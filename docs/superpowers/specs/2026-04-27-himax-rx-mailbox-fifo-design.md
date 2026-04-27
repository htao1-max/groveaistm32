# Himax I2C-slave RX mailbox FIFO — design

**Status:** Approved for implementation.
**Date:** 2026-04-27
**Related / supersedes (in part):** `2026-04-21-sensor-stream-logging-design.md`
(deferred Approach Z — this spec implements that approach for the
generic case).
**Bug it fixes:** `docs/debug-list.md` §2 — telemetry first-batch loss
caused by Himax single-mailbox RX coalescing.

## Goal

Make the Himax I2C-slave receive path tolerate back-to-back frames that
arrive while the scenario main loop is blocked (e.g., on JPEG `f_write`
during active recording), so that high-frequency STM32 → Himax data
transmission can run without losing frames.

Concrete target: **30 Hz telemetry, 4 samples per I2C batch (= 7.5 Hz
wire rate), zero data loss, JPEG/recording loop never preempted.**

The fix is in the shared driver layer (`evt_i2ccomm.c`), so it benefits
every present and future feature that uses the same path — telemetry
(0x83), `logToHimax` (0x82), boot/recording (0x80), and the deferred
sensor-stream feature.

## Why the current design loses data

`gRead_buf[USE_DW_IIC_SLV_0]` is a single buffer shared between the I2C
hardware and the main-loop event handler:

1. HW completes a frame → `i2cs_cb_rx` ISR → `hx_event_activate_ISR`.
2. Before the main loop services the event, a second frame arrives.
   HW writes it into the same `gRead_buf`, clobbering frame #1.
3. `hx_event_activate_ISR` coalesces both activations into a single
   wake — so the rx_cb runs once and sees only frame #2.

This was diagnosed in `docs/debug-list.md` §2 with a clean trace: two
`i2cs_cb_rx` prints, one `evt_i2ccomm_0_rx_cb`, only the second batch's
samples reaching `telemetry.csv`. Spec
`2026-04-21-sensor-stream-logging-design.md` already cited this as
Approach Z and deferred it; this spec implements it.

## Architecture

Insert a per-iic-id SPSC ring between the ISR and the event handler.
The ISR memcpys the just-completed `gRead_buf` into the next ring slot
before activating the event. The event handler drains all queued slots
in a `while (tail != head)` loop, restoring each slot into `gRead_buf`
before invoking the existing per-feature dispatch — so every scenario's
`i2c_cmd.c` consumer keeps working unchanged.

```
ISR  i2cs_cb_rx
   │
   ├─ memcpy gRead_buf[id]        →  s_rx_fifo[id][head % 16]
   ├─ s_rx_head[id]++              (or s_rx_dropped[id]++ if full)
   └─ hx_event_activate_ISR(EVT_INDEX_I2CS_<id>_RX)

main loop  evt_i2ccomm_<id>_rx_cb()
   while (s_rx_tail[id] != s_rx_head[id]):
       memcpy s_rx_fifo[id][tail % 16]  →  gRead_buf[id]
       prv_evt_i2ccomm_dispatch_one(id)        ← existing switch(feature)
       s_rx_tail[id]++
   (re-arm hx_lib_i2ccomm_enable_read happens inside dispatch_one,
    same call sites as today — no change to driver layer)
```

## Components

### New static state in `evt_i2ccomm.c`

```c
#define EVT_I2CCOMM_RX_FIFO_SLOTS  16U   /* power of 2 */

static uint8_t  s_rx_fifo[DW_IIC_S_NUM]
                         [EVT_I2CCOMM_RX_FIFO_SLOTS]
                         [I2CCOMM_MAX_RBUF_SIZE]
                __ALIGNED(__SCB_DCACHE_LINE_SIZE);
static volatile uint32_t s_rx_head[DW_IIC_S_NUM];     /* ISR writes */
static volatile uint32_t s_rx_tail[DW_IIC_S_NUM];     /* main writes */
static volatile uint32_t s_rx_dropped[DW_IIC_S_NUM];  /* ISR bumps */
```

SRAM cost: `2 × 16 × 256 = 8192 B` plus 24 B of indices. Both iic ids
get a FIFO for symmetry; only id 0 is used in current scenarios.

### Modified `i2cs_cb_rx` (ISR)

Pseudocode:

```c
static void i2cs_cb_rx(void *param) {
    HX_DRV_DEV_IIC *iic_obj = param;
    USE_DW_IIC_SLV_E id;

    if      (iic_obj->iic_info.slv_addr == EVT_I2CS_0_SLV_ADDR) id = USE_DW_IIC_SLV_0;
    else if (iic_obj->iic_info.slv_addr == EVT_I2CS_1_SLV_ADDR) id = USE_DW_IIC_SLV_1;
    else return;

    uint32_t head = s_rx_head[id];
    uint32_t tail = s_rx_tail[id];
    if ((head - tail) < EVT_I2CCOMM_RX_FIFO_SLOTS) {
        memcpy(s_rx_fifo[id][head % EVT_I2CCOMM_RX_FIFO_SLOTS],
               gRead_buf[id], I2CCOMM_MAX_RBUF_SIZE);
        __DMB();
        s_rx_head[id] = head + 1;
    } else {
        s_rx_dropped[id]++;
    }

    hx_event_activate_ISR(g_event[id == USE_DW_IIC_SLV_0
                                  ? EVT_INDEX_I2CS_0_RX
                                  : EVT_INDEX_I2CS_1_RX]);
}
```

Memcpy cost on CM55M @ 400 MHz: 256 B in a few µs. ISR remains short.

### Refactor of `evt_i2ccomm_<id>_rx_cb`

Split today's body. The `unsigned char feature = gRead_buf[...]; switch
(feature) {...}` block moves into a new private helper
`prv_evt_i2ccomm_dispatch_one(USE_DW_IIC_SLV_E id)` with **no
behavioral change**. The existing `evt_i2ccomm_<id>_rx_cb` becomes:

```c
uint8_t evt_i2ccomm_0_rx_cb(void) {
    while (s_rx_tail[USE_DW_IIC_SLV_0] != s_rx_head[USE_DW_IIC_SLV_0]) {
        uint32_t t = s_rx_tail[USE_DW_IIC_SLV_0];
        memcpy(gRead_buf[USE_DW_IIC_SLV_0],
               s_rx_fifo[USE_DW_IIC_SLV_0][t % EVT_I2CCOMM_RX_FIFO_SLOTS],
               I2CCOMM_MAX_RBUF_SIZE);
        __DMB();
        prv_evt_i2ccomm_dispatch_one(USE_DW_IIC_SLV_0);
        s_rx_tail[USE_DW_IIC_SLV_0] = t + 1;
    }
    return HX_EVENT_RETURN_DONE;
}
```

The id=1 twin is identical with `USE_DW_IIC_SLV_1`.

### Drop visibility

Add to `evt_i2ccomm.h`:

```c
uint32_t evt_i2ccomm_get_rx_dropped(USE_DW_IIC_SLV_E iic_id);
```

In `tflm_yolov8_od_sdlog` scenario loop, alongside the existing
`[SDLOG_TLM] dropped …` print site (sdlog.c:521 region), add a
print-on-change for the mailbox FIFO drops:

```c
static uint32_t s_last_iic_dropped;
uint32_t cur = evt_i2ccomm_get_rx_dropped(USE_DW_IIC_SLV_0);
if (cur != s_last_iic_dropped) {
    xprintf("[I2CCOMM] dropped %lu rx frames (mailbox FIFO full)\r\n",
            (unsigned long)cur);
    s_last_iic_dropped = cur;
}
```

If this counter ever increments in production, it means the worst-case
JPEG-save stall exceeded the buffer depth — bump
`EVT_I2CCOMM_RX_FIFO_SLOTS` and/or `SDLOG_TLM_RING_SLOTS`.

### Symmetric bump of `SDLOG_TLM_RING_SLOTS`

In `sdlog.c`, change `SDLOG_TLM_RING_SLOTS` from `4` to `16` so that
total end-to-end buffering between I2C wire and SD is consistent:

```
mailbox FIFO  16 slots × 256 B = 4 KB  (per iic id)
TLM SPSC ring 16 slots × 256 B blob + meta = ~4.2 KB
```

Both buffers absorb the full stall together — at 7.5 Hz frame rate
(30 Hz / 4-batch), 16 slots ≈ 2.1 s coverage each, ≥4.2 s total
across both buffers. The existing `SDLOG_TLM_BLOB_MAX` (256 B,
= `TLM_BATCH_SIZE × TLM_SAMPLE_BYTES`) is unchanged.

## Concurrency, error, edge cases

- **SPSC discipline.** ISR writes only `s_rx_head` and `s_rx_dropped`.
  Main-loop handler writes only `s_rx_tail`. ISR cannot preempt itself
  (single-priority I2C interrupt). Index difference `head - tail` is
  correct under unsigned wraparound for `EVT_I2CCOMM_RX_FIFO_SLOTS ≤ 2³¹`.
- **Memory ordering.** `__DMB()` between `memcpy` and `head+1` publish
  in the ISR; `__DMB()` between reading the slot and `tail+1` publish
  in the consumer. Same pattern STM32-side `sdlog_log_*` ring uses.
- **Cache.** `gRead_buf` cache treatment is unchanged — the fix doesn't
  re-enable or remove any existing cache maintenance. The new FIFO is
  CPU-only memory (memcpy in, memcpy out), no DMA touches it, so no
  cache ops are needed for it.
- **Re-arm.** `hx_lib_i2ccomm_enable_read` calls inside
  `prv_evt_i2ccomm_dispatch_one` stay where they are today, on the same
  `gRead_buf[id]` pointer. The driver/HAL is not touched.
- **Overflow policy.** Drop-newest. Matches existing `g_ring_dropped`
  policy in `sdlog.c`. Visibility via `evt_i2ccomm_get_rx_dropped`.
- **Zero-loss claim.** Holds **as long as the worst JPEG-save stall is
  shorter than the combined mailbox FIFO + TLM ring depth** (~4 s at
  7.5 Hz wire rate). True end-to-end zero loss requires back-pressure
  (STM32-side ack, deferred-spec Approach C); not in this iteration.
- **Boot.** Static-zero init of head/tail/dropped is sufficient. If
  `evt_i2ccomm_init` runs on a re-init path, FIFO contents are
  irrelevant when `head == tail`.
- **Error path.** `i2cs_cb_err` is unchanged. Frames triggering err do
  not reach `i2cs_cb_rx` and never enter the FIFO.

## Backwards compatibility

Every existing consumer of `gRead_buf[id]` reads identical bytes at
identical offsets — the per-frame restore-into-`gRead_buf` step
preserves the pre-fix contract exactly. Touched files:

- `EPII_CM55M_APP_S/app/scenario_app/event_handler/evt_i2ccomm/evt_i2ccomm.c`
- `EPII_CM55M_APP_S/app/scenario_app/event_handler/evt_i2ccomm/evt_i2ccomm.h`
  (new accessor declaration)
- `EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/sdlog.c`
  (`SDLOG_TLM_RING_SLOTS` 4 → 16; new drop-print site for mailbox FIFO)

Untouched:

- `hx_lib_i2ccomm` (HAL).
- Every scenario's `i2c_cmd.c` (telemetry 0x83, logToHimax 0x82, sys
  0x80, recording 0x80 cmd 0x02, etc.).
- All STM32 firmware.

## Testing

1. **Smoke (existing).** Re-run `DEBUG_LOGTLM_SMOKE`: 8 samples → 2
   batched I2C frames → expect all 8 rows in `telemetry.csv` (today
   only the second batch's 4 rows land).
2. **Stall-tolerance test.** With `startRecordingForHimax` active and
   `ALL/` saves running, run `logTelemetryToHimax` continuously at
   30 Hz for 60 seconds. Expect ≥1800 rows in `telemetry.csv` and
   `[I2CCOMM] dropped …` never printed. This is the production-rate
   verification cited in
   `2026-04-26-stm32-telemetry-binary-log-design.md`.
3. **Coalescing diagnostic.** Confirm Himax UART shows the "two
   `i2cs_cb_rx`, two enqueues" pattern (today: two cb_rx, one enqueue).
4. **Other scenarios unaffected.** Boot allon_sensor scenario or any
   scenario with `evt_i2ccomm_init` and confirm the existing sysinfo /
   logToHimax features still respond correctly.

## YAGNI / non-goals

- No HAL changes. `hx_lib_i2ccomm` stays as-is.
- No back-pressure protocol (STM32 doesn't know whether a frame was
  enqueued or dropped). Add only if test 2 shows real losses.
- No per-feature priority on drain. `dispatch_one` runs in arrival
  order; no fast-path for any feature.
- No multi-buffer rotation in the HAL (rejected as Approach 2 during
  design — invasive, race-prone, and unnecessary given the cheap
  memcpy approach works).

## Open items

None — both sizing (16 slots each side) and overflow policy
(drop-newest, visibility via counter) decided during design.
