# STM32 → Himax binary telemetry log (60 Hz, batched) — design

**Status:** Design approved by user 2026-04-26. Ready for implementation plan.
**Date:** 2026-04-26
**Builds on:** `2026-04-19-logToHimax-design.md`, `2026-04-21-sensor-stream-logging-design.md`

## Goal

Stream a fixed schema of structured numeric telemetry from the STM32 to the
Himax SD card at an effective rate of **60 Hz**, written as one CSV row per
sample to a dedicated `telemetry.csv` on the SD. Schema is fixed and known
to both ends; no string formatting on the wire.

Fields, in fixed order:

1. `q[0]` — orientation quaternion w (`float32`)
2. `q[1]` — orientation quaternion x (`float32`)
3. `q[2]` — orientation quaternion y (`float32`)
4. `q[3]` — orientation quaternion z (`float32`)
5. `temp_c` — temperature, Celsius (`float32`)
6. `vbat` — battery voltage, V (`float32`)
7. `vmotor[0]` — motor 1 voltage, V (`float32`)
8. `vmotor[1]` — motor 2 voltage, V (`float32`)
9. `vmotor[2]` — motor 3 voltage, V (`float32`)
10. `vmotor[3]` — motor 4 voltage, V (`float32`)
11. `stm32_tick_ms` — STM32 `HAL_GetTick()` at sample time (`uint32`)

Per-sample wire payload = 10 × 4 + 4 = **44 bytes**.

## Why a new feature, not an extension of `logToHimax`

`logToHimax` is `tag\0msg\0` text. Sending 11 floats as text would be
~80–110 bytes per sample (`"%.6f,%.6f,..."`), and Himax would still need
to re-parse them to write a structured CSV. Binary on the wire saves
parsing on Himax, makes the schema explicit, and keeps the existing
free-form text logger uncluttered.

Allocate a new feature/command pair:

- `I2C_FEATURE_TLM = 0x83`
- `I2C_CMD_TLM_WRITE = 0x01`

Reusing the existing `grove_send_cmd` framing/CRC unchanged.

## Why batching, not per-sample at 60 Hz

The deferred sensor-stream design
(`2026-04-21-sensor-stream-logging-design.md`) established that the Himax
I2C RX path is a **single mailbox** (`gRead_buf[USE_DW_IIC_SLV_0]`): if a
second frame arrives before the Himax main event loop drains the first,
the second clobbers the first. Empirically this caps reliable per-frame
rate at ~33 Hz, with 50 Hz already losing samples during JPEG saves.

60 Hz **per-sample** would drop samples whenever recording is active —
unacceptable for control-loop telemetry, where drops correlated with
image writes are systematically biased.

Solution: **batch 4 samples per I2C frame on the STM32 side.**

- Per-frame payload = 4 × 44 = **176 bytes** (well under the 256 B
  `grove_send_cmd` limit; total I2C packet = 4 + 176 + 2 = 182 B).
- Wire frame rate = 60 / 4 = **15 Hz** — comfortably below the 33 Hz
  Himax-side ceiling.
- Worst-case batching latency = 4 × 16.67 ms = **66.7 ms** — invisible
  for post-hoc analysis; acceptable for any non-hard-realtime use.
- Effective `telemetry.csv` row rate = full **60 Hz**.

Rejected alternatives:

- **Per-sample at 60 Hz, accept drops.** Drops are correlated with image
  writes — biases the dataset.
- **Compression / base64.** base64 *expands* by 33%; real compression on
  44 B of float-bit noise has overhead exceeding savings; bus bandwidth
  is not the bottleneck (50-byte frame = 1.1 ms at 400 kHz).
- **Himax-side double-buffer / inbound FIFO** (option Z in the prior
  spec). Largest scope — touches shared Seeed driver code. Revisit only
  if a future use case can't tolerate batching latency.

## Architecture

```
STM32                                Himax (Grove AI V2)
─────                                ───────────────────
fill telemetry_t                     i2ccomm RX
  q[4], temp_c, vbat,                  → feature dispatch
  vmotor[4]                            → I2C_FEATURE_TLM (0x83) handler
  stm32_tick_ms = HAL_GetTick()        → CRC verify
        │                              → memcpy into deferred-drain ring
        ▼                              (one ring entry per I2C frame)
  tlm_batch_add(&t)                          │
        │  (accumulator: 4 samples)          ▼
        ▼                              scenario loop drain:
  every 4 samples → flush:               for each frame:
    grove_send_cmd(                        for sample in 1..4:
      0x83, 0x01,                            himax_recv_ms = own tick
      pack[4*44]=176B)                       fprintf 1 CSV row
        │                                    f_write (no sync)
        └── 182-byte I2C frame at         every 60th sample: f_sync
            ~15 Hz (FM 400 kHz, ~3.7 ms)
```

### STM32 side

New types and functions in `Core/Inc/himax_sdk.h` /
`Core/Src/himax_sdk.c`:

```c
typedef struct {
    float    q[4];          /* quaternion w,x,y,z */
    float    temp_c;
    float    vbat;
    float    vmotor[4];     /* motors 1..4 */
    uint32_t stm32_tick_ms; /* filled by logTelemetryToHimax() */
} telemetry_t;

#define I2C_FEATURE_TLM      0x83
#define I2C_CMD_TLM_WRITE    0x01
#define TLM_BATCH_SIZE       4
#define TLM_SAMPLE_BYTES     44   /* 10*float32 + 1*uint32 */

/* Caller fills t->q, temp_c, vbat, vmotor; tick is filled internally.
 * Adds the sample to the batch accumulator. When the accumulator
 * holds TLM_BATCH_SIZE samples, flushes one I2C frame to Himax.
 * Fire-and-forget. Not ISR-safe. */
void logTelemetryToHimax(telemetry_t *t);
```

Internal state (file-static in `himax_sdk.c`):

```c
static uint8_t  s_tlm_batch[TLM_BATCH_SIZE * TLM_SAMPLE_BYTES];
static uint8_t  s_tlm_count;
```

Behavior:

1. Caller fills the struct (tick is `0` or anything; we overwrite).
2. `logTelemetryToHimax` sets `t->stm32_tick_ms = HAL_GetTick()`.
3. Pack the 11 fields little-endian into
   `s_tlm_batch + s_tlm_count * 44`. STM32G4 is little-endian native, so
   pack via byte-wise `memcpy` of each field (do not rely on struct
   layout — explicit packing avoids alignment / padding surprises).
4. `s_tlm_count++`.
5. If `s_tlm_count == TLM_BATCH_SIZE`: call
   `grove_send_cmd(0x83, 0x01, s_tlm_batch, 4*44)`, reset count, UART
   echo on success, UART warn on HAL failure.

Call site: main loop, throttled with `HAL_GetTick()` so
`logTelemetryToHimax` is called every ~17 ms (60 Hz). The flush only
happens every 4th call (~67 ms / ~15 Hz), so the I2C blocking cost is
amortised.

### Himax side

New case in the i2ccomm dispatcher (mirror of the existing
`I2C_FEATURE_LOG = 0x82` handler):

```c
case I2C_FEATURE_TLM: {  /* 0x83 */
    if (cmd != I2C_CMD_TLM_WRITE) break;
    if (plen == 0 || plen % 44 != 0) {
        xprintf("[I2C_TLM] bad payload len %u\n", plen);
        break;
    }
    /* enqueue (feature, plen, payload) into the existing
     * deferred-drain ring used by logToHimax — same race-avoidance
     * as feature 0x82. The ring entry already supports a binary
     * blob; we add a tag-discriminator so the drain can route. */
    sdlog_tlm_enqueue(payload, plen);
    break;
}
```

Drain (in scenario loop, alongside `sdlog_log_drain`):

```c
void sdlog_tlm_drain(void) {
    while (sdlog_tlm_dequeue(&blob, &blen)) {
        for (off = 0; off < blen; off += 44) {
            unpack_le_floats_uint(blob + off, &q0..q3, &temp, &vbat,
                                  &vm1..vm4, &stm32_tick);
            uint32_t himax_recv_ms = own_tick_ms();
            xsprintf(line,
                "%lu,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%lu\r\n",
                stm32_tick, q0, q1, q2, q3, temp, vbat,
                vm1, vm2, vm3, vm4, himax_recv_ms);
            f_write(&g_tlm_fil, line, strlen(line), &bw);
            if (++g_tlm_sync_counter >= 60) {
                f_sync(&g_tlm_fil);
                g_tlm_sync_counter = 0;
            }
        }
    }
}
```

`himax_recv_ms` is **per-sample, not per-frame** — it's the Himax tick
at the moment the drain unpacks that sample. Captures both per-frame
arrival timing and per-sample drain timing differences in one column.

### File lifecycle on Himax

`telemetry.csv` is opened at SD mount (in the same init path as
`session.log`), header written immediately:

```
stm32_tick_ms,qw,qx,qy,qz,temp_c,vbat,vm1,vm2,vm3,vm4,himax_recv_ms
```

Closed at session end / shutdown. Always-on: not gated on recording
state, matching `logToHimax`.

If a telemetry frame arrives before the SD is mounted, drop it silently
(consistent with `logToHimax`).

## Wire format detail

Per-sample 44 bytes, little-endian:

| offset | size | field           | type    |
|--------|------|-----------------|---------|
| 0      | 4    | q[0] (w)        | float32 |
| 4      | 4    | q[1] (x)        | float32 |
| 8      | 4    | q[2] (y)        | float32 |
| 12     | 4    | q[3] (z)        | float32 |
| 16     | 4    | temp_c          | float32 |
| 20     | 4    | vbat            | float32 |
| 24     | 4    | vmotor[0]       | float32 |
| 28     | 4    | vmotor[1]       | float32 |
| 32     | 4    | vmotor[2]       | float32 |
| 36     | 4    | vmotor[3]       | float32 |
| 40     | 4    | stm32_tick_ms   | uint32  |

Per-frame: 4 samples concatenated, no inter-sample padding, no frame
header beyond what `grove_send_cmd` already adds (`feature, cmd, plen_lo,
plen_hi`, then 176 B payload, then CRC16).

Both STM32G4 and the Himax MCU are little-endian, so reads/writes use
byte-wise pack/unpack (no host-order assumption).

## Error handling

| Condition                           | Behavior                                            |
|-------------------------------------|------------------------------------------------------|
| `t == NULL`                         | Return early (mirror `logToHimax` NULL guard).       |
| `HAL_I2C_Master_Transmit` fails     | UART warn `[logTlmToHimax] I2C TX failed rc=%d`; reset batch counter (drop the 4 samples — the next 4 batch will go through). |
| Himax: bad `plen` (not multiple of 44) | Log to UART, drop the frame.                       |
| Himax: SD not mounted               | Drop silently.                                       |
| Himax: `f_write` returns error      | Log once per session to UART, continue (don't spam). |

No retry on the STM32 side (fire-and-forget, same as `logToHimax`). At
60 Hz nominal, occasional drops are a non-event.

## Testing

### STM32-side unit-ish tests

Hard to add real unit tests in this CubeIDE workflow, but at minimum:

1. **Pack correctness**: in `main.c` (or a temporary scratch), populate a
   `telemetry_t` with known values, call the pack routine into a local
   buffer, dump it over UART4 in hex, and verify each 44-byte block
   matches the expected little-endian layout.
2. **Batch trigger**: call `logTelemetryToHimax` 4 times in a row and
   confirm via UART4 that exactly one I2C TX log line appears
   (`[->himax tlm] count=4 ...`).

### End-to-end smoke test (mirrors logToHimax E2E)

In `main.c`, after `initForHimax()` returns OK:

1. Send 8 telemetry samples with synthetic ramping values (sample N
   has `q[0] = N * 0.1`, `temp_c = 20.0 + N`, etc.).
2. Expect 2 I2C frames flushed to Himax.
3. Verify on Himax UART: `[I2C_TLM]` enqueue logs.
4. Pull the SD: `telemetry.csv` should have header + 8 rows with the
   ramped values, two `himax_recv_ms` cohorts (one per frame).

### Sustained-rate test

Run the full 60 Hz path for 60 seconds during an active recording
session (so JPEG saves are happening). Verify:

- `telemetry.csv` has 3600 ± a few rows. Anything below 3500 indicates
  frame loss in the I2C RX mailbox — escalate to the Himax-side FIFO
  fix (option Z in the prior deferred spec).
- `stm32_tick_ms` deltas cluster around 17 ms (60 Hz nominal).
- `himax_recv_ms` deltas show 4-sample bursts ~67 ms apart.

## Scope boundaries (explicit YAGNI)

- **No schema versioning byte** in the payload. Schema is fixed for
  this iteration. If we add fields later, bump the cmd code
  (`I2C_CMD_TLM_WRITE_V2 = 0x02`).
- **No back-pressure / ack** from Himax. STM32 is fire-and-forget.
- **No per-recording-session file rotation.** Single
  `telemetry.csv` for the SD lifetime.
- **No runtime-configurable rate or batch size.** Both compile-time
  constants for v1.
- **No Himax-side FIFO / double-buffer fix** for the single-mailbox
  race. Batching keeps wire rate well below the threshold; revisit
  only if measurements show drops.
- **No interaction with the JPEG f_write race** beyond using the
  existing deferred-drain ring (which already isolates I2C RX from
  the SD writer).

## Open questions deferred

- Sensor source for the actual quaternion / motor voltages — out of
  scope for this design. The API is a pure logging sink; whatever
  produces the values is a separate concern.
- Power-of-2 batching (16 or 32 samples for a faster wire rate) — not
  needed for v1 since 4 already gets us to 15 Hz wire rate. Revisit if
  CPU cost of the 15 Hz blocking I2C call ever shows up in profiling.

## Files touched

- **STM32 (`i2cScan` repo, branch TBD by writing-plans):**
  - `Core/Inc/himax_sdk.h` — add `telemetry_t`, `I2C_FEATURE_TLM`,
    `I2C_CMD_TLM_WRITE`, `TLM_BATCH_SIZE`, prototype.
  - `Core/Src/himax_sdk.c` — implementation: pack helper, batch state,
    `logTelemetryToHimax`.
  - `Core/Src/main.c` — E2E smoke test calls (gated under
    `#ifdef DEBUG_LOGTLM_SMOKE` per the holistic-review feedback on
    `logToHimax`).

- **Himax (`Seeed_Grove_Vision_AI_Module_V2` repo, branch TBD):**
  - I2C dispatcher (`i2c_cmd.c` or equivalent) — `case 0x83`.
  - `sdlog.c` (or new `sdlog_tlm.c`) — `g_tlm_fil`, header init,
    `sdlog_tlm_enqueue` / `sdlog_tlm_dequeue` / `sdlog_tlm_drain`,
    f_sync counter.
  - Scenario loop (`tflm_yolov8_od_sdlog.c`) — call `sdlog_tlm_drain`
    alongside the existing `sdlog_log_drain`.

## Success criteria

1. `telemetry.csv` is created on SD at session start with the header row.
2. After 60 s of operation with sustained `logTelemetryToHimax` calls at
   60 Hz, the file contains ≥ 3500 rows (≤ 3% loss).
3. Each row contains 12 well-formed numeric columns; quaternion values
   reproduce caller inputs to ≥ 6 significant figures.
4. `pandas.read_csv("telemetry.csv")` loads cleanly and produces the
   expected dtypes (`uint32` for the two tick columns, `float64` for
   the rest).
5. No regression in `session.log` or `logToHimax` behavior.
