# Sensor-stream logging over logToHimax (10–50 Hz) — design

**Status:** Deferred. Capture the design; do not implement yet.
**Date:** 2026-04-21
**Supersedes / builds on:** `2026-04-19-logToHimax-design.md`

## Goal

Extend the existing STM32 → Himax → SD logging path so a periodic sensor
stream (gyro, accel, timestamps, any text-formatted sample at 10–50 Hz) can
be written durably into `SESSION_XXXX/session.log`, interleaved with the
existing BOOT / `[I2C]` / `[DETECT]` / free-form `[STM32]` events.

Analysis model: one time-ordered human-readable log file. Downstream
tooling (grep / awk / pandas with a regex) pulls per-tag streams out when
needed. No per-stream files (`GYRO.csv` etc.) in this iteration.

## Why the current approach isn't enough

The wire protocol (`feature 0x82 / cmd 0x01`, payload `tag\0msg\0`) is
already generic string-in / string-out. A gyro logger on STM32 can call
`logToHimax("GYRO", "t=%lu gx=%.3f gy=%.3f gz=%.3f", ...)` unchanged.

The **receive path on Himax is the bottleneck.** Specifically:

- `gRead_buf[USE_DW_IIC_SLV_0]` (evt_i2ccomm.c:83) is a single buffer.
  When an I2C frame completes, the hardware fills it and the RX callback
  sets `EVT_INDEX_I2CS_0_RX`. If a second frame arrives before the Himax
  main event loop drains that event, the second frame overwrites the
  first — and `hx_event_activate_ISR` coalesces the two activations into
  one, so only the later frame's payload is seen.
- The main scenario loop in `tflm_yolov8_od_sdlog.c` blocks for tens of
  milliseconds every time it performs a JPEG save
  (`sdlog_save_all` / `sdlog_save_detect`). During that window, any
  inter-frame gap shorter than the stall causes lost samples.
- Already observed with just two back-to-back frames (the `startRecording`
  0x80 frame clobbered by the first `logToHimax` 0x82 frame; 200 ms
  post-send delay was the workaround).

This is not an I2C-bus-speed problem. The bus at 400 kHz carries
~1400 short frames/sec. It is a **single-mailbox-on-the-receiver** problem.

Memory depth downstream of the mailbox (the 8-slot SPSC ring added in the
logToHimax work, sdlog.c) is fine: 8 slots × ~33 ms/frame = ~260 ms of
buffering, comfortably longer than any single JPEG save. The ring only
fills if the *mailbox* delivers into it faster than the scenario loop
drains — which doesn't happen if frame arrivals are spaced.

## Approach (recommended: Y — STM32-side batching)

Keep the I2C frame rate well below any plausible JPEG-save stall, while
keeping the per-sample SD rate at the sensor rate.

### STM32 side

- Add a per-stream `samplebatch_t` accumulator:
  ```
  tag            char[16]
  lines          char[N][~48]   /* N = 4..8, tuned to keep frame ≤220 B */
  count          uint8_t
  last_flush_ms  uint32_t
  ```
- Sensor ISR / polling loop calls `samplebatch_add(&gyro_batch, "t=%lu gx=%.3f …", ...)`.
- A cooperative flush runs from the main loop (or a HAL timer callback)
  at ~10 Hz: if `count == N` *or* `HAL_GetTick() - last_flush_ms >= 100`,
  concatenate `lines[0..count-1]` with `\n` separators into one buffer
  and send via the existing `logToHimax(tag, "<batched>")`.
- No changes to the `grove_send_cmd` / CRC / framing layer.

Result: up to ~100 ms batching latency per sample, I2C frame rate ≤ 10 Hz,
effective SD-log rate = sensor rate.

### Himax side

One small change in `sdlog_log_drain` (sdlog.c): when the enqueued `msg`
contains `\n`, split it and emit one `[TAG] <segment>\r\n` line per
segment. The ring entry stays as `(tag, msg)`; only the writer changes.

Pseudocode:
```c
const char *p = e->msg;
while (*p) {
    const char *nl = strchr(p, '\n');
    size_t len = nl ? (size_t)(nl - p) : strlen(p);
    xsprintf(line, "[%s] %.*s\r\n", e->tag, (int)len, p);
    f_write(&g_log_fil, line, strlen(line), &bw);
    p = nl ? nl + 1 : p + len;
}
```

No changes to `gRead_buf`, `hx_lib_i2ccomm_*`, or the event-handler
dispatch. No impact on other Seeed scenarios that share the driver.

### Recording coexistence

Already solved by the logToHimax deferred-drain design: the I2C event
handler only does a `memcpy` into the SPSC ring; `f_write` happens in the
scenario loop after `sdlog_save_all`. Adding batched sensor frames
inherits that property — the new work is all upstream of the ring.

## Alternatives considered (rejected for this iteration)

- **X. Per-sample frames with strict STM32 pacing.** Send one frame per
  sample, rely on `HAL_Delay` spacing. Max reliable rate ~33 Hz; 50 Hz
  drops samples whenever a JPEG save stretches past the gap. Batching
  is strictly better for the same code cost.
- **Z. Himax ISR-side double-buffer / inbound FIFO.** Memcpy `gRead_buf`
  into a small FIFO inside `i2cs_cb_rx` before activating the event,
  so back-to-back frames don't clobber each other. Most general fix
  (works up to I2C wire speed) but touches `evt_i2ccomm.c` / possibly
  `hx_lib_i2ccomm` — shared driver code used by other Seeed scenarios.
  Revisit only if a future use case needs >50 Hz or cannot tolerate
  batching latency.

## Scope boundaries (explicit YAGNI list)

- No per-stream files. Everything into `session.log`.
- No binary / CSV-structured payload. Text lines only.
- No back-pressure signal from Himax to STM32. Drops stay silent on
  STM32; the Himax ring-overflow counter (`g_ring_dropped` in sdlog.c)
  continues to be the only visibility.
- No multi-sensor priority or scheduling on STM32. Each sensor owns its
  own `samplebatch_t`; the main loop flushes whichever is ready.

## Open questions (resolve before implementing)

1. STM32 flush trigger — HAL `SysTick`-driven callback, or cooperative
   poll in the main `while(1)`? Former needs a timer; latter is simpler
   but couples flush cadence to whatever else the main loop does.
2. Per-sample line budget. At N=5 samples × ~40 chars + separators the
   frame is ~210 B, inside the current 220 B payload cap. If a sensor
   formats wider, either N must drop or the Himax `SDLOG_RING_MSG`
   bumps (currently 208).
3. Overflow policy on STM32 — if a sensor produces samples faster than
   flush drains (shouldn't at 50 Hz, but defensive): drop newest, drop
   oldest, or block? Pick before coding.

## Non-goals

- High-rate (100 Hz+) streaming. Out of scope; requires Approach Z or a
  different transport (SPI slave, DMA ring) — separate design.
- Strict real-time / sub-ms sample timestamps. Sample timestamps come
  from STM32 tick at batch-add time, not at wire-send time; batching
  latency is not compensated for.
