# `logTelemetryToHimax` — usage guide

Send periodic sensor telemetry from the STM32 to the Himax, where it
lands in `SESSION_XXXX/telemetry.csv` on the Himax SD card. Verified
zero-loss at 30 Hz with concurrent JPEG recording (SESSION_0096:
1808/1808 rows, 172 contiguous JPEGs, no drops).

## API

```c
#include "himax_sdk.h"

typedef struct {
    float    q[4];          /* quaternion w, x, y, z              */
    float    temp_c;        /* IMU / board temperature, deg C     */
    float    vbat;          /* battery voltage, V                 */
    float    vmotor[4];     /* motor 1..4 supply voltage, V       */
    float    imotor[4];     /* motor 1..4 current, A              */
    float    depth;         /* depth sensor, m                    */
    uint32_t stm32_tick_ms; /* filled by logTelemetryToHimax()    */
} telemetry_t;

void logTelemetryToHimax(telemetry_t *t);
```

Fields are 15 × `float32` + 1 × `uint32_t` = 64 bytes per sample on the
wire (little-endian). `stm32_tick_ms` is overwritten with `HAL_GetTick()`
at call time — do not pre-fill it.

## Minimal example

```c
telemetry_t t = {0};
t.q[0] = qw;  t.q[1] = qx;  t.q[2] = qy;  t.q[3] = qz;
t.temp_c    = imu_temp_c();
t.vbat      = adc_vbat();
t.vmotor[0] = adc_vmotor(0);  /* ... 1..3 */
t.imotor[0] = adc_imotor(0);  /* ... 1..3 */
t.depth     = depth_sensor_m();

logTelemetryToHimax(&t);   /* fire and forget */
```

## How it works

- The function copies the sample into an internal 4-slot accumulator
  (`s_tlm_batch`).
- When the 4th sample is buffered, one I2C frame (256 B payload, feature
  `0x83`, cmd `0x01`) is flushed to Himax. The first three calls return
  immediately without bus traffic.
- Himax-side: ISR copies the frame into a 16-slot per-iic_id mailbox
  FIFO; main-loop drain hands it to `sdlog_tlm_enqueue`, which puts it
  in a 16-slot SPSC ring. `sdlog_tlm_drain` (called every scenario-loop
  iteration) pops samples and `f_write`s one CSV row each into
  `telemetry.csv`. Pipeline is described in
  `docs/superpowers/specs/2026-04-27-himax-rx-mailbox-fifo-design.md`.

## Calling cadence

- **Tested:** 30 Hz call rate (`HAL_Delay(33)`), sustained for 60 s.
  Wire rate is 7.5 Hz (one frame per 4 calls). Zero loss with concurrent
  `ALL/` JPEG recording.
- **Theoretical headroom:** mailbox FIFO + TLM ring give ~4 s of stall
  coverage at 7.5 Hz wire rate. If `[I2CCOMM] dropped …` or
  `[SDLOG_TLM] dropped …` ever appears on Himax UART, a JPEG-save
  stall exceeded the buffer depth — see "tuning" below.
- **Latency:** unbounded. Sample timestamp is `HAL_GetTick()` at call
  time, so downstream tooling can reconstruct the order even when
  batching delays the wire send by up to ~530 ms (4 samples × 133 ms).

## Pre-flight

Before the first call:

1. `initForHimax()` — opens I2C1, probes Himax at `0x62`.
2. `startRecordingForHimax()` — Himax creates `SESSION_XXXX/`,
   opens `telemetry.csv`, writes the CSV header.

If `startRecordingForHimax` was not called, telemetry frames are
silently dropped on the Himax side (no `telemetry.csv` to write to).

## CSV format on Himax

Header:

```
stm32_tick_ms,qw,qx,qy,qz,temp_c,vbat,vm1,vm2,vm3,vm4,im1,im2,im3,im4,depth,himax_recv_ms
```

`himax_recv_ms` is the Himax tick at the moment the row is written —
useful for measuring end-to-end pipeline latency.

## Constraints and gotchas

- **Not ISR-safe.** Call from main-loop / task context only. Internal
  state (`s_tlm_count`, batch buffer) is not protected.
- **One sample per call.** Don't try to feed multiple samples by
  reusing the same `telemetry_t` and calling repeatedly without
  refreshing fields — the batch will hold 4 copies of the last value.
- **No tail flush.** Samples 1-3 of a partial batch sit in RAM until
  the 4th call arrives. If your run ends mid-batch, those samples are
  lost. Send dummy samples or accept the truncation.
- **No back-pressure.** STM32 doesn't know if Himax dropped a frame.
  Watch the Himax UART for `[I2CCOMM] dropped` / `[SDLOG_TLM] dropped`
  diagnostics during development.

## Verification harness

`Core/Src/main.c` ships two compile-time-gated harnesses next to the
peripheral init:

- `DEBUG_LOGTLM_SMOKE` — 8 samples at 100 ms cadence (= 2 batched I2C
  frames). Verifies the path end-to-end. Expect 8 rows in `telemetry.csv`.
- `DEBUG_LOGTLM_30HZ` — 1800 samples at 33 ms cadence (= 60 s at 30 Hz).
  Verifies sustained throughput with concurrent JPEG recording. Expect
  1800 rows. Disabled by default; uncomment its `#define` to run.

## Tuning if `dropped` ever fires

Both buffers are sized for ~4 s of stall coverage at 7.5 Hz wire rate.
If real workload pushes past that:

- Bump `EVT_I2CCOMM_RX_FIFO_SLOTS` in
  `EPII_CM55M_APP_S/app/scenario_app/event_handler/evt_i2ccomm/evt_i2ccomm.c`
  (currently 16; doubles per id at 256 B/slot).
- Bump `SDLOG_TLM_RING_SLOTS` in
  `EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/sdlog.c`
  (currently 16; ~258 B/slot).
- Consider increasing `TLM_BATCH_SIZE` (currently 4) to lower the wire
  frame rate further — costs latency, gains headroom.

## Related files

- STM32 side: `Core/Inc/himax_sdk.h`, `Core/Src/himax_sdk.c` (around
  line 232 for `logTelemetryToHimax`, line 168 for the LE pack
  helper).
- Himax side (separate `Seeed_Grove_Vision_AI_Module_V2` repo, branch
  `red-circle-model`): `evt_i2ccomm.c` for the mailbox FIFO,
  `tflm_yolov8_od_sdlog/i2c_cmd.c` for the 0x83 dispatch,
  `tflm_yolov8_od_sdlog/sdlog.c` for the SPSC ring + CSV writer.
- Design: `docs/superpowers/specs/2026-04-26-stm32-telemetry-binary-log-design.md`,
  `docs/superpowers/specs/2026-04-27-himax-rx-mailbox-fifo-design.md`.
