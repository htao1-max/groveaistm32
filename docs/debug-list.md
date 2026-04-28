# Debug list — known issues to revisit

Running list of bugs / suspected races that have been diagnosed but not yet
fixed. Each entry: symptom, suspected root cause, proposed fix, status.

---

## 1. Scattered color-line corruption in saved JPEGs

- **First noted:** 2026-04-21
- **Where:** `SESSION_XXXX/ALL/img_*.jpg` (and presumably `DETECT/det_*.jpg`)
  on Himax SD card, tflm_yolov8_od_sdlog scenario.
- **Symptom:** Some saved JPEGs open to full size but have random streaks of
  random-coloured pixel rows at random positions within the image.
  Intermittent, no fixed correlation with frame number; file size is
  normal (container bytes intact).
- **What it is NOT:** not caused by I2C RX buffer overflow. The I2C path
  (`gRead_buf`, `sdlog_log_*` ring) touches completely separate memory
  and cannot affect the JPEG pixel buffer.

- **Suspected root cause:** XDMA-ring vs. `f_write` race on the JPEG
  output buffer.
  - `cisdp_get_jpginfo()` (cisdp_sensor.c:655–668 for OV5647 variant)
    returns the previously-completed slot in a short XDMA ring
    (`buffer_no`, typically 2–4 slots, managed by `hx_drv_jpeg_*`).
  - Scenario calls `sdlog_save_all(jpeg_addr, jpeg_sz, ...)` which runs a
    blocking SPI-to-SD `f_write` for tens of ms.
  - Sensor does not pause during the write. At ~30 fps, XDMA advances
    one slot per ~33 ms. If the ring wraps back into the slot `f_write`
    is reading, the bytes being streamed to SD mix old-frame and
    new-frame content.
  - JPEG is entropy-coded, so corruption of a handful of bytes cascades
    into visible block-row stripes of random colour until the decoder
    re-syncs at the next restart marker — exactly matching the observed
    symptom.
  - Correlation check this predicts: larger (more-detailed) scenes
    produce bigger JPEGs → longer write times → more frequent
    corruption. Confirm when investigating.

- **Candidate fixes (pick one later):**
  - **A. Copy JPEG to private scratch buffer before `f_write`**
    (recommended). In `sdlog_write_image`:
    `memcpy(scratch, (void*)addr, sz); f_write(&fil, scratch, sz, &bw);`
    Decouples SD latency from the XDMA ring. Needs ~64 KB scratch (JPEGs
    are small but size budget for worst case).
  - **B. Increase XDMA ring depth** (cisdp_sensor config) so a 30 fps
    sensor can't lap a 100 ms SD write. Touches driver layer; may bump
    SRAM budget.
  - **C. Pause sensor / XDMA during save.** Simple but drops frames and
    blinds detection every save. Poor fit when `ALL/` saves every frame.
  - **D. Speed up `f_write`** (FatFs cluster size, `f_expand`
    preallocation, 512 B-aligned writes). Reduces but does not remove
    the race. Complements A.

- **Status:** Diagnosed only. Not implemented. Revisit when we touch the
  scenario's save path again, or when fixing this is blocking analysis.

- **Evidence needed before committing to fix A:**
  - Confirm the current XDMA `buffer_no` for the OV5647 path.
  - Measure a typical `f_write(jpeg_sz)` time at 400 kHz SPI to SD.
  - Save two back-to-back frames and verify the larger one is more
    likely to corrupt.

---

## 2. Telemetry first batch lost — Himax single-mailbox RX coalescing

- **First noted:** 2026-04-26
- **Where:** `SESSION_XXXX/telemetry.csv` on Himax SD, smoke test path in
  STM32 `main.c` (`logTelemetryToHimax` × 8 → 2 batched I2C frames).
- **Symptom:** Only the second batch's 4 samples (i=4..7) reach
  `telemetry.csv`. The first batch's 4 samples (i=0..3) are silently
  dropped. Confirmed across two runs; persists with the smoke
  per-sample delay widened from 20 ms to 100 ms (gap between batch
  flushes raised from ~91 ms to ~400 ms).
- **Diagnostic trace (Himax UART):**

  ```
  i2cs_cb_rx(iic_id:0)         ← batch 1 arrives, fills gRead_buf
  i2cs_cb_rx(iic_id:0)         ← batch 2 arrives, OVERWRITES gRead_buf
  evt_i2ccomm_0_rx_cb(0x83)    ← handler runs ONCE — sees only batch 2
  [I2C_TLM] enqueued 256 bytes (4 samples)
  ```

  Two RX ISR callbacks, one event-handler invocation. Classic single-
  mailbox + event-coalescing pattern.

- **Root cause:** the deferred sensor-stream design
  (`docs/superpowers/specs/2026-04-21-sensor-stream-logging-design.md`)
  identified this exactly: `gRead_buf[USE_DW_IIC_SLV_0]` is a single
  buffer, and `hx_event_activate_ISR` coalesces multiple activations
  into one. When the scenario loop is busy (JPEG `f_write` during
  active recording), back-to-back I2C frames clobber each other in the
  mailbox before the event handler drains them.
  - `startRecordingForHimax` runs at boot, **before** the smoke. So
    JPEG saves are active during the smoke window.
  - Either JPEG `f_write` stalls on this hardware exceed 400 ms (worse
    than the deferred spec's 50 Hz / ~20 ms estimate), or the smoke
    happens during the heavy boot-burst when ALL/ saves every frame at
    sustained 30 fps and the scenario loop is hot.

- **Confirmed NOT the cause:**
  - SPSC ring overflow — only 2 frames, ring is 4 slots, no
    `[SDLOG_TLM] dropped ... frames` log line appeared.
  - `bad plen` rejection — earlier issue, fixed by re-flashing Himax
    with the schema bump (`I2C_TLM enqueued 256 bytes` now logs OK).
  - Float-formatting bug — separately fixed in `85cf72d`; rows that
    DO land in the file have correct decimal values now.

- **Candidate fixes (pick one later):**
  - **A. Move smoke before `startRecordingForHimax` in STM32 `main.c`.**
    Verifies CSV formatting cleanly with no JPEG races. Smoke-only
    workaround; does NOT solve the production problem.
  - **B. Implement Himax-side double-buffer / inbound FIFO inside
    `i2cs_cb_rx`** — option Z from the deferred spec. Memcpy
    `gRead_buf` into a small RAM FIFO before `hx_event_activate_ISR`,
    so back-to-back frames don't clobber. Touches shared driver code
    in `evt_i2ccomm.c` / `hx_lib_i2ccomm`. Real fix, larger scope.
  - **C. STM32-side serialize-with-ack.** STM32 waits for an explicit
    "frame consumed" ack from Himax before sending the next batch.
    Adds a return path on the I2C bus — invasive on both sides.

- **Status:** **Fixed 2026-04-27.** Implemented as the Himax RX
  mailbox FIFO (spec
  `docs/superpowers/specs/2026-04-27-himax-rx-mailbox-fifo-design.md`,
  plan `docs/superpowers/plans/2026-04-27-himax-rx-mailbox-fifo.md`).
  Final design: 16-slot per-iic_id SPSC FIFO interposed between
  `i2cs_cb_rx` ISR and main-loop drain; consumer reads FIFO slots
  directly (never `gRead_buf`); ISR re-arms HW immediately after
  enqueue; pending-flag guard around `hx_event_activate_ISR`; debug
  prints silenced in the per-frame hot path. Verified on hardware
  (SESSION_0096): 1808/1808 telemetry rows at 30 Hz × 60 s with
  `ALL/` JPEG recording running concurrently and uninterrupted (172
  contiguous frames, max gap 407 ms).

- **Reference:** spec
  `docs/superpowers/specs/2026-04-26-stm32-telemetry-binary-log-design.md`
  §"Why batching, not per-sample at 60 Hz" already cites this race;
  the production-rate verification (3500+ rows in 60 s) is the
  measurement that will quantify how bad it is.

---
