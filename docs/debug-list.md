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
