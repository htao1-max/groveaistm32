# STM32 Telemetry Binary Log Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `logTelemetryToHimax(telemetry_t *)` on STM32 so a fixed 11-field numeric telemetry sample (4× quaternion, temp, vbat, 4× motor V, STM32 tick) is appended at 60 Hz to a dedicated `telemetry.csv` on the Himax SD card.

**Architecture:** STM32 batches 4 samples per I2C frame (15 Hz wire rate, 60 Hz effective sample rate) using new feature `0x83` / cmd `0x01` over the existing `grove_send_cmd` framing. Himax dispatcher enqueues each frame into a new SPSC ring, scenario-loop drain unpacks 4 LE-binary samples per frame, stamps each with a Himax-side `himax_recv_ms`, and writes one CSV row per sample to `telemetry.csv` with `f_sync` every 60 rows.

**Tech Stack:** STM32G431 HAL C (`HAL_I2C_Master_Transmit`, `memcpy`-based LE pack), Himax i2ccomm + FatFS (`f_open` / `f_write` / `f_sync`, `xsprintf` for float formatting), CRC16-CCITT framing already used by `logToHimax`.

**Note on TDD:** Bare-metal firmware on two MCUs with no unit-test framework. "Tests" are manual hardware verifications: build → flash → observe UART trace + SD card contents. Each task ends with an explicit verification step.

**Reference spec:** `docs/superpowers/specs/2026-04-26-stm32-telemetry-binary-log-design.md`

**Branch:** Continue on existing branch `feature/i2ccomm-sdlog` (i2cScan) and `red-circle-model` (Seeed_Grove_Vision_AI_Module_V2). The `logToHimax` work on the same branches has not yet merged; this builds atop it.

**Files touched:**

STM32 (`i2cScan` repo):
- `Core/Inc/himax_sdk.h` — add `telemetry_t`, telemetry defines, prototype
- `Core/Src/himax_sdk.c` — add LE pack helper, batch state, `logTelemetryToHimax`
- `Core/Src/main.c` — E2E smoke test calls gated under `#ifdef DEBUG_LOGTLM_SMOKE`

Himax (`Seeed_Grove_Vision_AI_Module_V2/EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/`):
- `i2c_cmd.h` — add `I2C_FEATURE_TLM`, `I2C_CMD_TLM_WRITE`
- `i2c_cmd.c` — add `feature == 0x83` branch → `sdlog_tlm_enqueue`
- `sdlog.h` — add prototypes for `sdlog_tlm_init`, `sdlog_tlm_enqueue`, `sdlog_tlm_drain`
- `sdlog.c` — add `telemetry.csv` FIL, header write, SPSC ring, drain with sync-every-60 counter
- `tflm_yolov8_od_sdlog.c` — call `sdlog_tlm_init()` after `sdlog_session_init()`, call `sdlog_tlm_drain()` next to `sdlog_log_drain()`

---

## Task 1: STM32 — header changes

**Why first:** Defines and the public type are referenced by every subsequent STM32 task. No behavior change yet.

**Files:**
- Modify: `Core/Inc/himax_sdk.h`

- [ ] **Step 1: Add telemetry defines and `telemetry_t`**

In `Core/Inc/himax_sdk.h`, add the following block immediately after the `#define I2C_CMD_LOG_WRITE 0x01` line (line 19), and add the `logTelemetryToHimax` prototype after the existing `logToHimax` prototype:

```c
/* Telemetry binary log feature */
#define I2C_FEATURE_TLM       0x83
#define I2C_CMD_TLM_WRITE     0x01
#define TLM_BATCH_SIZE        4U     /* samples per I2C frame */
#define TLM_SAMPLE_BYTES      44U    /* 10*float32 + 1*uint32 */

typedef struct {
    float    q[4];          /* quaternion w,x,y,z */
    float    temp_c;
    float    vbat;
    float    vmotor[4];     /* motors 1..4 */
    uint32_t stm32_tick_ms; /* filled by logTelemetryToHimax() */
} telemetry_t;
```

After the existing `void logToHimax(...)` prototype:

```c
/* Append one telemetry sample to the batch accumulator. When the
 * accumulator holds TLM_BATCH_SIZE samples, flushes one I2C frame to
 * Himax. Caller fills q, temp_c, vbat, vmotor; stm32_tick_ms is filled
 * internally via HAL_GetTick(). Fire-and-forget. Not ISR-safe. */
void logTelemetryToHimax(telemetry_t *t);
```

- [ ] **Step 2: Verify compile**

Build in STM32CubeIDE (Project → Build Project, or `make -C Debug all` from a shell with the gcc-arm toolchain on PATH). Expected: clean build, zero warnings about the new declarations. The new symbols are not yet referenced anywhere, so the build should succeed without linker complaints.

- [ ] **Step 3: Commit**

```bash
git add Core/Inc/himax_sdk.h
git commit -m "Add telemetry_t and feature 0x83 defines to himax_sdk.h"
```

---

## Task 2: STM32 — implement `logTelemetryToHimax` (pack + batch + send)

**Files:**
- Modify: `Core/Src/himax_sdk.c`

- [ ] **Step 1: Add file-static batch state and LE pack helper**

In `Core/Src/himax_sdk.c`, immediately above the existing `void logToHimax(...)` definition (which starts at line 165), add:

```c
/* -----------------------------------------------------------------------
 * Telemetry batch state and LE pack helper
 * -------------------------------------------------------------------- */
static uint8_t s_tlm_batch[TLM_BATCH_SIZE * TLM_SAMPLE_BYTES];
static uint8_t s_tlm_count = 0;

/* Pack one telemetry_t into 44 little-endian bytes at dst. STM32G4 is
 * little-endian, so memcpy of each scalar is correct on this MCU. We
 * still pack field-by-field (rather than memcpy-ing the struct whole)
 * because the C struct layout has padding/alignment that does NOT match
 * the wire format — the wire format is exactly 44 bytes with no padding. */
static void tlm_pack_sample_le(uint8_t *dst, const telemetry_t *t)
{
    size_t off = 0;
    for (int i = 0; i < 4; i++) {
        memcpy(dst + off, &t->q[i], 4); off += 4;
    }
    memcpy(dst + off, &t->temp_c, 4); off += 4;
    memcpy(dst + off, &t->vbat,   4); off += 4;
    for (int i = 0; i < 4; i++) {
        memcpy(dst + off, &t->vmotor[i], 4); off += 4;
    }
    memcpy(dst + off, &t->stm32_tick_ms, 4); /* off += 4; final */
}
```

- [ ] **Step 2: Add `logTelemetryToHimax` definition**

Append at the end of `Core/Src/himax_sdk.c` (after the existing `logToHimax` function):

```c
void logTelemetryToHimax(telemetry_t *t)
{
    if (t == NULL) return;

    /* Stamp tick at call time. */
    t->stm32_tick_ms = HAL_GetTick();

    /* Pack into the next slot in the batch. */
    tlm_pack_sample_le(&s_tlm_batch[s_tlm_count * TLM_SAMPLE_BYTES], t);
    s_tlm_count++;

    if (s_tlm_count < TLM_BATCH_SIZE) return;

    /* Batch full — flush one I2C frame. */
    HAL_StatusTypeDef rc = grove_send_cmd(I2C_FEATURE_TLM,
                                          I2C_CMD_TLM_WRITE,
                                          s_tlm_batch,
                                          (uint16_t)(TLM_BATCH_SIZE * TLM_SAMPLE_BYTES));
    if (rc != HAL_OK) {
        uart_log("[logTlmToHimax] I2C TX failed rc=%d", rc);
    } else {
        uart_log("[->himax tlm] flushed %u samples (last tick=%lu)",
                 (unsigned)TLM_BATCH_SIZE, (unsigned long)t->stm32_tick_ms);
    }

    /* Reset batch regardless of TX outcome — drop the 4 samples on
     * failure rather than re-flushing the same payload next time. */
    s_tlm_count = 0;
}
```

- [ ] **Step 3: Verify compile**

Build. Expected: clean build, no warnings.

- [ ] **Step 4: Commit**

```bash
git add Core/Src/himax_sdk.c
git commit -m "Add logTelemetryToHimax with 4-sample batching and LE pack"
```

---

## Task 3: STM32 — E2E smoke test calls in `main.c`

**Why now:** Lets us exercise the STM32 side standalone (UART trace verifies pack + batch + I2C TX) before the Himax side exists. Himax will print `[I2C_CMD] Unknown customer cmd: feature=0x83 ...` until Task 7 lands — that's expected and is itself a useful smoke for Task 7.

**Files:**
- Modify: `Core/Src/main.c:113-118` (immediately after the existing `logToHimax` smoke calls)

- [ ] **Step 1: Add gated smoke calls**

In `Core/Src/main.c`, immediately after line 118 (`logToHimax("STM32", "formatted float: %.2f", 3.14159f);`) and **before** the `/* USER CODE END 2 */` marker on line 120, insert:

```c
#define DEBUG_LOGTLM_SMOKE   /* remove for production builds */
#ifdef DEBUG_LOGTLM_SMOKE
  HAL_Delay(500);
  uart_log("--- logTlmToHimax smoke: 8 samples (expect 2 I2C frames) ---");
  for (int i = 0; i < 8; i++) {
      telemetry_t t = {0};
      t.q[0] = 0.1f * (float)i;
      t.q[1] = 0.2f * (float)i;
      t.q[2] = 0.3f * (float)i;
      t.q[3] = 0.4f * (float)i;
      t.temp_c     = 20.0f + (float)i;
      t.vbat       = 11.5f + 0.05f * (float)i;
      t.vmotor[0]  = 3.30f + 0.01f * (float)i;
      t.vmotor[1]  = 3.31f + 0.01f * (float)i;
      t.vmotor[2]  = 3.32f + 0.01f * (float)i;
      t.vmotor[3]  = 3.33f + 0.01f * (float)i;
      logTelemetryToHimax(&t);
      HAL_Delay(20);  /* ~50 Hz sample cadence in smoke test */
  }
  uart_log("--- logTlmToHimax smoke done ---");
#endif
```

- [ ] **Step 2: Verify compile**

Build. Expected: clean build.

- [ ] **Step 3: Verify on hardware (STM32 side only)**

Flash STM32. Open UART4 at 115200 baud. Expected trace after the existing `logToHimax` lines:

```
--- logTlmToHimax smoke: 8 samples (expect 2 I2C frames) ---
[->himax tlm] flushed 4 samples (last tick=...)
[->himax tlm] flushed 4 samples (last tick=...)
--- logTlmToHimax smoke done ---
```

If you see `[logTlmToHimax] I2C TX failed rc=...`, treat as a hardware-side issue (Himax not powered / wires loose); see `project_logToHimax.md` for the same diagnosis chain — it applies identically here.

- [ ] **Step 4: Commit**

```bash
git add Core/Src/main.c
git commit -m "Add logTlmToHimax 8-sample E2E smoke test gated under DEBUG_LOGTLM_SMOKE"
```

---

## Task 4: Himax — header changes

**Files:**
- Modify: `EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/i2c_cmd.h`
- Modify: `EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/sdlog.h`

Note: Paths below are relative to the Seeed repo root (`Seeed_Grove_Vision_AI_Module_V2`).

- [ ] **Step 1: Add telemetry feature/cmd defines in `i2c_cmd.h`**

In `EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/i2c_cmd.h`, immediately after the existing `#define I2C_CMD_LOG_WRITE 0x01` line (line 11), add:

```c
#define I2C_FEATURE_TLM         0x83
#define I2C_CMD_TLM_WRITE       0x01
#define TLM_SAMPLE_BYTES        44U   /* 10*float32 + 1*uint32, see STM32 spec */
```

- [ ] **Step 2: Add telemetry prototypes in `sdlog.h`**

In `EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/sdlog.h`, immediately before the closing `#endif` on line 50, add:

```c
/**
 * sdlog_tlm_init() — open SESSION_XXXX/telemetry.csv (or top-level
 * telemetry.csv depending on convention) and write the CSV header row.
 * Must be called after sdlog_session_init() so the SD card and session
 * directory already exist.
 */
void sdlog_tlm_init(void);

/**
 * sdlog_tlm_enqueue() — SPSC ring enqueue called from the i2ccomm RX
 * event context. Copies one I2C frame's payload (TLM_BATCH_SIZE *
 * TLM_SAMPLE_BYTES bytes) into a RAM ring; no SD / SPI / FatFs access.
 */
void sdlog_tlm_enqueue(const uint8_t *payload, uint16_t plen);

/**
 * sdlog_tlm_drain() — called from the main scenario loop. Pops every
 * pending frame, unpacks each 44-byte sample, stamps a Himax-side
 * timestamp, and writes one CSV row per sample. f_syncs every 60
 * samples (~once per second at 60 Hz). Must NOT be called from
 * interrupt / event-callback context.
 */
void sdlog_tlm_drain(void);
```

- [ ] **Step 3: Verify compile (Himax side)**

Build the Himax `tflm_yolov8_od_sdlog` scenario. The new symbols are unreferenced; build should succeed.

- [ ] **Step 4: Commit (Himax repo)**

```bash
cd ../Seeed_Grove_Vision_AI_Module_V2
git add EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/i2c_cmd.h \
        EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/sdlog.h
git commit -m "Add telemetry feature 0x83 defines and sdlog_tlm_* prototypes"
```

---

## Task 5: Himax — open `telemetry.csv` and write header at session init

**Why before Task 6:** Task 6 needs `g_tlm_fil` and `g_tlm_ready` to exist.

**Files:**
- Modify: `EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/sdlog.c`

- [ ] **Step 1: Add module state and `sdlog_tlm_init`**

In `sdlog.c`, immediately after the existing `static int g_sdlog_ready = 0;` line (line 17), add:

```c
/* Telemetry CSV state */
static FIL      g_tlm_fil;
static int      g_tlm_ready    = 0;
static uint32_t g_tlm_sync_ctr = 0;
```

Then, immediately after the `sdlog_session_init` function definition (ends at line 157), add:

```c
/* -----------------------------------------------------------------------
 * sdlog_tlm_init — open telemetry.csv inside the active session dir and
 * write the CSV header row. Must run after sdlog_session_init.
 * -------------------------------------------------------------------- */
void sdlog_tlm_init(void)
{
    if (!g_sdlog_ready) {
        xprintf("[SDLOG_TLM] sdlog not ready — skipping telemetry.csv\r\n");
        return;
    }

    char path[80];
    xsprintf(path, "%s/telemetry.csv", g_session_dir);

    FRESULT res = f_open(&g_tlm_fil, path, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
        xprintf("[SDLOG_TLM] f_open(%s) res=%d — telemetry disabled\r\n", path, res);
        return;
    }

    static const char header[] =
        "stm32_tick_ms,qw,qx,qy,qz,temp_c,vbat,vm1,vm2,vm3,vm4,himax_recv_ms\r\n";
    UINT bw;
    res = f_write(&g_tlm_fil, header, (UINT)(sizeof(header) - 1), &bw);
    if (res != FR_OK) {
        xprintf("[SDLOG_TLM] header f_write res=%d — telemetry disabled\r\n", res);
        f_close(&g_tlm_fil);
        return;
    }
    f_sync(&g_tlm_fil);

    g_tlm_ready = 1;
    g_tlm_sync_ctr = 0;
    xprintf("[SDLOG_TLM] telemetry.csv ready in %s\r\n", g_session_dir);
}
```

- [ ] **Step 2: Verify compile**

Build the Himax scenario. Expected: clean build.

- [ ] **Step 3: Commit**

```bash
git add EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/sdlog.c
git commit -m "Open telemetry.csv with CSV header in sdlog_tlm_init"
```

---

## Task 6: Himax — telemetry SPSC ring + drain

**Files:**
- Modify: `EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/sdlog.c`

The ring stores whole I2C frames (one frame = up to `TLM_BATCH_SIZE * TLM_SAMPLE_BYTES` bytes = 176 bytes). 4 ring slots × 180 bytes ≈ 720 B of static RAM — comfortably small.

- [ ] **Step 1: Add ring storage and helpers below the existing log ring**

At the **end** of `sdlog.c` (after the `sdlog_log_drain` function, i.e. after line 305), add:

```c
/* -----------------------------------------------------------------------
 * Telemetry SPSC ring + drain
 *
 * Producer: i2c_customer_handler (i2ccomm RX event). Only memcpy +
 *           head advance.
 * Consumer: sdlog_tlm_drain(), called once per scenario loop iteration.
 * -------------------------------------------------------------------- */
#define SDLOG_TLM_RING_SLOTS  4
#define SDLOG_TLM_BLOB_MAX    (4U * 44U)  /* TLM_BATCH_SIZE * TLM_SAMPLE_BYTES */

typedef struct {
    uint16_t plen;
    uint8_t  blob[SDLOG_TLM_BLOB_MAX];
} sdlog_tlm_entry_t;

static sdlog_tlm_entry_t g_tlm_ring[SDLOG_TLM_RING_SLOTS];
static volatile uint32_t g_tlm_head = 0;   /* producer */
static volatile uint32_t g_tlm_tail = 0;   /* consumer */
static volatile uint32_t g_tlm_dropped = 0;

void sdlog_tlm_enqueue(const uint8_t *payload, uint16_t plen)
{
    if (payload == NULL) return;
    if (plen == 0 || plen > SDLOG_TLM_BLOB_MAX || (plen % TLM_SAMPLE_BYTES) != 0) {
        return;  /* drop malformed; logged at dispatcher */
    }

    uint32_t head = g_tlm_head;
    uint32_t tail = g_tlm_tail;
    if ((head - tail) >= SDLOG_TLM_RING_SLOTS) {
        g_tlm_dropped++;
        return;
    }

    sdlog_tlm_entry_t *e = &g_tlm_ring[head % SDLOG_TLM_RING_SLOTS];
    e->plen = plen;
    memcpy(e->blob, payload, plen);

    __DMB();
    g_tlm_head = head + 1;
}

/* Read 4 little-endian bytes as float (no host-order assumption). */
static float tlm_unpack_f32_le(const uint8_t *p)
{
    uint32_t u = (uint32_t)p[0]
               | ((uint32_t)p[1] << 8)
               | ((uint32_t)p[2] << 16)
               | ((uint32_t)p[3] << 24);
    float f;
    memcpy(&f, &u, 4);
    return f;
}

static uint32_t tlm_unpack_u32_le(const uint8_t *p)
{
    return (uint32_t)p[0]
         | ((uint32_t)p[1] << 8)
         | ((uint32_t)p[2] << 16)
         | ((uint32_t)p[3] << 24);
}

void sdlog_tlm_drain(void)
{
    if (!g_tlm_ready) return;

    int wrote_any = 0;

    while (g_tlm_tail != g_tlm_head) {
        wrote_any = 1;
        sdlog_tlm_entry_t *e = &g_tlm_ring[g_tlm_tail % SDLOG_TLM_RING_SLOTS];

        for (uint16_t off = 0; off < e->plen; off += TLM_SAMPLE_BYTES) {
            const uint8_t *p = &e->blob[off];
            float qw   = tlm_unpack_f32_le(p +  0);
            float qx   = tlm_unpack_f32_le(p +  4);
            float qy   = tlm_unpack_f32_le(p +  8);
            float qz   = tlm_unpack_f32_le(p + 12);
            float temp = tlm_unpack_f32_le(p + 16);
            float vbat = tlm_unpack_f32_le(p + 20);
            float vm1  = tlm_unpack_f32_le(p + 24);
            float vm2  = tlm_unpack_f32_le(p + 28);
            float vm3  = tlm_unpack_f32_le(p + 32);
            float vm4  = tlm_unpack_f32_le(p + 36);
            uint32_t stm32_tick = tlm_unpack_u32_le(p + 40);

            uint32_t himax_recv_ms = sdlog_now_ms();

            char line[200];
            int n = snprintf(line, sizeof(line),
                "%lu,%.6f,%.6f,%.6f,%.6f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%lu\r\n",
                (unsigned long)stm32_tick,
                qw, qx, qy, qz, temp, vbat, vm1, vm2, vm3, vm4,
                (unsigned long)himax_recv_ms);
            if (n < 0) continue;
            if (n > (int)sizeof(line)) n = (int)sizeof(line);

            UINT bw;
            f_write(&g_tlm_fil, line, (UINT)n, &bw);

            if (++g_tlm_sync_ctr >= 60U) {
                f_sync(&g_tlm_fil);
                g_tlm_sync_ctr = 0;
            }
        }

        __DMB();
        g_tlm_tail++;
    }

    /* Trailing sync: when we drained the ring empty, flush any residue
     * that the sync-every-60 counter didn't cover. At sustained 60 Hz
     * the ring rarely empties (drains pop 1 frame, more arrive), so
     * the every-60 counter still dominates. At low rates / end-of-run
     * this guarantees data hits the SD before idle. */
    if (wrote_any && g_tlm_tail == g_tlm_head) {
        f_sync(&g_tlm_fil);
        g_tlm_sync_ctr = 0;
    }

    if (g_tlm_dropped) {
        uint32_t d = g_tlm_dropped;
        g_tlm_dropped = 0;
        xprintf("[SDLOG_TLM] dropped %lu telemetry frames (ring full)\r\n",
                (unsigned long)d);
    }
}
```

Note: `snprintf` is used here (rather than `xsprintf`) because `xprintf`-family floats often only support `%f` with limited precision; the standard `snprintf` from newlib is already linked by FatFS / sdlog.c. If your toolchain's `snprintf` does not support `%.6f`, swap each `%.6f` / `%.4f` for `xsprintf` after a manual `snprintf("%ld", (long)(value*1000000))` — but try `snprintf` first since the existing `sdlog_write` already uses `vsnprintf` (line 216) so the dependency is already in.

- [ ] **Step 2: Verify compile**

Build. Expected: clean build, possibly a `-Wfloat-conversion` warning if your `tflm` build flags are strict — fix by casting `(float)i` if it appears.

- [ ] **Step 3: Commit**

```bash
git add EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/sdlog.c
git commit -m "Add SPSC ring and drain for telemetry CSV writes"
```

---

## Task 7: Himax — wire feature `0x83` into `i2c_customer_handler`

**Files:**
- Modify: `EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/i2c_cmd.c:98` (`else` branch — extend the `if/else` chain)

- [ ] **Step 1: Add the telemetry handler branch**

In `i2c_cmd.c`, change the existing chain at lines 49–100. Specifically, replace the final `else` block (lines 98–100):

```c
    } else {
        xprintf("[I2C_CMD] Unknown customer cmd: feature=0x%02x cmd=0x%02x\r\n", feature, cmd);
    }
```

with:

```c
    } else if (feature == I2C_FEATURE_TLM && cmd == I2C_CMD_TLM_WRITE) {
        uint16_t plen = ((uint16_t)gRead_buf[USE_DW_IIC_SLV_0][I2CFMT_PAYLOADLEN_MSB_OFFSET] << 8)
                      |  (uint16_t)gRead_buf[USE_DW_IIC_SLV_0][I2CFMT_PAYLOADLEN_LSB_OFFSET];

        if (plen == 0 || plen > I2CCOMM_MAX_PAYLOAD_SIZE
            || (plen % TLM_SAMPLE_BYTES) != 0) {
            xprintf("[I2C_TLM] bad plen=%u\r\n", (unsigned)plen);
        } else {
            const uint8_t *payload = (const uint8_t *)&gRead_buf[USE_DW_IIC_SLV_0][I2CFMT_PAYLOAD_OFFSET];
            sdlog_tlm_enqueue(payload, plen);
            xprintf("[I2C_TLM] enqueued %u bytes (%u samples)\r\n",
                    (unsigned)plen, (unsigned)(plen / TLM_SAMPLE_BYTES));
        }
    } else {
        xprintf("[I2C_CMD] Unknown customer cmd: feature=0x%02x cmd=0x%02x\r\n", feature, cmd);
    }
```

- [ ] **Step 2: Verify compile**

Build. Expected: clean build.

- [ ] **Step 3: Commit**

```bash
git add EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/i2c_cmd.c
git commit -m "Dispatch I2C feature 0x83 to sdlog_tlm_enqueue"
```

---

## Task 8: Himax — hook init and drain into the scenario lifecycle

**Files:**
- Modify: `EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/tflm_yolov8_od_sdlog.c:546` (drain call site)
- Modify: `EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/tflm_yolov8_od_sdlog.c:686` (init call site)

- [ ] **Step 1: Call `sdlog_tlm_init` after `sdlog_session_init`**

In `tflm_yolov8_od_sdlog.c` at line 686, the existing line is:

```c
    sdlog_session_init();
```

Replace with:

```c
    sdlog_session_init();
    sdlog_tlm_init();
```

- [ ] **Step 2: Call `sdlog_tlm_drain` next to `sdlog_log_drain`**

In `tflm_yolov8_od_sdlog.c` at line 546, the existing line is:

```c
        sdlog_log_drain();
```

Replace with:

```c
        sdlog_log_drain();
        sdlog_tlm_drain();
```

- [ ] **Step 3: Verify compile**

Build. Expected: clean build.

- [ ] **Step 4: Commit**

```bash
git add EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/tflm_yolov8_od_sdlog.c
git commit -m "Hook sdlog_tlm_init/_drain into scenario lifecycle"
```

---

## Task 9: End-to-end hardware verification

**Why:** This is the only way to verify correctness. No host-side test harness exists for either MCU.

- [ ] **Step 1: Flash both MCUs**

1. STM32CubeIDE → Run → Debug (uses `i2cScan Debug.launch`) on the i2cScan project. Or flash the existing `Debug/i2cScan.elf` via ST-Link.
2. Build and flash the Himax scenario (`tflm_yolov8_od_sdlog`) via the Seeed flash tool / dragon-tail USB drop, per the existing recording flow.
3. Power-cycle both boards together so they boot with the I2C bus quiet.

- [ ] **Step 2: Capture STM32 UART trace**

Connect a USB-UART to PC10 (UART4 TX) at 115200 8N1. Expected during the smoke test (after the existing logToHimax lines):

```
--- logTlmToHimax smoke: 8 samples (expect 2 I2C frames) ---
[<tick> ms] [->himax tlm] flushed 4 samples (last tick=...)
[<tick> ms] [->himax tlm] flushed 4 samples (last tick=...)
--- logTlmToHimax smoke done ---
```

If `[logTlmToHimax] I2C TX failed` appears: this is the same diagnosis chain as `logToHimax`. See `project_logToHimax.md`.

- [ ] **Step 3: Capture Himax UART trace**

Connect to the Himax debug UART. Expected:

```
[I2C_TLM] enqueued 176 bytes (4 samples)
[I2C_TLM] enqueued 176 bytes (4 samples)
```

Plus, near the start (after `[SDLOG] SD card mounted` and `[SDLOG] Session SESSION_XXXX initialised`):

```
[SDLOG_TLM] telemetry.csv ready in SESSION_XXXX
```

If `[I2C_CMD] Unknown customer cmd: feature=0x83` appears instead of `[I2C_TLM] enqueued`: the Himax firmware was not re-flashed with the Task 7 changes.

If `[I2C_TLM] bad plen=...` appears: STM32-side payload mismatch — re-verify `TLM_SAMPLE_BYTES == 44` on both ends and that `s_tlm_count` is reset to 0 before the first call.

- [ ] **Step 4: Pull SD card and inspect `telemetry.csv`**

After the smoke trace finishes on STM32 UART (the `--- logTlmToHimax smoke done ---` line) wait ~2 seconds, then power off the Himax. The trailing `f_sync` in `sdlog_tlm_drain` (Task 6) flushes the file once the ring drains empty — for 8 samples / 2 frames this happens within a few scenario-loop iterations (≤ 100 ms), well before you cut power.

Eject SD, mount on host, open `SESSION_XXXX/telemetry.csv`. Expected exactly 9 lines:

```
stm32_tick_ms,qw,qx,qy,qz,temp_c,vbat,vm1,vm2,vm3,vm4,himax_recv_ms
<tick0>,0.000000,0.000000,0.000000,0.000000,20.0000,11.5000,3.3000,3.3100,3.3200,3.3300,<recv0>
<tick1>,0.100000,0.200000,0.300000,0.400000,21.0000,11.5500,3.3100,3.3200,3.3300,3.3400,<recv1>
...
<tick7>,0.700000,1.400000,2.100000,2.800000,27.0000,11.8500,3.3700,3.3800,3.3900,3.4000,<recv7>
```

Sanity checks:
1. Header row exactly matches `Step 1` of Task 5.
2. 8 data rows.
3. Column count = 12 on every row (count commas: 11).
4. Quaternion values increase linearly (`0.0, 0.1, 0.2, ...` for `qw`), verifying the LE pack/unpack round-trips correctly.
5. `stm32_tick_ms` is monotonically increasing.
6. `himax_recv_ms` is monotonically increasing (drain order matches enqueue order).
7. Two `himax_recv_ms` cohorts: rows 1–4 are within ~ms of each other, rows 5–8 are within ~ms of each other, and the gap between cohorts is ~80–120 ms (the 4 × 20 ms STM32 smoke-side delay).

If `pandas.read_csv` from the host loads it and `df.dtypes` shows `uint64` (or `int64`) for the two tick columns and `float64` for the rest, the format is valid.

- [ ] **Step 5: Verify `session.log` and JPEG capture are unaffected**

Open `SESSION_XXXX/session.log`. The existing `[BOOT]`, `[I2C]`, `[STM32]`, `[DETECT]` lines must all still appear as before. The `[STM32]` lines from the prior `logToHimax` smoke calls should be present. No regression.

- [ ] **Step 6: Save memory note**

Update `project_logToHimax.md` with a pointer to a new `project_logTlmToHimax.md` recording: feature complete, hardware-verified (or list any failures observed in Steps 2–4 for debugging).

---

## Verification summary

| Property | How verified |
|----------|--------------|
| LE pack correctness | Quaternion ramp (0.0, 0.1, ...) shows up unchanged in `telemetry.csv` |
| Batch trigger every 4 samples | UART4: exactly 2 `[->himax tlm] flushed 4 samples` lines for 8 calls |
| Frame routing | Himax UART: `[I2C_TLM] enqueued 176 bytes (4 samples)` × 2 |
| File format | Exactly 1 header + 8 data rows; 12 columns each; pandas loads cleanly |
| Drain ordering | `himax_recv_ms` monotonically increasing; two cohorts ~80 ms apart |
| No regression | `session.log` still contains all prior `[STM32]` / `[BOOT]` / `[I2C]` lines |

## Out of scope (per spec YAGNI list)

- 60 Hz sustained-rate test (3500+ rows over 60 s during active recording) — listed in the spec as a separate validation milestone after the basic smoke passes; not a task in this plan.
- Schema versioning / per-recording-session file rotation / runtime-configurable rate — explicit YAGNI in spec §"Scope boundaries".
- Real sensor source (IMU / battery / motor monitor) — the API is a logging sink; producers are a separate concern.
