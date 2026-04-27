# Himax RX Mailbox FIFO Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Insert an 8-KB SPSC ring between the Himax I2C-slave ISR and the main-loop event handler so back-to-back I2C frames don't clobber each other in the single `gRead_buf` mailbox, eliminating the telemetry first-batch loss bug (debug-list §2) and unblocking 30 Hz STM32→Himax data transmission with zero loss.

**Architecture:** ISR memcpys the just-completed `gRead_buf[id]` into the next FIFO slot (or bumps a drop counter if full), then activates the event as today. Main-loop event handler drains in `while (tail != head)`, restoring each slot back into `gRead_buf[id]` before invoking the existing `switch (feature)` dispatch — so every scenario's `i2c_cmd.c` consumer sees identical bytes at identical offsets and needs zero changes. Symmetric bump of the downstream `SDLOG_TLM_RING_SLOTS` (4 → 16) keeps end-to-end stall coverage consistent at ~4 s.

**Tech Stack:** ARM Cortex-M55 (Himax WE2 / EPII_CM55M_APP_S firmware), C, plain SRAM-resident SPSC ring, `__DMB()` memory barriers, no HAL changes. Builds inside the Himax SDK toolchain (Seeed_Grove_Vision_AI_Module_V2 source tree).

**Spec:** `docs/superpowers/specs/2026-04-27-himax-rx-mailbox-fifo-design.md` (commit `ff8f742`).

**Code-tree note:** All source edits are in the **Seeed_Grove_Vision_AI_Module_V2** workspace, **NOT** the `i2cScan` repo where this plan lives. Paths below are absolute. Commits land in the Seeed_Grove_Vision_AI_Module_V2 git tree.

**Testing model:** This is embedded firmware with no unit-test harness. Each non-final task ends with a clean compile (Himax IDE build) as the bite-sized check. Behavioral verification is hardware-in-the-loop in the final task: smoke test (UART trace), production-rate stress test (`telemetry.csv` row count), and regression on a non-`tflm_yolov8_od_sdlog` scenario.

---

## File Structure

**Modified:**

- `C:\Users\frank\STM32CubeIDE\workspace_1.19.0\stm32ide\Seeed_Grove_Vision_AI_Module_V2\EPII_CM55M_APP_S\app\scenario_app\event_handler\evt_i2ccomm\evt_i2ccomm.c`
  Owns the FIFO storage, indices, ISR enqueue, drain loop, dispatch helper, and the drop-counter accessor. All new code lives here.

- `C:\Users\frank\STM32CubeIDE\workspace_1.19.0\stm32ide\Seeed_Grove_Vision_AI_Module_V2\EPII_CM55M_APP_S\app\scenario_app\event_handler\evt_i2ccomm\evt_i2ccomm.h`
  Adds one accessor declaration: `evt_i2ccomm_get_rx_dropped()`.

- `C:\Users\frank\STM32CubeIDE\workspace_1.19.0\stm32ide\Seeed_Grove_Vision_AI_Module_V2\EPII_CM55M_APP_S\app\scenario_app\tflm_yolov8_od_sdlog\sdlog.c`
  Bumps `SDLOG_TLM_RING_SLOTS` from 4 to 16.

- `C:\Users\frank\STM32CubeIDE\workspace_1.19.0\stm32ide\Seeed_Grove_Vision_AI_Module_V2\EPII_CM55M_APP_S\app\scenario_app\tflm_yolov8_od_sdlog\tflm_yolov8_od_sdlog.c`
  Adds one print-on-change site for the new mailbox-drop counter, near the existing `sdlog_tlm_drain()` call (currently line 547).

**Untouched (verified):** `hx_lib_i2ccomm` (HAL), every scenario's `i2c_cmd.c` (telemetry 0x83, log 0x82, sys 0x80, recording 0x80/0x02, …), all STM32 firmware in the `i2cScan` repo.

---

## Task 1: Add FIFO state and drop-counter accessor (scaffold; no behavior change)

**Files:**
- Modify: `EPII_CM55M_APP_S\app\scenario_app\event_handler\evt_i2ccomm\evt_i2ccomm.h`
- Modify: `EPII_CM55M_APP_S\app\scenario_app\event_handler\evt_i2ccomm\evt_i2ccomm.c`

This task only **adds** state and the accessor — `i2cs_cb_rx` and the rx_cb handlers are not yet rewired. After this task the firmware compiles and behaves exactly as before; the FIFO is dead storage.

- [ ] **Step 1: Add accessor declaration to header**

In `evt_i2ccomm.h`, just before the closing `#endif`, after the `i2ccomm_cmd_customer_register_cb` declaration (around line 268), insert:

```c
/**
 * \brief   Get cumulative count of RX frames dropped because the
 *          mailbox FIFO was full when an I2C frame arrived. Monotonic
 *          counter; reader is responsible for diff-tracking if it
 *          wants edge-triggered prints.
 * \param   iic_id  one of USE_DW_IIC_SLV_0 / USE_DW_IIC_SLV_1
 * \retval  cumulative drop count for that iic id
 */
uint32_t evt_i2ccomm_get_rx_dropped(USE_DW_IIC_SLV_E iic_id);
```

- [ ] **Step 2: Add FIFO state in `evt_i2ccomm.c`**

In `evt_i2ccomm.c`, add `#include "WE2_core.h"` is already present (line 25) — we need `__DMB()` from CMSIS, which the existing includes already pull in transitively (other event handlers use it). No new includes.

Add after the `EVT_I2CS_1_SLV_ADDR` `#define` (around line 58), in the constant-definition block:

```c
#define EVT_I2CCOMM_RX_FIFO_SLOTS  16U   /* depth = 16; ~2.1 s coverage at 7.5 Hz wire rate */
```

Add immediately after the `gWrite_buf` / `gRead_buf` declarations (around line 83), at file scope:

```c
/* RX mailbox FIFO — interposed between i2cs_cb_rx (ISR producer) and
 * evt_i2ccomm_<id>_rx_cb (main-loop consumer) to stop back-to-back
 * frames from clobbering gRead_buf during scenario-loop stalls.
 * SPSC discipline: ISR writes only s_rx_head + s_rx_dropped;
 * main-loop writes only s_rx_tail. */
static uint8_t  s_rx_fifo[DW_IIC_S_NUM]
                         [EVT_I2CCOMM_RX_FIFO_SLOTS]
                         [I2CCOMM_MAX_RBUF_SIZE]
                __ALIGNED(__SCB_DCACHE_LINE_SIZE);
static volatile uint32_t s_rx_head[DW_IIC_S_NUM];
static volatile uint32_t s_rx_tail[DW_IIC_S_NUM];
static volatile uint32_t s_rx_dropped[DW_IIC_S_NUM];
```

- [ ] **Step 3: Implement the accessor**

Append at the bottom of `evt_i2ccomm.c`:

```c
uint32_t evt_i2ccomm_get_rx_dropped(USE_DW_IIC_SLV_E iic_id)
{
    if (iic_id >= DW_IIC_S_NUM) return 0;
    return s_rx_dropped[iic_id];
}
```

- [ ] **Step 4: Build (Himax IDE clean rebuild)**

Open the Himax IDE workspace at `Seeed_Grove_Vision_AI_Module_V2`. Run a clean build of `EPII_CM55M_APP_S`. Expected: builds clean with zero new warnings. The new statics will be reported as "defined but not used" only if the compiler is unusually strict — they're referenced by the accessor, so this should be silent.

If the compiler complains about `__ALIGNED` or `__SCB_DCACHE_LINE_SIZE`, those are CMSIS macros already used by `gRead_buf` in this same file — no extra include needed.

- [ ] **Step 5: Commit**

```bash
git add EPII_CM55M_APP_S/app/scenario_app/event_handler/evt_i2ccomm/evt_i2ccomm.h \
        EPII_CM55M_APP_S/app/scenario_app/event_handler/evt_i2ccomm/evt_i2ccomm.c
git commit -m "Add RX mailbox FIFO state and drop-counter accessor"
```

---

## Task 2: Extract `prv_evt_i2ccomm_dispatch_one` (pure refactor; no behavior change)

**Files:**
- Modify: `EPII_CM55M_APP_S\app\scenario_app\event_handler\evt_i2ccomm\evt_i2ccomm.c`

This task collapses the two near-identical `evt_i2ccomm_<id>_rx_cb` switch bodies into one parametrised helper. Behavior is unchanged — both rx_cb functions still run the same dispatch, on the same `gRead_buf[id]`, exactly once per event activation. This isolates the dispatch logic so Task 3 can call it from inside a drain loop.

- [ ] **Step 1: Add the static declaration near the other static decls**

In `evt_i2ccomm.c`, in the `Static Function Declaration` block (around lines 68-75), add:

```c
static void prv_evt_i2ccomm_dispatch_one(USE_DW_IIC_SLV_E iic_id);
```

- [ ] **Step 2: Implement `prv_evt_i2ccomm_dispatch_one`**

Insert this function in `evt_i2ccomm.c` immediately before `evt_i2ccomm_0_rx_cb` (around line 188). It is the union of the existing `_0_rx_cb` and `_1_rx_cb` bodies. The id=0-only branches (`I2CCOMM_FEATURE_OTA_RESERVED`) are gated on `iic_id == USE_DW_IIC_SLV_0` so the merged helper preserves existing iic_id=1 behavior exactly.

```c
static void prv_evt_i2ccomm_dispatch_one(USE_DW_IIC_SLV_E iic_id)
{
    unsigned char feature = gRead_buf[iic_id][I2CFMT_FEATURE_OFFSET];
    dbg_evt_iics_cmd("\n");
    dbg_evt_iics_cmd("%s(iic_id:%d, feature:0x%02x) \n",
                     __FUNCTION__, (int)iic_id, feature);

    switch (feature)
    {
        case I2CCOMM_FEATURE_SYS:
            evt_i2cs_cmd_process_sysinfo(iic_id);
            prv_evt_i2ccomm_clear_read_buf_header(iic_id);
            break;

        case I2CCOMM_FEATURE_OTA_RESERVED:
            /* jump to 2ndloader — only meaningful on iic_id 0 */
            if (iic_id == USE_DW_IIC_SLV_0) {
                dbg_evt_iics_cmd("Into 2ndloader upgrade:\n");
                hx_drv_swreg_aon_set_ota_flag(SWREG_AON_OTA_YES_FLAG);
                setPS_PDNoVid();
            }
            break;

        case I2CCOMM_FEATURE_CUSTOMER_MIN ... I2CCOMM_FEATURE_CUSTOMER_MAX:
            if (i2ccomm_cmd_customer_process[iic_id] != NULL) {
                i2ccomm_cmd_customer_process[iic_id]();
            }
            break;

        case I2CCOMM_FEATURE_MAX:
            break;

        default:
            prv_evt_i2ccomm_clear_read_buf_header(iic_id);
            hx_lib_i2ccomm_enable_read(iic_id,
                                       (unsigned char *) &gRead_buf[iic_id],
                                       I2CCOMM_MAX_RBUF_SIZE);
            break;
    }
}
```

- [ ] **Step 3: Replace `evt_i2ccomm_0_rx_cb` body**

Replace the existing body (lines 189-224) with:

```c
uint8_t evt_i2ccomm_0_rx_cb(void)
{
    prv_evt_i2ccomm_dispatch_one(USE_DW_IIC_SLV_0);
    return HX_EVENT_RETURN_DONE;
}
```

- [ ] **Step 4: Replace `evt_i2ccomm_1_rx_cb` body**

Replace the existing body (lines 244-271, the body of `evt_i2ccomm_1_rx_cb`) with:

```c
uint8_t evt_i2ccomm_1_rx_cb(void)
{
    prv_evt_i2ccomm_dispatch_one(USE_DW_IIC_SLV_1);
    return HX_EVENT_RETURN_DONE;
}
```

- [ ] **Step 5: Remove the now-unused `funcptr_void ResetHandler;` local**

The original `evt_i2ccomm_0_rx_cb` declared a local `funcptr_void ResetHandler;` (line 194) that was never used. It's gone after the body replacement in Step 3. No action needed; just confirm the new body doesn't reintroduce it.

- [ ] **Step 6: Build (Himax IDE clean rebuild)**

Expected: builds clean. No new warnings. Behavior is identical to before Task 1.

- [ ] **Step 7: Commit**

```bash
git add EPII_CM55M_APP_S/app/scenario_app/event_handler/evt_i2ccomm/evt_i2ccomm.c
git commit -m "Extract prv_evt_i2ccomm_dispatch_one (pure refactor)"
```

---

## Task 3: Wire ISR memcpy enqueue and drain loop (the actual fix)

**Files:**
- Modify: `EPII_CM55M_APP_S\app\scenario_app\event_handler\evt_i2ccomm\evt_i2ccomm.c`

This task connects the FIFO. After this task, back-to-back I2C frames are buffered up to 16 deep per iic id, and the drain loop processes every one of them per main-loop iteration.

- [ ] **Step 1: Replace `i2cs_cb_rx`**

Replace the existing `i2cs_cb_rx` (lines 143-155) with:

```c
static void i2cs_cb_rx(void *param)
{
    HX_DRV_DEV_IIC *iic_obj = param;
    HX_DRV_DEV_IIC_INFO *iic_info_ptr = &(iic_obj->iic_info);
    USE_DW_IIC_SLV_E id;
    hx_event_index_e evt_idx;

    if (iic_info_ptr->slv_addr == EVT_I2CS_0_SLV_ADDR) {
        dbg_evt_iics_cb("%s(iic_id:0) \n", __FUNCTION__);
        id = USE_DW_IIC_SLV_0;
        evt_idx = EVT_INDEX_I2CS_0_RX;
    } else if (iic_info_ptr->slv_addr == EVT_I2CS_1_SLV_ADDR) {
        dbg_evt_iics_cb("%s(iic_id:1) \n", __FUNCTION__);
        id = USE_DW_IIC_SLV_1;
        evt_idx = EVT_INDEX_I2CS_1_RX;
    } else {
        return;
    }

    /* Enqueue the just-received frame into the SPSC FIFO. ISR is the
     * sole producer; main loop is the sole consumer. */
    uint32_t head = s_rx_head[id];
    uint32_t tail = s_rx_tail[id];
    if ((head - tail) < EVT_I2CCOMM_RX_FIFO_SLOTS) {
        memcpy(s_rx_fifo[id][head % EVT_I2CCOMM_RX_FIFO_SLOTS],
               (const void *)gRead_buf[id], I2CCOMM_MAX_RBUF_SIZE);
        __DMB();
        s_rx_head[id] = head + 1;
    } else {
        s_rx_dropped[id]++;
    }

    hx_event_activate_ISR(g_event[evt_idx]);
}
```

If `hx_event_index_e` is not the correct type for `g_event[]` indexing in this codebase, replace with `int` (the existing code uses raw enum names directly, so `int` is safe). Verify by checking that `g_event[EVT_INDEX_I2CS_0_TX]` already compiles in `i2cs_cb_tx` above.

- [ ] **Step 2: Replace `evt_i2ccomm_0_rx_cb` body with drain loop**

Replace the body written in Task 2 Step 3:

```c
uint8_t evt_i2ccomm_0_rx_cb(void)
{
    while (s_rx_tail[USE_DW_IIC_SLV_0] != s_rx_head[USE_DW_IIC_SLV_0]) {
        uint32_t t = s_rx_tail[USE_DW_IIC_SLV_0];
        memcpy((void *)gRead_buf[USE_DW_IIC_SLV_0],
               s_rx_fifo[USE_DW_IIC_SLV_0][t % EVT_I2CCOMM_RX_FIFO_SLOTS],
               I2CCOMM_MAX_RBUF_SIZE);
        __DMB();
        prv_evt_i2ccomm_dispatch_one(USE_DW_IIC_SLV_0);
        s_rx_tail[USE_DW_IIC_SLV_0] = t + 1;
    }
    return HX_EVENT_RETURN_DONE;
}
```

- [ ] **Step 3: Replace `evt_i2ccomm_1_rx_cb` body with drain loop**

```c
uint8_t evt_i2ccomm_1_rx_cb(void)
{
    while (s_rx_tail[USE_DW_IIC_SLV_1] != s_rx_head[USE_DW_IIC_SLV_1]) {
        uint32_t t = s_rx_tail[USE_DW_IIC_SLV_1];
        memcpy((void *)gRead_buf[USE_DW_IIC_SLV_1],
               s_rx_fifo[USE_DW_IIC_SLV_1][t % EVT_I2CCOMM_RX_FIFO_SLOTS],
               I2CCOMM_MAX_RBUF_SIZE);
        __DMB();
        prv_evt_i2ccomm_dispatch_one(USE_DW_IIC_SLV_1);
        s_rx_tail[USE_DW_IIC_SLV_1] = t + 1;
    }
    return HX_EVENT_RETURN_DONE;
}
```

- [ ] **Step 4: Build (Himax IDE clean rebuild)**

Expected: builds clean.

- [ ] **Step 5: Commit**

```bash
git add EPII_CM55M_APP_S/app/scenario_app/event_handler/evt_i2ccomm/evt_i2ccomm.c
git commit -m "Wire ISR enqueue and drain loop for RX mailbox FIFO"
```

---

## Task 4: Bump `SDLOG_TLM_RING_SLOTS` and add mailbox-drop print site

**Files:**
- Modify: `EPII_CM55M_APP_S\app\scenario_app\tflm_yolov8_od_sdlog\sdlog.c`
- Modify: `EPII_CM55M_APP_S\app\scenario_app\tflm_yolov8_od_sdlog\tflm_yolov8_od_sdlog.c`

This task adds the second-stage buffer depth and surface visibility into the new drop counter.

- [ ] **Step 1: Bump `SDLOG_TLM_RING_SLOTS` 4 → 16**

In `sdlog.c` line 357, change:

```c
#define SDLOG_TLM_RING_SLOTS  4
```

to:

```c
#define SDLOG_TLM_RING_SLOTS  16
```

No other change in `sdlog.c`. The existing `sdlog_tlm_enqueue` / `sdlog_tlm_drain` already use `% SDLOG_TLM_RING_SLOTS` for indexing, so they pick up the new depth automatically. Per-slot size (`sdlog_tlm_entry_t`) is `2 + 256 = 258 B` plus alignment padding; total ring grows from ~1 KB to ~4.2 KB.

- [ ] **Step 2: Add `evt_i2ccomm.h` include in scenario `.c`**

In `tflm_yolov8_od_sdlog.c`, locate the existing scenario includes (top of file). If `#include "evt_i2ccomm.h"` is not already present, add it. (It typically is — the scenario already calls `evt_i2ccomm_init`. Verify with a quick grep before editing.)

- [ ] **Step 3: Add mailbox-drop print-on-change site**

In `tflm_yolov8_od_sdlog.c`, around line 547 where `sdlog_tlm_drain()` is called, insert immediately **after** the `sdlog_tlm_drain()` call:

```c
        sdlog_tlm_drain();

        /* Surface I2C mailbox FIFO drops on change. Bumping means the
         * worst-case scenario-loop stall exceeded the 16-slot mailbox
         * depth — bump EVT_I2CCOMM_RX_FIFO_SLOTS and/or
         * SDLOG_TLM_RING_SLOTS. Pattern matches sdlog.c's
         * g_tlm_dropped print-on-change. */
        {
            static uint32_t s_last_iic_dropped = 0;
            uint32_t cur = evt_i2ccomm_get_rx_dropped(USE_DW_IIC_SLV_0);
            if (cur != s_last_iic_dropped) {
                xprintf("[I2CCOMM] dropped %lu rx frames (mailbox FIFO full)\r\n",
                        (unsigned long)cur);
                s_last_iic_dropped = cur;
            }
        }
```

The `static uint32_t s_last_iic_dropped` lives at function scope, so it persists across iterations and resets only on cold boot — same behavior as the FIFO drop counter itself.

- [ ] **Step 4: Build (Himax IDE clean rebuild)**

Expected: builds clean.

- [ ] **Step 5: Commit**

```bash
git add EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/sdlog.c \
        EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/tflm_yolov8_od_sdlog.c
git commit -m "Bump TLM ring to 16 slots; surface mailbox-FIFO drop count"
```

---

## Task 4.5: Add STM32-side 30 Hz × 60 s telemetry harness

**Files:**
- Modify: `C:\Users\frank\STM32CubeIDE\workspace_1.19.0\stm32ide\i2cScan\Core\Src\main.c`

This is the only STM32-side change in the whole effort. It adds a separate compile-time-gated harness next to the existing `DEBUG_LOGTLM_SMOKE` block so Task 5 Step 3 has something concrete to run. Mirrors the existing smoke pattern.

- [ ] **Step 1: Add `DEBUG_LOGTLM_30HZ` block in `main.c`**

In `main.c`, immediately after the closing `#endif` of the existing `DEBUG_LOGTLM_SMOKE` block (around line 151), insert:

```c
/* 30 Hz × 60 s production-rate stress test — verifies Himax RX mailbox
 * FIFO + bumped TLM ring (spec 2026-04-27-himax-rx-mailbox-fifo-design.md).
 * Expected: SESSION_XXXX/telemetry.csv has >=1800 rows; Himax UART shows
 * no [I2CCOMM] dropped or [SDLOG_TLM] dropped lines. */
//#define DEBUG_LOGTLM_30HZ   /* enable to run; remove for production */
#ifdef DEBUG_LOGTLM_30HZ
  HAL_Delay(500);
  uart_log("--- logTlmToHimax 30Hz x 60s stress (1800 samples) ---");
  for (int i = 0; i < 1800; i++) {
      telemetry_t t = {0};
      t.q[0] = 0.1f * (float)(i % 10);
      t.q[1] = 0.2f * (float)(i % 10);
      t.q[2] = 0.3f * (float)(i % 10);
      t.q[3] = 0.4f * (float)(i % 10);
      t.temp_c     = 20.0f + (float)(i % 10);
      t.vbat       = 11.5f + 0.05f * (float)(i % 10);
      t.vmotor[0]  = 3.30f + 0.01f * (float)(i % 10);
      t.vmotor[1]  = 3.31f + 0.01f * (float)(i % 10);
      t.vmotor[2]  = 3.32f + 0.01f * (float)(i % 10);
      t.vmotor[3]  = 3.33f + 0.01f * (float)(i % 10);
      t.imotor[0]  = 0.50f + 0.02f * (float)(i % 10);
      t.imotor[1]  = 0.51f + 0.02f * (float)(i % 10);
      t.imotor[2]  = 0.52f + 0.02f * (float)(i % 10);
      t.imotor[3]  = 0.53f + 0.02f * (float)(i % 10);
      t.depth      = 1.50f + 0.10f * (float)(i % 10);
      logTelemetryToHimax(&t);
      HAL_Delay(33);  /* 30 Hz call rate -> 7.5 Hz I2C wire rate
                       * (logTelemetryToHimax batches 4 samples/frame) */
  }
  uart_log("--- logTlmToHimax 30Hz stress done ---");
#endif
```

The block is left disabled by default (`//#define`) — Task 5 Step 3 enables it for a single stress run, then comments it back out.

- [ ] **Step 2: Build (STM32CubeIDE build of i2cScan)**

Open the i2cScan project in STM32CubeIDE and rebuild. Expected: builds clean.

- [ ] **Step 3: Commit (in i2cScan repo)**

```bash
git add Core/Src/main.c
git commit -m "Add DEBUG_LOGTLM_30HZ stress harness (1800 samples / 60 s)"
```

---

## Task 5: Hardware verification

**Files:** none modified — this task is end-to-end behavioral verification.

Three checks, in order. The smoke test is fastest to run; it's the regression on debug-list §2. The 30 Hz stress test is the production-rate proof. The third check guards against breaking unrelated scenarios.

- [ ] **Step 1: Flash Himax + STM32 with current branch builds**

Flash the Himax with the rebuilt `EPII_CM55M_APP_S` image. Re-flash the STM32 with the existing `i2cScan` firmware (no STM32 changes were made — last commit `7cfa215` is fine). Both devices boot, SD card mounts, `SESSION_XXXX` directory is created.

- [ ] **Step 2: Smoke test — debug-list §2 regression (the original bug)**

In STM32 `main.c`, `DEBUG_LOGTLM_SMOKE` is already wired (per memory: smoke widened to 100 ms / sample, 8 samples → 2 batched I2C frames). Boot with `DEBUG_LOGTLM_SMOKE` defined.

Expected Himax UART trace pattern:

```
i2cs_cb_rx(iic_id:0)            ← batch 1
i2cs_cb_rx(iic_id:0)            ← batch 2
prv_evt_i2ccomm_dispatch_one(iic_id:0, feature:0x83)   ← drain pass 1
[I2C_TLM] enqueued 256 bytes (4 samples)
prv_evt_i2ccomm_dispatch_one(iic_id:0, feature:0x83)   ← drain pass 2
[I2C_TLM] enqueued 256 bytes (4 samples)
```

(Two cb_rx prints, **two** dispatch_one prints, **two** `[I2C_TLM] enqueued` lines — pre-fix it was 2/1/1.)

Then mount the SD card on a host. `SESSION_XXXX/telemetry.csv` should contain **8 data rows** (excluding header). Pre-fix only 4 rows landed (the second batch's i=4..7). Spot-check the first row's `stm32_tick` field is the smallest, last row's the largest, so all 8 samples are present in order.

`[I2CCOMM] dropped …` line MUST NOT appear in the UART log during this test.

- [ ] **Step 3: 30 Hz × 60 s production-rate stress test**

Reconfigure STM32 to call `logTelemetryToHimax()` from the main loop at 30 Hz for 60 seconds, with `startRecordingForHimax` already active so `ALL/` JPEG saves are running concurrently. (If a 30 Hz harness doesn't exist, gate one under a new `DEBUG_LOGTLM_30HZ` flag matching the `DEBUG_LOGTLM_SMOKE` pattern. STM32-side change only — out of scope for this Himax-only plan but required for verification; create the harness as a single follow-up commit.)

Expected:

- `SESSION_XXXX/telemetry.csv` contains **≥ 1800 data rows** in 60 s of run time (30 Hz × 60 s = 1800 samples).
- `[I2CCOMM] dropped …` does NOT print on Himax UART.
- `[SDLOG_TLM] dropped …` does NOT print on Himax UART.
- JPEG save cadence in `ALL/` is uninterrupted (no missing frame indices in the directory listing).

If `[I2CCOMM] dropped` prints: the worst stall exceeded the 16-slot mailbox. Bump `EVT_I2CCOMM_RX_FIFO_SLOTS` to 32 and rerun.
If `[SDLOG_TLM] dropped` prints: the TLM ring is the bottleneck; bump `SDLOG_TLM_RING_SLOTS` to 32 and rerun.

- [ ] **Step 4: Regression on a non-`tflm_yolov8_od_sdlog` scenario**

Build and flash any other Himax scenario that calls `evt_i2ccomm_init` (e.g., `allon_sensor_tflm_freertos` from the existing tree — confirm via grep that it uses the same `evt_i2ccomm_*_rx_cb` registration). Boot it and exercise its existing I2C-slave commands (sysinfo `0x00` query, or whatever its established smoke is).

Expected: behaves identically to before this branch — the dispatch_one helper preserves the original switch dispatch byte-for-byte. This regression check protects the shared-driver claim in the spec.

- [ ] **Step 5: Update `docs/debug-list.md` to mark §2 fixed**

In the i2cScan repo, edit `docs/debug-list.md` §2:

```
- **Status:** Diagnosed. Smoke test verifies the float-format fix when
  rows land, but the first-batch loss makes E2E verification of
  `logTelemetryToHimax` incomplete. Production 60 Hz path will hit
  this race during recording. Plan to address in a follow-up by
  implementing fix B (the deferred-spec option Z).
```

Replace with:

```
- **Status:** Fixed by Himax RX mailbox FIFO (spec
  `2026-04-27-himax-rx-mailbox-fifo-design.md`, plan
  `2026-04-27-himax-rx-mailbox-fifo.md`). Smoke produces all 8 rows;
  30 Hz × 60 s stress yields ≥1800 rows with no mailbox or TLM-ring
  drops.
```

- [ ] **Step 6: Commit the debug-list update (in i2cScan repo)**

```bash
git add docs/debug-list.md
git commit -m "Mark debug-list §2 telemetry first-batch loss fixed"
```

---

## Self-review notes

Spec coverage check (against `2026-04-27-himax-rx-mailbox-fifo-design.md`):

- §Architecture (data flow before/after) — Tasks 1-3 implement it.
- §Components → "New static state in `evt_i2ccomm.c`" — Task 1 Step 2.
- §Components → "Modified `i2cs_cb_rx`" — Task 3 Step 1.
- §Components → "Refactor of `evt_i2ccomm_<id>_rx_cb`" — Tasks 2-3.
- §Components → "Drop visibility" — Task 1 Step 1 (header), Task 4 Step 3 (call site).
- §Components → "Symmetric bump of `SDLOG_TLM_RING_SLOTS`" — Task 4 Step 1.
- §Concurrency → memory ordering (`__DMB()`) — Task 3 Steps 1-3.
- §Testing — Task 5.
- §Backwards compatibility — preserved by Task 2 (refactor is byte-equivalent) + Task 3's per-slot restore-into-`gRead_buf`.

No placeholders, no "TBD", no `Similar to Task N` deferrals. Every code-bearing step has the actual code. Type names (`USE_DW_IIC_SLV_E`, `EVT_INDEX_I2CS_0_RX`, `__SCB_DCACHE_LINE_SIZE`, `prv_evt_i2ccomm_dispatch_one`, `evt_i2ccomm_get_rx_dropped`) are consistent across tasks.
