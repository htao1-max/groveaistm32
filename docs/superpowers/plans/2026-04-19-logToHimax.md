# logToHimax Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `logToHimax(tag, fmt, ...)` on STM32 so each call appends one `[tag] msg` line to the active `SESSION_XXXX/session.log` on the Himax SD card.

**Architecture:** Refactor the existing `grove_start_recording` into a generic `grove_send_cmd(feature, cmd, payload, plen)` helper on the STM32, then implement `logToHimax` on top of it with a `tag\0msg\0` payload and a new i2ccomm customer feature `0x82`/cmd `0x01`. On the Himax side, extend `i2c_customer_handler` with a branch that parses the two strings and calls `sdlog_write("[%s] %s\r\n", tag, msg)`.

**Tech Stack:** STM32G431 HAL C (`HAL_I2C_Master_Transmit`, `vsnprintf`), Himax i2ccomm library (`gRead_buf`, `I2CFMT_*` offsets, `sdlog_write`, `xprintf`), CRC16-CCITT framing already used by the existing recorder command.

**Note on TDD:** This is bare-metal firmware on two MCUs with no unit-test framework on either side. "Tests" are manual hardware verifications: build → flash → observe UART trace + SD card contents. Each task ends with an explicit build+observe verification step.

**Reference spec:** `docs/superpowers/specs/2026-04-19-logToHimax-design.md`

**Files touched:**
- `Core/Inc/himax_sdk.h` — add new defines and prototype
- `Core/Src/himax_sdk.c` — refactor sender, add `logToHimax`
- `../Seeed_Grove_Vision_AI_Module_V2/EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/i2c_cmd.h` — add new defines
- `../Seeed_Grove_Vision_AI_Module_V2/EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/i2c_cmd.c` — add new handler branch

---

## Task 1: STM32 — refactor `grove_start_recording` into `grove_send_cmd`

**Why first:** Extracts the framing/CRC/transmit logic so the new `logToHimax` (Task 2) can reuse it. This is a no-behaviour-change refactor — the existing recorder packet must look identical on the wire afterwards.

**Files:**
- Modify: `Core/Src/himax_sdk.c:29-65`

- [ ] **Step 1: Replace `grove_start_recording` with `grove_send_cmd` + thin wrapper**

In `Core/Src/himax_sdk.c`, replace the entire `grove_start_recording` function (lines 29–65) with:

```c
/*
 * Generic Himax i2ccomm sender.
 * Packet layout:
 *   [0] Feature
 *   [1] Command
 *   [2] Payload length LSB
 *   [3] Payload length MSB
 *   [4..4+N-1] Payload (optional)
 *   [N] CRC16 LSB
 *   [N+1] CRC16 MSB
 */
static HAL_StatusTypeDef grove_send_cmd(uint8_t feature,
                                        uint8_t cmd,
                                        const uint8_t *payload,
                                        uint16_t plen)
{
    /* Max packet = 4 header + 256 payload + 2 crc = 262 bytes. */
    uint8_t pkt[262];
    if (plen > 256) return HAL_ERROR;

    pkt[0] = feature;
    pkt[1] = cmd;
    pkt[2] = plen & 0xFF;
    pkt[3] = (plen >> 8) & 0xFF;

    if (plen > 0 && payload != NULL) {
        for (uint16_t i = 0; i < plen; i++) pkt[4 + i] = payload[i];
    }

    uint16_t total = 4 + plen;
    uint16_t crc = crc16_ccitt(pkt, total);
    pkt[total]     = crc & 0xFF;
    pkt[total + 1] = (crc >> 8) & 0xFF;
    total += 2;

    return HAL_I2C_Master_Transmit(&hi2c1, GROVE_I2C_ADDR, pkt, total, 1000);
}

static HAL_StatusTypeDef grove_start_recording(int threshold)
{
    if (threshold >= 0 && threshold <= 100) {
        uint8_t payload = (uint8_t)threshold;
        return grove_send_cmd(I2C_FEATURE_RECORDER, I2C_CMD_RECORD_START, &payload, 1);
    }
    return grove_send_cmd(I2C_FEATURE_RECORDER, I2C_CMD_RECORD_START, NULL, 0);
}
```

- [ ] **Step 2: Build via STM32CubeIDE**

In STM32CubeIDE: Project → Build Project (or Ctrl+B). Expected: `Debug/i2cScan.elf` generated with no warnings/errors.

If building from shell (optional): `make -C Debug all`

- [ ] **Step 3: Flash and verify the recorder still works (no regression)**

Flash the STM32 via STM32CubeIDE (Run → Debug or the flash button). Keep the Grove AI module powered. On UART4 @ 115200 baud you should still see:

```
Sending start-recording (threshold=50%)...
I2C TX OK — check Grove AI UART for [I2C_CMD] message
```

On the Himax UART (if attached) you should still see the existing `[I2C_CMD] Recording started (thresh=0.50)` banner and the SD `session.log` should still contain `[I2C] Recording started (threshold=0.50)` in the newest `SESSION_XXXX/` folder.

Expected: identical behaviour to pre-refactor. If the recorder stops working, the refactor broke framing — review byte ordering, CRC range, and total length.

- [ ] **Step 4: Commit**

```bash
git add Core/Src/himax_sdk.c
git commit -m "Refactor grove_start_recording into generic grove_send_cmd

No behaviour change. Extracts framing/CRC/transmit so the upcoming
logToHimax feature (feature 0x82) and the pending exposure-control
feature (0x81) can reuse the same sender.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 2: STM32 — add `logToHimax` API

**Why before Himax:** The packet will go out on the bus even if the Himax side doesn't yet recognise feature `0x82` — the existing handler logs `[I2C_CMD] Unknown customer cmd` for unknown features, which is a useful intermediate signal that STM32 is framing correctly.

**Files:**
- Modify: `Core/Inc/himax_sdk.h` (add defines + prototype)
- Modify: `Core/Src/himax_sdk.c` (add implementation + include)

- [ ] **Step 1: Add defines and prototype to `himax_sdk.h`**

Open `Core/Inc/himax_sdk.h`. Replace the existing defines block (lines 14–17) and function declarations (lines 19–22) with:

```c
/* Grove AI I2C comm protocol defines */
#define GROVE_I2C_ADDR       (0x62 << 1)  /* 7-bit 0x62 → 8-bit 0xC4 (Himax i2ccomm slave 0) */
#define I2C_FEATURE_RECORDER 0x80
#define I2C_CMD_RECORD_START 0x01
#define I2C_FEATURE_LOG      0x82
#define I2C_CMD_LOG_WRITE    0x01

void uart_log(const char *fmt, ...);
uint16_t initForHimax(void);
uint16_t startRecordingForHimax(void);
void logToHimax(const char *tag, const char *fmt, ...);
```

- [ ] **Step 2: Add `<string.h>` include to `himax_sdk.c`**

At the top of `Core/Src/himax_sdk.c`, after the existing `#include <stdarg.h>` (line 9), add:

```c
#include <string.h>
```

- [ ] **Step 3: Implement `logToHimax` at end of `himax_sdk.c`**

Append this function to the end of `Core/Src/himax_sdk.c`:

```c
void logToHimax(const char *tag, const char *fmt, ...)
{
    if (tag == NULL || fmt == NULL) return;

    /* Format the message (truncates cleanly at 200 chars). */
    char msg[201];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);
    if (n < 0) return;

    /* Tag length cap (15 + NUL). */
    size_t tag_len = strnlen(tag, 16);
    if (tag_len >= 16) {
        uart_log("[logToHimax] tag too long (max 15)");
        return;
    }

    /* Build payload: tag\0msg\0 */
    uint8_t payload[220];
    size_t off = 0;
    memcpy(payload + off, tag, tag_len); off += tag_len;
    payload[off++] = '\0';
    size_t msg_len = strnlen(msg, sizeof(msg));
    memcpy(payload + off, msg, msg_len); off += msg_len;
    payload[off++] = '\0';

    HAL_StatusTypeDef rc = grove_send_cmd(I2C_FEATURE_LOG,
                                          I2C_CMD_LOG_WRITE,
                                          payload, (uint16_t)off);
    if (rc != HAL_OK) {
        uart_log("[logToHimax] I2C TX failed rc=%d", rc);
    } else {
        uart_log("[->himax] [%s] %s", tag, msg);
    }
}
```

- [ ] **Step 4: Build via STM32CubeIDE**

Build Project. Expected: clean build, no warnings about `logToHimax`.

- [ ] **Step 5: Temporarily verify on the wire**

No flash required yet — just confirm the build symbol exists: in STM32CubeIDE Project Explorer, right-click the project → Show in Local Terminal → run:

```bash
arm-none-eabi-nm Debug/i2cScan.elf | grep logToHimax
```

Expected: one `T` symbol line showing `logToHimax` is compiled into the binary.

If not available in PATH, skip — Task 4's E2E test will catch any link issue.

- [ ] **Step 6: Commit**

```bash
git add Core/Inc/himax_sdk.h Core/Src/himax_sdk.c
git commit -m "Add logToHimax() STM32 API for SD session.log

Printf-style function that builds tag\\0msg\\0 payload and sends it
as i2ccomm feature 0x82/cmd 0x01. Himax handler not yet updated —
packet will currently be logged as [I2C_CMD] Unknown customer cmd
on the Himax UART until Task 3 lands.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Task 3: Himax — add `I2C_FEATURE_LOG` handler branch

**Files:**
- Modify: `../Seeed_Grove_Vision_AI_Module_V2/EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/i2c_cmd.h`
- Modify: `../Seeed_Grove_Vision_AI_Module_V2/EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/i2c_cmd.c`

- [ ] **Step 1: Add defines to `i2c_cmd.h`**

Open the Himax `i2c_cmd.h`. After the existing `I2C_CMD_RECORD_START` define (line 8), add:

```c
#define I2C_FEATURE_LOG         0x82
#define I2C_CMD_LOG_WRITE       0x01
```

- [ ] **Step 2: Add `<string.h>` include to `i2c_cmd.c`**

Verify `#include <string.h>` is already present (it is at line 2 in the current file). If missing, add it near the other includes.

- [ ] **Step 3: Add the new branch to `i2c_customer_handler`**

In Himax `i2c_cmd.c`, locate the existing `if (feature == I2C_FEATURE_RECORDER && cmd == I2C_CMD_RECORD_START)` block (currently ending at the line with `sdlog_write("[I2C] Recording started ...)`). Change the trailing `else` (the one that prints `Unknown customer cmd`) into `else if ... else` so the LOG branch sits between them. The final shape of the conditional:

```c
    if (feature == I2C_FEATURE_RECORDER && cmd == I2C_CMD_RECORD_START) {
        /* ...existing recorder body unchanged... */
    } else if (feature == I2C_FEATURE_LOG && cmd == I2C_CMD_LOG_WRITE) {
        uint16_t plen = ((uint16_t)gRead_buf[USE_DW_IIC_SLV_0][I2CFMT_PAYLOADLEN_MSB_OFFSET] << 8)
                      |  (uint16_t)gRead_buf[USE_DW_IIC_SLV_0][I2CFMT_PAYLOADLEN_LSB_OFFSET];

        if (plen < 2 || plen > I2CCOMM_MAX_PAYLOAD_SIZE) {
            xprintf("[I2C_LOG] bad plen=%u\r\n", plen);
        } else {
            const char *buf = (const char *)&gRead_buf[USE_DW_IIC_SLV_0][I2CFMT_PAYLOAD_OFFSET];
            size_t tag_len = strnlen(buf, 16);
            if (tag_len >= 16 || (tag_len + 1) >= plen) {
                xprintf("[I2C_LOG] malformed payload (tag_len=%u, plen=%u)\r\n",
                        (unsigned)tag_len, plen);
            } else {
                const char *tag = buf;
                const char *msg = buf + tag_len + 1;
                sdlog_write("[%s] %s\r\n", tag, msg);
                xprintf("[I2C_LOG] [%s] %s\r\n", tag, msg);
            }
        }
    } else {
        xprintf("[I2C_CMD] Unknown customer cmd: feature=0x%02x cmd=0x%02x\r\n", feature, cmd);
    }
```

Do **not** gate the write on `g_recording_active` — `session.log` exists from boot (created by `sdlog_session_init`) and log lines should appear any time after mount.

- [ ] **Step 4: Build Himax firmware**

In the Himax SDK tree, build the `tflm_yolov8_od_sdlog` scenario per the project's usual build command (e.g. `make -C EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog` or whatever the repo README documents). Expected: clean build, produces the flashable image used for Task 4.

- [ ] **Step 5: Flash Himax and watch UART for the branch being reachable**

Flash the Himax image. On boot, the `[I2C_CMD] Handler registered (addr=0x28)` banner should still appear in the Himax UART — confirms the handler is live. No functional change visible yet until Task 4 sends a packet.

- [ ] **Step 6: Commit (Himax repo)**

```bash
cd ../Seeed_Grove_Vision_AI_Module_V2
git add EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/i2c_cmd.h \
        EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/i2c_cmd.c
git commit -m "Add I2C_FEATURE_LOG (0x82) handler for SD session.log

Parses tag\\0msg\\0 payload from STM32 and appends [tag] msg\\r\\n to
the current SESSION_XXXX/session.log via sdlog_write. Mirrors the
printf line to xprintf for UART debugging. Not gated on recording
state — logs are accepted any time after SD mount.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
cd ../i2cScan
```

---

## Task 4: End-to-end verification with a sample call

**Files:**
- Modify: `Core/Src/main.c:109` (add sample calls in `USER CODE BEGIN 2` block)

- [ ] **Step 1: Add sample `logToHimax` calls in `main.c`**

Open `Core/Src/main.c`. Inside the `USER CODE BEGIN 2` block, after the existing `initForHimax` / `startRecordingForHimax` block (currently ending at line 109), insert:

```c
  /* E2E test: write a few lines to the Himax SD session.log. */
  logToHimax("STM32", "hello from stm32, tick=%lu", HAL_GetTick());
  HAL_Delay(100);
  logToHimax("STM32", "second line, counter=%d", 42);
  HAL_Delay(100);
  logToHimax("STM32", "formatted float: %.2f", 3.14159f);
```

These sit inside the existing `USER CODE BEGIN 2` / `USER CODE END 2` block, so CubeMX regen will not overwrite them.

- [ ] **Step 2: Build and flash STM32**

Build Project → flash. Expected UART4 output on STM32:

```
[->himax] [STM32] hello from stm32, tick=<N>
[->himax] [STM32] second line, counter=42
[->himax] [STM32] formatted float: 3.14
```

Each should be preceded by `I2C TX OK` in the existing recorder path (already happened at boot) but **not** by `I2C TX failed`. If you see `I2C TX failed`, stop and inspect — likely a framing or addressing error.

- [ ] **Step 3: Capture Himax UART**

On the Himax UART, you should see three `[I2C_LOG]` lines:

```
[I2C_LOG] [STM32] hello from stm32, tick=<N>
[I2C_LOG] [STM32] second line, counter=42
[I2C_LOG] [STM32] formatted float: 3.14
```

plus three `[I2C_CMD] raw 8-10 bytes: 82 01 ...` debug dumps (existing hex dump in the customer handler).

If you see `Unknown customer cmd: feature=0x82 cmd=0x01` instead, the Himax firmware wasn't re-flashed — repeat Task 3 step 5.

- [ ] **Step 4: Verify `session.log` on the SD card**

Power down, remove SD card, mount on PC. Open the newest `SESSION_XXXX/session.log`. Expected tail of the file:

```
[BOOT] Session SESSION_XXXX started
...
[I2C] Recording started (threshold=0.50)
[STM32] hello from stm32, tick=<N>
[STM32] second line, counter=42
[STM32] formatted float: 3.14
```

All three `[STM32]` lines must be present, in order, with CRLF line endings. If any are missing, the transport succeeded per UART but `sdlog_write` didn't flush — check `f_sync` path in `sdlog.c`.

- [ ] **Step 5: Commit the sample calls (or remove them)**

If you want to keep the three test lines as a smoke test that runs on every boot:

```bash
git add Core/Src/main.c
git commit -m "Add logToHimax E2E smoke test calls in main.c

Three boot-time logToHimax calls verify the STM32→Himax→SD path
end-to-end: tick-stamped hello, integer formatting, float formatting.
Can be removed once downstream callers are wired up.

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

If you'd rather remove them so boot stays clean, delete the three lines + comment and commit:

```bash
git add Core/Src/main.c
git commit -m "Remove logToHimax smoke test after E2E verification

Co-Authored-By: Claude Opus 4.7 <noreply@anthropic.com>"
```

---

## Final verification checklist

- [ ] STM32 builds without warnings touching `himax_sdk.c` or `himax_sdk.h`
- [ ] Existing recorder command (`startRecordingForHimax`) still works post-refactor
- [ ] `logToHimax("TAG", "fmt", ...)` produces `[TAG] fmt` in `SESSION_XXXX/session.log`
- [ ] Formatting (`%lu`, `%d`, `%.2f`) is preserved on the SD card
- [ ] STM32 UART shows `[->himax]` echo on success, `[logToHimax] I2C TX failed` on failure
- [ ] Himax UART shows `[I2C_LOG]` echo on each received log packet
- [ ] Malformed packets (e.g. missing tag NUL) are rejected with `[I2C_LOG] malformed payload`, not written to SD
- [ ] CRC-mismatched packets are handled by the existing bypass (same as recorder today)

---

## Notes for the implementer

- **Do not touch `i2cScan.ioc`.** Nothing in this plan requires CubeMX regen.
- **Respect `USER CODE BEGIN/END` guards** in `main.c`. The sample calls in Task 4 go inside `USER CODE BEGIN 2`.
- **The Himax side lives in a separate repo.** Task 3's commit happens in `../Seeed_Grove_Vision_AI_Module_V2`, not in `i2cScan`.
- **If Task 3 is skipped or deferred,** Task 2 is still valid — the STM32 transmit will succeed, but the Himax will log `Unknown customer cmd: feature=0x82 cmd=0x01` and nothing will reach `session.log`.
- **No heap allocation.** All buffers are stack; worst case ~480 B of stack (`msg[201]` + `payload[220]` + small locals). Current STM32G431 default stack is 4 KB, so safe.
