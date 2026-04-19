# logToHimax — STM32 → Himax SD session log

**Date:** 2026-04-19
**Branch:** feature/i2ccomm-sdlog
**Status:** Approved design, pending implementation plan

## Goal

Add a single printf-style function on the STM32 side that, each time it is
called, sends a string over I2C to the Grove AI (Himax) module, which appends
a tagged line to the current `SESSION_XXXX/session.log` on the SD card — the
same file that already receives `[BOOT]`, `[ALL]`, `[DETECT]`, and `[I2C]`
lines.

## Public API (STM32)

```c
void logToHimax(const char *tag, const char *fmt, ...);
```

- `tag` — short identifier, **without brackets** (e.g. `"STM32"`). Himax wraps
  it as `[tag]` when writing to `session.log`.
- `fmt, ...` — standard printf-style formatting.

Example:

```c
logToHimax("STM32", "boot ok, tick=%lu", HAL_GetTick());
logToHimax("STM32", "button pressed, count=%d", n);
```

Produces in `SESSION_XXXX/session.log`:

```
[STM32] boot ok, tick=1234
[STM32] button pressed, count=7
```

Fire-and-forget. No ACK, no retries. On I2C transmit failure, STM32 logs a
warning over UART4 and drops the message.

## Protocol extension (i2ccomm)

Reuses the existing Himax i2ccomm framing:
`[feature][cmd][plen_lsb][plen_msb][payload...][crc_lsb][crc_msb]`
with CRC16-CCITT over `feature + cmd + plen + payload`.

New customer feature/command:

| Name                  | Value | Notes                                            |
|-----------------------|-------|--------------------------------------------------|
| `I2C_FEATURE_LOG`     | 0x82  | 0x80 = recorder, 0x81 reserved for camera (WIP). |
| `I2C_CMD_LOG_WRITE`   | 0x01  |                                                  |

**Payload format:** `tag\0msg\0` — tag as a null-terminated C string,
immediately followed by the formatted message as a null-terminated C string.
Payload length in the header is `strlen(tag) + 1 + strlen(msg) + 1`.

**Size caps:**
- tag: ≤ 15 chars (plus null)
- msg: ≤ 200 chars (plus null)
- total payload ≤ 217 bytes, well under Himax's `I2CCOMM_MAX_PAYLOAD_SIZE` (256).

## STM32 implementation (`Core/Inc/himax_sdk.h`, `Core/Src/himax_sdk.c`)

### Refactor

Extract a generic sender from the existing `grove_start_recording`:

```c
static HAL_StatusTypeDef grove_send_cmd(uint8_t feature,
                                        uint8_t cmd,
                                        const uint8_t *payload,
                                        uint16_t plen);
```

It assembles header + payload + CRC into a stack buffer and calls
`HAL_I2C_Master_Transmit(&hi2c1, GROVE_I2C_ADDR, ...)`. The existing
`grove_start_recording` becomes a one-line wrapper. This refactor also unlocks
the pending runtime-exposure work (same pattern).

### New public function

```c
void logToHimax(const char *tag, const char *fmt, ...)
{
    char msg[201];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(msg, sizeof(msg), fmt, ap);  // truncates cleanly
    va_end(ap);
    if (n < 0) return;

    size_t tag_len = strnlen(tag, 16);
    if (tag_len >= 16) { uart_log("[logToHimax] tag too long"); return; }

    uint8_t payload[220];
    size_t off = 0;
    memcpy(payload + off, tag, tag_len); off += tag_len;
    payload[off++] = '\0';
    size_t msg_len = strnlen(msg, 201);
    memcpy(payload + off, msg, msg_len); off += msg_len;
    payload[off++] = '\0';

    HAL_StatusTypeDef rc = grove_send_cmd(I2C_FEATURE_LOG,
                                          I2C_CMD_LOG_WRITE,
                                          payload, (uint16_t)off);
    if (rc != HAL_OK) {
        uart_log("[logToHimax] I2C TX failed rc=%d", rc);
    } else {
        uart_log("[->himax] [%s] %s", tag, msg);  // local echo
    }
}
```

### Header additions

```c
#define I2C_FEATURE_LOG     0x82
#define I2C_CMD_LOG_WRITE   0x01
void logToHimax(const char *tag, const char *fmt, ...);
```

## Himax implementation (`EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/i2c_cmd.[ch]`)

### `i2c_cmd.h` additions

```c
#define I2C_FEATURE_LOG     0x82
#define I2C_CMD_LOG_WRITE   0x01
```

### `i2c_customer_handler` new branch

```c
else if (feature == I2C_FEATURE_LOG && cmd == I2C_CMD_LOG_WRITE) {
    uint16_t plen = ((uint16_t)gRead_buf[USE_DW_IIC_SLV_0][I2CFMT_PAYLOADLEN_MSB_OFFSET] << 8)
                  |  (uint16_t)gRead_buf[USE_DW_IIC_SLV_0][I2CFMT_PAYLOADLEN_LSB_OFFSET];
    if (plen < 2 || plen > I2CCOMM_MAX_PAYLOAD_SIZE) {
        xprintf("[I2C_LOG] bad plen=%u\r\n", plen);
    } else {
        const char *buf = (const char *)&gRead_buf[USE_DW_IIC_SLV_0][I2CFMT_PAYLOAD_OFFSET];
        size_t tag_len = strnlen(buf, 16);
        if (tag_len >= 16 || tag_len + 1 >= plen) {
            xprintf("[I2C_LOG] malformed payload\r\n");
        } else {
            const char *tag = buf;
            const char *msg = buf + tag_len + 1;
            sdlog_write("[%s] %s\r\n", tag, msg);
            xprintf("[I2C_LOG] [%s] %s\r\n", tag, msg);
        }
    }
}
```

Logging is **not** gated on `g_recording_active` — `session.log` is created at
boot by `sdlog_session_init`, so `logToHimax` should work any time after the
Himax session mount completes.

## Error handling summary

| Failure                                | Behaviour                                           |
|----------------------------------------|-----------------------------------------------------|
| STM32 called before Himax ready        | `HAL_I2C_Master_Transmit` NACKs → UART warning, drop|
| `vsnprintf` overflow                   | Truncated cleanly at 200 chars                      |
| tag too long on STM32                  | UART warning, message dropped                       |
| Malformed payload on Himax             | `xprintf` warning, nothing written to `session.log` |
| CRC mismatch                           | Already logged and bypassed in existing handler     |

## Out of scope

- No ACK / response path.
- No rate limiting (caller's responsibility).
- No buffering while Himax SD mount is not ready — caller decides when to log.
- No binary payloads; text/ASCII only.
- Not coupled to the recording state — log lines go to the session even when
  recording is inactive.

## Test plan (manual)

1. Build STM32 firmware, flash.
2. Build Himax firmware with the new handler, flash.
3. Boot both; wait for `initForHimax()` to confirm 0x62 ACKs.
4. From `main.c`, add a few test calls:
   ```c
   logToHimax("STM32", "hello world");
   logToHimax("STM32", "tick=%lu", HAL_GetTick());
   ```
5. Eject SD card; open `SESSION_XXXX/session.log` and verify:
   - Lines appear in order with correct `[STM32]` prefix.
   - Formatting is preserved.
   - No truncation for normal-length messages.
6. Check Himax UART: `[I2C_LOG] ...` echo lines appear.
7. Check STM32 UART4: `[->himax] ...` echo lines appear.

## Files touched

- `Core/Inc/himax_sdk.h` — new defines, new prototype.
- `Core/Src/himax_sdk.c` — refactor sender, add `logToHimax`.
- `Seeed_Grove_Vision_AI_Module_V2/EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/i2c_cmd.h` — new defines.
- `Seeed_Grove_Vision_AI_Module_V2/EPII_CM55M_APP_S/app/scenario_app/tflm_yolov8_od_sdlog/i2c_cmd.c` — new handler branch.
