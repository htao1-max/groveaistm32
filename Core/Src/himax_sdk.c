/*
 * himax_sdk.c
 *
 *  Created on: Apr 3, 2026
 *      Author: frank
 */
#include "himax_sdk.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart4;

static uint16_t crc16_ccitt(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)buf[i] << 8;
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

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

void uart_log(const char *fmt, ...)
{
    char buf[160];
    va_list args;
    int prefix = snprintf(buf, sizeof(buf), "[%010lu ms] ", HAL_GetTick());
    va_start(args, fmt);
    int msg = vsnprintf(buf + prefix, (int)sizeof(buf) - prefix - 2, fmt, args);
    va_end(args);
    int total = prefix + msg;
    if (total > (int)sizeof(buf) - 2) total = (int)sizeof(buf) - 2;
    buf[total++] = '\r';
    buf[total++] = '\n';
    HAL_UART_Transmit(&huart4, (uint8_t*)buf, (uint16_t)total, 1000);
}

uint16_t initForHimax(void)
{
	uart_log("========================================");
	uart_log("  STM32 + Grove AI (i2ccomm SD-log)");
	uart_log("========================================");
	uart_log("Target i2ccomm slave addr: 0x%02X (7-bit)", GROVE_I2C_ADDR >> 1);

	uart_log("Waiting 5s for Grove AI to boot...");
	HAL_Delay(5000);

	// I2C bus scan — probe all 7-bit addresses on I2C1
	uart_log("--- I2C bus scan on I2C1 (PB7=SDA, PB8=SCL) ---");
	int found_count = 0;
	int found_0x62 = 0;
	for (uint8_t addr = 0x03; addr <= 0x77; addr++)
	{
	  if (HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK)
	  {
		  uart_log("  0x%02X  ACK%s", addr,
				   (addr == (GROVE_I2C_ADDR >> 1)) ? "  <-- i2ccomm target" : "");
		  found_count++;
		  if (addr == (GROVE_I2C_ADDR >> 1)) found_0x62 = 1;
	  }
	}
	uart_log("  %d device(s) found", found_count);
	//remove i2cscan
	if (!found_0x62)
	{
	  uart_log("WARNING: 0x62 not found in scan! Retrying every 2s...");
	  for (int retry = 0; retry < 10 && !found_0x62; retry++)
	  {
		  HAL_Delay(2000);
		  if (HAL_I2C_IsDeviceReady(&hi2c1, GROVE_I2C_ADDR, 3, 100) == HAL_OK)
		  {
			  uart_log("  0x62 appeared on retry %d", retry + 1);
			  found_0x62 = 1;
		  }
		  else
		  {
			  uart_log("  retry %d: 0x62 still NACK", retry + 1);
		  }
	  }
	  if (!found_0x62)
	  {
		  uart_log("FAIL: 0x62 never responded.");
		  uart_log("  Check: SDA/SCL wires go to Grove connector (not camera I2C)");
		  uart_log("  Check: GND is shared between STM32 and Grove AI");
		  uart_log("  Note: 0x28 = OV5647 camera, NOT the i2ccomm slave");
	  }

	  return 0;
	}

	return 1;
}

uint16_t startRecordingForHimax(void)
{
	// Send start-recording command with 50% threshold
	uart_log("Sending start-recording (threshold=50%%)...");
	HAL_StatusTypeDef rc = grove_start_recording(50);
	if (rc == HAL_OK)
	{
	  uart_log("I2C TX OK — check Grove AI UART for [I2C_CMD] message");
	  return 1;
	}
	else
	{
	  uart_log("I2C TX FAILED (HAL rc=%d)", rc);
	  return 0;
	}
}

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
