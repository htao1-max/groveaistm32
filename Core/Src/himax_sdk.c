/*
 * himax_sdk.c
 *
 *  Created on: Apr 3, 2026
 *      Author: frank
 */
#include "himax_sdk.h"
#include <stdio.h>
#include <stdarg.h>

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

static HAL_StatusTypeDef grove_start_recording(int threshold)
{
    /*
     * Packet layout (Himax i2ccomm):
     *   [0] Feature
     *   [1] Command
     *   [2] Payload length LSB
     *   [3] Payload length MSB
     *   [4..4+N-1] Payload (optional)
     *   [N] CRC16 LSB
     *   [N+1] CRC16 MSB
     */
    uint8_t pkt[8];  /* max: 4 header + 1 payload + 2 crc + spare */
    uint16_t payload_len = 0;
    uint16_t total;

    pkt[0] = I2C_FEATURE_RECORDER;  /* feature */
    pkt[1] = I2C_CMD_RECORD_START;  /* command */

    if (threshold >= 0 && threshold <= 100) {
        payload_len = 1;
        pkt[4] = (uint8_t)threshold;
    }

    pkt[2] = payload_len & 0xFF;         /* payload len LSB (offset 2) */
    pkt[3] = (payload_len >> 8) & 0xFF;  /* payload len MSB (offset 3) */

    total = 4 + payload_len;

    /* Append CRC-16 over header + payload */
    uint16_t crc = crc16_ccitt(pkt, total);
    pkt[total]     = crc & 0xFF;        /* CRC LSB */
    pkt[total + 1] = (crc >> 8) & 0xFF; /* CRC MSB */
    total += 2;

    return HAL_I2C_Master_Transmit(&hi2c1, GROVE_I2C_ADDR, pkt, total, 1000);
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

	uart_log("Waiting 15s for Grove AI to boot...");
	HAL_Delay(15000);

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
