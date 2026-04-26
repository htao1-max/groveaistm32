/*
 * himax_sdk.h
 *
 *  Created on: Apr 3, 2026
 *      Author: frank
 */

#ifndef INC_HIMAX_SDK_H_
#define INC_HIMAX_SDK_H_

#include "main.h"
#include <stdint.h>

/* Grove AI I2C comm protocol defines */
#define GROVE_I2C_ADDR       (0x62 << 1)  /* 7-bit 0x62 → 8-bit 0xC4 (Himax i2ccomm slave 0) */
#define I2C_FEATURE_RECORDER 0x80
#define I2C_CMD_RECORD_START 0x01
#define I2C_FEATURE_LOG      0x82
#define I2C_CMD_LOG_WRITE    0x01

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

void uart_log(const char *fmt, ...);
uint16_t initForHimax(void);
uint16_t startRecordingForHimax(void);
void logToHimax(const char *tag, const char *fmt, ...);

/* Append one telemetry sample to the batch accumulator. When the
 * accumulator holds TLM_BATCH_SIZE samples, flushes one I2C frame to
 * Himax. Caller fills q, temp_c, vbat, vmotor; stm32_tick_ms is filled
 * internally via HAL_GetTick(). Fire-and-forget. Not ISR-safe. */
void logTelemetryToHimax(telemetry_t *t);

#endif /* INC_HIMAX_SDK_H_ */
