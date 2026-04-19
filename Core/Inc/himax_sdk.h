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

void uart_log(const char *fmt, ...);
uint16_t initForHimax(void);
uint16_t startRecordingForHimax(void);
void logToHimax(const char *tag, const char *fmt, ...);

#endif /* INC_HIMAX_SDK_H_ */
