#ifndef __EP2_ISP_I2C_H__
#define __EP2_ISP_I2C_H__

#include "stm32g4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>

/* Call once before using any other function */
void ep2_isp_init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);

/* Scan I2C bus for WE2 device at address 0x28. Returns true if found. */
bool ep2_probe_device(void);

/*
 * Flash one 256-byte chunk to the WE2 via I2C ISP.
 *   chunk     : pointer to 256 bytes of firmware data (pad with 0xFF if short)
 *   addr      : target flash address (starts at ISP_DATA_PORT_ADDR, increments by 256)
 *   chunk_idx : 1-based chunk counter (used for PP_DONE verification)
 *
 * Returns 0 on success, negative on error.
 */
int ep2_isp_write_chunk(const uint8_t *chunk, uint32_t addr, uint32_t chunk_idx);

/* Initialize ISP control registers. Call once before writing chunks. */
void ep2_isp_begin(void);

/* Finalize ISP and disable programming mode. Call after all chunks are written. */
void ep2_isp_end(void);

#endif /* __EP2_ISP_I2C_H__ */
