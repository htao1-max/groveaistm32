/*
 * Seeed_test.h
 *
 *  Created on: Dec 24, 2025
 *      Author: frank
 */

#ifndef INC_SEEED_TEST_H_
#define INC_SEEED_TEST_H_

#include <Seeed_Arduino_SSCMA1.h>

// 全局SSCMA句柄
extern sscma_handle_t g_ai_handle;

void sscma_app_init(UART_HandleTypeDef* ai_uart);
void sscma_app_task(void);



#endif /* INC_SEEED_TEST_H_ */
