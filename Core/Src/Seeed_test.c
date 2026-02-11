///*
// * Seeed_test.c
// *
// *  Created on: Dec 24, 2025
// *      Author: frank
// */
//#include "Seeed_test.h"
//#include "main.h"  // 包含您的main.h以访问HAL句柄
//#include <stdio.h>
//
//sscma_handle_t g_ai_handle;
//
//void sscma_app_init(UART_HandleTypeDef* ai_uart) {
//    printf("Initializing SSCMA AI module...\n");
//
//    // 初始化SSCMA句柄
//    sscma_init(&g_ai_handle);
//
//    // 配置AI模块(使用UART接口)
//    int result = sscma_begin_i2c(&AI, &hi2c1, SSCMA_DEFAULT_I2C_ADDRESS, 2);
//    if (result != SSCMA_OK) {
//        printf("Error initializing AI module: %d\n", result);
//        return;
//    }
//
//    // 获取设备信息
//    char device_id[32];
//    if (sscma_get_id(&g_ai_handle, device_id, sizeof(device_id), true) == SSCMA_OK) {
//        printf("Device ID: %s\n", device_id);
//    }
//
//    printf("AI module initialized successfully\n");
//}
//
//void sscma_app_task(void) {
//    static uint32_t last_invoke_time = 0;
//    uint32_t current_time = HAL_GetTick();
//
//    // 每2秒执行一次推理
//    if (current_time - last_invoke_time >= 2000) {
//        last_invoke_time = current_time;
//
//        printf("\n[AI] Starting inference...\n");
//
//        // 执行推理
//        int result = sscma_invoke(&g_ai_handle, 1, true, false);
//
//        if (result == SSCMA_OK) {
//            printf("[AI] Inference successful!\n");
//            printf("[AI] Performance (ms): preprocess=%d, inference=%d, postprocess=%d\n",
//                   g_ai_handle.perf.prepocess,
//                   g_ai_handle.perf.inference,
//                   g_ai_handle.perf.postprocess);
//
//            // 打印检测结果
//            if (g_ai_handle.boxes.count > 0) {
//                printf("[AI] Detected %d objects:\n", g_ai_handle.boxes.count);
//                for (uint16_t i = 0; i < g_ai_handle.boxes.count; i++) {
//                    printf("[AI] Box %d: x=%d, y=%d, w=%d, h=%d, score=%d%%, target=%d\n",
//                           i,
//                           g_ai_handle.boxes.items[i].x,
//                           g_ai_handle.boxes.items[i].y,
//                           g_ai_handle.boxes.items[i].w,
//                           g_ai_handle.boxes.items[i].h,
//                           g_ai_handle.boxes.items[i].score,
//                           g_ai_handle.boxes.items[i].target);
//                }
//            }
//        } else {
//            printf("[AI] Inference failed: error %d\n", result);
//        }
//    }
//}
//
//
//
