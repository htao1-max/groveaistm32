///**
// * SSCMA_STM32.h
// * Description: A driver for Seeed Grove AI Family adapted for STM32
// * Ported from Arduino version to STM32 HAL
// * Original Author: Hongtai Liu(lht856@foxmail.com)
// * STM32 Port: 2025
// *
// * Copyright (C) 2020  Seeed Technology Co.,Ltd.
// */
//
//#ifndef SSCMA_STM32_H
//#define SSCMA_STM32_H
//
//#ifdef __cplusplus
//extern "C" {
//#endif
//
//#include <stdio.h>
//#include <stdint.h>
//#include <string.h>
//#include <stdbool.h>
//#include <stdlib.h>
//
//// Include STM32 HAL - adjust based on your STM32 family
//// For STM32G4 (your chip):
//#include "stm32g4xx_hal.h"
//
//// Configuration defines
//#ifndef SSCMA_UART_BAUD
//#define SSCMA_UART_BAUD 921600
//#endif
//
//#ifndef SSCMA_IIC_CLOCK
//#define SSCMA_IIC_CLOCK 400000
//#endif
//
//#ifndef SSCMA_MAX_RX_SIZE
//#define SSCMA_MAX_RX_SIZE (4 * 1024)
//#endif
//
//#ifndef SSCMA_MAX_TX_SIZE
//#define SSCMA_MAX_TX_SIZE (4 * 1024)
//#endif
//
//// Protocol defines
//#define I2C_ADDRESS (0x62 << 1)  // STM32 HAL uses 8-bit address
//
//#define HEADER_LEN 4
//#define MAX_PL_LEN 250
//#define CHECKSUM_LEN 2
//#define PACKET_SIZE (HEADER_LEN + MAX_PL_LEN + CHECKSUM_LEN)
//
//#define FEATURE_TRANSPORT 0x10
//#define FEATURE_TRANSPORT_CMD_READ 0x01
//#define FEATURE_TRANSPORT_CMD_WRITE 0x02
//#define FEATURE_TRANSPORT_CMD_AVAILABLE 0x03
//#define FEATURE_TRANSPORT_CMD_START 0x04
//#define FEATURE_TRANSPORT_CMD_STOP 0x05
//#define FEATURE_TRANSPORT_CMD_RESET 0x06
//
//#define RESPONSE_PREFIX "\r{"
//#define RESPONSE_SUFFIX "}\n"
//#define RESPONSE_PREFIX_LEN 2
//#define RESPONSE_SUFFIX_LEN 2
//
//#define CMD_PREFIX "AT+"
//#define CMD_SUFFIX "\r\n"
//
//#define CMD_TYPE_RESPONSE 0
//#define CMD_TYPE_EVENT 1
//#define CMD_TYPE_LOG 2
//
//// Command strings
//#define CMD_AT_ID "ID?"
//#define CMD_AT_NAME "NAME?"
//#define CMD_AT_VERSION "VER?"
//#define CMD_AT_STATS "STAT"
//#define CMD_AT_BREAK "BREAK"
//#define CMD_AT_RESET "RST"
//#define CMD_AT_WIFI "WIFI"
//#define CMD_AT_WIFI_VER "WIFIVER"
//#define CMD_AT_WIFI_STA "WIFISTA"
//#define CMD_AT_WIFI_IN4 "WIFIIN4"
//#define CMD_AT_MQTTSERVER "MQTTSERVER"
//#define CMD_AT_MQTTSERVER_STA "MQTTSERVERSTA"
//#define CMD_AT_INVOKE "INVOKE"
//#define CMD_AT_SAMPLE "SAMPLE"
//#define CMD_AT_INFO "INFO"
//#define CMD_AT_ACTION "ACTION"
//
//// Error codes
//#define CMD_OK 0
//#define CMD_AGAIN 1
//#define CMD_ELOG 2
//#define CMD_ETIMEDOUT 3
//#define CMD_EIO 4
//#define CMD_EINVAL 5
//#define CMD_ENOMEM 6
//#define CMD_EBUSY 7
//#define CMD_ENOTSUP 8
//#define CMD_EPERM 9
//#define CMD_EUNKNOWN 10
//
//// Data structures
//typedef struct {
//    uint16_t x;
//    uint16_t y;
//    uint16_t w;
//    uint16_t h;
//    uint8_t score;
//    uint8_t target;
//} boxes_t;
//
//typedef struct {
//    uint8_t target;
//    uint8_t score;
//} classes_t;
//
//typedef struct {
//    uint16_t x;
//    uint16_t y;
//    uint16_t z;
//    uint8_t score;
//    uint8_t target;
//} point_t;
//
//typedef struct {
//    boxes_t box;
//    point_t points[32];  // Fixed array for C compatibility
//    uint8_t point_count;
//} keypoints_t;
//
//typedef struct {
//    uint16_t preprocess;
//    uint16_t inference;
//    uint16_t postprocess;
//} perf_t;
//
//typedef struct {
//    int status;
//    int security;
//    char ssid[64];
//    char password[64];
//} wifi_t;
//
//typedef struct {
//    int status;
//    char ipv4[16];
//    char netmask[16];
//    char gateway[16];
//} wifi_status_t;
//
//typedef struct {
//    int status;
//    uint16_t port;
//    bool use_ssl;
//    char server[128];
//    char username[128];
//    char password[128];
//    char client_id[128];
//} mqtt_t;
//
//typedef struct {
//    int status;
//} mqtt_status_t;
//
//// Communication interface enum
//typedef enum {
//    SSCMA_INTERFACE_I2C,
//    SSCMA_INTERFACE_UART
//} SSCMA_Interface_t;
//
//// Callback function type
//typedef void (*ResponseCallback)(const char *resp, size_t len);
//
//typedef struct {
//    uint16_t x;
//    uint16_t y;
//    uint16_t w;
//    uint16_t h;
//    uint8_t score;
//    uint8_t target;
//} SSCMA_Box_t;
//
//// Main SSCMA structure
//typedef struct {
//    // Communication interface
//    SSCMA_Interface_t interface;
//
//    // HAL handles
//    I2C_HandleTypeDef *hi2c;
//    UART_HandleTypeDef *huart;
//
//
////    UART_HandleTypeDef *debug_uart;
//
//
//    // GPIO pins (for reset only)
//    GPIO_TypeDef *rst_port;
//    uint16_t rst_pin;
//
//    // Settings
//    uint16_t i2c_address;
//    uint32_t wait_delay;
//
//    // Buffers
//    char *tx_buf;
//    uint32_t tx_len;
//    char *rx_buf;
//    uint32_t rx_len;
//    uint32_t rx_end;
//
//    // Data
//    perf_t perf;
//    boxes_t boxes[16];
//    uint8_t boxes_count;
//    classes_t classes[16];
//    uint8_t classes_count;
//    point_t points[32];
//    uint8_t points_count;
//    keypoints_t keypoints[8];
//    uint8_t keypoints_count;
//    SSCMA_Box_t last_box;
//    uint8_t box_detected;
//
//    char name[32];
//    char id[32];
//    char image[256];
//    char info[256];
//
//} SSCMA_t;
//
//// Function prototypes
//
//// Initialization functions
//bool SSCMA_Init_I2C(SSCMA_t *sscma, I2C_HandleTypeDef *hi2c,
//                    GPIO_TypeDef *rst_port, uint16_t rst_pin,
//                    uint16_t address, uint32_t wait_delay);
//
//bool SSCMA_Init_UART(SSCMA_t *sscma, UART_HandleTypeDef *huart,
//                     GPIO_TypeDef *rst_port, uint16_t rst_pin,
//                     uint32_t wait_delay);
//
//// Core functions
//int SSCMA_Invoke(SSCMA_t *sscma, int times, bool filter, bool show);
//int SSCMA_Available(SSCMA_t *sscma);
//int SSCMA_Read(SSCMA_t *sscma, char *data, int length);
//int SSCMA_Write(SSCMA_t *sscma, const char *data, int length);
//void SSCMA_Fetch(SSCMA_t *sscma, ResponseCallback callback);
//
//// Query functions
//char* SSCMA_GetID(SSCMA_t *sscma, bool cache);
//char* SSCMA_GetName(SSCMA_t *sscma, bool cache);
//char* SSCMA_GetInfo(SSCMA_t *sscma, bool cache);
//
//// Configuration functions
//int SSCMA_GetWiFi(SSCMA_t *sscma, wifi_t *wifi);
//int SSCMA_GetMQTT(SSCMA_t *sscma, mqtt_t *mqtt);
//int SSCMA_SetWiFiStatus(SSCMA_t *sscma, wifi_status_t *wifi_status);
//int SSCMA_SetMQTTStatus(SSCMA_t *sscma, mqtt_status_t *mqtt_status);
//int SSCMA_GetModelInfo(SSCMA_t *sscma);
//int SSCMA_LoadModel(SSCMA_t *sscma);
//int SSCMA_QueryModel(SSCMA_t *sscma);
//int SSCMA_SelectModel(SSCMA_t *sscma, int id);
//int SSCMA_QueryStatus(SSCMA_t *sscma);
//int SSCMA_InvokeMulti(SSCMA_t *sscma);
//int SSCMA_InvokeOnce(SSCMA_t *sscma);
//
//
//// Action functions
//int SSCMA_CleanActions(SSCMA_t *sscma);
//int SSCMA_SaveJPEG(SSCMA_t *sscma);
//
//// Buffer management
//bool SSCMA_SetRxBuffer(SSCMA_t *sscma, uint32_t size);
//bool SSCMA_SetTxBuffer(SSCMA_t *sscma, uint32_t size);
//
//// Utility functions
//char* strnstr_custom(const char *haystack, const char *needle, size_t n);
//
//#ifdef __cplusplus
//}
//#endif
//
//#endif // SSCMA_STM32_H
