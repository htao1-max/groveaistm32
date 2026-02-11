///*
// * Seeed_Arduino_SSCMA.h
// *
// *  Created on: Dec 24, 2025
// *      Author: frank
// */
//
//
//
//
//
//
//
//
///**
// * sscmadrv.h
// * Description: STM32 native driver for Seeed Grove AI modules (SSCMA)
// * 2024 Copyright (c) Seeed Technology Inc. All right reserved.
// *
// * This is a non-Arduino implementation optimized for STM32 microcontrollers.
// * Based on original Seeed_Arduino_SSCMA.h by Hongtai Liu(lht856@foxmail.com)
// */
//
//#ifndef SEEED_ARDUINO_SSCMA_SRC_SEEED_ARDUINO_SSCMA_H_
//#define SEEED_ARDUINO_SSCMA_SRC_SEEED_ARDUINO_SSCMA_H_
//#ifdef __cplusplus
//extern "C" {
//#endif
//
//#include <stdint.h>
//#include <stdbool.h>
//#include <stddef.h>
//#include <string.h>
//
///* STM32 HAL includes - adjust for your specific MCU series */
//#include "stm32g4xx_hal.h"  /* Example for STM32G4 series, modify as needed */
//
///* Configuration options - modify according to your hardware */
//#define SSCMA_DEFAULT_I2C_ADDRESS   0x62
//#define SSCMA_DEFAULT_UART_BAUDRATE 921600
//#define SSCMA_DEFAULT_SPI_BAUDRATE  15000000
//
///* Memory configuration - adjust based on available RAM */
//#ifdef STM32G431xx  /* Example for STM32G4 with sufficient RAM */
//#define SSCMA_MAX_RX_SIZE  16384  /* 16KB buffer */
//#else
//#define SSCMA_MAX_RX_SIZE  4096   /* 4KB buffer for resource-constrained MCUs */
//#endif
//#define SSCMA_MAX_TX_SIZE  1024   /* TX buffer size */
//
///* Protocol definitions */
//#define RESPONSE_PREFIX     "\r{"
//#define RESPONSE_SUFFIX     "}\n"
//#define RESPONSE_PREFIX_LEN (sizeof(RESPONSE_PREFIX) - 1)
//#define RESPONSE_SUFFIX_LEN (sizeof(RESPONSE_SUFFIX) - 1)
//
//#define CMD_PREFIX          "AT+"
//#define CMD_SUFFIX          "\r\n"
//
//#define CMD_TYPE_RESPONSE   0
//#define CMD_TYPE_EVENT      1
//#define CMD_TYPE_LOG        2
//
///* Command definitions */
//#define CMD_AT_ID           "ID?"
//#define CMD_AT_NAME         "NAME?"
//#define CMD_AT_WIFI         "WIFI"
//#define CMD_AT_WIFI_VER     "WIFIVER"
//#define CMD_AT_WIFI_STA     "WIFISTA"
//#define CMD_AT_WIFI_IN4     "WIFIIN4"
//#define CMD_AT_MQTTSERVER   "MQTTSERVER"
//#define CMD_AT_MQTTSERVER_STA "MQTTSERVERSTA"
//#define CMD_AT_INVOKE       "INVOKE"
//#define CMD_AT_INFO         "INFO"
//#define CMD_AT_ACTION       "ACTION"
//#define CMD_AT_SAVE_JPEG    "save_jpeg()"
//
///* Error codes */
//#define SSCMA_OK            0
//#define SSCMA_AGAIN         1
//#define SSCMA_ELOG          2
//#define SSCMA_ETIMEDOUT     3
//#define SSCMA_EIO           4
//#define SSCMA_EINVAL        5
//#define SSCMA_ENOMEM        6
//#define SSCMA_EBUSY         7
//#define SSCMA_ENOTSUP       8
//#define SSCMA_EPERM         9
//#define SSCMA_EUNKNOWN      10
//
///* Data structures */
//typedef struct {
//    uint16_t x;
//    uint16_t y;
//    uint16_t w;
//    uint16_t h;
//    uint8_t score;
//    uint8_t target;
//} sscma_box_t;
//
//typedef struct {
//    uint8_t target;
//    uint8_t score;
//} sscma_class_t;
//
//typedef struct {
//    uint16_t x;
//    uint16_t y;
//    uint8_t score;
//    uint8_t target;
//} sscma_point_t;
//
//typedef struct {
//    sscma_box_t box;
//    sscma_point_t* points;
//    uint8_t num_points;
//} sscma_keypoints_t;
//
//typedef struct {
//    uint16_t prepocess;
//    uint16_t inference;
//    uint16_t postprocess;
//} sscma_perf_t;
//
//typedef struct {
//    int status;
//    int security;
//    char ssid[64];
//    char password[64];
//} sscma_wifi_config_t;
//
//typedef struct {
//    int status;
//    uint16_t port;
//    bool use_ssl;
//    char server[128];
//    char username[128];
//    char password[128];
//    char client_id[128];
//} sscma_mqtt_config_t;
//
//typedef struct {
//    int status;
//    char ipv4[16];
//    char netmask[16];
//    char gateway[16];
//} sscma_wifi_status_t;
//
//typedef struct {
//    int status;
//} sscma_mqtt_status_t;
//
///* Dynamic array implementations */
//typedef struct {
//    sscma_box_t* items;
//    uint16_t count;
//    uint16_t capacity;
//} sscma_box_array_t;
//
//typedef struct {
//    sscma_class_t* items;
//    uint16_t count;
//    uint16_t capacity;
//} sscma_class_array_t;
//
//typedef struct {
//    sscma_point_t* items;
//    uint16_t count;
//    uint16_t capacity;
//} sscma_point_array_t;
//
//typedef struct {
//    sscma_keypoints_t* items;
//    uint16_t count;
//    uint16_t capacity;
//} sscma_keypoints_array_t;
//
///* Main context structure */
//typedef struct {
//    /* Communication interface selection */
//    enum {
//        SSCMA_IFACE_NONE = 0,
//        SSCMA_IFACE_I2C,
//        SSCMA_IFACE_UART,
//        SSCMA_IFACE_SPI
//    } interface_type;
//
//    /* I2C interface */
//    I2C_HandleTypeDef* hi2c;
//    uint16_t i2c_address;
//
//    /* UART interface */
//    UART_HandleTypeDef* huart;
//
//    /* SPI interface */
//    SPI_HandleTypeDef* hspi;
//    GPIO_TypeDef* cs_port;
//    uint16_t cs_pin;
//    GPIO_TypeDef* sync_port;
//    uint16_t sync_pin;
//    GPIO_TypeDef* rst_port;
//    uint16_t rst_pin;
//
//    /* Common parameters */
//    uint32_t wait_delay_ms;
//
//    /* Performance data */
//    sscma_perf_t perf;
//
//    /* Detection results */
//    sscma_box_array_t boxes;
//    sscma_class_array_t classes;
//    sscma_point_array_t points;
//    sscma_keypoints_array_t keypoints;
//
//    /* Device information */
//    char device_name[32];
//    char device_id[32];
//    char firmware_info[128];
//
//    /* Last captured image (base64 string) */
//    char* last_image;
//    uint32_t last_image_len;
//
//    /* Buffer management */
//    uint8_t* tx_buffer;
//    uint32_t tx_buffer_size;
//    uint8_t* rx_buffer;
//    uint32_t rx_buffer_size;
//    uint32_t rx_index;
//
//    /* Internal state */
//    void* json_context;  /* For JSON parsing (implementation dependent) */
//} sscma_handle_t;
//
///* Function prototypes */
///**
// * @brief Initialize SSCMA handle with default values
// * @param handle Pointer to sscma_handle_t structure to initialize
// */
//void sscma_init(sscma_handle_t* handle);
//
///**
// * @brief Initialize SSCMA module via I2C interface
// * @param handle Pointer to initialized sscma_handle_t
// * @param hi2c Pointer to I2C handle (HAL_I2C structure)
// * @param address I2C address of the device (default: 0x62)
// * @param wait_delay_ms Delay between commands in milliseconds
// * @return SSCMA_OK on success, error code otherwise
// */
//int sscma_begin_i2c(sscma_handle_t* handle, I2C_HandleTypeDef* hi2c,
//                   uint16_t address, uint32_t wait_delay_ms);
//
///**
// * @brief Initialize SSCMA module via UART interface
// * @param handle Pointer to initialized sscma_handle_t
// * @param huart Pointer to UART handle (HAL_UART structure)
// * @param baudrate Communication baudrate (default: 921600)
// * @param wait_delay_ms Delay between commands in milliseconds
// * @return SSCMA_OK on success, error code otherwise
// */
//int sscma_begin_uart(sscma_handle_t* handle, UART_HandleTypeDef* huart,
//                    uint32_t baudrate, uint32_t wait_delay_ms);
//
///**
// * @brief Initialize SSCMA module via SPI interface
// * @param handle Pointer to initialized sscma_handle_t
// * @param hspi Pointer to SPI handle (HAL_SPI structure)
// * @param cs_port GPIO port for CS pin
// * @param cs_pin GPIO pin number for CS
// * @param sync_port GPIO port for SYNC pin (optional, can be NULL)
// * @param sync_pin GPIO pin number for SYNC (optional, can be 0)
// * @param rst_port GPIO port for RST pin (optional, can be NULL)
// * @param rst_pin GPIO pin number for RST (optional, can be 0)
// * @param baudrate SPI baudrate (default: 15000000)
// * @param wait_delay_ms Delay between commands in milliseconds
// * @return SSCMA_OK on success, error code otherwise
// */
//int sscma_begin_spi(sscma_handle_t* handle, SPI_HandleTypeDef* hspi,
//                   GPIO_TypeDef* cs_port, uint16_t cs_pin,
//                   GPIO_TypeDef* sync_port, uint16_t sync_pin,
//                   GPIO_TypeDef* rst_port, uint16_t rst_pin,
//                   uint32_t baudrate, uint32_t wait_delay_ms);
//
///**
// * @brief Execute inference on the device
// * @param handle Pointer to initialized sscma_handle_t
// * @param times Number of times to run inference
// * @param filter Enable post-processing filtering
// * @param show If true, returns image data (requires larger RX buffer)
// * @return SSCMA_OK on success, error code otherwise
// */
//int sscma_invoke(sscma_handle_t* handle, uint8_t times, bool filter, bool show);
//
///**
// * @brief Get device ID
// * @param handle Pointer to initialized sscma_handle_t
// * @param id_buffer Buffer to store device ID
// * @param buffer_size Size of id_buffer
// * @param use_cache If true, returns cached value if available
// * @return SSCMA_OK on success, error code otherwise
// */
//int sscma_get_id(sscma_handle_t* handle, char* id_buffer, size_t buffer_size, bool use_cache);
//
///**
// * @brief Get device name
// * @param handle Pointer to initialized sscma_handle_t
// * @param name_buffer Buffer to store device name
// * @param buffer_size Size of name_buffer
// * @param use_cache If true, returns cached value if available
// * @return SSCMA_OK on success, error code otherwise
// */
//int sscma_get_name(sscma_handle_t* handle, char* name_buffer, size_t buffer_size, bool use_cache);
//
///**
// * @brief Get firmware information
// * @param handle Pointer to initialized sscma_handle_t
// * @param info_buffer Buffer to store firmware info
// * @param buffer_size Size of info_buffer
// * @param use_cache If true, returns cached value if available
// * @return SSCMA_OK on success, error code otherwise
// */
//int sscma_get_info(sscma_handle_t* handle, char* info_buffer, size_t buffer_size, bool use_cache);
//
///**
// * @brief Get WiFi configuration
// * @param handle Pointer to initialized sscma_handle_t
// * @param wifi_config Pointer to sscma_wifi_config_t structure to fill
// * @return SSCMA_OK on success, error code otherwise
// */
//int sscma_get_wifi_config(sscma_handle_t* handle, sscma_wifi_config_t* wifi_config);
//
///**
// * @brief Get MQTT configuration
// * @param handle Pointer to initialized sscma_handle_t
// * @param mqtt_config Pointer to sscma_mqtt_config_t structure to fill
// * @return SSCMA_OK on success, error code otherwise
// */
//int sscma_get_mqtt_config(sscma_handle_t* handle, sscma_mqtt_config_t* mqtt_config);
//
///**
// * @brief Set WiFi status
// * @param handle Pointer to initialized sscma_handle_t
// * @param wifi_status Pointer to sscma_wifi_status_t structure with status data
// * @return SSCMA_OK on success, error code otherwise
// */
//int sscma_set_wifi_status(sscma_handle_t* handle, sscma_wifi_status_t* wifi_status);
//
///**
// * @brief Set MQTT status
// * @param handle Pointer to initialized sscma_handle_t
// * @param mqtt_status Pointer to sscma_mqtt_status_t structure with status data
// * @return SSCMA_OK on success, error code otherwise
// */
//int sscma_set_mqtt_status(sscma_handle_t* handle, sscma_mqtt_status_t* mqtt_status);
//
///**
// * @brief Save JPEG image
// * @param handle Pointer to initialized sscma_handle_t
// * @return SSCMA_OK on success, error code otherwise
// */
//int sscma_save_jpeg(sscma_handle_t* handle);
//
///**
// * @brief Clear all actions
// * @param handle Pointer to initialized sscma_handle_t
// * @return SSCMA_OK on success, error code otherwise
// */
//int sscma_clear_actions(sscma_handle_t* handle);
//
///**
// * @brief Get last captured image as base64 string
// * @param handle Pointer to initialized sscma_handle_t
// * @param image_buffer Buffer to store image data
// * @param buffer_size Size of image_buffer
// * @return Number of bytes copied, or negative error code
// */
//int sscma_get_last_image(sscma_handle_t* handle, char* image_buffer, size_t buffer_size);
//
///**
// * @brief Free resources allocated by the driver
// * @param handle Pointer to sscma_handle_t to clean up
// */
//void sscma_deinit(sscma_handle_t* handle);
//
//#ifdef __cplusplus
//}
//#endif
//
//#endif /* SEEED_ARDUINO_SSCMA_SRC_SEEED_ARDUINO_SSCMA_H_ */
