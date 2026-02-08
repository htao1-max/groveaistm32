/*
 * Seeed_Arduino_SSCMA.cpp
 *
 *  Created on: Dec 24, 2025
 *      Author: frank
 */

/**
 * sscmadrv.c
 * Description: STM32 native driver implementation for Seeed Grove AI modules (SSCMA)
 * 2024 Copyright (c) Seeed Technology Inc. All right reserved.
 */

#include "Seeed_Arduino_SSCMA.h"
#include <stdlib.h>
#include <string.h>
#include "json_parser.h"  /* Simple JSON parser implementation for STM32 */

/* Internal constants */
#define MAX_CMD_LENGTH      128
#define DEFAULT_TIMEOUT_MS  2000
#define MIN(a, b)           ((a) < (b) ? (a) : (b))
#define MAX(a, b)           ((a) > (b) ? (a) : (b))

/* Internal function prototypes */
static int sscma_send_command(sscma_handle_t* handle, const char* cmd, size_t cmd_len);
static int sscma_wait_for_response(sscma_handle_t* handle, const char* expected_cmd,
                                   uint8_t expected_type, uint32_t timeout_ms);
static int sscma_parse_event(sscma_handle_t* handle, json_object_t* json_obj);
static int sscma_i2c_write(sscma_handle_t* handle, const uint8_t* data, size_t len);
static int sscma_i2c_read(sscma_handle_t* handle, uint8_t* data, size_t len);
static int sscma_i2c_available(sscma_handle_t* handle);
static int sscma_uart_write(sscma_handle_t* handle, const uint8_t* data, size_t len);
static int sscma_uart_read(sscma_handle_t* handle, uint8_t* data, size_t len);
static int sscma_uart_available(sscma_handle_t* handle);
static int sscma_spi_write(sscma_handle_t* handle, const uint8_t* data, size_t len);
static int sscma_spi_read(sscma_handle_t* handle, uint8_t* data, size_t len);
static int sscma_spi_available(sscma_handle_t* handle);
static void sscma_spi_cs_select(sscma_handle_t* handle, bool state);
static void sscma_spi_reset_device(sscma_handle_t* handle);
static char* sscma_strnstr(const char* haystack, const char* needle, size_t len);
static int sscma_grow_array(void** array, uint16_t* count, uint16_t* capacity,
                           size_t item_size, uint16_t required_capacity);

/* Memory allocation helpers */
static void* sscma_malloc(sscma_handle_t* handle, size_t size) {
    /* In production code, consider using a memory pool or static allocation */
    return malloc(size);
}

static void sscma_free(sscma_handle_t* handle, void* ptr) {
    if (ptr) free(ptr);
}

/* Array management functions */
static int sscma_init_box_array(sscma_box_array_t* array, uint16_t initial_capacity) {
    array->items = (sscma_box_t*)malloc(initial_capacity * sizeof(sscma_box_t));
    if (!array->items) return SSCMA_ENOMEM;
    array->count = 0;
    array->capacity = initial_capacity;
    return SSCMA_OK;
}

static void sscma_free_box_array(sscma_box_array_t* array) {
    if (array->items) free(array->items);
    array->items = NULL;
    array->count = 0;
    array->capacity = 0;
}

static int sscma_add_box(sscma_box_array_t* array, const sscma_box_t* box) {
    if (array->count >= array->capacity) {
        /* Grow array by 50% */
        uint16_t new_capacity = array->capacity + (array->capacity >> 1) + 1;
        sscma_box_t* new_items = (sscma_box_t*)realloc(array->items,
                                                     new_capacity * sizeof(sscma_box_t));
        if (!new_items) return SSCMA_ENOMEM;
        array->items = new_items;
        array->capacity = new_capacity;
    }

    array->items[array->count] = *box;
    array->count++;
    return SSCMA_OK;
}

/* Similar functions for other array types (classes, points, keypoints) would be implemented */

/* JSON parsing helpers */
static bool json_get_string(json_object_t* obj, const char* key, char* buffer, size_t buffer_size) {
    json_value_t* value = json_object_get(obj, key);
    if (!value || value->type != JSON_STRING || !value->string_value) return false;

    strncpy(buffer, value->string_value, buffer_size - 1);
    buffer[buffer_size - 1] = '\0';
    return true;
}

static bool json_get_int(json_object_t* obj, const char* key, int* value) {
    json_value_t* json_val = json_object_get(obj, key);
    if (!json_val || (json_val->type != JSON_NUMBER && json_val->type != JSON_INT))
        return false;

    *value = (int)json_val->number_value;
    return true;
}

/* Hardware abstraction layer functions */
static void sscma_delay_ms(uint32_t ms) {
    HAL_Delay(ms);
}

static uint32_t sscma_get_tick(void) {
    return HAL_GetTick();
}

static void sscma_gpio_write(GPIO_TypeDef* port, uint16_t pin, uint8_t state) {
    if (port) {
        if (state) {
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
        } else {
            HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
        }
    }
}

static uint8_t sscma_gpio_read(GPIO_TypeDef* port, uint16_t pin) {
    if (port) {
        return HAL_GPIO_ReadPin(port, pin);
    }
    return 0;
}

/* String utilities */
static char* sscma_strnstr(const char* haystack, const char* needle, size_t len) {
    if (!haystack || !needle || len == 0) return NULL;

    size_t needle_len = strlen(needle);
    if (needle_len == 0) return (char*)haystack;

    for (size_t i = 0; i < len && haystack[i] != '\0'; i++) {
        if (i + needle_len > len) break;

        bool match = true;
        for (size_t j = 0; j < needle_len; j++) {
            if (haystack[i + j] != needle[j]) {
                match = false;
                break;
            }
        }

        if (match) return (char*)&haystack[i];
    }

    return NULL;
}

/* Core driver functions */
void sscma_init(sscma_handle_t* handle) {
    if (!handle) return;

    memset(handle, 0, sizeof(sscma_handle_t));

    /* Initialize arrays */
    sscma_init_box_array(&handle->boxes, 4);
    /* Similar initialization for other arrays */

    /* Set default buffer sizes */
    handle->tx_buffer_size = SSCMA_MAX_TX_SIZE;
    handle->rx_buffer_size = SSCMA_MAX_RX_SIZE;

    /* Allocate buffers */
    handle->tx_buffer = (uint8_t*)malloc(handle->tx_buffer_size);
    handle->rx_buffer = (uint8_t*)malloc(handle->rx_buffer_size);

    /* Default wait delay */
    handle->wait_delay_ms = 2;
}

void sscma_deinit(sscma_handle_t* handle) {
    if (!handle) return;

    /* Free arrays */
    sscma_free_box_array(&handle->boxes);
    /* Free other arrays similarly */

    /* Free buffers */
    if (handle->tx_buffer) free(handle->tx_buffer);
    if (handle->rx_buffer) free(handle->rx_buffer);
    if (handle->last_image) free(handle->last_image);

    /* Free JSON context if used */
    if (handle->json_context) {
        json_parser_free_context(handle->json_context);
        handle->json_context = NULL;
    }

    memset(handle, 0, sizeof(sscma_handle_t));
}

static void sscma_hardware_reset(sscma_handle_t* handle) {
    if (handle->rst_port) {
        /* Configure as output */
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = handle->rst_pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        HAL_GPIO_Init(handle->rst_port, &GPIO_InitStruct);

        /* Perform reset sequence */
        sscma_gpio_write(handle->rst_port, handle->rst_pin, 0);
        sscma_delay_ms(50);

        /* Reconfigure as input (floating) */
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(handle->rst_port, &GPIO_InitStruct);

        sscma_delay_ms(500);
    }
}

int sscma_begin_i2c(sscma_handle_t* handle, I2C_HandleTypeDef* hi2c,
                   uint16_t address, uint32_t wait_delay_ms) {
    if (!handle || !hi2c) return SSCMA_EINVAL;

    handle->interface_type = SSCMA_IFACE_I2C;
    handle->hi2c = hi2c;
    handle->i2c_address = (address) ? address : SSCMA_DEFAULT_I2C_ADDRESS;
    handle->wait_delay_ms = (wait_delay_ms) ? wait_delay_ms : 2;

    /* Initialize JSON parser context */
    handle->json_context = json_parser_init_context(2048);
    if (!handle->json_context) return SSCMA_ENOMEM;

    /* Reset device if RST pin is configured */
    sscma_hardware_reset(handle);

    /* Check communication with device */
    char id[32];
    if (sscma_get_id(handle, id, sizeof(id), false) != SSCMA_OK) {
        return SSCMA_EIO;
    }

    return SSCMA_OK;
}

int sscma_begin_uart(sscma_handle_t* handle, UART_HandleTypeDef* huart,
                    uint32_t baudrate, uint32_t wait_delay_ms) {
    if (!handle || !huart) return SSCMA_EINVAL;

    handle->interface_type = SSCMA_IFACE_UART;
    handle->huart = huart;
    handle->wait_delay_ms = (wait_delay_ms) ? wait_delay_ms : 2;

    /* Initialize JSON parser context */
    handle->json_context = json_parser_init_context(2048);
    if (!handle->json_context) return SSCMA_ENOMEM;

    /* Configure UART baudrate if needed */
    if (baudrate && baudrate != SSCMA_DEFAULT_UART_BAUDRATE) {
        /* Reconfigure UART with new baudrate */
        /* Implementation depends on HAL version and STM32 series */
        huart->Init.BaudRate = baudrate;
        if (HAL_UART_Init(huart) != HAL_OK) {
            return SSCMA_EIO;
        }
    }

    /* Reset device if RST pin is configured */
    sscma_hardware_reset(handle);

    /* Check communication with device */
    char id[32];
    if (sscma_get_id(handle, id, sizeof(id), false) != SSCMA_OK) {
        return SSCMA_EIO;
    }

    return SSCMA_OK;
}

static void sscma_spi_reset_device(sscma_handle_t* handle) {
    /* Send reset command over SPI */
    uint8_t tx_buffer[8] = {0x10, 0x06, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF};

    sscma_spi_cs_select(handle, true);
    sscma_spi_write(handle, tx_buffer, sizeof(tx_buffer));
    sscma_spi_cs_select(handle, false);
}

int sscma_begin_spi(sscma_handle_t* handle, SPI_HandleTypeDef* hspi,
                   GPIO_TypeDef* cs_port, uint16_t cs_pin,
                   GPIO_TypeDef* sync_port, uint16_t sync_pin,
                   GPIO_TypeDef* rst_port, uint16_t rst_pin,
                   uint32_t baudrate, uint32_t wait_delay_ms) {
    if (!handle || !hspi) return SSCMA_EINVAL;

    handle->interface_type = SSCMA_IFACE_SPI;
    handle->hspi = hspi;
    handle->cs_port = cs_port;
    handle->cs_pin = cs_pin;
    handle->sync_port = sync_port;
    handle->sync_pin = sync_pin;
    handle->rst_port = rst_port;
    handle->rst_pin = rst_pin;
    handle->wait_delay_ms = (wait_delay_ms) ? wait_delay_ms : 2;

    /* Initialize CS pin */
    if (cs_port) {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = cs_pin;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(cs_port, &GPIO_InitStruct);
        sscma_gpio_write(cs_port, cs_pin, 1);  /* Deselect by default */
    }

    /* Initialize SYNC pin (input) if provided */
    if (sync_port) {
        GPIO_InitTypeDef GPIO_InitStruct = {0};
        GPIO_InitStruct.Pin = sync_pin;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(sync_port, &GPIO_InitStruct);
    }

    /* Initialize JSON parser context */
    handle->json_context = json_parser_init_context(2048);
    if (!handle->json_context) return SSCMA_ENOMEM;

    /* Reset device */
    sscma_hardware_reset(handle);
    sscma_delay_ms(100);
    sscma_spi_reset_device(handle);

    /* Check communication with device */
    char id[32];
    if (sscma_get_id(handle, id, sizeof(id), false) != SSCMA_OK) {
        return SSCMA_EIO;
    }

    return SSCMA_OK;
}

static void sscma_spi_cs_select(sscma_handle_t* handle, bool select) {
    if (handle->cs_port) {
        sscma_gpio_write(handle->cs_port, handle->cs_pin, select ? 0 : 1);
    }
}

static int sscma_send_command(sscma_handle_t* handle, const char* cmd, size_t cmd_len) {
    if (!handle || !cmd || cmd_len == 0) return SSCMA_EINVAL;

    /* Add small delay before sending command */
    if (handle->wait_delay_ms) {
        sscma_delay_ms(handle->wait_delay_ms);
    }

    /* Send command based on interface type */
    switch (handle->interface_type) {
        case SSCMA_IFACE_I2C:
            return sscma_i2c_write(handle, (const uint8_t*)cmd, cmd_len);
        case SSCMA_IFACE_UART:
            return sscma_uart_write(handle, (const uint8_t*)cmd, cmd_len);
        case SSCMA_IFACE_SPI:
            return sscma_spi_write(handle, (const uint8_t*)cmd, cmd_len);
        default:
            return SSCMA_EINVAL;
    }
}

static int sscma_wait_for_response(sscma_handle_t* handle, const char* expected_cmd,
                                   uint8_t expected_type, uint32_t timeout_ms) {
    if (!handle) return SSCMA_EINVAL;

    uint32_t start_tick = sscma_get_tick();
    uint32_t elapsed = 0;
    int result = SSCMA_ETIMEDOUT;

    /* Clear RX buffer index */
    handle->rx_index = 0;

    while (elapsed <= timeout_ms) {
        /* Check available data */
        int available = 0;
        switch (handle->interface_type) {
            case SSCMA_IFACE_I2C:
                available = sscma_i2c_available(handle);
                break;
            case SSCMA_IFACE_UART:
                available = sscma_uart_available(handle);
                break;
            case SSCMA_IFACE_SPI:
                available = sscma_spi_available(handle);
                break;
            default:
                return SSCMA_EINVAL;
        }

        if (available > 0) {
            /* Ensure we have enough space in RX buffer */
            uint32_t remaining_space = handle->rx_buffer_size - handle->rx_index;
            if (remaining_space < (uint32_t)available) {
                /* Shift buffer contents to make space */
                memmove(handle->rx_buffer,
                        handle->rx_buffer + (handle->rx_index - remaining_space + available),
                        remaining_space);
                handle->rx_index = remaining_space;
            }

            /* Read available data */
            int read_count = 0;
            switch (handle->interface_type) {
                case SSCMA_IFACE_I2C:
                    read_count = sscma_i2c_read(handle,
                                              handle->rx_buffer + handle->rx_index,
                                              MIN(available, remaining_space));
                    break;
                case SSCMA_IFACE_UART:
                    read_count = sscma_uart_read(handle,
                                               handle->rx_buffer + handle->rx_index,
                                               MIN(available, remaining_space));
                    break;
                case SSCMA_IFACE_SPI:
                    read_count = sscma_spi_read(handle,
                                              handle->rx_buffer + handle->rx_index,
                                              MIN(available, remaining_space));
                    break;
            }

            if (read_count > 0) {
                handle->rx_index += read_count;
                handle->rx_buffer[handle->rx_index] = '\0';  /* Null terminate */
            }
        }

        /* Process complete responses in buffer */
        char* suffix;
        while ((suffix = sscma_strnstr((char*)handle->rx_buffer, RESPONSE_SUFFIX, handle->rx_index))) {
            char* prefix = sscma_strnstr((char*)handle->rx_buffer, RESPONSE_PREFIX, suffix - (char*)handle->rx_buffer);

            if (prefix) {
                /* Found complete response */
                size_t payload_len = suffix - prefix + RESPONSE_SUFFIX_LEN;
                char* payload = (char*)malloc(payload_len + 1);
                if (!payload) {
                    return SSCMA_ENOMEM;
                }

                /* Copy payload and null-terminate */
                memcpy(payload, prefix, payload_len);
                payload[payload_len] = '\0';

                /* Parse JSON payload */
                json_object_t* json_obj = json_parse_object(handle->json_context, payload + 1, payload_len - 2);
                free(payload);

                if (json_obj) {
                    int code = 0;
                    char cmd_name[32] = {0};
                    int type = 0;

                    /* Extract basic response fields */
                    json_get_int(json_obj, "code", &code);
                    json_get_int(json_obj, "type", &type);
                    json_get_string(json_obj, "name", cmd_name, sizeof(cmd_name));

                    /* Check if this is the expected response */
                    if (expected_cmd && expected_type) {
                        if (type == expected_type && strcmp(cmd_name, expected_cmd) == 0) {
                            result = code;

                            /* Parse event data if needed */
                            if (type == CMD_TYPE_EVENT) {
                                sscma_parse_event(handle, json_obj);
                            }

                            /* Clean up and return */
                            json_free_object(json_obj);
                            return result;
                        }
                    }

                    /* Clean up JSON object */
                    json_free_object(json_obj);
                }

                /* Remove processed response from buffer */
                memmove(handle->rx_buffer,
                        suffix + RESPONSE_SUFFIX_LEN,
                        handle->rx_index - (suffix - (char*)handle->rx_buffer) - RESPONSE_SUFFIX_LEN);
                handle->rx_index -= (suffix - (char*)handle->rx_buffer) + RESPONSE_SUFFIX_LEN;
            } else {
                /* Discard incomplete or invalid data before prefix */
                memmove(handle->rx_buffer,
                        suffix + RESPONSE_SUFFIX_LEN,
                        handle->rx_index - (suffix - (char*)handle->rx_buffer) - RESPONSE_SUFFIX_LEN);
                handle->rx_index -= (suffix - (char*)handle->rx_buffer) + RESPONSE_SUFFIX_LEN;
            }
        }

        /* Check for timeout */
        elapsed = sscma_get_tick() - start_tick;
    }

    return result;
}

int sscma_invoke(sscma_handle_t* handle, uint8_t times, bool filter, bool show) {
    if (!handle) return SSCMA_EINVAL;

    /* Check if we have enough buffer space for image data */
    if (show && handle->rx_buffer_size < 16384) {
        return SSCMA_ENOTSUP;
    }

    /* Format command */
    char cmd[MAX_CMD_LENGTH];
    int cmd_len = snprintf(cmd, sizeof(cmd), CMD_PREFIX "%s=%d,%d,%d" CMD_SUFFIX,
                          CMD_AT_INVOKE, times, !filter, filter);

    if (cmd_len <= 0 || (size_t)cmd_len >= sizeof(cmd)) {
        return SSCMA_EINVAL;
    }

    /* Send command */
    if (sscma_send_command(handle, cmd, cmd_len) != cmd_len) {
        return SSCMA_EIO;
    }

    /* Wait for command response */
    int result = sscma_wait_for_response(handle, CMD_AT_INVOKE, CMD_TYPE_RESPONSE, DEFAULT_TIMEOUT_MS);
    if (result != SSCMA_OK) {
        return result;
    }

    /* Wait for event response */
    result = sscma_wait_for_response(handle, CMD_AT_INVOKE, CMD_TYPE_EVENT, DEFAULT_TIMEOUT_MS);
    return result;
}

static int sscma_parse_event(sscma_handle_t* handle, json_object_t* json_obj) {
    if (!handle || !json_obj) return SSCMA_EINVAL;

    json_value_t* data_val = json_object_get(json_obj, "data");
    if (!data_val || data_val->type != JSON_OBJECT) return SSCMA_EINVAL;

    json_object_t* data_obj = data_val->object_value;

    /* Parse performance data */
    json_value_t* perf_val = json_object_get(data_obj, "perf");
    if (perf_val && perf_val->type == JSON_ARRAY && perf_val->array_value->count >= 3) {
        handle->perf.prepocess = (uint16_t)perf_val->array_value->items[0]->number_value;
        handle->perf.inference = (uint16_t)perf_val->array_value->items[1]->number_value;
        handle->perf.postprocess = (uint16_t)perf_val->array_value->items[2]->number_value;
    }

    /* Parse bounding boxes */
    json_value_t* boxes_val = json_object_get(data_obj, "boxes");
    if (boxes_val && boxes_val->type == JSON_ARRAY) {
        /* Clear existing boxes */
        handle->boxes.count = 0;

        for (uint16_t i = 0; i < boxes_val->array_value->count; i++) {
            json_value_t* box_val = boxes_val->array_value->items[i];
            if (box_val->type == JSON_ARRAY && box_val->array_value->count >= 6) {
                sscma_box_t box;
                box.x = (uint16_t)box_val->array_value->items[0]->number_value;
                box.y = (uint16_t)box_val->array_value->items[1]->number_value;
                box.w = (uint16_t)box_val->array_value->items[2]->number_value;
                box.h = (uint16_t)box_val->array_value->items[3]->number_value;
                box.score = (uint8_t)box_val->array_value->items[4]->number_value;
                box.target = (uint8_t)box_val->array_value->items[5]->number_value;

                sscma_add_box(&handle->boxes, &box);
            }
        }
    }

    /* Parse image data if available */
    json_value_t* image_val = json_object_get(data_obj, "image");
    if (image_val && image_val->type == JSON_STRING && image_val->string_value) {
        /* Free existing image data */
        if (handle->last_image) {
            free(handle->last_image);
            handle->last_image = NULL;
            handle->last_image_len = 0;
        }

        /* Allocate new buffer and copy image data */
        handle->last_image_len = strlen(image_val->string_value);
        handle->last_image = (char*)malloc(handle->last_image_len + 1);
        if (handle->last_image) {
            strcpy(handle->last_image, image_val->string_value);
        }
    }

    return SSCMA_OK;
}

int sscma_get_id(sscma_handle_t* handle, char* id_buffer, size_t buffer_size, bool use_cache) {
    if (!handle || !id_buffer || buffer_size == 0) return SSCMA_EINVAL;

    /* Return cached value if available and requested */
    if (use_cache && handle->device_id[0] != '\0') {
        strncpy(id_buffer, handle->device_id, buffer_size);
        id_buffer[buffer_size - 1] = '\0';
        return SSCMA_OK;
    }

    /* Format command */
    char cmd[MAX_CMD_LENGTH];
    int cmd_len = snprintf(cmd, sizeof(cmd), CMD_PREFIX "%s" CMD_SUFFIX, CMD_AT_ID);
    if (cmd_len <= 0 || (size_t)cmd_len >= sizeof(cmd)) {
        return SSCMA_EINVAL;
    }

    /* Send command */
    if (sscma_send_command(handle, cmd, cmd_len) != cmd_len) {
        return SSCMA_EIO;
    }

    /* Wait for response */
    int result = sscma_wait_for_response(handle, CMD_AT_ID, CMD_TYPE_RESPONSE, DEFAULT_TIMEOUT_MS);
    if (result != SSCMA_OK) {
        return result;
    }

    /* Extract ID from response (simplified - actual implementation would parse JSON) */
    json_object_t* json_obj = (json_object_t*)handle->json_context; /* In real code, get from response */
    if (json_obj) {
        json_value_t* data_val = json_object_get(json_obj, "data");
        if (data_val && data_val->type == JSON_STRING && data_val->string_value) {
            strncpy(handle->device_id, data_val->string_value, sizeof(handle->device_id) - 1);
            handle->device_id[sizeof(handle->device_id) - 1] = '\0';

            strncpy(id_buffer, handle->device_id, buffer_size);
            id_buffer[buffer_size - 1] = '\0';
            return SSCMA_OK;
        }
    }

    return SSCMA_EUNKNOWN;
}

/* Other function implementations follow similar patterns */

/* Interface-specific implementations */

/* I2C implementation */
static int sscma_i2c_write(sscma_handle_t* handle, const uint8_t* data, size_t len) {
    if (!handle->hi2c) return SSCMA_EINVAL;

    /* In a real implementation, this would handle the SSCMA I2C protocol
       with header and payload splitting as needed */
    HAL_StatusTypeDef status = HAL_I2C_Master_Transmit(handle->hi2c,
                                                     handle->i2c_address << 1,
                                                     (uint8_t*)data, len, 100);
    return (status == HAL_OK) ? len : SSCMA_EIO;
}

static int sscma_i2c_read(sscma_handle_t* handle, uint8_t* data, size_t len) {
    if (!handle->hi2c) return SSCMA_EINVAL;

    /* In a real implementation, this would handle the SSCMA I2C protocol
       with proper command structure */
    HAL_StatusTypeDef status = HAL_I2C_Master_Receive(handle->hi2c,
                                                    handle->i2c_address << 1,
                                                    data, len, 100);
    return (status == HAL_OK) ? len : SSCMA_EIO;
}

static int sscma_i2c_available(sscma_handle_t* handle) {
    /* For SSCMA over I2C, we need to query available data */
    uint8_t cmd[6] = {0x10, 0x03, 0x00, 0x00, 0x00, 0x00}; /* FEATURE_TRANSPORT_CMD_AVAILABLE */
    uint8_t response[2] = {0};

    /* Send command to check available bytes */
    if (sscma_i2c_write(handle, cmd, sizeof(cmd)) != sizeof(cmd)) {
        return 0;
    }

    sscma_delay_ms(1);

    /* Read response (2 bytes indicating available data length) */
    if (sscma_i2c_read(handle, response, sizeof(response)) != sizeof(response)) {
        return 0;
    }

    return (response[0] << 8) | response[1];
}

/* UART implementation */
static int sscma_uart_write(sscma_handle_t* handle, const uint8_t* data, size_t len) {
    if (!handle->huart) return SSCMA_EINVAL;

    HAL_StatusTypeDef status = HAL_UART_Transmit(handle->huart, (uint8_t*)data, len, 100);
    return (status == HAL_OK) ? len : SSCMA_EIO;
}

static int sscma_uart_read(sscma_handle_t* handle, uint8_t* data, size_t len) {
    if (!handle->huart) return SSCMA_EINVAL;

    HAL_StatusTypeDef status = HAL_UART_Receive(handle->huart, data, len, 100);
    return (status == HAL_OK) ? len : SSCMA_EIO;
}

static int sscma_uart_available(sscma_handle_t* handle) {
    /* For UART, we can check the RX buffer level */
    if (!handle->huart) return 0;

    /* This is a simplified implementation - actual implementation
       would depend on UART buffering strategy */
    return __HAL_UART_GET_FLAG(handle->huart, UART_FLAG_RXNE) ? 1 : 0;
}

/* SPI implementation */
static int sscma_spi_write(sscma_handle_t* handle, const uint8_t* data, size_t len) {
    if (!handle->hspi) return SSCMA_EINVAL;

    sscma_spi_cs_select(handle, true);
    HAL_StatusTypeDef status = HAL_SPI_Transmit(handle->hspi, (uint8_t*)data, len, 100);
    sscma_spi_cs_select(handle, false);

    return (status == HAL_OK) ? len : SSCMA_EIO;
}

static int sscma_spi_read(sscma_handle_t* handle, uint8_t* data, size_t len) {
    if (!handle->hspi) return SSCMA_EINVAL;

    sscma_spi_cs_select(handle, true);
    HAL_StatusTypeDef status = HAL_SPI_Receive(handle->hspi, data, len, 100);
    sscma_spi_cs_select(handle, false);

    return (status == HAL_OK) ? len : SSCMA_EIO;
}

static int sscma_spi_available(sscma_handle_t* handle) {
    /* Check SYNC pin if available */
    if (handle->sync_port) {
        if (sscma_gpio_read(handle->sync_port, handle->sync_pin) == 0) {
            return 0; /* No data available */
        }
    }

    /* Query available data over SPI */
    uint8_t cmd[8] = {0x10, 0x03, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF}; /* FEATURE_TRANSPORT_CMD_AVAILABLE */
    uint8_t response[2] = {0};

    sscma_spi_cs_select(handle, true);
    HAL_SPI_Transmit(handle->hspi, cmd, sizeof(cmd), 100);
    HAL_SPI_Receive(handle->hspi, response, sizeof(response), 100);
    sscma_spi_cs_select(handle, false);

    return (response[0] << 8) | response[1];
}

/* Additional functions (save_jpeg, clear_actions, etc.) would follow similar patterns */

/* Simple JSON parser implementation - minimal version for STM32 */
/* This would normally be in a separate json_parser.c/.h file */
typedef struct {
    char* buffer;
    size_t buffer_size;
    void* internal_state; /* For more complex parsers */
} json_parser_context_t;

void* json_parser_init_context(size_t buffer_size) {
    json_parser_context_t* ctx = (json_parser_context_t*)malloc(sizeof(json_parser_context_t));
    if (!ctx) return NULL;

    ctx->buffer = (char*)malloc(buffer_size);
    if (!ctx->buffer) {
        free(ctx);
        return NULL;
    }

    ctx->buffer_size = buffer_size;
    ctx->internal_state = NULL;
    return ctx;
}

void json_parser_free_context(void* context) {
    if (!context) return;

    json_parser_context_t* ctx = (json_parser_context_t*)context;
    if (ctx->buffer) free(ctx->buffer);
    free(ctx);
}

json_object_t* json_parse_object(void* context, const char* json_str, size_t len) {
    /* Simplified stub - real implementation would parse JSON */
    if (!context || !json_str || len == 0) return NULL;

    /* In a real implementation, this would parse the JSON string
       and build an object tree */

    json_parser_context_t* ctx = (json_parser_context_t*)context;
    if (len + 1 > ctx->buffer_size) return NULL;

    strncpy(ctx->buffer, json_str, ctx->buffer_size - 1);
    ctx->buffer[ctx->buffer_size - 1] = '\0';

    /* Return dummy object for now */
    static json_object_t dummy_obj;
    return &dummy_obj;
}

void json_free_object(json_object_t* obj) {
    /* Free object resources */
}

json_value_t* json_object_get(json_object_t* obj, const char* key) {
    /* Get value by key - simplified stub */
    return NULL;
}

/* End of file */


