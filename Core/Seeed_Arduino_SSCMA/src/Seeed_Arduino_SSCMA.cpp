/**
 * SSCMA_STM32.c
 * Description: A driver for Seeed Grove AI Family adapted for STM32
 * Ported from Arduino version to STM32 HAL
 * Original Author: Hongtai Liu(lht856@foxmail.com)
 * STM32 Port: 2025
 */

#include "Seeed_Arduino_SSCMA.h"
#include <stdlib.h>


extern "C" {
    #include "main.h"
    extern UART_HandleTypeDef huart4;
}

// JSON parsing requires a library - You'll need to add one or parse manually
// For now, we'll add stubs that you can implement with your preferred JSON library
// Options: cJSON, jsmn, parson, etc.

// Private function prototypes
static int i2c_write(SSCMA_t *sscma, const char *data, int length);
static int i2c_read(SSCMA_t *sscma, char *data, int length);
static int i2c_available(SSCMA_t *sscma);
static void i2c_cmd(SSCMA_t *sscma, uint8_t feature, uint8_t cmd, uint16_t len, uint8_t *data);

static int wait_response(SSCMA_t *sscma, int type, const char *cmd, uint32_t timeout);
static void parse_event(SSCMA_t *sscma, const char *json);
static void reset_pin(GPIO_TypeDef *port, uint16_t pin);

// Utility function - custom strnstr implementation
char* strnstr_custom(const char *haystack, const char *needle, size_t n) {
    if (!needle || n == 0) {
        return NULL;
    }

    size_t needle_len = strlen(needle);
    if (needle_len == 0) {
        return (char *)haystack;
    }

    for (size_t i = 0; i < n && haystack[i] != '\0'; i++) {
        if (i + needle_len <= n && haystack[i] == needle[0]) {
            size_t j = 1;
            while (j < needle_len && haystack[i + j] == needle[j]) {
                j++;
            }
            if (j == needle_len) {
                return (char *)&haystack[i];
            }
        }
    }
    return NULL;
}

// Reset GPIO pin
static void reset_pin(GPIO_TypeDef *port, uint16_t pin) {
    if (port != NULL) {
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
        HAL_Delay(50);
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
        HAL_Delay(500);
    }
}

// I2C Initialization
bool SSCMA_Init_I2C(SSCMA_t *sscma, I2C_HandleTypeDef *hi2c,
                    GPIO_TypeDef *rst_port, uint16_t rst_pin,
                    uint16_t address, uint32_t wait_delay) {
    memset(sscma, 0, sizeof(SSCMA_t));
    
    sscma->interface = SSCMA_INTERFACE_I2C;
    sscma->hi2c = hi2c;
    sscma->rst_port = rst_port;
    sscma->rst_pin = rst_pin;
    sscma->i2c_address = address;
    sscma->wait_delay = wait_delay;
    


//    sscma->debug_uart = &huart4;


    SSCMA_SetRxBuffer(sscma, SSCMA_MAX_RX_SIZE);
    SSCMA_SetTxBuffer(sscma, SSCMA_MAX_TX_SIZE);
    
    reset_pin(rst_port, rst_pin);
    
    return (SSCMA_GetID(sscma, false) != NULL && SSCMA_GetName(sscma, false) != NULL);
}

// UART Initialization
bool SSCMA_Init_UART(SSCMA_t *sscma, UART_HandleTypeDef *huart,
                     GPIO_TypeDef *rst_port, uint16_t rst_pin,
                     uint32_t wait_delay) {
    memset(sscma, 0, sizeof(SSCMA_t));
    
    sscma->interface = SSCMA_INTERFACE_UART;
    sscma->huart = huart;
    sscma->rst_port = rst_port;
    sscma->rst_pin = rst_pin;
    sscma->wait_delay = wait_delay;
    
    SSCMA_SetRxBuffer(sscma, SSCMA_MAX_RX_SIZE);
    SSCMA_SetTxBuffer(sscma, SSCMA_MAX_TX_SIZE);
    
    reset_pin(rst_port, rst_pin);
    
    return (SSCMA_GetID(sscma, false) != NULL && SSCMA_GetName(sscma, false) != NULL);
}

// Write data
int SSCMA_Write(SSCMA_t *sscma, const char *data, int length) {
    switch (sscma->interface) {
        case SSCMA_INTERFACE_I2C:
            return i2c_write(sscma, data, length);
        case SSCMA_INTERFACE_UART:
            if (HAL_UART_Transmit(sscma->huart, (uint8_t*)data, length, 1000) == HAL_OK) {
                return length;
            }
            return 0;
        default:
            return 0;
    }
}

int SSCMA_SelectModel(SSCMA_t *sscma, int id)
{
    char cmd[32];
    snprintf(cmd, sizeof(cmd), "AT+MODEL=%d\r\n", id);

    SSCMA_Write(sscma, cmd, strlen(cmd));

    return wait_response(sscma, CMD_TYPE_RESPONSE, "MODEL", 3000);
}



// Read data
int SSCMA_Read(SSCMA_t *sscma, char *data, int length) {
    switch (sscma->interface) {
        case SSCMA_INTERFACE_I2C:
            return i2c_read(sscma, data, length);
        case SSCMA_INTERFACE_UART:
            if (HAL_UART_Receive(sscma->huart, (uint8_t*)data, length, 100) == HAL_OK) {
                return length;
            }
            return 0;
        default:
            return 0;
    }
}

int SSCMA_InvokeOnce(SSCMA_t *sscma)
{
    sscma->box_detected = 0;

    SSCMA_Write(sscma, "AT+INVOKE=1,0,1\r\n", 16);

    // MUST wait for final EVENT (type 1)
    return wait_response(sscma, CMD_TYPE_EVENT, "INVOKE", 3000);
}



// Check available data
int SSCMA_Available(SSCMA_t *sscma) {
    switch (sscma->interface) {
        case SSCMA_INTERFACE_I2C:
            return i2c_available(sscma);
        case SSCMA_INTERFACE_UART:
            // For UART, you'll need to implement a ring buffer or DMA
            // This is a simplified version
            return 0;
        default:
            return 0;
    }
}

// I2C command
static void i2c_cmd(SSCMA_t *sscma, uint8_t feature, uint8_t cmd, uint16_t len, uint8_t *data) {

//
//	HAL_UART_Transmit(sscma->debug_uart,
//	                  (uint8_t*)"Entered i2c_write\r\n",
//	                  20,
//	                  1000);


    uint8_t tx_data[PACKET_SIZE];
    uint16_t idx = 0;
    
    HAL_Delay(sscma->wait_delay);
    
    tx_data[idx++] = feature;
    tx_data[idx++] = cmd;
    tx_data[idx++] = len >> 8;
    tx_data[idx++] = len & 0xFF;
    
    if (data != NULL && len > 0) {
        memcpy(&tx_data[idx], data, len);
        idx += len;
    }
    
    tx_data[idx++] = 0; // Checksum placeholder
    tx_data[idx++] = 0;
    
    HAL_I2C_Master_Transmit(sscma->hi2c, sscma->i2c_address, tx_data, idx, 1000);

//    if (HAL_I2C_Master_Transmit(sscma->hi2c, sscma->i2c_address, tx_data, idx, 1000) == HAL_OK)
//    {
//        HAL_UART_Transmit(&huart4, (uint8_t*)"TX OK\r\n", 7, 1000);
//    }
//    else
//    {
//        HAL_UART_Transmit(&huart4, (uint8_t*)"TX FAIL\r\n", 9, 1000);
//    }

}

// I2C available
static int i2c_available(SSCMA_t *sscma) {
    uint8_t buf[2] = {0};
    uint8_t cmd[6] = {FEATURE_TRANSPORT, FEATURE_TRANSPORT_CMD_AVAILABLE, 0, 0, 0, 0};

//    HAL_UART_Transmit(sscma->debug_uart, (uint8_t*)"Checking available...\r\n", 24, 1000);

    if (HAL_I2C_Master_Transmit(sscma->hi2c, sscma->i2c_address, cmd, 6, 1000) != HAL_OK) {
//        HAL_UART_Transmit(sscma->debug_uart, (uint8_t*)"TX FAIL\r\n", 9, 1000);
        return 0;
    }

    HAL_Delay(sscma->wait_delay);

    if (HAL_I2C_Master_Receive(sscma->hi2c, sscma->i2c_address, buf, 2, 1000) != HAL_OK) {
//        HAL_UART_Transmit(sscma->debug_uart, (uint8_t*)"RX FAIL\r\n", 9, 1000);
        return 0;
    }

    char dbg[50];
    sprintf(dbg, "Available raw: %02X %02X\r\n", buf[0], buf[1]);
//    HAL_UART_Transmit(sscma->debug_uart, (uint8_t*)dbg, strlen(dbg), 1000);

    return (buf[0] << 8) | buf[1];
}


// I2C read
static int i2c_read(SSCMA_t *sscma, char *data, int length) {
    uint16_t packets = length / MAX_PL_LEN;
    uint8_t remain = length % MAX_PL_LEN;
    
    for (uint16_t i = 0; i < packets; i++) {
        i2c_cmd(sscma, FEATURE_TRANSPORT, FEATURE_TRANSPORT_CMD_READ, MAX_PL_LEN, NULL);
        HAL_Delay(sscma->wait_delay);
        HAL_I2C_Master_Receive(sscma->hi2c, sscma->i2c_address, 
                              (uint8_t*)(data + i * MAX_PL_LEN), MAX_PL_LEN, 1000);
    }
    
    if (remain) {
        i2c_cmd(sscma, FEATURE_TRANSPORT, FEATURE_TRANSPORT_CMD_READ, remain, NULL);
        HAL_Delay(sscma->wait_delay);
        HAL_I2C_Master_Receive(sscma->hi2c, sscma->i2c_address,
                              (uint8_t*)(data + packets * MAX_PL_LEN), remain, 1000);
    }
    
    return length;
}

// I2C write
static int i2c_write(SSCMA_t *sscma, const char *data, int length) {

//	HAL_UART_Transmit(sscma->debug_uart,
//	                  (uint8_t*)"Entered i2c_write\r\n",
//	                  20,
//	                  1000);

    uint16_t packets = length / MAX_PL_LEN;
    uint16_t remain = length % MAX_PL_LEN;
    
    for (uint16_t i = 0; i < packets; i++) {
        i2c_cmd(sscma, FEATURE_TRANSPORT, FEATURE_TRANSPORT_CMD_WRITE, 
                MAX_PL_LEN, (uint8_t*)(data + i * MAX_PL_LEN));
    }
    
    if (remain) {
        i2c_cmd(sscma, FEATURE_TRANSPORT, FEATURE_TRANSPORT_CMD_WRITE,
                remain, (uint8_t*)(data + packets * MAX_PL_LEN));
    }
    
    return length;
}

// Wait for response
static int wait_response(SSCMA_t *sscma, int expected_type, const char *expected_cmd, uint32_t timeout)
{
    uint32_t startTime = HAL_GetTick();
    char *payload = NULL;

    while (HAL_GetTick() - startTime <= timeout)
    {
        int len = SSCMA_Available(sscma);
        if (len == 0)
            continue;

        if (len + sscma->rx_end > sscma->rx_len)
        {
            len = sscma->rx_len - sscma->rx_end;
            if (len <= 0)
            {
                sscma->rx_end = 0;
                continue;
            }
        }

        sscma->rx_end += SSCMA_Read(sscma, sscma->rx_buf + sscma->rx_end, len);
        sscma->rx_buf[sscma->rx_end] = '\0';

        char *suffix = strnstr_custom(sscma->rx_buf, RESPONSE_SUFFIX, sscma->rx_end);

        while (suffix != NULL)
        {
            char *prefix = strnstr_custom(sscma->rx_buf, RESPONSE_PREFIX, suffix - sscma->rx_buf);

            if (prefix != NULL)
            {
                len = suffix - prefix + RESPONSE_SUFFIX_LEN;
                payload = (char *)malloc(len);

                if (!payload)
                    break;

                memcpy(payload, prefix + 1, len - 1);
                payload[len - 1] = '\0';
//                HAL_UART_Transmit(&huart4, (uint8_t*)payload, strlen(payload), 1000);
//                HAL_UART_Transmit(&huart4, (uint8_t*)"\r\n\r\n", 4, 1000);
                parse_event(sscma, payload);

                if (sscma->box_detected)
                {
                    HAL_UART_Transmit(&huart4,
                        (uint8_t*)payload,
                        strlen(payload),
                        1000);

                    HAL_UART_Transmit(&huart4,
                        (uint8_t*)"\r\n\r\n",
                        4,
                        1000);
                }


                memmove(sscma->rx_buf,
                        suffix + RESPONSE_SUFFIX_LEN,
                        sscma->rx_end - (suffix - sscma->rx_buf) - RESPONSE_SUFFIX_LEN);

                sscma->rx_end -= (suffix - sscma->rx_buf) + RESPONSE_SUFFIX_LEN;

                /* ---- Minimal manual parsing ---- */

                int msg_type = -1;
                char msg_name[32] = {0};

                char *type_ptr = strstr(payload, "\"type\":");
                if (type_ptr)
                    sscanf(type_ptr, "\"type\": %d", &msg_type);

                char *name_ptr = strstr(payload, "\"name\":");
                if (name_ptr)
                    sscanf(name_ptr, "\"name\": \"%31[^\"]\"", msg_name);

                /* If this is EVENT, parse it */
                if (msg_type == CMD_TYPE_EVENT)
                {
                    parse_event(sscma, payload);
                }

                /* Return only when type and name match expected */
                if (msg_type == expected_type &&
                    strcmp(msg_name, expected_cmd) == 0)
                {
                    free(payload);
                    return CMD_OK;
                }

                free(payload);
            }
            else
            {
                memmove(sscma->rx_buf,
                        suffix + RESPONSE_PREFIX_LEN,
                        sscma->rx_end - (suffix - sscma->rx_buf) - RESPONSE_PREFIX_LEN);

                sscma->rx_end -= (suffix - sscma->rx_buf) + RESPONSE_PREFIX_LEN;
                sscma->rx_buf[sscma->rx_end] = '\0';
            }

            suffix = strnstr_custom(sscma->rx_buf, RESPONSE_SUFFIX, sscma->rx_end);
        }
    }

    return CMD_ETIMEDOUT;
}


// Parse event
static void parse_event(SSCMA_t *sscma, const char *json)
{
    // Reset detection each event
    sscma->box_detected = 0;

    // Only care about INVOKE result events
    if (strstr(json, "\"name\": \"INVOKE\"") == NULL)
        return;

    if (strstr(json, "\"type\": 1") == NULL)
        return;

    // Find start of boxes array
    char *boxes_ptr = strstr(json, "\"boxes\":");
    if (!boxes_ptr)
        return;

    // If empty boxes
    if (strstr(boxes_ptr, "[]"))
        return;

    // Find first [[
    char *start = strstr(boxes_ptr, "[[");
    if (!start)
        return;

    int x, y, w, h, score, target;

    // Parse first box only
    if (sscanf(start, "[[%d,%d,%d,%d,%d,%d",
               &x, &y, &w, &h, &score, &target) == 6)
    {
        sscma->last_box.x = (uint16_t)x;
        sscma->last_box.y = (uint16_t)y;
        sscma->last_box.w = (uint16_t)w;
        sscma->last_box.h = (uint16_t)h;
        sscma->last_box.score = (uint8_t)score;
        sscma->last_box.target = (uint8_t)target;

        sscma->box_detected = 1;
    }
}



// Invoke
int SSCMA_Invoke(SSCMA_t *sscma, int times, bool filter, bool show) {
    char cmd[64];
    
    if (show && sscma->rx_len < 16 * 1024) {
        return CMD_ENOTSUP;
    }
    
    snprintf(cmd, sizeof(cmd), "AT+INVOKE=%d,%d,%d\r\n", times, !filter, filter);
    SSCMA_Write(sscma, cmd, strlen(cmd));
    
    if (wait_response(sscma, CMD_TYPE_RESPONSE, CMD_AT_INVOKE, 1000) == CMD_OK) {
        if (wait_response(sscma, CMD_TYPE_EVENT, CMD_AT_INVOKE, 5000) == CMD_OK) {
            return CMD_OK;
        }
    }
    
    return CMD_ETIMEDOUT;
}

int SSCMA_InvokeMulti(SSCMA_t *sscma)
{
	while (1)
	    {
	        SSCMA_Write(sscma, "AT+INVOKE=1,0,1\r\n", 16);

	        wait_response(sscma, CMD_TYPE_EVENT, "INVOKE", 3000);

	        HAL_Delay(50);   // small pacing delay
	    }
}


// Get ID
char* SSCMA_GetInfo(SSCMA_t *sscma, bool cache)
{
    if (cache && sscma->info[0])
        return sscma->info;

    char cmd[] = "AT+INFO?\r\n";
    SSCMA_Write(sscma, cmd, strlen(cmd));

    if (wait_response(sscma, CMD_TYPE_RESPONSE, "INFO?", 3000) == CMD_OK)
    {
        char *data_ptr = strstr(sscma->rx_buf, "\"info\":");
        if (data_ptr)
        {
            sscanf(data_ptr, "\"info\": \"%255[^\"]\"", sscma->info);
            return sscma->info;
        }
    }

    return NULL;
}


// Get Name
char* SSCMA_GetName(SSCMA_t *sscma, bool cache)
{
    if (cache && sscma->name[0])
        return sscma->name;

    char cmd[] = "AT+NAME?\r\n";
    SSCMA_Write(sscma, cmd, strlen(cmd));

    if (wait_response(sscma, CMD_TYPE_RESPONSE, "NAME?", 3000) == CMD_OK)
    {
        char *data_ptr = strstr(sscma->rx_buf, "\"data\":");
        if (data_ptr)
        {
            sscanf(data_ptr, "\"data\": \"%63[^\"]\"", sscma->name);
            return sscma->name;
        }
    }

    return NULL;
}


// Get Info
char* SSCMA_GetID(SSCMA_t *sscma, bool cache)
{
    if (cache && sscma->id[0])
        return sscma->id;

    char cmd[] = "AT+ID?\r\n";
    SSCMA_Write(sscma, cmd, strlen(cmd));

    if (wait_response(sscma, CMD_TYPE_RESPONSE, "ID?", 3000) == CMD_OK)
    {
        char *data_ptr = strstr(sscma->rx_buf, "\"data\":");
        if (data_ptr)
        {
            sscanf(data_ptr, "\"data\": \"%31[^\"]\"", sscma->id);
            return sscma->id;
        }
    }

    return NULL;
}


int SSCMA_GetModelInfo(SSCMA_t *sscma)
{
    char cmd[] = "AT+MODEL?\r\n";
    SSCMA_Write(sscma, cmd, strlen(cmd));

    return wait_response(sscma, CMD_TYPE_RESPONSE, "MODEL?", 3000);
}

int SSCMA_LoadModel(SSCMA_t *sscma)
{
    char cmd[] = "AT+MODEL\r\n";
    SSCMA_Write(sscma, cmd, strlen(cmd));
    return wait_response(sscma, CMD_TYPE_RESPONSE, "MODEL", 3000);
}


int SSCMA_QueryModel(SSCMA_t *sscma)
{
    char cmd[] = "AT+MODEL?\r\n";
    SSCMA_Write(sscma, cmd, strlen(cmd));
    return wait_response(sscma, CMD_TYPE_RESPONSE, "MODEL?", 3000);
}

int SSCMA_QueryStatus(SSCMA_t *sscma)
{
    SSCMA_Write(sscma, "AT+STAT?\r\n", 10);
    return wait_response(sscma, CMD_TYPE_RESPONSE, "STAT?", 3000);
}


// Clean actions
int SSCMA_CleanActions(SSCMA_t *sscma) {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+ACTION=\"\"\r\n");
    SSCMA_Write(sscma, cmd, strlen(cmd));
    
    if (wait_response(sscma, CMD_TYPE_RESPONSE, CMD_AT_ACTION, 1000) == CMD_OK) {
        return CMD_OK;
    }
    return CMD_ETIMEDOUT;
}

// Save JPEG
int SSCMA_SaveJPEG(SSCMA_t *sscma) {
    char cmd[64];
    snprintf(cmd, sizeof(cmd), "AT+ACTION=\"save_jpeg()\"\r\n");
    SSCMA_Write(sscma, cmd, strlen(cmd));
    
    if (wait_response(sscma, CMD_TYPE_RESPONSE, CMD_AT_ACTION, 1000) == CMD_OK) {
        return CMD_OK;
    }
    return CMD_ETIMEDOUT;
}

// Buffer management
bool SSCMA_SetRxBuffer(SSCMA_t *sscma, uint32_t size) {
    if (size == 0) return false;
    
    if (sscma->rx_len == 0) {
        sscma->rx_buf = (char *)malloc(size);
    } else {
        sscma->rx_buf = (char *)realloc(sscma->rx_buf, size);
    }
    
    if (sscma->rx_buf) {
        sscma->rx_end = 0;
        sscma->rx_len = size;
        return true;
    }
    return false;
}

bool SSCMA_SetTxBuffer(SSCMA_t *sscma, uint32_t size) {
    if (size == 0) return false;
    
    if (sscma->tx_len == 0) {
        sscma->tx_buf = (char *)malloc(size);
    } else {
        sscma->tx_buf = (char *)realloc(sscma->tx_buf, size);
    }
    
    if (sscma->tx_buf) {
        sscma->tx_len = size;
        return true;
    }
    return false;
}

// Fetch with callback
void SSCMA_Fetch(SSCMA_t *sscma, ResponseCallback callback) {
    int len = SSCMA_Available(sscma);
    if (len == 0) return;
    
    if (len + sscma->rx_end > sscma->rx_len) {
        len = sscma->rx_len - sscma->rx_end;
        if (len <= 0) {
            sscma->rx_end = 0;
            return;
        }
    }
    
    sscma->rx_end += SSCMA_Read(sscma, sscma->rx_buf + sscma->rx_end, len);
    sscma->rx_buf[sscma->rx_end] = '\0';
    
    char *suffix = strnstr_custom(sscma->rx_buf, RESPONSE_SUFFIX, sscma->rx_end);
    while (suffix != NULL) {
        char *prefix = strnstr_custom(sscma->rx_buf, RESPONSE_PREFIX, suffix - sscma->rx_buf);
        if (prefix != NULL) {
            len = suffix - prefix + RESPONSE_SUFFIX_LEN;
            char *payload = (char *)malloc(len + 1);
            
            if (payload) {
                memcpy(payload, prefix, len);
                memmove(sscma->rx_buf, suffix + RESPONSE_SUFFIX_LEN,
                       sscma->rx_end - (suffix - sscma->rx_buf) - RESPONSE_SUFFIX_LEN);
                sscma->rx_end -= suffix - sscma->rx_buf + RESPONSE_SUFFIX_LEN;
                payload[len] = '\0';
                
                if (callback) {
                    callback(payload, len);
                }
                
                free(payload);
            }
        } else {
            memmove(sscma->rx_buf, suffix + RESPONSE_PREFIX_LEN,
                   sscma->rx_end - (suffix - sscma->rx_buf) - RESPONSE_PREFIX_LEN);
            sscma->rx_end -= suffix - sscma->rx_buf + RESPONSE_PREFIX_LEN;
            sscma->rx_buf[sscma->rx_end] = '\0';
        }
        suffix = strnstr_custom(sscma->rx_buf, RESPONSE_SUFFIX, sscma->rx_end);
    }
}

// WiFi and MQTT functions - implement as needed following the pattern above


//Not implemented yet
int SSCMA_GetWiFi(SSCMA_t *sscma, wifi_t *wifi) {
    // Implementation similar to other query functions
    return CMD_ENOTSUP;
}

int SSCMA_GetMQTT(SSCMA_t *sscma, mqtt_t *mqtt) {
    return CMD_ENOTSUP;
}

int SSCMA_SetWiFiStatus(SSCMA_t *sscma, wifi_status_t *wifi_status) {
    return CMD_ENOTSUP;
}

int SSCMA_SetMQTTStatus(SSCMA_t *sscma, mqtt_status_t *mqtt_status) {
    return CMD_ENOTSUP;
}
