/*
 * ep2_isp_i2c.c
 *
 * WE2 I2C ISP bootloader recovery — STM32 HAL port.
 * Replaces Arduino Wire.h calls with HAL_I2C_Master_Transmit / HAL_I2C_Mem_Read.
 */
#include "ep2_isp_i2c.h"
#include "common.h"
#include <stdio.h>
#include <string.h>

#define I2C_TIMEOUT_MS  100

static I2C_HandleTypeDef  *g_hi2c;
static UART_HandleTypeDef *g_huart;

/* ------------------------------------------------------------------ */
/*  Helpers                                                           */
/* ------------------------------------------------------------------ */

static void dbg(const char *msg)
{
    HAL_UART_Transmit(g_huart, (uint8_t *)msg, strlen(msg), 500);
}

/* Simple I2C master write (START + addr+W + data + STOP) */
static HAL_StatusTypeDef i2c_write(uint8_t slave7, const uint8_t *data, uint16_t len)
{
    return HAL_I2C_Master_Transmit(g_hi2c, slave7 << 1,
                                   (uint8_t *)data, len, I2C_TIMEOUT_MS);
}

/*
 * Write one register-index byte, then repeated-start read one byte.
 * Equivalent to the original WriteEx(START) + ReadEx(REPEATED_START|STOP).
 */
static HAL_StatusTypeDef i2c_reg_read(uint8_t slave7, uint8_t reg, uint8_t *out)
{
    return HAL_I2C_Mem_Read(g_hi2c, slave7 << 1,
                            reg, I2C_MEMADD_SIZE_8BIT,
                            out, 1, I2C_TIMEOUT_MS);
}

/* ------------------------------------------------------------------ */
/*  Low-level ISP primitives (same protocol as original)              */
/* ------------------------------------------------------------------ */

/* Concatenate addr+data and write in one transaction */
static void i2c_burst_write(uint8_t id,
                             uint8_t *addr_buf, uint32_t addr_size,
                             uint8_t *data_buf, uint32_t data_size)
{
    uint8_t pkt[8];   /* max we ever send here is 2 bytes */
    memcpy(pkt, addr_buf, addr_size);
    memcpy(pkt + addr_size, data_buf, data_size);
    if (i2c_write(id, pkt, addr_size + data_size) != HAL_OK)
        dbg("i2c_burst_write: fail\r\n");
}

/*
 * Write a 4-byte address and 4-byte data word via the ISP register
 * interface, then poll for ACK.
 */
static void i2c_single_write(uint8_t id, uint8_t *addr, uint8_t *data)
{
    uint8_t pkt[2];

    /* Address bytes → registers 0x00..0x03 */
    for (int i = 0; i < 4; i++) {
        pkt[0] = (uint8_t)i;
        pkt[1] = addr[i];
        i2c_write(id, pkt, 2);
    }

    /* Data bytes → registers 0x04..0x07 */
    for (int i = 0; i < 4; i++) {
        pkt[0] = 0x04 + i;
        pkt[1] = data[i];
        i2c_write(id, pkt, 2);
    }

    /* ACK command: write */
    pkt[0] = 0x0C;
    pkt[1] = 0x01;
    i2c_write(id, pkt, 2);

    /* Poll status register 0x0F until 0x00 or timeout */
    uint8_t status = 0xFF;
    for (int t = 0; t < 1000; t++) {
        if (i2c_reg_read(id, 0x0F, &status) == HAL_OK && status == 0x00)
            break;
    }
}

/*
 * Read a 4-byte word from the ISP register interface at the given address.
 */
static uint32_t i2c_single_read(uint8_t id, uint8_t *addr)
{
    uint8_t pkt[2];

    /* Address bytes → registers 0x00..0x03 */
    for (int i = 0; i < 4; i++) {
        pkt[0] = (uint8_t)i;
        pkt[1] = addr[i];
        i2c_write(id, pkt, 2);
    }

    /* ACK command: read */
    pkt[0] = 0x0C;
    pkt[1] = 0x00;
    i2c_write(id, pkt, 2);

    /* Poll status register 0x0F until 0x00 or timeout */
    uint8_t no_timeout = 0;
    uint8_t status = 0xFF;
    for (int t = 0; t < 1000; t++) {
        if (i2c_reg_read(id, 0x0F, &status) == HAL_OK && status == 0x00) {
            no_timeout = 1;
            break;
        }
    }

    uint8_t d[4] = {0xFF, 0xFF, 0xFF, 0xFF};
    if (no_timeout) {
        /* Read data from registers 0x08..0x0B */
        for (int i = 0; i < 4; i++)
            i2c_reg_read(id, 0x08 + i, &d[i]);
    }

    return ((uint32_t)d[3] << 24) | ((uint32_t)d[2] << 16) |
           ((uint32_t)d[1] << 8)  | (uint32_t)d[0];
}

/* ------------------------------------------------------------------ */
/*  Public API                                                        */
/* ------------------------------------------------------------------ */

void ep2_isp_init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart)
{
    g_hi2c  = hi2c;
    g_huart = huart;
}

bool ep2_probe_device(void)
{
    for (uint8_t a = 1; a < 127; a++) {
        if (HAL_I2C_IsDeviceReady(g_hi2c, a << 1, 1, 5) == HAL_OK) {
            if (a == CTRL_SLVID)
                return true;
        }
    }
    return false;
}

void ep2_isp_begin(void)
{
    uint32_t reg_addr_scu = ISP_ENABLE_REG_ADDR;
    uint32_t val_scu      = REG_D8_TEST_MODE + REG_D8_ISP_EN;

    /* Enable ISP mode: S|50|W|ACK|D8|ACK|03|ACK|P */
    i2c_burst_write(CTRL_SLVID, (uint8_t *)&reg_addr_scu, 1,
                                (uint8_t *)&val_scu, 1);

    /* Write ISP control register default value */
    uint32_t ctrl_addr = ISP_CONTROL_ADDR;
    uint32_t ctrl_val  = ISP_REG_DEFAULT_VAL;
    i2c_single_write(CTRL_SLVID, (uint8_t *)&ctrl_addr, (uint8_t *)&ctrl_val);
}

void ep2_isp_end(void)
{
    /* Clear CRC info */
    uint32_t tmp_addr = ISP_CONTROL_ADDR + ISP_STATUS_CLR_OFFEST;
    i2c_single_read(CTRL_SLVID, (uint8_t *)&tmp_addr);

    /* Disable ISP mode */
    uint32_t reg_addr_scu = ISP_ENABLE_REG_ADDR;
    uint32_t val_off      = REG_OFF;
    i2c_burst_write(CTRL_SLVID, (uint8_t *)&reg_addr_scu, 1,
                                (uint8_t *)&val_off, 1);
}

int ep2_isp_write_chunk(const uint8_t *chunk, uint32_t addr, uint32_t chunk_idx)
{
    /* Build the 261-byte I2C packet: [0xFA, addr_LE(4), data(256)] */
    uint8_t buf[261];
    buf[0] = 0xFA;
    buf[1] = (uint8_t)(addr & 0xFF);
    buf[2] = (uint8_t)((addr >> 8) & 0xFF);
    buf[3] = (uint8_t)((addr >> 16) & 0xFF);
    buf[4] = (uint8_t)((addr >> 24) & 0xFF);
    memcpy(&buf[5], chunk, 256);

    /* Write chunk to WE2 */
    if (i2c_write(DATA_SLVID, buf, 261) != HAL_OK)
        return -1;

    /* ---- Check PP_DONE ---- */
#if (EPII_VERC_PPDONE == 0x01)
    {
        uint32_t pp_stat;
        uint32_t tmp_addr = ISP_CONTROL_ADDR + ISP_PPDONE_COUNTER_OFFSET;
        int timeout = 0;

        do {
            pp_stat = i2c_single_read(CTRL_SLVID, (uint8_t *)&tmp_addr);
            if (++timeout > 1000)
                return -2;  /* PP_DONE timeout */
        } while (!((pp_stat >> 28) == 1 || (pp_stat & 0xFFFFF) == chunk_idx));
    }
#else
    {
        uint32_t pp_stat;
        uint32_t tmp_addr = ISP_CONTROL_ADDR + ISP_PPDONE_OFFSET;
        do {
            pp_stat = i2c_single_read(CTRL_SLVID, (uint8_t *)&tmp_addr);
    #if (EPII_VERB_PPDONE_WORKAROUND == 0x01)
        } while (!((pp_stat >> 28) == 1 || (pp_stat >> 28) == 2 || (pp_stat >> 28) == 3 ||
                   (pp_stat & 0xFFFFFF) == (addr & 0xFFFFFF) - 4));
    #else
        } while (!((pp_stat >> 28) == 1 || (pp_stat >> 28) == 2 || (pp_stat >> 28) == 3));
    #endif
    }
#endif

    /* ---- CRC check ---- */
    {
        uint32_t tmp_addr = ISP_CONTROL_ADDR + ISP_PPDONE_COUNTER_OFFSET;
        uint32_t pp_stat  = i2c_single_read(CTRL_SLVID, (uint8_t *)&tmp_addr);

        if ((pp_stat >> 28) == 1 || (pp_stat >> 28) == 3) {
            uint32_t crc_w_addr = ISP_CONTROL_ADDR + 0x04;
            uint32_t crc_r_addr = ISP_CONTROL_ADDR + 0x08;
            uint32_t crc_wout = i2c_single_read(CTRL_SLVID, (uint8_t *)&crc_w_addr);
            uint32_t crc_rout = i2c_single_read(CTRL_SLVID, (uint8_t *)&crc_r_addr);

            /* Clear CRC info */
            uint32_t clr_addr = ISP_CONTROL_ADDR + ISP_STATUS_CLR_OFFEST;
            i2c_single_read(CTRL_SLVID, (uint8_t *)&clr_addr);

            if (crc_wout != crc_rout)
                return -3;  /* CRC mismatch */
        }
    }

    return 0;
}
