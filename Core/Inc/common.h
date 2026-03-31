#ifndef __COMMON_H__
#define __COMMON_H__

#include <stdint.h>

#define CTRL_SLVID                  (0x28)
#define DATA_SLVID                  (0x28)

#define ISP_CONTROL_ADDR            (0x51010000)
#define ISP_DATA_PORT_ADDR          (0x38000000)
#define ISP_ENABLE_REG_ADDR         (0xD8)
#define ISP_REG_DEFAULT_VAL         (0x0288208F)
#define ISP_STATUS_CLR_OFFEST       (0x30)
#define ISP_PPDONE_OFFSET           (0x50)
#define ISP_PPDONE_COUNTER_OFFSET   (0x40)

#define EPII_VERC_PPDONE            (0x01)
#define EPII_VERB_PPDONE_WORKAROUND (0x01)

#define PROGRESS_STEP               (2048)

typedef enum {
    REG_OFF           = 0x00,
    REG_ISP_WRITE_EN  = 0x01,
    REG_XIP_EN        = 0x02,
    REG_ISP_TEST_MODE = 0x04,
    REG_D8_ISP_EN     = 0x01,
    REG_D8_TEST_MODE  = 0x02,
    REG_D8_SPI_DO_EN  = 0x04,
} FlashSubMod_t;

#endif /* __COMMON_H__ */
