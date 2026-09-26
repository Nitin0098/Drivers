/**
 * @file    ism330dhcx_example.c
 * @brief   Example usage of the ISM330DHCX driver
 *
 * This file shows three platform stubs and a complete usage demo.
 * Pick the section that matches your hardware and delete the rest.
 *
 * Sections:
 *   1. STM32 HAL (I2C)
 *   2. Linux i2c-dev (user-space I2C)
 *   3. Generic / Arduino-style SPI (template)
 *   4. usage_demo() — works with any of the above
 */

#include "ism330dhcx.h"
#include <stdio.h>
#include <stdint.h>

/* =========================================================================
 * 1.  STM32 HAL — I2C
 * ====================================================================== */
#ifdef PLATFORM_STM32_I2C

#include "stm32XXxx_hal.h"   /* adjust for your MCU series */

/* The HAL handle pointer becomes the "handle" in ism330dhcx_dev_t */

static int stm32_i2c_read(void *handle, uint8_t reg, uint8_t *buf, size_t len)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)handle;
    uint8_t addr = ISM330DHCX_I2C_ADDR_HIGH << 1;   /* adjust for SA0 pin */
    HAL_StatusTypeDef s;

    s = HAL_I2C_Mem_Read(hi2c, addr, reg, I2C_MEMADD_SIZE_8BIT,
                          buf, (uint16_t)len, HAL_MAX_DELAY);
    return (s == HAL_OK) ? 0 : -1;
}

static int stm32_i2c_write(void *handle, uint8_t reg,
                             const uint8_t *buf, size_t len)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)handle;
    uint8_t addr = ISM330DHCX_I2C_ADDR_HIGH << 1;
    /* We need a local buffer because HAL_I2C_Mem_Write expects non-const */
    uint8_t tmp[32];
    if (len > sizeof(tmp)) return -1;
    for (size_t i = 0; i < len; i++) tmp[i] = buf[i];
    HAL_StatusTypeDef s;
    s = HAL_I2C_Mem_Write(hi2c, addr, reg, I2C_MEMADD_SIZE_8BIT,
                           tmp, (uint16_t)len, HAL_MAX_DELAY);
    return (s == HAL_OK) ? 0 : -1;
}

/* Usage:
 *   extern I2C_HandleTypeDef hi2c1;
 *   ism330dhcx_dev_t imu;
 *   ism330dhcx_init(&imu, stm32_i2c_read, stm32_i2c_write, &hi2c1);
 */

#endif /* PLATFORM_STM32_I2C */


/* =========================================================================
 * 2.  Linux user-space — i2c-dev
 * ====================================================================== */
#ifdef PLATFORM_LINUX_I2CDEV

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdlib.h>
#include <string.h>

typedef struct { int fd; uint8_t addr; } linux_i2c_ctx_t;

static int linux_i2c_read(void *handle, uint8_t reg, uint8_t *buf, size_t len)
{
    linux_i2c_ctx_t *ctx = (linux_i2c_ctx_t *)handle;
    if (ioctl(ctx->fd, I2C_SLAVE, ctx->addr) < 0) return -1;
    if (write(ctx->fd, &reg, 1) != 1)              return -1;
    if ((size_t)read(ctx->fd, buf, len) != len)    return -1;
    return 0;
}

static int linux_i2c_write(void *handle, uint8_t reg,
                             const uint8_t *buf, size_t len)
{
    linux_i2c_ctx_t *ctx = (linux_i2c_ctx_t *)handle;
    uint8_t tmp[64];
    if (len + 1 > sizeof(tmp)) return -1;
    tmp[0] = reg;
    memcpy(tmp + 1, buf, len);
    if (ioctl(ctx->fd, I2C_SLAVE, ctx->addr) < 0)       return -1;
    if ((size_t)write(ctx->fd, tmp, len + 1) != len + 1) return -1;
    return 0;
}

/* Usage:
 *   linux_i2c_ctx_t ctx = { open("/dev/i2c-1", O_RDWR), ISM330DHCX_I2C_ADDR_HIGH };
 *   ism330dhcx_dev_t imu;
 *   ism330dhcx_init(&imu, linux_i2c_read, linux_i2c_write, &ctx);
 */

#endif /* PLATFORM_LINUX_I2CDEV */


/* =========================================================================
 * 3.  Generic SPI template
 *     Adapt to your MCU's SPI API — the only requirement is to assert CS,
 *     send reg (with read/write bit set per your wiring), then rx/tx data.
 * ====================================================================== */
#ifdef PLATFORM_SPI_TEMPLATE

/*
 * ISM330DHCX SPI protocol:
 *   READ  : send (reg | 0x80), then receive N bytes
 *   WRITE : send reg (bit7=0), then send N bytes
 */

typedef struct {
    void (*cs_assert)(void);
    void (*cs_deassert)(void);
    uint8_t (*spi_transfer)(uint8_t byte);  /* full-duplex byte exchange */
} spi_ctx_t;

static int spi_read(void *handle, uint8_t reg, uint8_t *buf, size_t len)
{
    spi_ctx_t *ctx = (spi_ctx_t *)handle;
    ctx->cs_assert();
    ctx->spi_transfer(reg | 0x80);
    for (size_t i = 0; i < len; i++) buf[i] = ctx->spi_transfer(0xFF);
    ctx->cs_deassert();
    return 0;
}

static int spi_write(void *handle, uint8_t reg,
                      const uint8_t *buf, size_t len)
{
    spi_ctx_t *ctx = (spi_ctx_t *)handle;
    ctx->cs_assert();
    ctx->spi_transfer(reg & 0x7F);
    for (size_t i = 0; i < len; i++) ctx->spi_transfer(buf[i]);
    ctx->cs_deassert();
    return 0;
}

#endif /* PLATFORM_SPI_TEMPLATE */


/* =========================================================================
 * 4.  Usage demo
 *     Call usage_demo() from main() after wiring up a dev handle.
 * ====================================================================== */

void usage_demo(void)
{
    /*
     * ---- Setup ----------------------------------------------------------
     * Replace these stubs with your actual callbacks and handle.
     */
    extern int my_read (void*, uint8_t, uint8_t*, size_t);
    extern int my_write(void*, uint8_t, const uint8_t*, size_t);
    void *my_handle = NULL;   /* e.g. &hi2c1 or &spi_ctx */

    ism330dhcx_dev_t imu;
    ism330dhcx_err_t err;

    /* 1. Initialise — verifies WHO_AM_I = 0x6B */
    err = ism330dhcx_init(&imu, my_read, my_write, my_handle);
    if (err != ISM330DHCX_OK) {
        printf("ISM330DHCX not found (err=%d)\n", err);
        return;
    }
    printf("ISM330DHCX found.\n");

    /* 2. (Optional) Software reset to defaults */
    ism330dhcx_reset(&imu);

    /* 3. Block Data Update — prevents reading stale half-updated values */
    ism330dhcx_enable_bdu(&imu);

    /* 4. Configure accel: 104 Hz, ±4 g */
    ism330dhcx_config_accel(&imu, ISM330DHCX_XL_ODR_104HZ, ISM330DHCX_XL_FS_4G);

    /* 5. Configure gyro: 104 Hz, ±2000 dps */
    ism330dhcx_config_gyro(&imu, ISM330DHCX_GY_ODR_104HZ, ISM330DHCX_GY_FS_2000DPS);

    /* 6. Route data-ready interrupts to INT1 */
    ism330dhcx_set_int1(&imu, ISM330DHCX_INT_DRDY_XL | ISM330DHCX_INT_DRDY_G);


    /* ---- Polling loop ---------------------------------------------------
     * In a real project you would wait for the DRDY interrupt instead.    */
    for (int i = 0; i < 10; i++) {

        /* 7a. Wait for data ready */
        uint8_t status = 0;
        while (!(status & (ISM330DHCX_STATUS_XLDA | ISM330DHCX_STATUS_GDA))) {
            ism330dhcx_get_status(&imu, &status);
        }

        /* 7b. Burst-read accel + gyro in one I2C/SPI transaction */
        ism330dhcx_accel_t accel;
        ism330dhcx_gyro_t  gyro;
        ism330dhcx_read_all(&imu, &accel, &gyro);

        printf("A: x=%6.1f y=%6.1f z=%6.1f mg   "
               "G: x=%8.1f y=%8.1f z=%8.1f mdps\n",
               accel.x, accel.y, accel.z,
               gyro.x,  gyro.y,  gyro.z);

        /* 7c. Temperature */
        float temp;
        ism330dhcx_read_temp(&imu, &temp);
        printf("   Temp: %.2f °C\n", temp);
    }


    /* ---- FIFO example ---------------------------------------------------
     * Continuous mode, both sensors batched at 104 Hz, watermark=32 */
    printf("\n-- FIFO demo --\n");
    ism330dhcx_fifo_config(&imu,
                            ISM330DHCX_FIFO_CONTINUOUS,
                            0x04,   /* BDR_XL = 0x04 → 104 Hz */
                            0x04,   /* BDR_GY = 0x04 → 104 Hz */
                            32);

    /* Wait for watermark — in a real system use the INT1_FIFO_TH interrupt */
    uint16_t fifo_count = 0;
    while (fifo_count < 32) {
        ism330dhcx_fifo_get_count(&imu, &fifo_count);
    }

    printf("FIFO has %u samples. Reading:\n", fifo_count);
    for (uint16_t n = 0; n < fifo_count; n++) {
        ism330dhcx_fifo_sample_t sample;
        ism330dhcx_fifo_read_sample(&imu, &sample);

        if (sample.tag == ISM330DHCX_FIFO_TAG_ACCEL_NC) {
            float s = ism330dhcx_xl_sensitivity(imu.xl_fs);
            printf("  ACCEL  x=%7.1f y=%7.1f z=%7.1f mg\n",
                   sample.x * s, sample.y * s, sample.z * s);
        } else if (sample.tag == ISM330DHCX_FIFO_TAG_GYRO_NC) {
            float s = ism330dhcx_gy_sensitivity(imu.gy_fs);
            printf("  GYRO   x=%9.1f y=%9.1f z=%9.1f mdps\n",
                   sample.x * s, sample.y * s, sample.z * s);
        }
        /* Other tags: ISM330DHCX_FIFO_TAG_TEMP, ISM330DHCX_FIFO_TAG_TIMESTAMP */
    }

    /* Disable FIFO (Bypass) */
    ism330dhcx_fifo_config(&imu, ISM330DHCX_FIFO_BYPASS, 0, 0, 0);


    /* ---- Self-test ------------------------------------------------------*/
    printf("\n-- Self-test --\n");
    ism330dhcx_accel_t a_delta;
    ism330dhcx_gyro_t  g_delta;
    err = ism330dhcx_self_test(&imu, &a_delta, &g_delta);
    printf("Accel delta (mg):  x=%.1f y=%.1f z=%.1f  — %s\n",
           a_delta.x, a_delta.y, a_delta.z,
           (err == ISM330DHCX_OK) ? "PASS" : "FAIL");
    printf("Gyro  delta (mdps): x=%.0f y=%.0f z=%.0f — %s\n",
           g_delta.x, g_delta.y, g_delta.z,
           (err == ISM330DHCX_OK) ? "PASS" : "FAIL");

    /* ---- Raw register access (e.g. for Machine Learning Core) ----------*/
    uint8_t ctrl1;
    ism330dhcx_reg_read(&imu, ISM330DHCX_REG_CTRL1_XL, &ctrl1);
    printf("\nCTRL1_XL = 0x%02X\n", ctrl1);

    /* Set a specific bit without disturbing others */
    ism330dhcx_reg_update(&imu,
                           ISM330DHCX_REG_CTRL1_XL,
                           0x02,    /* mask: LPF2_XL_EN bit */
                           0x02);   /* value: enable LPF2   */
}
