/**
 * @file mmc5603nj_example.c
 * @brief Example usage of the MMC5603NJ driver.
 *
 * This file shows how to:
 *  1. Implement the three HAL callbacks for your platform
 *  2. Initialize the sensor
 *  3. Run the self-test
 *  4. Take a single-shot measurement
 *  5. Calibrate bridge offset (SET/RESET method)
 *  6. Run continuous measurement mode
 *
 * Replace the HAL stub bodies with your platform's actual I2C and delay calls.
 */

#include "mmc5603nj.h"
#include <stdio.h>

/* ─── HAL Implementation (replace with your platform's drivers) ───────────── */

int mmc5603nj_hal_i2c_write(uint8_t addr, uint8_t reg,
                              const uint8_t *data, uint16_t len)
{
    /*
     * STM32 HAL example:
     *   return HAL_I2C_Mem_Write(&hi2c1, addr << 1, reg,
     *                            I2C_MEMADD_SIZE_8BIT,
     *                            (uint8_t *)data, len, HAL_MAX_DELAY);
     *
     * Arduino Wire example:
     *   Wire.beginTransmission(addr);
     *   Wire.write(reg);
     *   for (int i = 0; i < len; i++) Wire.write(data[i]);
     *   return Wire.endTransmission();
     */
    (void)addr; (void)reg; (void)data; (void)len;
    return 0; /* stub */
}

int mmc5603nj_hal_i2c_read(uint8_t addr, uint8_t reg,
                             uint8_t *data, uint16_t len)
{
    /*
     * STM32 HAL example:
     *   return HAL_I2C_Mem_Read(&hi2c1, addr << 1, reg,
     *                           I2C_MEMADD_SIZE_8BIT,
     *                           data, len, HAL_MAX_DELAY);
     *
     * Arduino Wire example:
     *   Wire.beginTransmission(addr);
     *   Wire.write(reg);
     *   Wire.endTransmission(false);
     *   Wire.requestFrom(addr, (uint8_t)len);
     *   for (int i = 0; i < len; i++) data[i] = Wire.read();
     *   return 0;
     */
    (void)addr; (void)reg; (void)data; (void)len;
    return 0; /* stub */
}

void mmc5603nj_hal_delay_ms(uint32_t ms)
{
    /*
     * STM32 HAL:   HAL_Delay(ms);
     * Arduino:     delay(ms);
     * FreeRTOS:    vTaskDelay(pdMS_TO_TICKS(ms));
     */
    (void)ms; /* stub */
}

/* ─── Example Main ────────────────────────────────────────────────────────── */

int main(void)
{
    mmc5603nj_t     mag;
    mmc5603nj_data_t field;
    float            temperature;
    bool             ready  = false;
    bool             passed = false;
    int              ret;

    /* ── 1. Initialize handle (20-bit mode, 6.6ms bandwidth) ────────────── */
    mmc5603nj_init_handle(&mag, MMC5603NJ_RES_20BIT, MMC5603NJ_BW_6_6MS);

    /* ── 2. Initialize sensor (reset + product ID check + bandwidth) ─────── */
    ret = mmc5603nj_init(&mag);
    if (ret != 0) {
        printf("MMC5603NJ init failed (ret=%d)\n", ret);
        return ret;
    }
    printf("MMC5603NJ initialized OK\n");

    /* ── 3. Self-test ────────────────────────────────────────────────────── */
    ret = mmc5603nj_selftest(&mag, &passed);
    if (ret != 0) {
        printf("Self-test communication error (ret=%d)\n", ret);
    } else {
        printf("Self-test: %s\n", passed ? "PASS" : "FAIL");
    }

    /* ── 4. Calibrate bridge offset (recommended before measurements) ────── */
    ret = mmc5603nj_calibrate_offset(&mag);
    if (ret != 0) {
        printf("Offset calibration failed (ret=%d)\n", ret);
        return ret;
    }
    printf("Offset calibrated — X:%lu Y:%lu Z:%lu (counts)\n",
           (unsigned long)mag.offset.x,
           (unsigned long)mag.offset.y,
           (unsigned long)mag.offset.z);

    /* ── 5. Single-shot measurement ─────────────────────────────────────── */
    ret = mmc5603nj_read_gauss(&mag, &field);
    if (ret == 0) {
        printf("Magnetic field — X: %.4f G  Y: %.4f G  Z: %.4f G\n",
               field.x, field.y, field.z);
    }

    /* ── 6. Temperature reading ──────────────────────────────────────────── */
    ret = mmc5603nj_read_temperature(&mag, &temperature);
    if (ret == 0) {
        printf("Temperature: %.1f °C\n", temperature);
    }

    /* ── 7. Continuous mode at 50 Hz, BW=2ms, auto SET/RESET ─────────────── */
    ret = mmc5603nj_start_continuous(&mag,
                                     50,                   /* ODR = 50 Hz   */
                                     MMC5603NJ_BW_2_0MS,   /* 2ms meas time */
                                     true,                 /* auto SET/RESET */
                                     false);               /* normal power   */
    if (ret != 0) {
        printf("Failed to start continuous mode\n");
        return ret;
    }
    printf("Continuous mode started at 50 Hz\n");

    /* Poll for 5 samples */
    for (int i = 0; i < 5; i++) {
        ready = false;
        while (!ready) {
            mmc5603nj_is_data_ready(&mag, &ready);
            mmc5603nj_hal_delay_ms(1);
        }

        ret = mmc5603nj_read_gauss(&mag, &field);
        if (ret == 0) {
            printf("Sample %d — X: %.4f G  Y: %.4f G  Z: %.4f G\n",
                   i + 1, field.x, field.y, field.z);
        }
    }

    /* ── 8. Stop continuous mode ─────────────────────────────────────────── */
    mmc5603nj_stop_continuous(&mag);
    printf("Continuous mode stopped\n");

    return 0;
}
