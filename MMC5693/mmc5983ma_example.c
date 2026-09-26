/**
 * @file mmc5983ma_example.c
 * @brief Example usage of the MMC5983MA driver.
 *
 * This file shows how to:
 *  1. Implement the three HAL callbacks for your platform
 *  2. Initialize the sensor
 *  3. Take a single-shot measurement
 *  4. Use offset calibration (SET/RESET method)
 *  5. Run continuous measurement mode
 *
 * Replace the HAL stub bodies with your platform's actual I2C and delay calls.
 */

#include "mmc5983ma.h"
#include <stdio.h>

/* ─── HAL Implementation (replace with your platform's drivers) ───────────── */

int mmc5983ma_hal_i2c_write(uint8_t addr, uint8_t reg,
                             const uint8_t *data, uint16_t len)
{
    /*
     * Example for STM32 HAL:
     *   return HAL_I2C_Mem_Write(&hi2c1, addr << 1, reg,
     *                            I2C_MEMADD_SIZE_8BIT,
     *                            (uint8_t *)data, len, HAL_MAX_DELAY);
     *
     * Example for Arduino Wire:
     *   Wire.beginTransmission(addr);
     *   Wire.write(reg);
     *   for (int i = 0; i < len; i++) Wire.write(data[i]);
     *   return Wire.endTransmission();
     */
    (void)addr; (void)reg; (void)data; (void)len;
    return 0; /* stub */
}

int mmc5983ma_hal_i2c_read(uint8_t addr, uint8_t reg,
                            uint8_t *data, uint16_t len)
{
    /*
     * Example for STM32 HAL:
     *   return HAL_I2C_Mem_Read(&hi2c1, addr << 1, reg,
     *                           I2C_MEMADD_SIZE_8BIT,
     *                           data, len, HAL_MAX_DELAY);
     *
     * Example for Arduino Wire:
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

void mmc5983ma_hal_delay_ms(uint32_t ms)
{
    /*
     * Example for STM32 HAL:  HAL_Delay(ms);
     * Example for Arduino:    delay(ms);
     * Example for FreeRTOS:   vTaskDelay(pdMS_TO_TICKS(ms));
     */
    (void)ms; /* stub */
}

/* ─── Example Main ────────────────────────────────────────────────────────── */

int main(void)
{
    mmc5983ma_t mag;
    mmc5983ma_data_t field;
    float temperature;
    int ret;

    /* ── 1. Initialize handle (18-bit mode) ─────────────────────────────── */
    mmc5983ma_init_handle(&mag, true);

    /* ── 2. Initialize sensor (reset + verify product ID + set bandwidth) ── */
    ret = mmc5983ma_init(&mag);
    if (ret != 0) {
        printf("MMC5983MA init failed (ret=%d)\n", ret);
        return ret;
    }
    printf("MMC5983MA initialized OK\n");

    /* ── 3. Optional: calibrate bridge offset (recommended for accuracy) ── */
    ret = mmc5983ma_calibrate_offset(&mag);
    if (ret != 0) {
        printf("Offset calibration failed (ret=%d)\n", ret);
        return ret;
    }
    printf("Offset calibrated — X:%lu Y:%lu Z:%lu (counts)\n",
           (unsigned long)mag.offset.x,
           (unsigned long)mag.offset.y,
           (unsigned long)mag.offset.z);

    /* ── 4. Single-shot measurement ─────────────────────────────────────── */
    ret = mmc5983ma_read_gauss(&mag, &field);
    if (ret == 0) {
        printf("Magnetic field — X: %.4f G  Y: %.4f G  Z: %.4f G\n",
               field.x, field.y, field.z);
    }

    /* ── 5. Temperature reading ──────────────────────────────────────────── */
    ret = mmc5983ma_read_temperature(&mag, &temperature);
    if (ret == 0) {
        printf("Temperature: %.1f °C\n", temperature);
    }

    /* ── 6. Continuous mode at 100 Hz with auto SET/RESET ────────────────── */
    ret = mmc5983ma_start_continuous(&mag,
                                     MMC5983MA_CM_100HZ,
                                     MMC5983MA_BW_200HZ,
                                     true);
    if (ret != 0) {
        printf("Failed to start continuous mode\n");
        return ret;
    }
    printf("Continuous mode started at 100 Hz\n");

    /* Poll for 5 samples */
    for (int i = 0; i < 5; i++) {
        bool ready = false;

        /* Wait until data is ready */
        while (!ready) {
            mmc5983ma_is_data_ready(&mag, &ready);
            mmc5983ma_hal_delay_ms(1);
        }

        ret = mmc5983ma_read_gauss(&mag, &field);
        if (ret == 0) {
            printf("Sample %d — X: %.4f G  Y: %.4f G  Z: %.4f G\n",
                   i + 1, field.x, field.y, field.z);
        }
    }

    /* ── 7. Stop continuous mode ─────────────────────────────────────────── */
    mmc5983ma_stop_continuous(&mag);
    printf("Continuous mode stopped\n");

    return 0;
}
