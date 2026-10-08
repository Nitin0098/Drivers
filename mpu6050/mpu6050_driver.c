#include "mpu6050_driver.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* Sensitivity tables (datasheet sections 4.18 / 4.20), indexed by FS_SEL / AFS_SEL */
static const float k_accel_lsb_per_g[4]   = { 16384.0f, 8192.0f, 4096.0f, 2048.0f };
static const float k_gyro_lsb_per_dps[4]  = { 131.0f,   65.5f,   32.8f,   16.4f   };

void mpu6050_init_handle(mpu6050_t *dev, mpu6050_accel_fs_t afs,
                         mpu6050_gyro_fs_t gfs, mpu6050_dlpf_t dlpf)
{
    memset(dev, 0, sizeof(*dev));
    dev->accel_fs = afs;
    dev->gyro_fs  = gfs;
    dev->dlpf     = dlpf;
    dev->clk      = MPU6050_CLK_PLL_XGYRO;   /* datasheet: gyro PLL recommended over 8 MHz osc */
}

int mpu6050_reset(mpu6050_t *dev)
{
    uint8_t v = MPU6050_PWR1_DEVICE_RESET;
    if (mpu6050_hal_i2c_write(MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1, &v, 1) != 0) return -1;

    /* Datasheet gives no reset time; 100 ms is the commonly used safe value.
     * After reset the part is in SLEEP (PWR_MGMT_1 = 0x40), so mpu6050_init()
     * must be called afterwards to wake it. */
    HAL_Delay(100);

    dev->pwr1_shadow   = 0x40;   /* reset value */
    dev->int_en_shadow = 0x00;
    dev->smplrt_div    = 0x00;
    return 0;
}

int mpu6050_init(mpu6050_t *dev)
{
    uint8_t id = 0x00;
    if (mpu6050_hal_i2c_read(MPU6050_I2C_ADDR, MPU6050_REG_WHO_AM_I, &id, 1) != 0) {
        return -1;
    }
    if (id != MPU6050_WHO_AM_I_VAL) {
        return -2;
    }

    /* Wake up (clear SLEEP) and select clock source in a single write */
    uint8_t pwr1 = (uint8_t)((uint8_t)dev->clk & MPU6050_PWR1_CLKSEL_MASK);
    if (mpu6050_hal_i2c_write(MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1, &pwr1, 1) != 0) return -3;
    dev->pwr1_shadow = pwr1;
    HAL_Delay(50);                                   /* let the gyro PLL settle */

    /* All accel + gyro axes active, no low-power wake cycling */
    uint8_t pwr2 = 0x00;
    if (mpu6050_hal_i2c_write(MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_2, &pwr2, 1) != 0) return -4;

    if (mpu6050_configure(dev) != 0) return -5;

    /* Default to 1 kHz output rate with DLPF on (gyro rate 1 kHz, div = 0),
     * or 8 kHz if DLPF is off. Change with mpu6050_set_sample_rate(). */
    uint8_t div = 0x00;
    if (mpu6050_hal_i2c_write(MPU6050_I2C_ADDR, MPU6050_REG_SMPLRT_DIV, &div, 1) != 0) return -6;
    dev->smplrt_div = div;

    return 0;
}

int mpu6050_configure(mpu6050_t *dev)
{
    /* CONFIG: EXT_SYNC_SET = 0 (FSYNC disabled), DLPF_CFG in [2:0] */
    uint8_t cfg = (uint8_t)(dev->dlpf & 0x07);
    if (cfg == 7) cfg = 6;                           /* 7 is RESERVED */
    if (mpu6050_hal_i2c_write(MPU6050_I2C_ADDR, MPU6050_REG_CONFIG, &cfg, 1) != 0) return -1;

    /* GYRO_CONFIG: self-test bits cleared, FS_SEL in [4:3] */
    uint8_t gcfg = (uint8_t)(((uint8_t)dev->gyro_fs << MPU6050_FS_SHIFT) & MPU6050_FS_MASK);
    if (mpu6050_hal_i2c_write(MPU6050_I2C_ADDR, MPU6050_REG_GYRO_CONFIG, &gcfg, 1) != 0) return -2;

    /* ACCEL_CONFIG: self-test bits cleared, AFS_SEL in [4:3] */
    uint8_t acfg = (uint8_t)(((uint8_t)dev->accel_fs << MPU6050_FS_SHIFT) & MPU6050_FS_MASK);
    if (mpu6050_hal_i2c_write(MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_CONFIG, &acfg, 1) != 0) return -3;

    return 0;
}

int mpu6050_set_sample_rate(mpu6050_t *dev, uint16_t hz)
{
    /* Sample Rate = GyroOutputRate / (1 + SMPLRT_DIV)
     * GyroOutputRate = 8 kHz when DLPF_CFG = 0, otherwise 1 kHz.
     * Note: accel output rate is always 1 kHz, so >1 kHz repeats accel samples. */
    uint32_t base = (dev->dlpf == MPU6050_DLPF_260HZ) ? 8000u : 1000u;

    if (hz == 0 || hz > base) return -1;

    uint32_t div = (base / hz);
    if (div == 0) div = 1;
    div -= 1;
    if (div > 255) div = 255;                        /* clamp: lowest rate = base/256 */

    uint8_t v = (uint8_t)div;
    if (mpu6050_hal_i2c_write(MPU6050_I2C_ADDR, MPU6050_REG_SMPLRT_DIV, &v, 1) != 0) return -2;
    dev->smplrt_div = v;
    return 0;
}

int mpu6050_enable_data_ready_int(mpu6050_t *dev, bool active_low, bool open_drain, bool latch)
{
    uint8_t pin_cfg = 0;
    if (active_low)  pin_cfg |= MPU6050_INTCFG_INT_LEVEL;
    if (open_drain)  pin_cfg |= MPU6050_INTCFG_INT_OPEN;
    if (latch)       pin_cfg |= MPU6050_INTCFG_LATCH_INT_EN | MPU6050_INTCFG_INT_RD_CLEAR;
    if (mpu6050_hal_i2c_write(MPU6050_I2C_ADDR, MPU6050_REG_INT_PIN_CFG, &pin_cfg, 1) != 0) return -1;

    uint8_t en = (uint8_t)(dev->int_en_shadow | MPU6050_INT_DATA_RDY);
    if (mpu6050_hal_i2c_write(MPU6050_I2C_ADDR, MPU6050_REG_INT_ENABLE, &en, 1) != 0) return -2;
    dev->int_en_shadow = en;
    return 0;
}

int mpu6050_set_sleep(mpu6050_t *dev, bool sleep)
{
    uint8_t pwr1 = dev->pwr1_shadow;                 /* use what we know we wrote */
    if (sleep) pwr1 |=  MPU6050_PWR1_SLEEP;
    else       pwr1 &= (uint8_t)~MPU6050_PWR1_SLEEP;
    if (mpu6050_hal_i2c_write(MPU6050_I2C_ADDR, MPU6050_REG_PWR_MGMT_1, &pwr1, 1) != 0) return -1;
    dev->pwr1_shadow = pwr1;
    return 0;
}

bool mpu6050_is_data_ready(mpu6050_t *dev)
{
    (void)dev;
    uint8_t status = 0;
    /* Reading INT_STATUS clears DATA_RDY_INT (datasheet 4.17), unless INT_RD_CLEAR
     * semantics were changed. Error returns false, same convention as the MMC driver. */
    if (mpu6050_hal_i2c_read(MPU6050_I2C_ADDR, MPU6050_REG_INT_STATUS, &status, 1) != 0) {
        return false;
    }
    return (status & MPU6050_INT_DATA_RDY) != 0;
}

int mpu6050_read_raw_burst(mpu6050_t *dev, uint8_t *buf14)
{
    (void)dev;
    /* Burst read guarantees all 14 bytes come from the same sampling instant (datasheet 4.18) */
    return mpu6050_hal_i2c_read(MPU6050_I2C_ADDR, MPU6050_REG_ACCEL_XOUT_H, buf14, 14);
}

void mpu6050_parse_raw(const uint8_t *b, mpu6050_raw_t *raw)
{
    /* Register order: AX_H AX_L AY_H AY_L AZ_H AZ_L T_H T_L GX_H GX_L GY_H GY_L GZ_H GZ_L */
    raw->accel.x = (int16_t)(((uint16_t)b[0]  << 8) | b[1]);
    raw->accel.y = (int16_t)(((uint16_t)b[2]  << 8) | b[3]);
    raw->accel.z = (int16_t)(((uint16_t)b[4]  << 8) | b[5]);
    raw->temp    = (int16_t)(((uint16_t)b[6]  << 8) | b[7]);
    raw->gyro.x  = (int16_t)(((uint16_t)b[8]  << 8) | b[9]);
    raw->gyro.y  = (int16_t)(((uint16_t)b[10] << 8) | b[11]);
    raw->gyro.z  = (int16_t)(((uint16_t)b[12] << 8) | b[13]);
}

int mpu6050_read_raw(mpu6050_t *dev, mpu6050_raw_t *raw)
{
    uint8_t buf[14];
    if (mpu6050_read_raw_burst(dev, buf) != 0) return -1;
    mpu6050_parse_raw(buf, raw);
    return 0;
}

void mpu6050_convert(const mpu6050_t *dev, const mpu6050_raw_t *raw, mpu6050_data_t *out)
{
    const float a_sens = k_accel_lsb_per_g[dev->accel_fs & 0x03];
    const float g_sens = k_gyro_lsb_per_dps[dev->gyro_fs & 0x03];

    out->accel_g.x  = ((int32_t)raw->accel.x - dev->accel_offset.x) / a_sens;
    out->accel_g.y  = ((int32_t)raw->accel.y - dev->accel_offset.y) / a_sens;
    out->accel_g.z  = ((int32_t)raw->accel.z - dev->accel_offset.z) / a_sens;

    out->gyro_dps.x = ((int32_t)raw->gyro.x  - dev->gyro_offset.x) / g_sens;
    out->gyro_dps.y = ((int32_t)raw->gyro.y  - dev->gyro_offset.y) / g_sens;
    out->gyro_dps.z = ((int32_t)raw->gyro.z  - dev->gyro_offset.z) / g_sens;

    /* Datasheet 4.19: Temp(C) = TEMP_OUT / 340 + 36.53 */
    out->temp_c = (float)raw->temp / 340.0f + 36.53f;
}

int mpu6050_read(mpu6050_t *dev, mpu6050_data_t *out)
{
    mpu6050_raw_t raw;
    if (mpu6050_read_raw(dev, &raw) != 0) return -1;
    mpu6050_convert(dev, &raw, out);
    return 0;
}

/* Keep the board STILL and FLAT (Z axis pointing up) while this runs.
 * Gyro offset = mean gyro reading (true rate is 0).
 * Accel offset = mean reading minus expected gravity (+1 g on Z, 0 on X/Y).
 * Offsets are stored in raw LSB for the FS ranges currently configured. */
int mpu6050_calibrate_offset(mpu6050_t *dev, uint16_t samples)
{
    if (samples == 0) return -1;

    int64_t sum_ax = 0, sum_ay = 0, sum_az = 0;
    int64_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
    uint8_t buf[14];
    mpu6050_raw_t raw;

    /* Clear any existing offsets so they don't leak into this run */
    memset(&dev->accel_offset, 0, sizeof(dev->accel_offset));
    memset(&dev->gyro_offset,  0, sizeof(dev->gyro_offset));

    for (uint16_t i = 0; i < samples; i++) {
        /* Wait for a fresh sample, with timeout so a dead bus can't hang us */
        uint32_t t0 = HAL_GetTick();
        while (!mpu6050_is_data_ready(dev)) {
            if ((HAL_GetTick() - t0) > 100) return -2;
        }
        if (mpu6050_read_raw_burst(dev, buf) != 0) return -3;
        mpu6050_parse_raw(buf, &raw);

        sum_ax += raw.accel.x;  sum_ay += raw.accel.y;  sum_az += raw.accel.z;
        sum_gx += raw.gyro.x;   sum_gy += raw.gyro.y;   sum_gz += raw.gyro.z;
    }

    const int32_t one_g = (int32_t)k_accel_lsb_per_g[dev->accel_fs & 0x03];

    dev->accel_offset.x = (int32_t)(sum_ax / samples);
    dev->accel_offset.y = (int32_t)(sum_ay / samples);
    dev->accel_offset.z = (int32_t)(sum_az / samples) - one_g;

    dev->gyro_offset.x  = (int32_t)(sum_gx / samples);
    dev->gyro_offset.y  = (int32_t)(sum_gy / samples);
    dev->gyro_offset.z  = (int32_t)(sum_gz / samples);

    return 0;
}