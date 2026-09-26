/**
 * @file mmc5983ma.c
 * @brief Driver implementation for the MEMSIC MMC5983MA 3-axis Magnetic Sensor
 *
 * Plug in your platform's I2C read/write and delay functions by implementing
 * the three HAL callbacks declared in mmc5983ma.h:
 *   - mmc5983ma_hal_i2c_write()
 *   - mmc5983ma_hal_i2c_read()
 *   - mmc5983ma_hal_delay_ms()
 */

#include "mmc5983ma.h"

/* ─── Internal Helpers ────────────────────────────────────────────────────── */

/**
 * Write a single byte to a register.
 */
static int write_reg(mmc5983ma_t *dev, uint8_t reg, uint8_t val)
{
    return mmc5983ma_hal_i2c_write(dev->i2c_addr, reg, &val, 1);
}

/**
 * Read a single byte from a register.
 */
static int read_reg(mmc5983ma_t *dev, uint8_t reg, uint8_t *val)
{
    return mmc5983ma_hal_i2c_read(dev->i2c_addr, reg, val, 1);
}

/**
 * Poll the Status register until the given done bit is set, with a timeout.
 * Returns 0 when bit is set, -1 on timeout.
 */
static int wait_for_status(mmc5983ma_t *dev, uint8_t bit, uint32_t timeout_ms)
{
    uint8_t status = 0;
    uint32_t elapsed = 0;
    const uint32_t poll_ms = 1;

    while (elapsed < timeout_ms) {
        if (read_reg(dev, MMC5983MA_REG_STATUS, &status) != 0)
            return -1;
        if (status & bit)
            return 0;
        mmc5983ma_hal_delay_ms(poll_ms);
        elapsed += poll_ms;
    }
    return -1; /* timeout */
}

/**
 * Convert a raw 18-bit count to Gauss using the stored (or default) offset.
 */
static float raw_to_gauss(mmc5983ma_t *dev, uint32_t raw, uint32_t offset)
{
    float sensitivity = dev->mode_18bit ? MMC5983MA_SENSITIVITY_18BIT
                                        : MMC5983MA_SENSITIVITY_16BIT;
    return ((float)raw - (float)offset) / sensitivity;
}

/* ─── Public API Implementation ───────────────────────────────────────────── */

void mmc5983ma_init_handle(mmc5983ma_t *dev, bool mode_18bit)
{
    dev->i2c_addr     = MMC5983MA_I2C_ADDR;
    dev->bandwidth    = MMC5983MA_BW_100HZ;
    dev->mode_18bit   = mode_18bit;
    dev->offset_valid = false;
    dev->offset.x     = dev->mode_18bit ? MMC5983MA_NULL_FIELD_18BIT
                                        : MMC5983MA_NULL_FIELD_16BIT;
    dev->offset.y     = dev->offset.x;
    dev->offset.z     = dev->offset.x;
}

int mmc5983ma_init(mmc5983ma_t *dev)
{
    int ret;
    uint8_t id = 0;

    /* Software reset */
    ret = mmc5983ma_reset(dev);
    if (ret != 0) return ret;
    mmc5983ma_hal_delay_ms(15); /* Power-on/reset time: ~10ms per datasheet */

    /* Verify product ID */
    ret = mmc5983ma_read_product_id(dev, &id);
    if (ret != 0)           return ret;
    if (id != MMC5983MA_PRODUCT_ID) return -1;

    /* Apply default bandwidth */
    ret = mmc5983ma_set_bandwidth(dev, dev->bandwidth);
    return ret;
}

int mmc5983ma_reset(mmc5983ma_t *dev)
{
    return write_reg(dev, MMC5983MA_REG_CTRL1, MMC5983MA_CTRL1_SW_RST);
}

int mmc5983ma_read_product_id(mmc5983ma_t *dev, uint8_t *id)
{
    return read_reg(dev, MMC5983MA_REG_PRODUCT_ID, id);
}

int mmc5983ma_set(mmc5983ma_t *dev)
{
    int ret = write_reg(dev, MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_SET);
    if (ret != 0) return ret;
    mmc5983ma_hal_delay_ms(1); /* SET pulse lasts 500ns; 1ms is safe margin */
    return 0;
}

int mmc5983ma_reset_coil(mmc5983ma_t *dev)
{
    int ret = write_reg(dev, MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_RESET);
    if (ret != 0) return ret;
    mmc5983ma_hal_delay_ms(1); /* RESET pulse lasts 500ns; 1ms is safe margin */
    return 0;
}

int mmc5983ma_read_raw(mmc5983ma_t *dev, mmc5983ma_raw_t *raw)
{
    int ret;
    uint8_t buf[7]; /* Registers 0x00 through 0x06 */

    /* Trigger a single measurement */
    ret = write_reg(dev, MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_TM_M);
    if (ret != 0) return ret;

    /*
     * Wait for Meas_M_Done. Timeout depends on bandwidth:
     * BW=00 → 8ms, BW=01 → 4ms, BW=10 → 2ms, BW=11 → 0.5ms
     * Using 20ms as a conservative ceiling.
     */
    ret = wait_for_status(dev, MMC5983MA_STATUS_MEAS_M_DONE, 20);
    if (ret != 0) return ret;

    /* Burst-read registers 0x00–0x06 */
    ret = mmc5983ma_hal_i2c_read(dev->i2c_addr, MMC5983MA_REG_XOUT0, buf, 7);
    if (ret != 0) return ret;

    if (dev->mode_18bit) {
        /* 18-bit: combine Xout[17:10] | Xout[9:2] | Xout[1:0] */
        raw->x = ((uint32_t)buf[0] << 10) |
                 ((uint32_t)buf[1] <<  2) |
                 ((uint32_t)(buf[6] >> 6) & 0x03);

        raw->y = ((uint32_t)buf[2] << 10) |
                 ((uint32_t)buf[3] <<  2) |
                 ((uint32_t)(buf[6] >> 4) & 0x03);

        raw->z = ((uint32_t)buf[4] << 10) |
                 ((uint32_t)buf[5] <<  2) |
                 ((uint32_t)(buf[6] >> 2) & 0x03);
    } else {
        /* 16-bit: use only Xout[17:2] (upper 16 bits of 18-bit word) */
        raw->x = ((uint32_t)buf[0] << 8) | (uint32_t)buf[1];
        raw->y = ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
        raw->z = ((uint32_t)buf[4] << 8) | (uint32_t)buf[5];
    }

    return 0;
}

int mmc5983ma_read_gauss(mmc5983ma_t *dev, mmc5983ma_data_t *data)
{
    mmc5983ma_raw_t raw;
    int ret = mmc5983ma_read_raw(dev, &raw);
    if (ret != 0) return ret;

    data->x = raw_to_gauss(dev, raw.x, dev->offset.x);
    data->y = raw_to_gauss(dev, raw.y, dev->offset.y);
    data->z = raw_to_gauss(dev, raw.z, dev->offset.z);
    return 0;
}

int mmc5983ma_read_temperature(mmc5983ma_t *dev, float *temp_c)
{
    int ret;
    uint8_t tout = 0;

    /* Trigger temperature measurement (cannot be simultaneous with TM_M) */
    ret = write_reg(dev, MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_TM_T);
    if (ret != 0) return ret;

    /* Wait for Meas_T_Done; temperature conversion is quick (~1ms) */
    ret = wait_for_status(dev, MMC5983MA_STATUS_MEAS_T_DONE, 10);
    if (ret != 0) return ret;

    ret = read_reg(dev, MMC5983MA_REG_TOUT, &tout);
    if (ret != 0) return ret;

    /* T(°C) = -75 + count × 0.8 */
    *temp_c = MMC5983MA_TEMP_OFFSET + (float)tout * MMC5983MA_TEMP_SCALE;
    return 0;
}

int mmc5983ma_calibrate_offset(mmc5983ma_t *dev)
{
    mmc5983ma_raw_t raw_set, raw_reset;
    int ret;

    /* Measurement after SET */
    ret = mmc5983ma_set(dev);
    if (ret != 0) return ret;
    ret = mmc5983ma_read_raw(dev, &raw_set);
    if (ret != 0) return ret;

    /* Measurement after RESET */
    ret = mmc5983ma_reset_coil(dev);
    if (ret != 0) return ret;
    ret = mmc5983ma_read_raw(dev, &raw_reset);
    if (ret != 0) return ret;

    /*
     * Bridge offset = (SET_output + RESET_output) / 2
     * Stored as integer counts; the division by 2 is lossless for summed
     * 18-bit values (max sum = 2 × 262143 = 524286, fits in uint32_t).
     */
    dev->offset.x = (raw_set.x + raw_reset.x) / 2;
    dev->offset.y = (raw_set.y + raw_reset.y) / 2;
    dev->offset.z = (raw_set.z + raw_reset.z) / 2;
    dev->offset_valid = true;

    /* Restore sensor to SET state for normal operation */
    return mmc5983ma_set(dev);
}

int mmc5983ma_set_auto_sr(mmc5983ma_t *dev, bool enable)
{
    uint8_t val = enable ? MMC5983MA_CTRL0_AUTO_SR_EN : 0x00;
    return write_reg(dev, MMC5983MA_REG_CTRL0, val);
}

int mmc5983ma_start_continuous(mmc5983ma_t *dev, mmc5983ma_cm_freq_t freq,
                               mmc5983ma_bw_t bw, bool auto_sr)
{
    int ret;

    /* Set bandwidth in CTRL1 */
    ret = write_reg(dev, MMC5983MA_REG_CTRL1, (uint8_t)bw & 0x03);
    if (ret != 0) return ret;
    dev->bandwidth = bw;

    /* Enable auto SET/RESET if requested */
    if (auto_sr) {
        ret = write_reg(dev, MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_AUTO_SR_EN);
        if (ret != 0) return ret;
    }

    /* Enable continuous mode: set CMM_en and CM_Freq in CTRL2 */
    uint8_t ctrl2 = MMC5983MA_CTRL2_CMM_EN | ((uint8_t)freq & 0x07);
    return write_reg(dev, MMC5983MA_REG_CTRL2, ctrl2);
}

int mmc5983ma_stop_continuous(mmc5983ma_t *dev)
{
    /* Clear CTRL2 to disable CMM */
    return write_reg(dev, MMC5983MA_REG_CTRL2, 0x00);
}

int mmc5983ma_is_data_ready(mmc5983ma_t *dev, bool *ready)
{
    uint8_t status = 0;
    int ret = read_reg(dev, MMC5983MA_REG_STATUS, &status);
    if (ret != 0) return ret;
    *ready = (status & MMC5983MA_STATUS_MEAS_M_DONE) != 0;
    return 0;
}

int mmc5983ma_set_bandwidth(mmc5983ma_t *dev, mmc5983ma_bw_t bw)
{
    int ret = write_reg(dev, MMC5983MA_REG_CTRL1, (uint8_t)bw & 0x03);
    if (ret == 0)
        dev->bandwidth = bw;
    return ret;
}

int mmc5983ma_enable_interrupt(mmc5983ma_t *dev, bool enable)
{
    uint8_t val = enable ? MMC5983MA_CTRL0_INT_MEAS_EN : 0x00;
    return write_reg(dev, MMC5983MA_REG_CTRL0, val);
}

int mmc5983ma_enable_periodic_set(mmc5983ma_t *dev, mmc5983ma_prd_set_t interval)
{
    /*
     * Periodic SET requires Auto_SR_en (CTRL0) and Cmm_en (CTRL2) to also
     * be active. This function sets the interval and En_prd_set flag in CTRL2
     * alongside the existing CMM_en bit.
     */
    uint8_t ctrl2 = 0;
    int ret = read_reg(dev, MMC5983MA_REG_CTRL2, &ctrl2);
    if (ret != 0) return ret;

    /* Preserve CMM_en and CM_Freq; set En_prd_set and Prd_set[2:0] */
    ctrl2 &= 0x0F; /* keep CM_Freq and CMM_en bits */
    ctrl2 |= MMC5983MA_CTRL2_EN_PRD_SET;
    ctrl2 |= ((uint8_t)interval & 0x07) << 4;
    return write_reg(dev, MMC5983MA_REG_CTRL2, ctrl2);
}

int mmc5983ma_read_register(mmc5983ma_t *dev, uint8_t reg, uint8_t *val)
{
    return read_reg(dev, reg, val);
}

int mmc5983ma_write_register(mmc5983ma_t *dev, uint8_t reg, uint8_t val)
{
    return write_reg(dev, reg, val);
}
