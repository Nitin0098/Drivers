/**
 * @file mmc5603nj.c
 * @brief Driver implementation for the MEMSIC MMC5603NJ 3-axis Magnetic Sensor
 *
 * Plug in your platform's I2C read/write and delay functions by implementing
 * the three HAL callbacks declared in mmc5603nj.h:
 *   - mmc5603nj_hal_i2c_write()
 *   - mmc5603nj_hal_i2c_read()
 *   - mmc5603nj_hal_delay_ms()
 *
 * Notable MMC5603NJ quirks handled here:
 *   - Output registers are 20-bit split across three bytes each (Xout0/1/2)
 *   - Xout2[7:4] = X[3:0], Yout2[7:4] = Y[3:0], Zout2[7:4] = Z[3:0]
 *     (lower nibble of each Xout2/Yout2/Zout2 is always 0)
 *   - Meas_m_done is bit 6 of Status1 (not bit 0 as on MMC5983MA)
 *   - Meas_m_done clears automatically when any magnetic output register is read
 *   - Continuous mode: ODR → Cmm_freq_en → Cmm_en (strict sequence)
 *   - Minimum 1ms gap (tSR) required after SET/RESET before other operations
 *   - Software reset takes ~20ms
 */

#include "mmc5603nj.h"

/* ─── Internal Helpers ────────────────────────────────────────────────────── */

static int write_reg(mmc5603nj_t *dev, uint8_t reg, uint8_t val)
{
    return mmc5603nj_hal_i2c_write(dev->i2c_addr, reg, &val, 1);
}

static int read_reg(mmc5603nj_t *dev, uint8_t reg, uint8_t *val)
{
    return mmc5603nj_hal_i2c_read(dev->i2c_addr, reg, val, 1);
}

/**
 * Poll Status1 until the given bit is set, with a millisecond timeout.
 */
static int wait_for_status(mmc5603nj_t *dev, uint8_t bit, uint32_t timeout_ms)
{
    uint8_t  status  = 0;
    uint32_t elapsed = 0;
    const uint32_t poll_ms = 1;

    while (elapsed < timeout_ms) {
        if (read_reg(dev, MMC5603NJ_REG_STATUS1, &status) != 0)
            return -1;
        if (status & bit)
            return 0;
        mmc5603nj_hal_delay_ms(poll_ms);
        elapsed += poll_ms;
    }
    return -1;
}

/**
 * Return the null-field (zero-field) offset count for the active resolution.
 */
static uint32_t null_field(mmc5603nj_t *dev)
{
    switch (dev->resolution) {
        case MMC5603NJ_RES_20BIT: return MMC5603NJ_NULL_FIELD_20BIT;
        case MMC5603NJ_RES_18BIT: return MMC5603NJ_NULL_FIELD_18BIT;
        default:                  return MMC5603NJ_NULL_FIELD_16BIT;
    }
}

/**
 * Return the sensitivity (counts/G) for the active resolution.
 */
static float sensitivity(mmc5603nj_t *dev)
{
    switch (dev->resolution) {
        case MMC5603NJ_RES_20BIT: return MMC5603NJ_SENSITIVITY_20BIT;
        case MMC5603NJ_RES_18BIT: return MMC5603NJ_SENSITIVITY_18BIT;
        default:                  return MMC5603NJ_SENSITIVITY_16BIT;
    }
}

/**
 * Convert a raw count to Gauss using the stored (or default) offset.
 */
static float raw_to_gauss(mmc5603nj_t *dev, uint32_t raw, uint32_t offset)
{
    return ((float)raw - (float)offset) / sensitivity(dev);
}

/* ─── Public API Implementation ───────────────────────────────────────────── */

void mmc5603nj_init_handle(mmc5603nj_t *dev, mmc5603nj_res_t resolution,
                            mmc5603nj_bw_t bw)
{
    dev->i2c_addr     = MMC5603NJ_I2C_ADDR;
    dev->resolution   = resolution;
    dev->bandwidth    = bw;
    dev->offset_valid = false;

    uint32_t nf = (resolution == MMC5603NJ_RES_20BIT) ? MMC5603NJ_NULL_FIELD_20BIT :
                  (resolution == MMC5603NJ_RES_18BIT) ? MMC5603NJ_NULL_FIELD_18BIT :
                                                        MMC5603NJ_NULL_FIELD_16BIT;
    dev->offset.x = nf;
    dev->offset.y = nf;
    dev->offset.z = nf;
}

int mmc5603nj_init(mmc5603nj_t *dev)
{
    int     ret;
    uint8_t id = 0;

    ret = mmc5603nj_reset(dev);
    if (ret != 0) return ret;

    /* Datasheet: power-on time after reset is 20ms */
    mmc5603nj_hal_delay_ms(25);

    ret = mmc5603nj_read_product_id(dev, &id);
    if (ret != 0)                       return ret;
    if (id != MMC5603NJ_PRODUCT_ID)     return -1;

    ret = mmc5603nj_set_bandwidth(dev, dev->bandwidth);
    return ret;
}

int mmc5603nj_reset(mmc5603nj_t *dev)
{
    return write_reg(dev, MMC5603NJ_REG_CTRL1, MMC5603NJ_CTRL1_SW_RST);
}

int mmc5603nj_read_product_id(mmc5603nj_t *dev, uint8_t *id)
{
    return read_reg(dev, MMC5603NJ_REG_PRODUCT_ID, id);
}

int mmc5603nj_set(mmc5603nj_t *dev)
{
    int ret = write_reg(dev, MMC5603NJ_REG_CTRL0, MMC5603NJ_CTRL0_DO_SET);
    if (ret != 0) return ret;
    /* Datasheet tSR: minimum 1ms between SET/RESET and next operation */
    mmc5603nj_hal_delay_ms(1);
    return 0;
}

int mmc5603nj_reset_coil(mmc5603nj_t *dev)
{
    int ret = write_reg(dev, MMC5603NJ_REG_CTRL0, MMC5603NJ_CTRL0_DO_RESET);
    if (ret != 0) return ret;
    mmc5603nj_hal_delay_ms(1);
    return 0;
}

int mmc5603nj_read_raw(mmc5603nj_t *dev, mmc5603nj_raw_t *raw)
{
    int     ret;
    uint8_t buf[9]; /* Registers 0x00–0x08: Xout0,Xout1, Yout0,Yout1, Zout0,Zout1, Xout2,Yout2,Zout2 */

    /*
     * Trigger single measurement with Auto_SR_en recommended by datasheet.
     * (Auto_SR_en | Take_meas_M = 0x21)
     */
    ret = write_reg(dev, MMC5603NJ_REG_CTRL0,
                    MMC5603NJ_CTRL0_AUTO_SR_EN | MMC5603NJ_CTRL0_TAKE_MEAS_M);
    if (ret != 0) return ret;

    /*
     * Wait for Meas_m_done (bit 6 of Status1).
     * Worst-case measurement time: 6.6ms (BW=00) + margin.
     */
    ret = wait_for_status(dev, MMC5603NJ_STATUS_MEAS_M_DONE, 25);
    if (ret != 0) return ret;

    /* Burst-read all 9 output registers starting at Xout0 (0x00) */
    ret = mmc5603nj_hal_i2c_read(dev->i2c_addr, MMC5603NJ_REG_XOUT0, buf, 9);
    if (ret != 0) return ret;

    /*
     * Register layout (unsigned, MSB first):
     *   buf[0] = Xout0: X[19:12]
     *   buf[1] = Xout1: X[11:4]
     *   buf[2] = Yout0: Y[19:12]
     *   buf[3] = Yout1: Y[11:4]
     *   buf[4] = Zout0: Z[19:12]
     *   buf[5] = Zout1: Z[11:4]
     *   buf[6] = Xout2: X[3:0] in bits [7:4], lower nibble = 0
     *   buf[7] = Yout2: Y[3:0] in bits [7:4], lower nibble = 0
     *   buf[8] = Zout2: Z[3:0] in bits [7:4], lower nibble = 0
     */
    switch (dev->resolution) {
        case MMC5603NJ_RES_20BIT:
            raw->x = ((uint32_t)buf[0] << 12) |
                     ((uint32_t)buf[1] <<  4) |
                     ((uint32_t)buf[6] >>  4);
            raw->y = ((uint32_t)buf[2] << 12) |
                     ((uint32_t)buf[3] <<  4) |
                     ((uint32_t)buf[7] >>  4);
            raw->z = ((uint32_t)buf[4] << 12) |
                     ((uint32_t)buf[5] <<  4) |
                     ((uint32_t)buf[8] >>  4);
            break;

        case MMC5603NJ_RES_18BIT:
            /* Use X[19:2]: top 16 bits shifted, plus bits [3:2] from Xout2 */
            raw->x = ((uint32_t)buf[0] << 10) |
                     ((uint32_t)buf[1] <<  2) |
                     ((uint32_t)buf[6] >>  6);
            raw->y = ((uint32_t)buf[2] << 10) |
                     ((uint32_t)buf[3] <<  2) |
                     ((uint32_t)buf[7] >>  6);
            raw->z = ((uint32_t)buf[4] << 10) |
                     ((uint32_t)buf[5] <<  2) |
                     ((uint32_t)buf[8] >>  6);
            break;

        default: /* MMC5603NJ_RES_16BIT: X[19:4] */
            raw->x = ((uint32_t)buf[0] << 8) | (uint32_t)buf[1];
            raw->y = ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
            raw->z = ((uint32_t)buf[4] << 8) | (uint32_t)buf[5];
            break;
    }

    return 0;
}

int mmc5603nj_read_gauss(mmc5603nj_t *dev, mmc5603nj_data_t *data)
{
    mmc5603nj_raw_t raw;
    int ret = mmc5603nj_read_raw(dev, &raw);
    if (ret != 0) return ret;

    data->x = raw_to_gauss(dev, raw.x, dev->offset.x);
    data->y = raw_to_gauss(dev, raw.y, dev->offset.y);
    data->z = raw_to_gauss(dev, raw.z, dev->offset.z);
    return 0;
}

int mmc5603nj_read_temperature(mmc5603nj_t *dev, float *temp_c)
{
    int     ret;
    uint8_t tout = 0;

    /* Temperature and magnetic measurements cannot be triggered simultaneously */
    ret = write_reg(dev, MMC5603NJ_REG_CTRL0, MMC5603NJ_CTRL0_TAKE_MEAS_T);
    if (ret != 0) return ret;

    ret = wait_for_status(dev, MMC5603NJ_STATUS_MEAS_T_DONE, 15);
    if (ret != 0) return ret;

    ret = read_reg(dev, MMC5603NJ_REG_TOUT, &tout);
    if (ret != 0) return ret;

    /* T(°C) = -75 + count × 0.8 */
    *temp_c = MMC5603NJ_TEMP_OFFSET + (float)tout * MMC5603NJ_TEMP_SCALE;
    return 0;
}

int mmc5603nj_calibrate_offset(mmc5603nj_t *dev)
{
    mmc5603nj_raw_t raw_set, raw_reset;
    int ret;

    /* Measurement after SET */
    ret = mmc5603nj_set(dev);
    if (ret != 0) return ret;
    ret = mmc5603nj_read_raw(dev, &raw_set);
    if (ret != 0) return ret;

    /* Measurement after RESET */
    ret = mmc5603nj_reset_coil(dev);
    if (ret != 0) return ret;
    ret = mmc5603nj_read_raw(dev, &raw_reset);
    if (ret != 0) return ret;

    /*
     * Bridge offset = (SET_output + RESET_output) / 2
     * Max sum for 20-bit values: 2 × 1048575 = 2097150 — fits in uint32_t.
     */
    dev->offset.x = (raw_set.x + raw_reset.x) / 2;
    dev->offset.y = (raw_set.y + raw_reset.y) / 2;
    dev->offset.z = (raw_set.z + raw_reset.z) / 2;
    dev->offset_valid = true;

    /* Restore to SET state for normal operation */
    return mmc5603nj_set(dev);
}

int mmc5603nj_set_auto_sr(mmc5603nj_t *dev, bool enable)
{
    uint8_t val = enable ? MMC5603NJ_CTRL0_AUTO_SR_EN : 0x00;
    return write_reg(dev, MMC5603NJ_REG_CTRL0, val);
}

int mmc5603nj_start_continuous(mmc5603nj_t *dev, uint8_t odr,
                                mmc5603nj_bw_t bw, bool auto_sr, bool hpower)
{
    int ret;

    /* Guard: ODR must be non-zero to enter continuous mode */
    if (odr == 0) return -1;

    /* Step 1: Set bandwidth in CTRL1 */
    ret = write_reg(dev, MMC5603NJ_REG_CTRL1, (uint8_t)bw & 0x03);
    if (ret != 0) return ret;
    dev->bandwidth = bw;

    /* Step 2: Write ODR register */
    ret = write_reg(dev, MMC5603NJ_REG_ODR, odr);
    if (ret != 0) return ret;

    /* Step 3: Enable Auto_SR if requested, then set Cmm_freq_en */
    uint8_t ctrl0 = MMC5603NJ_CTRL0_CMM_FREQ_EN;
    if (auto_sr) ctrl0 |= MMC5603NJ_CTRL0_AUTO_SR_EN;
    ret = write_reg(dev, MMC5603NJ_REG_CTRL0, ctrl0);
    if (ret != 0) return ret;

    /* Brief wait for internal period calculation to complete */
    mmc5603nj_hal_delay_ms(1);

    /* Step 4: Enable continuous mode in CTRL2 (+ hpower if 1000Hz) */
    uint8_t ctrl2 = MMC5603NJ_CTRL2_CMM_EN;
    if (hpower) ctrl2 |= MMC5603NJ_CTRL2_HPOWER;
    return write_reg(dev, MMC5603NJ_REG_CTRL2, ctrl2);
}

int mmc5603nj_stop_continuous(mmc5603nj_t *dev)
{
    return write_reg(dev, MMC5603NJ_REG_CTRL2, 0x00);
}

int mmc5603nj_is_data_ready(mmc5603nj_t *dev, bool *ready)
{
    uint8_t status = 0;
    int ret = read_reg(dev, MMC5603NJ_REG_STATUS1, &status);
    if (ret != 0) return ret;
    *ready = (status & MMC5603NJ_STATUS_MEAS_M_DONE) != 0;
    return 0;
}

int mmc5603nj_set_bandwidth(mmc5603nj_t *dev, mmc5603nj_bw_t bw)
{
    int ret = write_reg(dev, MMC5603NJ_REG_CTRL1, (uint8_t)bw & 0x03);
    if (ret == 0) dev->bandwidth = bw;
    return ret;
}

int mmc5603nj_enable_periodic_set(mmc5603nj_t *dev, mmc5603nj_prd_set_t interval)
{
    /*
     * Periodic SET requires Auto_SR_en (CTRL0) and Cmm_en (CTRL2) active.
     * Read-modify-write CTRL2 to preserve the CMM_en and hpower bits.
     */
    uint8_t ctrl2 = 0;
    int ret = read_reg(dev, MMC5603NJ_REG_CTRL2, &ctrl2);
    if (ret != 0) return ret;

    /* Preserve CMM_en and hpower; set En_prd_set and Prd_set[2:0] */
    ctrl2 &= (MMC5603NJ_CTRL2_CMM_EN | MMC5603NJ_CTRL2_HPOWER);
    ctrl2 |= MMC5603NJ_CTRL2_EN_PRD_SET;
    ctrl2 |= ((uint8_t)interval & 0x07);
    return write_reg(dev, MMC5603NJ_REG_CTRL2, ctrl2);
}

int mmc5603nj_selftest(mmc5603nj_t *dev, bool *passed)
{
    int     ret;
    uint8_t st_x = 0, st_y = 0, st_z = 0;
    uint8_t status = 0;

    /* Step 1: Read factory self-test reference values */
    ret  = read_reg(dev, MMC5603NJ_REG_ST_X, &st_x);
    ret |= read_reg(dev, MMC5603NJ_REG_ST_Y, &st_y);
    ret |= read_reg(dev, MMC5603NJ_REG_ST_Z, &st_z);
    if (ret != 0) return ret;

    /* Step 2: Set thresholds to 80% of factory values */
    ret  = write_reg(dev, MMC5603NJ_REG_ST_X_TH, (uint8_t)(st_x * 4 / 5));
    ret |= write_reg(dev, MMC5603NJ_REG_ST_Y_TH, (uint8_t)(st_y * 4 / 5));
    ret |= write_reg(dev, MMC5603NJ_REG_ST_Z_TH, (uint8_t)(st_z * 4 / 5));
    if (ret != 0) return ret;

    /* Step 3: Trigger measurement with Auto_st_en and Take_meas_M */
    ret = write_reg(dev, MMC5603NJ_REG_CTRL0,
                    MMC5603NJ_CTRL0_AUTO_ST_EN | MMC5603NJ_CTRL0_TAKE_MEAS_M);
    if (ret != 0) return ret;

    /* Step 4: Wait for measurement to complete */
    ret = wait_for_status(dev, MMC5603NJ_STATUS_MEAS_M_DONE, 25);
    if (ret != 0) return ret;

    /* Step 5: Check Sat_sensor bit — 0 = PASS, 1 = FAIL */
    ret = read_reg(dev, MMC5603NJ_REG_STATUS1, &status);
    if (ret != 0) return ret;

    *passed = !(status & MMC5603NJ_STATUS_SAT_SENSOR);
    return 0;
}

int mmc5603nj_read_register(mmc5603nj_t *dev, uint8_t reg, uint8_t *val)
{
    return read_reg(dev, reg, val);
}

int mmc5603nj_write_register(mmc5603nj_t *dev, uint8_t reg, uint8_t val)
{
    return write_reg(dev, reg, val);
}
