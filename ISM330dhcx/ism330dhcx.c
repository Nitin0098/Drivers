/**
 * @file    ism330dhcx.c
 * @brief   Driver implementation for ST ISM330DHCX 6-axis IMU
 *
 * Plug in your platform by filling ism330dhcx_dev_t.read / .write.
 * See ism330dhcx.h for details.
 *
 * DS13012 Rev 7
 */

#include "ism330dhcx.h"
#include <string.h>   /* memcpy */

/* =========================================================================
 * Internal helpers
 * ====================================================================== */

static ism330dhcx_err_t _read(ism330dhcx_dev_t *dev, uint8_t reg,
                                uint8_t *buf, size_t len)
{
    if (!dev || !dev->read) return ISM330DHCX_ERR;
    return dev->read(dev->handle, reg, buf, len) == 0
           ? ISM330DHCX_OK : ISM330DHCX_ERR;
}

static ism330dhcx_err_t _write(ism330dhcx_dev_t *dev, uint8_t reg,
                                 const uint8_t *buf, size_t len)
{
    if (!dev || !dev->write) return ISM330DHCX_ERR;
    return dev->write(dev->handle, reg, buf, len) == 0
           ? ISM330DHCX_OK : ISM330DHCX_ERR;
}

/* Combine two bytes into a signed 16-bit value (little-endian) */
static inline int16_t _to_int16(uint8_t lo, uint8_t hi)
{
    return (int16_t)((uint16_t)hi << 8 | lo);
}

/* =========================================================================
 * Low-level register helpers
 * ====================================================================== */

ism330dhcx_err_t ism330dhcx_reg_read(ism330dhcx_dev_t *dev,
                                       uint8_t reg, uint8_t *val)
{
    return _read(dev, reg, val, 1);
}

ism330dhcx_err_t ism330dhcx_reg_write(ism330dhcx_dev_t *dev,
                                        uint8_t reg, uint8_t val)
{
    return _write(dev, reg, &val, 1);
}

ism330dhcx_err_t ism330dhcx_reg_update(ism330dhcx_dev_t *dev,
                                         uint8_t reg,
                                         uint8_t mask,
                                         uint8_t val)
{
    uint8_t tmp;
    ism330dhcx_err_t err;

    err = ism330dhcx_reg_read(dev, reg, &tmp);
    if (err != ISM330DHCX_OK) return err;

    tmp = (tmp & ~mask) | (val & mask);
    return ism330dhcx_reg_write(dev, reg, tmp);
}

/* =========================================================================
 * Sensitivity tables
 * ====================================================================== */

float ism330dhcx_xl_sensitivity(ism330dhcx_xl_fs_t fs)
{
    switch (fs) {
    case ISM330DHCX_XL_FS_2G:  return 0.061f;   /* mg/LSB */
    case ISM330DHCX_XL_FS_4G:  return 0.122f;
    case ISM330DHCX_XL_FS_8G:  return 0.244f;
    case ISM330DHCX_XL_FS_16G: return 0.488f;
    default:                    return 0.061f;
    }
}

float ism330dhcx_gy_sensitivity(ism330dhcx_gy_fs_t fs)
{
    switch (fs) {
    case ISM330DHCX_GY_FS_125DPS:  return 4.375f;  /* mdps/LSB */
    case ISM330DHCX_GY_FS_250DPS:  return 8.75f;
    case ISM330DHCX_GY_FS_500DPS:  return 17.5f;
    case ISM330DHCX_GY_FS_1000DPS: return 35.0f;
    case ISM330DHCX_GY_FS_2000DPS: return 70.0f;
    case ISM330DHCX_GY_FS_4000DPS: return 140.0f;
    default:                        return 8.75f;
    }
}

/* =========================================================================
 * Initialisation
 * ====================================================================== */

ism330dhcx_err_t ism330dhcx_init(ism330dhcx_dev_t      *dev,
                                   ism330dhcx_read_fn     read_fn,
                                   ism330dhcx_write_fn    write_fn,
                                   void                  *handle)
{
    uint8_t who;

    if (!dev || !read_fn || !write_fn) return ISM330DHCX_ERR;

    dev->read   = read_fn;
    dev->write  = write_fn;
    dev->handle = handle;

    /* Sane defaults so sensitivity functions work even before config */
    dev->xl_fs  = ISM330DHCX_XL_FS_2G;
    dev->gy_fs  = ISM330DHCX_GY_FS_250DPS;

    /* Verify device identity */
    if (_read(dev, ISM330DHCX_REG_WHO_AM_I, &who, 1) != ISM330DHCX_OK)
        return ISM330DHCX_ERR;

    if (who != ISM330DHCX_WHO_AM_I_VAL) return ISM330DHCX_ENODEV;

    /*
     * Enable auto-increment (IF_INC=1).  This is the default (CTRL3_C
     * reset value = 0x04) but set it explicitly so the driver works
     * even after a partial init sequence.
     */
    return ism330dhcx_reg_update(dev, ISM330DHCX_REG_CTRL3_C,
                                  ISM330DHCX_CTRL3_IF_INC,
                                  ISM330DHCX_CTRL3_IF_INC);
}

/* =========================================================================
 * Software reset
 * ====================================================================== */

ism330dhcx_err_t ism330dhcx_reset(ism330dhcx_dev_t *dev)
{
    ism330dhcx_err_t err;
    uint8_t val;
    int timeout = 100;

    err = ism330dhcx_reg_update(dev, ISM330DHCX_REG_CTRL3_C,
                                  ISM330DHCX_CTRL3_SW_RESET,
                                  ISM330DHCX_CTRL3_SW_RESET);
    if (err != ISM330DHCX_OK) return err;

    /* Wait for SW_RESET bit to self-clear (≤ ~50 µs typical) */
    do {
        err = ism330dhcx_reg_read(dev, ISM330DHCX_REG_CTRL3_C, &val);
        if (err != ISM330DHCX_OK) return err;
        timeout--;
    } while ((val & ISM330DHCX_CTRL3_SW_RESET) && timeout > 0);

    if (timeout == 0) return ISM330DHCX_ERR;

    /* Restore IF_INC after reset */
    return ism330dhcx_reg_update(dev, ISM330DHCX_REG_CTRL3_C,
                                  ISM330DHCX_CTRL3_IF_INC,
                                  ISM330DHCX_CTRL3_IF_INC);
}

/* =========================================================================
 * BDU
 * ====================================================================== */

ism330dhcx_err_t ism330dhcx_enable_bdu(ism330dhcx_dev_t *dev)
{
    return ism330dhcx_reg_update(dev, ISM330DHCX_REG_CTRL3_C,
                                  ISM330DHCX_CTRL3_BDU,
                                  ISM330DHCX_CTRL3_BDU);
}

/* =========================================================================
 * Sensor configuration
 * ====================================================================== */

ism330dhcx_err_t ism330dhcx_config_accel(ism330dhcx_dev_t   *dev,
                                           ism330dhcx_xl_odr_t odr,
                                           ism330dhcx_xl_fs_t  fs)
{
    /*
     * CTRL1_XL layout:
     *   [7:4] ODR_XL[3:0]
     *   [3:2] FS[1:0]_XL
     *   [1]   LPF2_XL_EN
     *   [0]   must be 0
     */
    uint8_t val = (uint8_t)(((odr & 0x0F) << 4) | ((fs & 0x03) << 2));
    ism330dhcx_err_t err = ism330dhcx_reg_write(dev, ISM330DHCX_REG_CTRL1_XL, val);
    if (err == ISM330DHCX_OK) dev->xl_fs = fs;
    return err;
}

ism330dhcx_err_t ism330dhcx_config_gyro(ism330dhcx_dev_t  *dev,
                                          ism330dhcx_gy_odr_t odr,
                                          ism330dhcx_gy_fs_t  fs)
{
    /*
     * CTRL2_G layout:
     *   [7:4] ODR_G[3:0]
     *   [3:2] FS[1:0]_G
     *   [1]   FS_125
     *   [0]   FS_4000
     */
    uint8_t val = (uint8_t)(((odr & 0x0F) << 4) | (fs & 0x0F));
    ism330dhcx_err_t err = ism330dhcx_reg_write(dev, ISM330DHCX_REG_CTRL2_G, val);
    if (err == ISM330DHCX_OK) dev->gy_fs = fs;
    return err;
}

/* =========================================================================
 * Status
 * ====================================================================== */

ism330dhcx_err_t ism330dhcx_get_status(ism330dhcx_dev_t *dev, uint8_t *status)
{
    return _read(dev, ISM330DHCX_REG_STATUS_REG, status, 1);
}

/* =========================================================================
 * Data reads
 * ====================================================================== */

ism330dhcx_err_t ism330dhcx_read_accel_raw(ism330dhcx_dev_t  *dev,
                                             ism330dhcx_raw3_t *raw)
{
    uint8_t buf[6];
    ism330dhcx_err_t err = _read(dev, ISM330DHCX_REG_OUTX_L_A, buf, 6);
    if (err != ISM330DHCX_OK) return err;

    raw->x = _to_int16(buf[0], buf[1]);
    raw->y = _to_int16(buf[2], buf[3]);
    raw->z = _to_int16(buf[4], buf[5]);
    return ISM330DHCX_OK;
}

ism330dhcx_err_t ism330dhcx_read_accel(ism330dhcx_dev_t  *dev,
                                         ism330dhcx_accel_t *out)
{
    ism330dhcx_raw3_t raw;
    ism330dhcx_err_t  err = ism330dhcx_read_accel_raw(dev, &raw);
    if (err != ISM330DHCX_OK) return err;

    float sens = ism330dhcx_xl_sensitivity(dev->xl_fs);
    out->x = raw.x * sens;
    out->y = raw.y * sens;
    out->z = raw.z * sens;
    return ISM330DHCX_OK;
}

ism330dhcx_err_t ism330dhcx_read_gyro_raw(ism330dhcx_dev_t  *dev,
                                            ism330dhcx_raw3_t *raw)
{
    uint8_t buf[6];
    ism330dhcx_err_t err = _read(dev, ISM330DHCX_REG_OUTX_L_G, buf, 6);
    if (err != ISM330DHCX_OK) return err;

    raw->x = _to_int16(buf[0], buf[1]);
    raw->y = _to_int16(buf[2], buf[3]);
    raw->z = _to_int16(buf[4], buf[5]);
    return ISM330DHCX_OK;
}

ism330dhcx_err_t ism330dhcx_read_gyro(ism330dhcx_dev_t *dev,
                                        ism330dhcx_gyro_t *out)
{
    ism330dhcx_raw3_t raw;
    ism330dhcx_err_t  err = ism330dhcx_read_gyro_raw(dev, &raw);
    if (err != ISM330DHCX_OK) return err;

    float sens = ism330dhcx_gy_sensitivity(dev->gy_fs);
    out->x = raw.x * sens;
    out->y = raw.y * sens;
    out->z = raw.z * sens;
    return ISM330DHCX_OK;
}

ism330dhcx_err_t ism330dhcx_read_temp(ism330dhcx_dev_t *dev, float *temp_c)
{
    /*
     * OUT_TEMP: 16-bit two's complement.
     * Sensitivity: 256 LSB/°C.  0 LSB = 25 °C.
     */
    uint8_t buf[2];
    ism330dhcx_err_t err = _read(dev, ISM330DHCX_REG_OUT_TEMP_L, buf, 2);
    if (err != ISM330DHCX_OK) return err;

    int16_t raw = _to_int16(buf[0], buf[1]);
    *temp_c = 25.0f + (float)raw / 256.0f;
    return ISM330DHCX_OK;
}

ism330dhcx_err_t ism330dhcx_read_all(ism330dhcx_dev_t  *dev,
                                       ism330dhcx_accel_t *accel,
                                       ism330dhcx_gyro_t  *gyro)
{
    /*
     * Burst-read 12 bytes starting at OUTX_L_G (0x22):
     *   [0–5]  Gyro  X_L, X_H, Y_L, Y_H, Z_L, Z_H
     *   [6–11] Accel X_L, X_H, Y_L, Y_H, Z_L, Z_H
     */
    uint8_t buf[12];
    ism330dhcx_err_t err = _read(dev, ISM330DHCX_REG_OUTX_L_G, buf, 12);
    if (err != ISM330DHCX_OK) return err;

    float gs = ism330dhcx_gy_sensitivity(dev->gy_fs);
    gyro->x = _to_int16(buf[0],  buf[1])  * gs;
    gyro->y = _to_int16(buf[2],  buf[3])  * gs;
    gyro->z = _to_int16(buf[4],  buf[5])  * gs;

    float as = ism330dhcx_xl_sensitivity(dev->xl_fs);
    accel->x = _to_int16(buf[6],  buf[7])  * as;
    accel->y = _to_int16(buf[8],  buf[9])  * as;
    accel->z = _to_int16(buf[10], buf[11]) * as;

    return ISM330DHCX_OK;
}

/* =========================================================================
 * FIFO
 * ====================================================================== */

ism330dhcx_err_t ism330dhcx_fifo_config(ism330dhcx_dev_t      *dev,
                                          ism330dhcx_fifo_mode_t mode,
                                          uint8_t                bdr_xl,
                                          uint8_t                bdr_gy,
                                          uint16_t               watermark)
{
    ism330dhcx_err_t err;

    /* FIFO_CTRL1: WTM[7:0] */
    err = ism330dhcx_reg_write(dev, ISM330DHCX_REG_FIFO_CTRL1,
                                (uint8_t)(watermark & 0xFF));
    if (err != ISM330DHCX_OK) return err;

    /* FIFO_CTRL2: WTM8 (bit 0) */
    err = ism330dhcx_reg_update(dev, ISM330DHCX_REG_FIFO_CTRL2,
                                  0x01, (uint8_t)((watermark >> 8) & 0x01));
    if (err != ISM330DHCX_OK) return err;

    /* FIFO_CTRL3: BDR_GY[7:4] | BDR_XL[3:0] */
    uint8_t ctrl3 = (uint8_t)(((bdr_gy & 0x0F) << 4) | (bdr_xl & 0x0F));
    err = ism330dhcx_reg_write(dev, ISM330DHCX_REG_FIFO_CTRL3, ctrl3);
    if (err != ISM330DHCX_OK) return err;

    /* FIFO_CTRL4: FIFO_MODE[2:0] */
    return ism330dhcx_reg_update(dev, ISM330DHCX_REG_FIFO_CTRL4,
                                   0x07, (uint8_t)(mode & 0x07));
}

ism330dhcx_err_t ism330dhcx_fifo_get_count(ism330dhcx_dev_t *dev,
                                             uint16_t         *count)
{
    uint8_t buf[2];
    ism330dhcx_err_t err = _read(dev, ISM330DHCX_REG_FIFO_STATUS1, buf, 2);
    if (err != ISM330DHCX_OK) return err;

    /* DIFF_FIFO[9:0]: buf[1] bits[1:0] are high bits */
    *count = (uint16_t)(buf[0] | ((buf[1] & 0x03) << 8));
    return ISM330DHCX_OK;
}

ism330dhcx_err_t ism330dhcx_fifo_read_sample(ism330dhcx_dev_t        *dev,
                                               ism330dhcx_fifo_sample_t *sample)
{
    /*
     * Read 7 bytes: TAG (1) + DATA X_L, X_H, Y_L, Y_H, Z_L, Z_H (6)
     * Starting at FIFO_DATA_OUT_TAG (0x78) with auto-increment.
     */
    uint8_t buf[7];
    ism330dhcx_err_t err = _read(dev, ISM330DHCX_REG_FIFO_DATA_OUT_TAG, buf, 7);
    if (err != ISM330DHCX_OK) return err;

    sample->tag = (buf[0] >> 3) & 0x1F;   /* bits [7:3] */
    sample->x   = _to_int16(buf[1], buf[2]);
    sample->y   = _to_int16(buf[3], buf[4]);
    sample->z   = _to_int16(buf[5], buf[6]);
    return ISM330DHCX_OK;
}

/* =========================================================================
 * Interrupts
 * ====================================================================== */

ism330dhcx_err_t ism330dhcx_set_int1(ism330dhcx_dev_t *dev, uint8_t mask)
{
    return ism330dhcx_reg_write(dev, ISM330DHCX_REG_INT1_CTRL, mask);
}

ism330dhcx_err_t ism330dhcx_set_int2(ism330dhcx_dev_t *dev, uint8_t mask)
{
    return ism330dhcx_reg_write(dev, ISM330DHCX_REG_INT2_CTRL, mask);
}

/* =========================================================================
 * Self-test
 * ====================================================================== */

/*
 * Minimal delay shim — replace with your platform's delay if needed.
 * The self-test requires the sensor to settle after enabling ST mode.
 * Per datasheet a wait of ~200 ms is needed after enabling.
 */
#ifndef ISM330DHCX_DELAY_MS
/* Default: busy-loop — replace with e.g. HAL_Delay(ms) on STM32 */
static void _delay_ms_default(uint32_t ms)
{
    /* On a bare-metal system without an OS timer, this is a placeholder.
     * Replace with your RTOS delay, hardware timer, or HAL call.       */
    volatile uint32_t i;
    for (i = 0; i < ms * 10000UL; i++) { /* crude; tune to your clock */ }
}
#define ISM330DHCX_DELAY_MS(ms) HAL_Delay(ms)
#endif

ism330dhcx_err_t ism330dhcx_self_test(ism330dhcx_dev_t  *dev,
                                        ism330dhcx_accel_t *accel_delta,
                                        ism330dhcx_gyro_t  *gyro_delta)
{
#define ST_SAMPLES  5
    ism330dhcx_err_t  err;
    ism330dhcx_accel_t a_normal = {0}, a_st = {0};
    ism330dhcx_gyro_t  g_normal = {0}, g_st = {0};
    int i;

    /* Configure: accel ±2g @ 52 Hz, gyro ±2000dps @ 208 Hz */
    err = ism330dhcx_config_accel(dev, ISM330DHCX_XL_ODR_52HZ, ISM330DHCX_XL_FS_2G);
    if (err != ISM330DHCX_OK) return err;
    err = ism330dhcx_config_gyro(dev, ISM330DHCX_GY_ODR_208HZ, ISM330DHCX_GY_FS_2000DPS);
    if (err != ISM330DHCX_OK) return err;
    err = ism330dhcx_enable_bdu(dev);
    if (err != ISM330DHCX_OK) return err;

    /* Settle */
    ISM330DHCX_DELAY_MS(200);

    /* Average normal output */
    for (i = 0; i < ST_SAMPLES; i++) {
        ism330dhcx_accel_t a; ism330dhcx_gyro_t g;
        err = ism330dhcx_read_all(dev, &a, &g);
        if (err != ISM330DHCX_OK) return err;
        a_normal.x += a.x; a_normal.y += a.y; a_normal.z += a.z;
        g_normal.x += g.x; g_normal.y += g.y; g_normal.z += g.z;
    }
    a_normal.x /= ST_SAMPLES; a_normal.y /= ST_SAMPLES; a_normal.z /= ST_SAMPLES;
    g_normal.x /= ST_SAMPLES; g_normal.y /= ST_SAMPLES; g_normal.z /= ST_SAMPLES;

    /* Enable positive self-test on both sensors
     * CTRL5_C: ST[1:0]_G=01 (bits[3:2]), ST[1:0]_XL=01 (bits[1:0]) */
    err = ism330dhcx_reg_write(dev, ISM330DHCX_REG_CTRL5_C, 0x05);
    if (err != ISM330DHCX_OK) return err;

    ISM330DHCX_DELAY_MS(200);

    /* Average self-test output */
    for (i = 0; i < ST_SAMPLES; i++) {
        ism330dhcx_accel_t a; ism330dhcx_gyro_t g;
        err = ism330dhcx_read_all(dev, &a, &g);
        if (err != ISM330DHCX_OK) return err;
        a_st.x += a.x; a_st.y += a.y; a_st.z += a.z;
        g_st.x += g.x; g_st.y += g.y; g_st.z += g.z;
    }
    a_st.x /= ST_SAMPLES; a_st.y /= ST_SAMPLES; a_st.z /= ST_SAMPLES;
    g_st.x /= ST_SAMPLES; g_st.y /= ST_SAMPLES; g_st.z /= ST_SAMPLES;

    /* Disable self-test */
    err = ism330dhcx_reg_write(dev, ISM330DHCX_REG_CTRL5_C, 0x00);
    if (err != ISM330DHCX_OK) return err;

    /* Compute absolute deltas */
#define ABS_F(x) ((x) < 0.0f ? -(x) : (x))
    if (accel_delta) {
        accel_delta->x = ABS_F(a_st.x - a_normal.x);
        accel_delta->y = ABS_F(a_st.y - a_normal.y);
        accel_delta->z = ABS_F(a_st.z - a_normal.z);
    }
    if (gyro_delta) {
        /* Convert mdps → dps for limit comparison */
        gyro_delta->x = ABS_F(g_st.x - g_normal.x);
        gyro_delta->y = ABS_F(g_st.y - g_normal.y);
        gyro_delta->z = ABS_F(g_st.z - g_normal.z);
    }
#undef ABS_F

    /* Validate against datasheet limits
     * Accel: 40–1700 mg, Gyro: 150000–700000 mdps (i.e. 150–700 dps) */
    ism330dhcx_err_t result = ISM330DHCX_OK;
    if (accel_delta) {
        if (accel_delta->x < 40.0f || accel_delta->x > 1700.0f) result = ISM330DHCX_ERR;
        if (accel_delta->y < 40.0f || accel_delta->y > 1700.0f) result = ISM330DHCX_ERR;
        if (accel_delta->z < 40.0f || accel_delta->z > 1700.0f) result = ISM330DHCX_ERR;
    }
    if (gyro_delta) {
        /* Gyro deltas are in mdps; limits from datasheet: 150 dps to 700 dps */
        if (gyro_delta->x < 150000.0f || gyro_delta->x > 700000.0f) result = ISM330DHCX_ERR;
        if (gyro_delta->y < 150000.0f || gyro_delta->y > 700000.0f) result = ISM330DHCX_ERR;
        if (gyro_delta->z < 150000.0f || gyro_delta->z > 700000.0f) result = ISM330DHCX_ERR;
    }

    return result;
#undef ST_SAMPLES
}
