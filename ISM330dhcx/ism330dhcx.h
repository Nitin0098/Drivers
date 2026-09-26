/**
 * @file    ism330dhcx.h
 * @brief   Driver for ST ISM330DHCX 6-axis IMU (3D Accel + 3D Gyro)
 *
 * Ported to any platform via two function pointers:
 *   - ism330dhcx_read_fn  : read N bytes from a register
 *   - ism330dhcx_write_fn : write N bytes to a register
 *
 * I2C addresses:
 *   ISM330DHCX_I2C_ADDR_LOW  (0x6A) when SDO/SA0 is tied to GND
 *   ISM330DHCX_I2C_ADDR_HIGH (0x6B) when SDO/SA0 is tied to VDD
 *
 * WHO_AM_I fixed value: 0x6B
 *
 * DS13012 Rev 7
 */

#ifndef ISM330DHCX_H
#define ISM330DHCX_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------
 * I2C Addresses
 * ---------------------------------------------------------------------- */
#define ISM330DHCX_I2C_ADDR_LOW   0x6A   /* SDO/SA0 → GND */
#define ISM330DHCX_I2C_ADDR_HIGH  0x6B   /* SDO/SA0 → VDD */

/* -------------------------------------------------------------------------
 * WHO_AM_I expected value
 * ---------------------------------------------------------------------- */
#define ISM330DHCX_WHO_AM_I_VAL   0x6B

/* -------------------------------------------------------------------------
 * Register Map
 * ---------------------------------------------------------------------- */
#define ISM330DHCX_REG_FUNC_CFG_ACCESS     0x01
#define ISM330DHCX_REG_PIN_CTRL            0x02
#define ISM330DHCX_REG_FIFO_CTRL1          0x07
#define ISM330DHCX_REG_FIFO_CTRL2          0x08
#define ISM330DHCX_REG_FIFO_CTRL3          0x09
#define ISM330DHCX_REG_FIFO_CTRL4          0x0A
#define ISM330DHCX_REG_COUNTER_BDR_REG1    0x0B
#define ISM330DHCX_REG_COUNTER_BDR_REG2    0x0C
#define ISM330DHCX_REG_INT1_CTRL           0x0D
#define ISM330DHCX_REG_INT2_CTRL           0x0E
#define ISM330DHCX_REG_WHO_AM_I            0x0F
#define ISM330DHCX_REG_CTRL1_XL            0x10
#define ISM330DHCX_REG_CTRL2_G             0x11
#define ISM330DHCX_REG_CTRL3_C             0x12
#define ISM330DHCX_REG_CTRL4_C             0x13
#define ISM330DHCX_REG_CTRL5_C             0x14
#define ISM330DHCX_REG_CTRL6_C             0x15
#define ISM330DHCX_REG_CTRL7_G             0x16
#define ISM330DHCX_REG_CTRL8_XL            0x17
#define ISM330DHCX_REG_CTRL9_XL            0x18
#define ISM330DHCX_REG_CTRL10_C            0x19
#define ISM330DHCX_REG_ALL_INT_SRC         0x1A
#define ISM330DHCX_REG_WAKE_UP_SRC         0x1B
#define ISM330DHCX_REG_TAP_SRC             0x1C
#define ISM330DHCX_REG_D6D_SRC             0x1D
#define ISM330DHCX_REG_STATUS_REG          0x1E
#define ISM330DHCX_REG_OUT_TEMP_L          0x20
#define ISM330DHCX_REG_OUT_TEMP_H          0x21
#define ISM330DHCX_REG_OUTX_L_G            0x22
#define ISM330DHCX_REG_OUTX_H_G            0x23
#define ISM330DHCX_REG_OUTY_L_G            0x24
#define ISM330DHCX_REG_OUTY_H_G            0x25
#define ISM330DHCX_REG_OUTZ_L_G            0x26
#define ISM330DHCX_REG_OUTZ_H_G            0x27
#define ISM330DHCX_REG_OUTX_L_A            0x28
#define ISM330DHCX_REG_OUTX_H_A            0x29
#define ISM330DHCX_REG_OUTY_L_A            0x2A
#define ISM330DHCX_REG_OUTY_H_A            0x2B
#define ISM330DHCX_REG_OUTZ_L_A            0x2C
#define ISM330DHCX_REG_OUTZ_H_A            0x2D
#define ISM330DHCX_REG_FIFO_STATUS1        0x3A
#define ISM330DHCX_REG_FIFO_STATUS2        0x3B
#define ISM330DHCX_REG_TIMESTAMP0          0x40
#define ISM330DHCX_REG_TIMESTAMP1          0x41
#define ISM330DHCX_REG_TIMESTAMP2          0x42
#define ISM330DHCX_REG_TIMESTAMP3          0x43
#define ISM330DHCX_REG_TAP_CFG0            0x56
#define ISM330DHCX_REG_TAP_CFG1            0x57
#define ISM330DHCX_REG_TAP_CFG2            0x58
#define ISM330DHCX_REG_TAP_THS_6D          0x59
#define ISM330DHCX_REG_INT_DUR2            0x5A
#define ISM330DHCX_REG_WAKE_UP_THS         0x5B
#define ISM330DHCX_REG_WAKE_UP_DUR         0x5C
#define ISM330DHCX_REG_FREE_FALL            0x5D
#define ISM330DHCX_REG_MD1_CFG             0x5E
#define ISM330DHCX_REG_MD2_CFG             0x5F
#define ISM330DHCX_REG_FIFO_DATA_OUT_TAG   0x78
#define ISM330DHCX_REG_FIFO_DATA_OUT_X_L   0x79
#define ISM330DHCX_REG_FIFO_DATA_OUT_X_H   0x7A
#define ISM330DHCX_REG_FIFO_DATA_OUT_Y_L   0x7B
#define ISM330DHCX_REG_FIFO_DATA_OUT_Y_H   0x7C
#define ISM330DHCX_REG_FIFO_DATA_OUT_Z_L   0x7D
#define ISM330DHCX_REG_FIFO_DATA_OUT_Z_H   0x7E

/* -------------------------------------------------------------------------
 * STATUS_REG bit masks
 * ---------------------------------------------------------------------- */
#define ISM330DHCX_STATUS_TDA  (1 << 2)   /* Temperature data available */
#define ISM330DHCX_STATUS_GDA  (1 << 1)   /* Gyro data available        */
#define ISM330DHCX_STATUS_XLDA (1 << 0)   /* Accel data available       */

/* -------------------------------------------------------------------------
 * CTRL3_C bit masks
 * ---------------------------------------------------------------------- */
#define ISM330DHCX_CTRL3_BOOT      (1 << 7)
#define ISM330DHCX_CTRL3_BDU       (1 << 6)
#define ISM330DHCX_CTRL3_H_LACTIVE (1 << 5)
#define ISM330DHCX_CTRL3_PP_OD     (1 << 4)
#define ISM330DHCX_CTRL3_SIM       (1 << 3)
#define ISM330DHCX_CTRL3_IF_INC    (1 << 2)
#define ISM330DHCX_CTRL3_SW_RESET  (1 << 0)

/* -------------------------------------------------------------------------
 * INT1_CTRL / INT2_CTRL bit masks
 * ---------------------------------------------------------------------- */
#define ISM330DHCX_INT_CNT_BDR    (1 << 6)
#define ISM330DHCX_INT_FIFO_FULL  (1 << 5)
#define ISM330DHCX_INT_FIFO_OVR   (1 << 4)
#define ISM330DHCX_INT_FIFO_TH    (1 << 3)
#define ISM330DHCX_INT_DRDY_TEMP  (1 << 2)
#define ISM330DHCX_INT_DRDY_G     (1 << 1)
#define ISM330DHCX_INT_DRDY_XL    (1 << 0)

/* -------------------------------------------------------------------------
 * Accelerometer ODR (CTRL1_XL bits [7:4])
 * ---------------------------------------------------------------------- */
typedef enum {
    ISM330DHCX_XL_ODR_OFF    = 0x00,  /* Power-down         */
    ISM330DHCX_XL_ODR_1_6HZ  = 0x0B,  /* 1.6 Hz  (low power only) */
    ISM330DHCX_XL_ODR_12_5HZ = 0x01,  /* 12.5 Hz            */
    ISM330DHCX_XL_ODR_26HZ   = 0x02,  /* 26 Hz              */
    ISM330DHCX_XL_ODR_52HZ   = 0x03,  /* 52 Hz              */
    ISM330DHCX_XL_ODR_104HZ  = 0x04,  /* 104 Hz             */
    ISM330DHCX_XL_ODR_208HZ  = 0x05,  /* 208 Hz             */
    ISM330DHCX_XL_ODR_416HZ  = 0x06,  /* 416 Hz             */
    ISM330DHCX_XL_ODR_833HZ  = 0x07,  /* 833 Hz             */
    ISM330DHCX_XL_ODR_1660HZ = 0x08,  /* 1.66 kHz           */
    ISM330DHCX_XL_ODR_3330HZ = 0x09,  /* 3.33 kHz           */
    ISM330DHCX_XL_ODR_6660HZ = 0x0A,  /* 6.66 kHz           */
} ism330dhcx_xl_odr_t;

/* -------------------------------------------------------------------------
 * Accelerometer full-scale (CTRL1_XL bits [3:2])
 * Sensitivity: ±2g=0.061, ±4g=0.122, ±8g=0.244, ±16g=0.488 (mg/LSB)
 * ---------------------------------------------------------------------- */
typedef enum {
    ISM330DHCX_XL_FS_2G  = 0x00,  /* ±2  g  — 0.061 mg/LSB */
    ISM330DHCX_XL_FS_16G = 0x01,  /* ±16 g  — 0.488 mg/LSB */
    ISM330DHCX_XL_FS_4G  = 0x02,  /* ±4  g  — 0.122 mg/LSB */
    ISM330DHCX_XL_FS_8G  = 0x03,  /* ±8  g  — 0.244 mg/LSB */
} ism330dhcx_xl_fs_t;

/* -------------------------------------------------------------------------
 * Gyroscope ODR (CTRL2_G bits [7:4])
 * ---------------------------------------------------------------------- */
typedef enum {
    ISM330DHCX_GY_ODR_OFF    = 0x00,
    ISM330DHCX_GY_ODR_12_5HZ = 0x01,
    ISM330DHCX_GY_ODR_26HZ   = 0x02,
    ISM330DHCX_GY_ODR_52HZ   = 0x03,
    ISM330DHCX_GY_ODR_104HZ  = 0x04,
    ISM330DHCX_GY_ODR_208HZ  = 0x05,
    ISM330DHCX_GY_ODR_416HZ  = 0x06,
    ISM330DHCX_GY_ODR_833HZ  = 0x07,
    ISM330DHCX_GY_ODR_1660HZ = 0x08,
    ISM330DHCX_GY_ODR_3330HZ = 0x09,
    ISM330DHCX_GY_ODR_6660HZ = 0x0A,
} ism330dhcx_gy_odr_t;

/* -------------------------------------------------------------------------
 * Gyroscope full-scale (CTRL2_G bits [3:0])
 * Sensitivity (mdps/LSB): 125→4.375, 250→8.75, 500→17.5, 1000→35,
 *                         2000→70, 4000→140
 * ---------------------------------------------------------------------- */
typedef enum {
    ISM330DHCX_GY_FS_250DPS  = 0x00,  /* bits FS[1:0]=00, FS_125=0, FS_4000=0 */
    ISM330DHCX_GY_FS_500DPS  = 0x04,  /* bits FS[1:0]=01 */
    ISM330DHCX_GY_FS_1000DPS = 0x08,  /* bits FS[1:0]=10 */
    ISM330DHCX_GY_FS_2000DPS = 0x0C,  /* bits FS[1:0]=11 */
    ISM330DHCX_GY_FS_125DPS  = 0x02,  /* FS_125=1        */
    ISM330DHCX_GY_FS_4000DPS = 0x01,  /* FS_4000=1       */
} ism330dhcx_gy_fs_t;

/* -------------------------------------------------------------------------
 * FIFO modes (FIFO_CTRL4 bits [2:0])
 * ---------------------------------------------------------------------- */
typedef enum {
    ISM330DHCX_FIFO_BYPASS         = 0x00,
    ISM330DHCX_FIFO_FIFO_MODE      = 0x01,
    ISM330DHCX_FIFO_CONTINUOUS      = 0x06,
    ISM330DHCX_FIFO_CONT_TO_FIFO   = 0x03,
    ISM330DHCX_FIFO_BYPASS_TO_CONT = 0x04,
    ISM330DHCX_FIFO_BYPASS_TO_FIFO = 0x07,
} ism330dhcx_fifo_mode_t;

/* -------------------------------------------------------------------------
 * FIFO tag types (from FIFO_DATA_OUT_TAG[7:3])
 * ---------------------------------------------------------------------- */
#define ISM330DHCX_FIFO_TAG_GYRO_NC    0x01
#define ISM330DHCX_FIFO_TAG_ACCEL_NC   0x02
#define ISM330DHCX_FIFO_TAG_TEMP       0x03
#define ISM330DHCX_FIFO_TAG_TIMESTAMP  0x04

/* -------------------------------------------------------------------------
 * Error codes
 * ---------------------------------------------------------------------- */
typedef enum {
    ISM330DHCX_OK    =  0,
    ISM330DHCX_ERR   = -1,   /* Generic / bus error        */
    ISM330DHCX_ENODEV = -2,  /* WHO_AM_I mismatch          */
    ISM330DHCX_EBUSY = -3,   /* Data not ready             */
} ism330dhcx_err_t;

/* -------------------------------------------------------------------------
 * Platform I/O function pointer types
 *
 *   handle  : user context (e.g. I2C handle, SPI CS pin, file descriptor)
 *   reg     : first register address
 *   data    : pointer to data buffer
 *   len     : number of bytes
 *   returns : 0 on success, non-zero on failure
 * ---------------------------------------------------------------------- */
typedef int (*ism330dhcx_read_fn)(void *handle, uint8_t reg,
                                   uint8_t *data, size_t len);
typedef int (*ism330dhcx_write_fn)(void *handle, uint8_t reg,
                                    const uint8_t *data, size_t len);

/* -------------------------------------------------------------------------
 * Sensitivity lookup (use ism330dhcx_xl_sensitivity() /
 *                         ism330dhcx_gy_sensitivity())
 * ---------------------------------------------------------------------- */

/* -------------------------------------------------------------------------
 * Device handle
 * ---------------------------------------------------------------------- */
typedef struct {
    ism330dhcx_read_fn   read;     /* Platform read callback  */
    ism330dhcx_write_fn  write;    /* Platform write callback */
    void                *handle;   /* Passed verbatim to callbacks */

    /* Cached configuration (set by init / config functions) */
    ism330dhcx_xl_fs_t  xl_fs;
    ism330dhcx_gy_fs_t  gy_fs;
} ism330dhcx_dev_t;

/* -------------------------------------------------------------------------
 * Raw data containers
 * ---------------------------------------------------------------------- */
typedef struct { int16_t x, y, z; } ism330dhcx_raw3_t;

typedef struct {
    float x;   /* mg   */
    float y;   /* mg   */
    float z;   /* mg   */
} ism330dhcx_accel_t;

typedef struct {
    float x;   /* mdps */
    float y;   /* mdps */
    float z;   /* mdps */
} ism330dhcx_gyro_t;

/* -------------------------------------------------------------------------
 * FIFO sample (7 bytes: 1 tag + 6 data)
 * ---------------------------------------------------------------------- */
typedef struct {
    uint8_t  tag;       /* FIFO_DATA_OUT_TAG[7:3] — sensor ID */
    int16_t  x;
    int16_t  y;
    int16_t  z;
} ism330dhcx_fifo_sample_t;

/* =========================================================================
 * Public API
 * ====================================================================== */

/**
 * @brief  Initialise the device handle with I/O callbacks and verify
 *         WHO_AM_I.  Call this before any other function.
 *
 * @param  dev      Pointer to device handle (caller-allocated)
 * @param  read_fn  Platform read function
 * @param  write_fn Platform write function
 * @param  handle   Opaque pointer forwarded to read_fn / write_fn
 *
 * @return ISM330DHCX_OK on success, ISM330DHCX_ENODEV if device not found.
 */
ism330dhcx_err_t ism330dhcx_init(ism330dhcx_dev_t      *dev,
                                   ism330dhcx_read_fn     read_fn,
                                   ism330dhcx_write_fn    write_fn,
                                   void                  *handle);

/**
 * @brief  Software-reset the device and wait for it to clear.
 *         All registers return to defaults.
 */
ism330dhcx_err_t ism330dhcx_reset(ism330dhcx_dev_t *dev);

/**
 * @brief  Configure the accelerometer ODR and full-scale range.
 *         Passing ISM330DHCX_XL_ODR_OFF powers the accel down.
 */
ism330dhcx_err_t ism330dhcx_config_accel(ism330dhcx_dev_t *dev,
                                           ism330dhcx_xl_odr_t odr,
                                           ism330dhcx_xl_fs_t  fs);

/**
 * @brief  Configure the gyroscope ODR and full-scale range.
 *         Passing ISM330DHCX_GY_ODR_OFF powers the gyro down.
 */
ism330dhcx_err_t ism330dhcx_config_gyro(ism330dhcx_dev_t *dev,
                                          ism330dhcx_gy_odr_t odr,
                                          ism330dhcx_gy_fs_t  fs);

/**
 * @brief  Enable Block Data Update (BDU) — recommended for all use-cases.
 *         Prevents MSB/LSB from being updated while a read is in progress.
 */
ism330dhcx_err_t ism330dhcx_enable_bdu(ism330dhcx_dev_t *dev);

/**
 * @brief  Read STATUS_REG (data-ready flags).
 * @param  status  Output: bitmask of ISM330DHCX_STATUS_* flags.
 */
ism330dhcx_err_t ism330dhcx_get_status(ism330dhcx_dev_t *dev,
                                         uint8_t          *status);

/**
 * @brief  Read raw 16-bit accelerometer output (two's complement).
 */
ism330dhcx_err_t ism330dhcx_read_accel_raw(ism330dhcx_dev_t  *dev,
                                             ism330dhcx_raw3_t *raw);

/**
 * @brief  Read accelerometer and convert to milli-g.
 */
ism330dhcx_err_t ism330dhcx_read_accel(ism330dhcx_dev_t  *dev,
                                         ism330dhcx_accel_t *out);

/**
 * @brief  Read raw 16-bit gyroscope output (two's complement).
 */
ism330dhcx_err_t ism330dhcx_read_gyro_raw(ism330dhcx_dev_t  *dev,
                                            ism330dhcx_raw3_t *raw);

/**
 * @brief  Read gyroscope and convert to milli-degrees-per-second.
 */
ism330dhcx_err_t ism330dhcx_read_gyro(ism330dhcx_dev_t *dev,
                                        ism330dhcx_gyro_t *out);

/**
 * @brief  Read temperature in degrees Celsius.
 *         Sensitivity: 256 LSB/°C, 0 LSB = 25 °C.
 */
ism330dhcx_err_t ism330dhcx_read_temp(ism330dhcx_dev_t *dev, float *temp_c);

/**
 * @brief  Read all six output axes (accel + gyro) in a single burst.
 *         More efficient than separate calls; uses auto-increment.
 */
ism330dhcx_err_t ism330dhcx_read_all(ism330dhcx_dev_t  *dev,
                                       ism330dhcx_accel_t *accel,
                                       ism330dhcx_gyro_t  *gyro);

/* ---- Sensitivity helpers ------------------------------------------------ */
/**
 * @brief  Return accelerometer sensitivity in mg/LSB for the configured FS.
 */
float ism330dhcx_xl_sensitivity(ism330dhcx_xl_fs_t fs);

/**
 * @brief  Return gyroscope sensitivity in mdps/LSB for the configured FS.
 */
float ism330dhcx_gy_sensitivity(ism330dhcx_gy_fs_t fs);

/* ---- FIFO ---------------------------------------------------------------- */
/**
 * @brief  Configure FIFO: mode, batch rates for accel and gyro, optional
 *         watermark (set watermark=0 to disable).
 *
 * @param  mode         FIFO operating mode
 * @param  bdr_xl       Accelerometer batch data rate (same encoding as ODR)
 * @param  bdr_gy       Gyroscope batch data rate (same encoding as ODR)
 * @param  watermark    Number of samples (0 = disabled, max 511)
 */
ism330dhcx_err_t ism330dhcx_fifo_config(ism330dhcx_dev_t    *dev,
                                          ism330dhcx_fifo_mode_t mode,
                                          uint8_t               bdr_xl,
                                          uint8_t               bdr_gy,
                                          uint16_t              watermark);

/**
 * @brief  Return the number of unread words currently in the FIFO.
 * @param  count  Output: sample count (each sample = 7 bytes).
 */
ism330dhcx_err_t ism330dhcx_fifo_get_count(ism330dhcx_dev_t *dev,
                                             uint16_t         *count);

/**
 * @brief  Pop one sample from the FIFO.
 *         Check sample->tag to know which sensor the data belongs to.
 */
ism330dhcx_err_t ism330dhcx_fifo_read_sample(ism330dhcx_dev_t       *dev,
                                               ism330dhcx_fifo_sample_t *sample);

/* ---- Interrupts --------------------------------------------------------- */
/**
 * @brief  Configure INT1 pin.
 * @param  mask  Bitwise OR of ISM330DHCX_INT_* flags.
 */
ism330dhcx_err_t ism330dhcx_set_int1(ism330dhcx_dev_t *dev, uint8_t mask);

/**
 * @brief  Configure INT2 pin.
 * @param  mask  Bitwise OR of ISM330DHCX_INT_* flags.
 */
ism330dhcx_err_t ism330dhcx_set_int2(ism330dhcx_dev_t *dev, uint8_t mask);

/* ---- Self-test ---------------------------------------------------------- */
/**
 * @brief  Run a basic self-test by enabling the built-in positive self-test
 *         mode for both accel and gyro, reading data, then disabling it.
 *         Prints PASS/FAIL via a user-supplied printf-compatible function or
 *         returns the raw deltas.
 *
 * @param  accel_delta  Output: |output_ST - output_normal| in mg  (X,Y,Z)
 * @param  gyro_delta   Output: |output_ST - output_normal| in mdps (X,Y,Z)
 *
 * Pass/fail limits per datasheet:
 *   Accel: 40 – 1700 mg
 *   Gyro:  150 – 700 dps (at ±2000 dps FS)
 *
 * @return ISM330DHCX_OK if within limits, ISM330DHCX_ERR otherwise.
 */
ism330dhcx_err_t ism330dhcx_self_test(ism330dhcx_dev_t  *dev,
                                        ism330dhcx_accel_t *accel_delta,
                                        ism330dhcx_gyro_t  *gyro_delta);

/* ---- Low-level register access ----------------------------------------- */
/**
 * @brief  Read raw bytes from a register.  Useful for features not covered
 *         by the high-level API (e.g. Machine Learning Core, sensor hub).
 */
ism330dhcx_err_t ism330dhcx_reg_read(ism330dhcx_dev_t *dev,
                                       uint8_t reg, uint8_t *val);

/**
 * @brief  Write a byte to a register.
 */
ism330dhcx_err_t ism330dhcx_reg_write(ism330dhcx_dev_t *dev,
                                        uint8_t reg, uint8_t val);

/**
 * @brief  Read-modify-write: change only the bits covered by mask.
 */
ism330dhcx_err_t ism330dhcx_reg_update(ism330dhcx_dev_t *dev,
                                         uint8_t reg,
                                         uint8_t mask,
                                         uint8_t val);

#ifdef __cplusplus
}
#endif

#endif /* ISM330DHCX_H */
