/**
 * @file mmc5983ma.h
 * @brief Driver for the MEMSIC MMC5983MA 3-axis Magnetic Sensor
 *
 * Supports I2C interface. Provides functions for initialization,
 * single-shot and continuous measurements, SET/RESET degaussing,
 * temperature reading, and offset calibration.
 *
 * Datasheet: MEMSIC MMC5983MA Rev A (4/3/2019)
 */

#ifndef MMC5983MA_H
#define MMC5983MA_H

#include <stdint.h>
#include <stdbool.h>

/* ─── I2C Address ─────────────────────────────────────────────────────────── */
#define MMC5983MA_I2C_ADDR          0x30    /**< 7-bit address: 0110000 */

/* ─── Register Addresses ──────────────────────────────────────────────────── */
#define MMC5983MA_REG_XOUT0         0x00    /**< X output bits [17:10]         */
#define MMC5983MA_REG_XOUT1         0x01    /**< X output bits [9:2]           */
#define MMC5983MA_REG_YOUT0         0x02    /**< Y output bits [17:10]         */
#define MMC5983MA_REG_YOUT1         0x03    /**< Y output bits [9:2]           */
#define MMC5983MA_REG_ZOUT0         0x04    /**< Z output bits [17:10]         */
#define MMC5983MA_REG_ZOUT1         0x05    /**< Z output bits [9:2]           */
#define MMC5983MA_REG_XYZOUT2       0x06    /**< X[1:0], Y[1:0], Z[1:0] LSBs  */
#define MMC5983MA_REG_TOUT          0x07    /**< Temperature output            */
#define MMC5983MA_REG_STATUS        0x08    /**< Device status                 */
#define MMC5983MA_REG_CTRL0         0x09    /**< Internal control register 0   */
#define MMC5983MA_REG_CTRL1         0x0A    /**< Internal control register 1   */
#define MMC5983MA_REG_CTRL2         0x0B    /**< Internal control register 2   */
#define MMC5983MA_REG_CTRL3         0x0C    /**< Internal control register 3   */
#define MMC5983MA_REG_PRODUCT_ID    0x2F    /**< Product ID (expected: 0x30)   */

/* ─── Status Register Bits ────────────────────────────────────────────────── */
#define MMC5983MA_STATUS_MEAS_M_DONE    (1 << 0)  /**< Magnetic meas complete  */
#define MMC5983MA_STATUS_MEAS_T_DONE    (1 << 1)  /**< Temperature meas done   */
#define MMC5983MA_STATUS_OTP_RD_DONE    (1 << 4)  /**< OTP read complete       */

/* ─── Control Register 0 Bits ────────────────────────────────────────────── */
#define MMC5983MA_CTRL0_TM_M            (1 << 0)  /**< Take magnetic meas      */
#define MMC5983MA_CTRL0_TM_T            (1 << 1)  /**< Take temperature meas   */
#define MMC5983MA_CTRL0_INT_MEAS_EN     (1 << 2)  /**< Enable meas done INT    */
#define MMC5983MA_CTRL0_SET             (1 << 3)  /**< Perform SET operation   */
#define MMC5983MA_CTRL0_RESET           (1 << 4)  /**< Perform RESET operation */
#define MMC5983MA_CTRL0_AUTO_SR_EN      (1 << 5)  /**< Auto SET/RESET enable   */
#define MMC5983MA_CTRL0_OTP_READ        (1 << 6)  /**< Re-read OTP             */

/* ─── Control Register 1 Bits ────────────────────────────────────────────── */
#define MMC5983MA_CTRL1_SW_RST          (1 << 7)  /**< Software reset          */
#define MMC5983MA_CTRL1_YZ_INHIBIT      (1 << 3)  /**< Disable Y and Z axes    */
#define MMC5983MA_CTRL1_X_INHIBIT       (1 << 2)  /**< Disable X axis          */

/* ─── Control Register 2 Bits ────────────────────────────────────────────── */
#define MMC5983MA_CTRL2_CMM_EN          (1 << 3)  /**< Enable continuous mode  */
#define MMC5983MA_CTRL2_EN_PRD_SET      (1 << 7)  /**< Enable periodic SET     */

/* ─── Control Register 3 Bits ────────────────────────────────────────────── */
#define MMC5983MA_CTRL3_SPI_3W          (1 << 6)  /**< Enable 3-wire SPI mode  */
#define MMC5983MA_CTRL3_ST_ENP          (1 << 1)  /**< Self-test positive      */
#define MMC5983MA_CTRL3_ST_ENM          (1 << 2)  /**< Self-test negative      */

/* ─── Sensor Constants ────────────────────────────────────────────────────── */
#define MMC5983MA_PRODUCT_ID            0x30      /**< Expected product ID      */
#define MMC5983MA_NULL_FIELD_18BIT      131072UL  /**< Zero-field output (18b)  */
#define MMC5983MA_NULL_FIELD_16BIT      32768U    /**< Zero-field output (16b)  */
#define MMC5983MA_SENSITIVITY_18BIT     16384.0f  /**< Counts per Gauss (18b)   */
#define MMC5983MA_SENSITIVITY_16BIT     4096.0f   /**< Counts per Gauss (16b)   */
#define MMC5983MA_TEMP_OFFSET           (-75.0f)  /**< Temp sensor zero point   */
#define MMC5983MA_TEMP_SCALE            (0.8f)    /**< Degrees C per LSB        */

/* ─── Bandwidth / Measurement Time Settings ───────────────────────────────── */
typedef enum {
    MMC5983MA_BW_100HZ  = 0x00,  /**< 8ms  meas time, 100Hz  bandwidth */
    MMC5983MA_BW_200HZ  = 0x01,  /**< 4ms  meas time, 200Hz  bandwidth */
    MMC5983MA_BW_400HZ  = 0x02,  /**< 2ms  meas time, 400Hz  bandwidth */
    MMC5983MA_BW_800HZ  = 0x03,  /**< 0.5ms meas time, 800Hz bandwidth */
} mmc5983ma_bw_t;

/* ─── Continuous Measurement Mode Frequencies ─────────────────────────────── */
typedef enum {
    MMC5983MA_CM_OFF    = 0x00,  /**< Continuous mode disabled    */
    MMC5983MA_CM_1HZ    = 0x01,
    MMC5983MA_CM_10HZ   = 0x02,
    MMC5983MA_CM_20HZ   = 0x03,
    MMC5983MA_CM_50HZ   = 0x04,
    MMC5983MA_CM_100HZ  = 0x05,
    MMC5983MA_CM_200HZ  = 0x06,  /**< Requires BW=01              */
    MMC5983MA_CM_1000HZ = 0x07,  /**< Requires BW=11              */
} mmc5983ma_cm_freq_t;

/* ─── Periodic SET Interval ───────────────────────────────────────────────── */
typedef enum {
    MMC5983MA_PRD_SET_1    = 0x00,
    MMC5983MA_PRD_SET_25   = 0x01,
    MMC5983MA_PRD_SET_75   = 0x02,
    MMC5983MA_PRD_SET_100  = 0x03,
    MMC5983MA_PRD_SET_250  = 0x04,
    MMC5983MA_PRD_SET_500  = 0x05,
    MMC5983MA_PRD_SET_1000 = 0x06,
    MMC5983MA_PRD_SET_2000 = 0x07,
} mmc5983ma_prd_set_t;

/* ─── Raw Magnetic Data ───────────────────────────────────────────────────── */
typedef struct {
    uint32_t x;   /**< Raw X output (18-bit unsigned) */
    uint32_t y;   /**< Raw Y output (18-bit unsigned) */
    uint32_t z;   /**< Raw Z output (18-bit unsigned) */
} mmc5983ma_raw_t;

/* ─── Calibrated Magnetic Data (in Gauss) ────────────────────────────────── */
typedef struct {
    float x;
    float y;
    float z;
} mmc5983ma_data_t;

/* ─── Stored Offset (for bridge offset removal) ──────────────────────────── */
typedef struct {
    uint32_t x;
    uint32_t y;
    uint32_t z;
} mmc5983ma_offset_t;

/* ─── Device Handle ───────────────────────────────────────────────────────── */
typedef struct {
    uint8_t             i2c_addr;       /**< I2C address (default 0x30)   */
    mmc5983ma_bw_t      bandwidth;      /**< Active bandwidth setting      */
    bool                mode_18bit;     /**< true = 18-bit, false = 16-bit */
    mmc5983ma_offset_t  offset;         /**< Stored bridge offset          */
    bool                offset_valid;   /**< Whether offset has been set   */
} mmc5983ma_t;

/* ─── HAL Callbacks (implement these for your platform) ───────────────────── */
/**
 * Write bytes over I2C.
 * @param addr   7-bit I2C device address
 * @param reg    Register address to write to
 * @param data   Pointer to data buffer
 * @param len    Number of bytes to write
 * @return 0 on success, non-zero on error
 */
extern int mmc5983ma_hal_i2c_write(uint8_t addr, uint8_t reg,
                                   const uint8_t *data, uint16_t len);

/**
 * Read bytes over I2C.
 * @param addr   7-bit I2C device address
 * @param reg    Register address to read from
 * @param data   Pointer to receive buffer
 * @param len    Number of bytes to read
 * @return 0 on success, non-zero on error
 */
extern int mmc5983ma_hal_i2c_read(uint8_t addr, uint8_t reg,
                                  uint8_t *data, uint16_t len);

/**
 * Delay for the given number of milliseconds.
 * @param ms  Milliseconds to delay
 */
extern void mmc5983ma_hal_delay_ms(uint32_t ms);

/* ─── Public API ──────────────────────────────────────────────────────────── */

/**
 * Initialize the device handle with default settings.
 * Call this before any other function.
 *
 * @param dev        Pointer to device handle to initialize
 * @param mode_18bit true for 18-bit output, false for 16-bit
 */
void mmc5983ma_init_handle(mmc5983ma_t *dev, bool mode_18bit);

/**
 * Initialize and verify the sensor over I2C.
 * Performs a software reset, reads the product ID, and sets bandwidth.
 *
 * @param dev  Pointer to initialized device handle
 * @return 0 on success, -1 if product ID mismatch or communication error
 */
int mmc5983ma_init(mmc5983ma_t *dev);

/**
 * Perform a software reset (clears all registers, re-reads OTP).
 * Wait at least 10ms after calling this before further communication.
 *
 * @param dev  Pointer to device handle
 * @return 0 on success
 */
int mmc5983ma_reset(mmc5983ma_t *dev);

/**
 * Read the product ID register.
 * Expected value is 0x30.
 *
 * @param dev  Pointer to device handle
 * @param id   Output: product ID byte
 * @return 0 on success
 */
int mmc5983ma_read_product_id(mmc5983ma_t *dev, uint8_t *id);

/**
 * Perform the SET operation (magnetizes sensing elements in SET direction).
 * Used before a measurement to establish a known sensor state.
 *
 * @param dev  Pointer to device handle
 * @return 0 on success
 */
int mmc5983ma_set(mmc5983ma_t *dev);

/**
 * Perform the RESET operation (magnetizes sensing elements in RESET direction).
 * Used in combination with SET for bridge offset removal.
 *
 * @param dev  Pointer to device handle
 * @return 0 on success
 */
int mmc5983ma_reset_coil(mmc5983ma_t *dev);

/**
 * Take a single magnetic field measurement (blocking).
 * Waits for Meas_M_Done, then reads raw X/Y/Z registers.
 *
 * @param dev  Pointer to device handle
 * @param raw  Output: raw 18-bit (or 16-bit) unsigned counts
 * @return 0 on success, -1 on timeout
 */
int mmc5983ma_read_raw(mmc5983ma_t *dev, mmc5983ma_raw_t *raw);

/**
 * Take a single magnetic field measurement and convert to Gauss.
 * Uses the null-field offset to produce a signed result.
 * If offset calibration has been performed, it is applied automatically.
 *
 * @param dev   Pointer to device handle
 * @param data  Output: magnetic field in Gauss for each axis
 * @return 0 on success, -1 on error
 */
int mmc5983ma_read_gauss(mmc5983ma_t *dev, mmc5983ma_data_t *data);

/**
 * Take a temperature measurement (blocking).
 * Range: -75°C to +125°C, ~0.8°C per LSB.
 *
 * @param dev      Pointer to device handle
 * @param temp_c   Output: temperature in degrees Celsius
 * @return 0 on success, -1 on timeout
 */
int mmc5983ma_read_temperature(mmc5983ma_t *dev, float *temp_c);

/**
 * Calibrate bridge offset using SET/RESET subtraction method.
 *
 * Performs: SET → measure → RESET → measure, then stores
 * the average as the offset. Subsequent calls to mmc5983ma_read_gauss()
 * will use this stored offset instead of the fixed null-field value.
 *
 * Formula: offset = (output_set + output_reset) / 2
 *          field  = (output_set - output_reset) / 2
 *
 * @param dev  Pointer to device handle
 * @return 0 on success, -1 on error
 */
int mmc5983ma_calibrate_offset(mmc5983ma_t *dev);

/**
 * Enable or disable automatic SET/RESET before each measurement.
 *
 * @param dev     Pointer to device handle
 * @param enable  true to enable, false to disable
 * @return 0 on success
 */
int mmc5983ma_set_auto_sr(mmc5983ma_t *dev, bool enable);

/**
 * Configure and start continuous measurement mode.
 *
 * @param dev      Pointer to device handle
 * @param freq     Desired output data rate
 * @param bw       Bandwidth / measurement time setting
 * @param auto_sr  Enable automatic SET/RESET
 * @return 0 on success
 */
int mmc5983ma_start_continuous(mmc5983ma_t *dev, mmc5983ma_cm_freq_t freq,
                               mmc5983ma_bw_t bw, bool auto_sr);

/**
 * Stop continuous measurement mode.
 *
 * @param dev  Pointer to device handle
 * @return 0 on success
 */
int mmc5983ma_stop_continuous(mmc5983ma_t *dev);

/**
 * Check whether a magnetic measurement is ready to be read.
 * Useful when polling in continuous mode without using the INT pin.
 *
 * @param dev   Pointer to device handle
 * @param ready Output: true if Meas_M_Done bit is set
 * @return 0 on success
 */
int mmc5983ma_is_data_ready(mmc5983ma_t *dev, bool *ready);

/**
 * Set the measurement bandwidth (decimation filter duration).
 *
 * @param dev  Pointer to device handle
 * @param bw   Bandwidth selection
 * @return 0 on success
 */
int mmc5983ma_set_bandwidth(mmc5983ma_t *dev, mmc5983ma_bw_t bw);

/**
 * Enable or disable the measurement-done interrupt on the INT pin.
 *
 * @param dev     Pointer to device handle
 * @param enable  true to enable, false to disable
 * @return 0 on success
 */
int mmc5983ma_enable_interrupt(mmc5983ma_t *dev, bool enable);

/**
 * Enable periodic automatic SET during continuous mode.
 * Requires continuous mode and auto SET/RESET to also be enabled.
 *
 * @param dev      Pointer to device handle
 * @param interval How often to perform a SET (every N measurements)
 * @return 0 on success
 */
int mmc5983ma_enable_periodic_set(mmc5983ma_t *dev,
                                  mmc5983ma_prd_set_t interval);

/**
 * Read a raw register byte (useful for debugging).
 *
 * @param dev  Pointer to device handle
 * @param reg  Register address
 * @param val  Output: register value
 * @return 0 on success
 */
int mmc5983ma_read_register(mmc5983ma_t *dev, uint8_t reg, uint8_t *val);

/**
 * Write a raw register byte (useful for advanced configuration).
 *
 * @param dev  Pointer to device handle
 * @param reg  Register address
 * @param val  Value to write
 * @return 0 on success
 */
int mmc5983ma_write_register(mmc5983ma_t *dev, uint8_t reg, uint8_t val);

#endif /* MMC5983MA_H */
