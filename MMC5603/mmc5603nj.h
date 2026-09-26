/**
 * @file mmc5603nj.h
 * @brief Driver for the MEMSIC MMC5603NJ 3-axis Magnetic Sensor
 *
 * Supports I2C interface. Provides functions for initialization,
 * single-shot and continuous measurements, SET/RESET degaussing,
 * temperature reading, offset calibration, and self-test.
 *
 * Datasheet: MEMSIC MMC5603NJ Rev B (1/17/2022)
 *
 * Key differences vs MMC5983MA:
 *  - ±30 G FSR (vs ±8 G)
 *  - 20-bit output (vs 18-bit); separate Xout2/Yout2/Zout2 registers
 *  - 1.62V–3.6V supply (vs 3.0V), 1.2V logic IO
 *  - ODR set via dedicated ODR register (1–255); 1000Hz via hpower bit
 *  - Cmm_freq_en must be written before Cmm_en
 *  - X/Y/Z channels measured sequentially (not in parallel)
 *  - On-chip self-test with threshold registers
 *  - Software reset takes ~20ms (vs ~10ms)
 */

#ifndef MMC5603NJ_H
#define MMC5603NJ_H

#include <stdint.h>
#include <stdbool.h>

/* ─── I2C Address ─────────────────────────────────────────────────────────── */
#define MMC5603NJ_I2C_ADDR          0x30    /**< 7-bit address: 0110000 */

/* ─── Register Addresses ──────────────────────────────────────────────────── */
#define MMC5603NJ_REG_XOUT0         0x00    /**< X output bits [19:12]         */
#define MMC5603NJ_REG_XOUT1         0x01    /**< X output bits [11:4]          */
#define MMC5603NJ_REG_YOUT0         0x02    /**< Y output bits [19:12]         */
#define MMC5603NJ_REG_YOUT1         0x03    /**< Y output bits [11:4]          */
#define MMC5603NJ_REG_ZOUT0         0x04    /**< Z output bits [19:12]         */
#define MMC5603NJ_REG_ZOUT1         0x05    /**< Z output bits [11:4]          */
#define MMC5603NJ_REG_XOUT2         0x06    /**< X output bits [3:0]           */
#define MMC5603NJ_REG_YOUT2         0x07    /**< Y output bits [3:0]           */
#define MMC5603NJ_REG_ZOUT2         0x08    /**< Z output bits [3:0]           */
#define MMC5603NJ_REG_TOUT          0x09    /**< Temperature output            */
#define MMC5603NJ_REG_STATUS1       0x18    /**< Device status 1               */
#define MMC5603NJ_REG_ODR           0x1A    /**< Output data rate (1–255)      */
#define MMC5603NJ_REG_CTRL0         0x1B    /**< Internal control register 0   */
#define MMC5603NJ_REG_CTRL1         0x1C    /**< Internal control register 1   */
#define MMC5603NJ_REG_CTRL2         0x1D    /**< Internal control register 2   */
#define MMC5603NJ_REG_ST_X_TH       0x1E    /**< X self-test threshold         */
#define MMC5603NJ_REG_ST_Y_TH       0x1F    /**< Y self-test threshold         */
#define MMC5603NJ_REG_ST_Z_TH       0x20    /**< Z self-test threshold         */
#define MMC5603NJ_REG_ST_X          0x27    /**< X self-test factory value     */
#define MMC5603NJ_REG_ST_Y          0x28    /**< Y self-test factory value     */
#define MMC5603NJ_REG_ST_Z          0x29    /**< Z self-test factory value     */
#define MMC5603NJ_REG_PRODUCT_ID    0x39    /**< Product ID (expected: 0x10)   */

/* ─── Status Register 1 Bits ─────────────────────────────────────────────── */
#define MMC5603NJ_STATUS_MEAS_M_DONE    (1 << 6)  /**< Magnetic meas complete  */
#define MMC5603NJ_STATUS_MEAS_T_DONE    (1 << 7)  /**< Temperature meas done   */
#define MMC5603NJ_STATUS_OTP_RD_DONE    (1 << 4)  /**< OTP read complete       */
#define MMC5603NJ_STATUS_SAT_SENSOR     (1 << 5)  /**< Self-test saturation    */

/* ─── Control Register 0 Bits (0x1B) ─────────────────────────────────────── */
#define MMC5603NJ_CTRL0_TAKE_MEAS_M     (1 << 0)  /**< Take magnetic meas      */
#define MMC5603NJ_CTRL0_TAKE_MEAS_T     (1 << 1)  /**< Take temperature meas   */
#define MMC5603NJ_CTRL0_DO_SET          (1 << 3)  /**< Perform SET operation   */
#define MMC5603NJ_CTRL0_DO_RESET        (1 << 4)  /**< Perform RESET operation */
#define MMC5603NJ_CTRL0_AUTO_SR_EN      (1 << 5)  /**< Auto SET/RESET enable   */
#define MMC5603NJ_CTRL0_AUTO_ST_EN      (1 << 6)  /**< Auto self-test enable   */
#define MMC5603NJ_CTRL0_CMM_FREQ_EN     (1 << 7)  /**< Calc CMM period (write before Cmm_en) */

/* ─── Control Register 1 Bits (0x1C) ─────────────────────────────────────── */
#define MMC5603NJ_CTRL1_SW_RST          (1 << 7)  /**< Software reset          */
#define MMC5603NJ_CTRL1_ST_ENM          (1 << 6)  /**< Self-test negative      */
#define MMC5603NJ_CTRL1_ST_ENP          (1 << 5)  /**< Self-test positive      */
#define MMC5603NJ_CTRL1_Z_INHIBIT       (1 << 4)  /**< Disable Z axis          */
#define MMC5603NJ_CTRL1_Y_INHIBIT       (1 << 3)  /**< Disable Y axis          */
#define MMC5603NJ_CTRL1_X_INHIBIT       (1 << 2)  /**< Disable X axis          */

/* ─── Control Register 2 Bits (0x1D) ─────────────────────────────────────── */
#define MMC5603NJ_CTRL2_CMM_EN          (1 << 4)  /**< Enable continuous mode  */
#define MMC5603NJ_CTRL2_EN_PRD_SET      (1 << 3)  /**< Enable periodic SET     */
#define MMC5603NJ_CTRL2_HPOWER          (1 << 7)  /**< High power (for 1000Hz) */

/* ─── Sensor Constants ────────────────────────────────────────────────────── */
#define MMC5603NJ_PRODUCT_ID            0x10      /**< Expected product ID      */
#define MMC5603NJ_NULL_FIELD_20BIT      524288UL  /**< Zero-field output (20b)  */
#define MMC5603NJ_NULL_FIELD_18BIT      131072UL  /**< Zero-field output (18b)  */
#define MMC5603NJ_NULL_FIELD_16BIT      32768U    /**< Zero-field output (16b)  */
#define MMC5603NJ_SENSITIVITY_20BIT     16384.0f  /**< Counts per Gauss (20b)   */
#define MMC5603NJ_SENSITIVITY_18BIT     4096.0f   /**< Counts per Gauss (18b)   */
#define MMC5603NJ_SENSITIVITY_16BIT     1024.0f   /**< Counts per Gauss (16b)   */
#define MMC5603NJ_TEMP_OFFSET           (-75.0f)  /**< Temp sensor zero point   */
#define MMC5603NJ_TEMP_SCALE            (0.8f)    /**< Degrees C per LSB        */

/* ─── Output Resolution Mode ─────────────────────────────────────────────── */
typedef enum {
    MMC5603NJ_RES_16BIT = 0,  /**< Use Xout[19:4]  — 16 significant bits */
    MMC5603NJ_RES_18BIT = 1,  /**< Use Xout[19:2]  — 18 significant bits */
    MMC5603NJ_RES_20BIT = 2,  /**< Use Xout[19:0]  — 20 significant bits */
} mmc5603nj_res_t;

/* ─── Bandwidth / Measurement Time Settings ───────────────────────────────── */
typedef enum {
    MMC5603NJ_BW_6_6MS  = 0x00,  /**< 6.6ms meas time, max 75Hz   (BW=00) */
    MMC5603NJ_BW_3_5MS  = 0x01,  /**< 3.5ms meas time, max 150Hz  (BW=01) */
    MMC5603NJ_BW_2_0MS  = 0x02,  /**< 2.0ms meas time, max 255Hz  (BW=10) */
    MMC5603NJ_BW_1_2MS  = 0x03,  /**< 1.2ms meas time, max 1000Hz (BW=11) */
} mmc5603nj_bw_t;

/* ─── Periodic SET Interval ───────────────────────────────────────────────── */
typedef enum {
    MMC5603NJ_PRD_SET_1    = 0x00,  /**< SET every 1    sample  */
    MMC5603NJ_PRD_SET_25   = 0x01,  /**< SET every 25   samples */
    MMC5603NJ_PRD_SET_75   = 0x02,  /**< SET every 75   samples */
    MMC5603NJ_PRD_SET_100  = 0x03,  /**< SET every 100  samples */
    MMC5603NJ_PRD_SET_250  = 0x04,  /**< SET every 250  samples */
    MMC5603NJ_PRD_SET_500  = 0x05,  /**< SET every 500  samples */
    MMC5603NJ_PRD_SET_1000 = 0x06,  /**< SET every 1000 samples */
    MMC5603NJ_PRD_SET_2000 = 0x07,  /**< SET every 2000 samples */
} mmc5603nj_prd_set_t;

/* ─── Raw Magnetic Data ───────────────────────────────────────────────────── */
typedef struct {
    uint32_t x;   /**< Raw X output (up to 20-bit unsigned) */
    uint32_t y;   /**< Raw Y output (up to 20-bit unsigned) */
    uint32_t z;   /**< Raw Z output (up to 20-bit unsigned) */
} mmc5603nj_raw_t;

/* ─── Calibrated Magnetic Data (in Gauss) ────────────────────────────────── */
typedef struct {
    float x;
    float y;
    float z;
} mmc5603nj_data_t;

/* ─── Stored Offset (for bridge offset removal) ──────────────────────────── */
typedef struct {
    uint32_t x;
    uint32_t y;
    uint32_t z;
} mmc5603nj_offset_t;

/* ─── Device Handle ───────────────────────────────────────────────────────── */
typedef struct {
    uint8_t              i2c_addr;      /**< I2C address (default 0x30)   */
    mmc5603nj_bw_t       bandwidth;     /**< Active bandwidth setting      */
    mmc5603nj_res_t      resolution;    /**< Output resolution mode        */
    mmc5603nj_offset_t   offset;        /**< Stored bridge offset          */
    bool                 offset_valid;  /**< Whether offset has been set   */
} mmc5603nj_t;

/* ─── HAL Callbacks (implement these for your platform) ───────────────────── */

/**
 * Write bytes over I2C.
 * @param addr   7-bit I2C device address
 * @param reg    Register address to write to
 * @param data   Pointer to data buffer
 * @param len    Number of bytes to write
 * @return 0 on success, non-zero on error
 */
extern int mmc5603nj_hal_i2c_write(uint8_t addr, uint8_t reg,
                                   const uint8_t *data, uint16_t len);

/**
 * Read bytes over I2C.
 * @param addr   7-bit I2C device address
 * @param reg    Register address to read from
 * @param data   Pointer to receive buffer
 * @param len    Number of bytes to read
 * @return 0 on success, non-zero on error
 */
extern int mmc5603nj_hal_i2c_read(uint8_t addr, uint8_t reg,
                                  uint8_t *data, uint16_t len);

/**
 * Delay for the given number of milliseconds.
 * @param ms  Milliseconds to delay
 */
extern void mmc5603nj_hal_delay_ms(uint32_t ms);

/* ─── Public API ──────────────────────────────────────────────────────────── */

/**
 * Initialize the device handle with default settings.
 * Call this before any other function.
 *
 * @param dev         Pointer to device handle to initialize
 * @param resolution  Output resolution (16, 18, or 20 bit)
 * @param bw          Bandwidth / measurement time setting
 */
void mmc5603nj_init_handle(mmc5603nj_t *dev, mmc5603nj_res_t resolution,
                            mmc5603nj_bw_t bw);

/**
 * Initialize and verify the sensor over I2C.
 * Performs a software reset, reads the product ID, and sets bandwidth.
 *
 * @param dev  Pointer to initialized device handle
 * @return 0 on success, -1 if product ID mismatch or communication error
 */
int mmc5603nj_init(mmc5603nj_t *dev);

/**
 * Perform a software reset (clears all registers, re-reads OTP).
 * The datasheet specifies a 20ms power-on time after reset.
 *
 * @param dev  Pointer to device handle
 * @return 0 on success
 */
int mmc5603nj_reset(mmc5603nj_t *dev);

/**
 * Read the product ID register.
 * Expected value is 0x10.
 *
 * @param dev  Pointer to device handle
 * @param id   Output: product ID byte
 * @return 0 on success
 */
int mmc5603nj_read_product_id(mmc5603nj_t *dev, uint8_t *id);

/**
 * Perform the SET operation (magnetizes sensing elements in SET direction).
 * Wait at least 1ms before the next operation (tSR).
 *
 * @param dev  Pointer to device handle
 * @return 0 on success
 */
int mmc5603nj_set(mmc5603nj_t *dev);

/**
 * Perform the RESET operation (magnetizes sensing elements opposite to SET).
 * Wait at least 1ms before the next operation (tSR).
 *
 * @param dev  Pointer to device handle
 * @return 0 on success
 */
int mmc5603nj_reset_coil(mmc5603nj_t *dev);

/**
 * Take a single magnetic field measurement (blocking).
 * Polls Meas_m_done, then reads all output registers in one burst.
 *
 * @param dev  Pointer to device handle
 * @param raw  Output: raw unsigned counts (16/18/20-bit depending on resolution)
 * @return 0 on success, -1 on timeout
 */
int mmc5603nj_read_raw(mmc5603nj_t *dev, mmc5603nj_raw_t *raw);

/**
 * Take a single magnetic field measurement and convert to Gauss.
 * If offset calibration has been performed, it is applied automatically.
 *
 * @param dev   Pointer to device handle
 * @param data  Output: magnetic field in Gauss for each axis
 * @return 0 on success, -1 on error
 */
int mmc5603nj_read_gauss(mmc5603nj_t *dev, mmc5603nj_data_t *data);

/**
 * Take a temperature measurement (blocking).
 * Range: -75°C to +125°C, ~0.8°C per LSB.
 *
 * @param dev      Pointer to device handle
 * @param temp_c   Output: temperature in degrees Celsius
 * @return 0 on success, -1 on timeout
 */
int mmc5603nj_read_temperature(mmc5603nj_t *dev, float *temp_c);

/**
 * Calibrate bridge offset using the SET/RESET subtraction method.
 *
 * Performs SET → measure → RESET → measure, stores the average as offset.
 * Subsequent calls to mmc5603nj_read_gauss() use this offset automatically.
 *
 * Formula: offset = (output_set + output_reset) / 2
 *          field  = (output_set - output_reset) / 2
 *
 * @param dev  Pointer to device handle
 * @return 0 on success, -1 on error
 */
int mmc5603nj_calibrate_offset(mmc5603nj_t *dev);

/**
 * Enable or disable automatic SET/RESET before each measurement.
 * Recommended to keep enabled (bit Auto_SR_en in CTRL0).
 *
 * @param dev     Pointer to device handle
 * @param enable  true to enable, false to disable
 * @return 0 on success
 */
int mmc5603nj_set_auto_sr(mmc5603nj_t *dev, bool enable);

/**
 * Configure and start continuous measurement mode.
 *
 * Sequence per datasheet:
 *   1. Write non-zero ODR value to ODR register
 *   2. Write Cmm_freq_en=1 to CTRL0 (triggers internal period calculation)
 *   3. Write Cmm_en=1 to CTRL2
 *
 * @param dev      Pointer to device handle
 * @param odr      Output data rate in Hz (1–255; use 255 + hpower for 1000Hz)
 * @param bw       Bandwidth setting
 * @param auto_sr  Enable automatic SET/RESET
 * @param hpower   Set true to enable 1000Hz mode (requires odr=255, bw=BW_1_2MS)
 * @return 0 on success
 */
int mmc5603nj_start_continuous(mmc5603nj_t *dev, uint8_t odr,
                                mmc5603nj_bw_t bw, bool auto_sr, bool hpower);

/**
 * Stop continuous measurement mode (clears Cmm_en in CTRL2).
 *
 * @param dev  Pointer to device handle
 * @return 0 on success
 */
int mmc5603nj_stop_continuous(mmc5603nj_t *dev);

/**
 * Check whether a magnetic measurement result is ready to be read.
 *
 * @param dev   Pointer to device handle
 * @param ready Output: true if Meas_m_done bit is set
 * @return 0 on success
 */
int mmc5603nj_is_data_ready(mmc5603nj_t *dev, bool *ready);

/**
 * Set the measurement bandwidth (decimation filter duration).
 *
 * @param dev  Pointer to device handle
 * @param bw   Bandwidth selection
 * @return 0 on success
 */
int mmc5603nj_set_bandwidth(mmc5603nj_t *dev, mmc5603nj_bw_t bw);

/**
 * Enable periodic automatic SET during continuous mode.
 * Requires Auto_SR_en and Cmm_en to also be active.
 *
 * @param dev      Pointer to device handle
 * @param interval How often to perform a SET (every N measurements)
 * @return 0 on success
 */
int mmc5603nj_enable_periodic_set(mmc5603nj_t *dev,
                                  mmc5603nj_prd_set_t interval);

/**
 * Run the on-chip self-test.
 *
 * Reads the factory self-test reference values (ST_X/Y/Z), sets thresholds
 * at 80% of those values, triggers auto_st_en, then checks Sat_sensor.
 *
 * @param dev    Pointer to device handle
 * @param passed Output: true if self-test passed (Sat_sensor = 0)
 * @return 0 on success (communication OK), -1 on I2C error
 */
int mmc5603nj_selftest(mmc5603nj_t *dev, bool *passed);

/**
 * Read a raw register byte (useful for debugging).
 *
 * @param dev  Pointer to device handle
 * @param reg  Register address
 * @param val  Output: register value
 * @return 0 on success
 */
int mmc5603nj_read_register(mmc5603nj_t *dev, uint8_t reg, uint8_t *val);

/**
 * Write a raw register byte (useful for advanced configuration).
 *
 * @param dev  Pointer to device handle
 * @param reg  Register address
 * @param val  Value to write
 * @return 0 on success
 */
int mmc5603nj_write_register(mmc5603nj_t *dev, uint8_t reg, uint8_t val);

#endif /* MMC5603NJ_H */
