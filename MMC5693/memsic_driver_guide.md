# MEMSIC Magnetometer Driver Guide

Covers the **MMC5983MA** and **MMC5603NJ** C drivers. Both follow the same
design pattern — a device handle, three HAL callbacks you implement, and a set
of functions that build on top of them.

---

## Table of Contents

1. [Quick Comparison](#1-quick-comparison)
2. [Architecture Overview](#2-architecture-overview)
3. [HAL Callbacks — What You Must Implement](#3-hal-callbacks--what-you-must-implement)
4. [MMC5983MA Driver Reference](#4-mmc5983ma-driver-reference)
   - [Device Handle & Enums](#41-device-handle--enums)
   - [Initialization](#42-initialization)
   - [SET / RESET Degaussing](#43-set--reset-degaussing)
   - [Single-Shot Measurements](#44-single-shot-measurements)
   - [Offset Calibration](#45-offset-calibration)
   - [Continuous Measurement Mode](#46-continuous-measurement-mode)
   - [Configuration Helpers](#47-configuration-helpers)
   - [Raw Register Access](#48-raw-register-access)
5. [MMC5603NJ Driver Reference](#5-mmc5603nj-driver-reference)
   - [Device Handle & Enums](#51-device-handle--enums)
   - [Initialization](#52-initialization)
   - [SET / RESET Degaussing](#53-set--reset-degaussing)
   - [Single-Shot Measurements](#54-single-shot-measurements)
   - [Offset Calibration](#55-offset-calibration)
   - [Continuous Measurement Mode](#56-continuous-measurement-mode)
   - [Self-Test](#57-self-test)
   - [Configuration Helpers](#58-configuration-helpers)
   - [Raw Register Access](#59-raw-register-access)
6. [Return Values & Error Handling](#6-return-values--error-handling)
7. [Typical Usage Flows](#7-typical-usage-flows)
8. [Key Differences Between the Two Sensors](#8-key-differences-between-the-two-sensors)

---

## 1. Quick Comparison

| Feature               | MMC5983MA              | MMC5603NJ               |
|-----------------------|------------------------|-------------------------|
| Field range           | ±8 G                   | ±30 G                   |
| Max resolution        | 18-bit                 | 20-bit                  |
| Sensitivity (max res) | 16384 counts/G         | 16384 counts/G          |
| Supply voltage        | 2.8 V – 3.6 V          | 1.62 V – 3.6 V          |
| I2C address (7-bit)   | 0x30                   | 0x30                    |
| Max ODR               | 1000 Hz                | 1000 Hz                 |
| ODR configuration     | Enum (fixed presets)   | Register (1–255 Hz)     |
| Self-test             | No                     | Yes (on-chip)           |
| RMS noise (typ.)      | 0.4 mG                 | 2 mG                    |
| Reset time            | ~10 ms                 | ~20 ms                  |
| Product ID register   | 0x2F → expects 0x30    | 0x39 → expects 0x10     |

---

## 2. Architecture Overview

```
Your Application
      │
      ├── mmc5983ma_read_gauss()   (or mmc5603nj_*)
      │         │
      │   [driver internals]
      │         │
      ├── mmc5983ma_hal_i2c_read()    ← YOU implement this
      ├── mmc5983ma_hal_i2c_write()   ← YOU implement this
      └── mmc5983ma_hal_delay_ms()    ← YOU implement this
```

The driver never calls any platform API directly. All hardware access goes
through the three HAL callbacks, making the driver fully portable.

---

## 3. HAL Callbacks — What You Must Implement

Both drivers need the same three function signatures, just with a different
prefix (`mmc5983ma_` vs `mmc5603nj_`).

### `hal_i2c_write`

```c
// MMC5983MA version
int mmc5983ma_hal_i2c_write(uint8_t addr, uint8_t reg,
                             const uint8_t *data, uint16_t len);

// MMC5603NJ version
int mmc5603nj_hal_i2c_write(uint8_t addr, uint8_t reg,
                             const uint8_t *data, uint16_t len);
```

- `addr` — 7-bit I2C device address (always `0x30` for both sensors)
- `reg`  — register address to write to
- `data` — byte buffer to send
- `len`  — number of bytes in `data`
- **Returns** 0 on success, any non-zero value on failure

**STM32 HAL example:**
```c
int mmc5983ma_hal_i2c_write(uint8_t addr, uint8_t reg,
                             const uint8_t *data, uint16_t len) {
    return HAL_I2C_Mem_Write(&hi2c1, addr << 1, reg,
                             I2C_MEMADD_SIZE_8BIT,
                             (uint8_t *)data, len, HAL_MAX_DELAY);
}
```

**Arduino Wire example:**
```c
int mmc5983ma_hal_i2c_write(uint8_t addr, uint8_t reg,
                             const uint8_t *data, uint16_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    for (int i = 0; i < len; i++) Wire.write(data[i]);
    return Wire.endTransmission();
}
```

---

### `hal_i2c_read`

```c
int mmc5983ma_hal_i2c_read(uint8_t addr, uint8_t reg,
                            uint8_t *data, uint16_t len);
```

- Same parameters as write, but `data` is the receive buffer
- **Returns** 0 on success, any non-zero value on failure

---

### `hal_delay_ms`

```c
void mmc5983ma_hal_delay_ms(uint32_t ms);
```

A blocking delay in milliseconds. Used after reset and after SET/RESET pulses.

| Platform   | Implementation              |
|------------|-----------------------------|
| STM32 HAL  | `HAL_Delay(ms)`             |
| Arduino    | `delay(ms)`                 |
| FreeRTOS   | `vTaskDelay(pdMS_TO_TICKS(ms))` |
| Linux      | `usleep(ms * 1000)`         |

---

## 4. MMC5983MA Driver Reference

### 4.1 Device Handle & Enums

The device handle (`mmc5983ma_t`) stores all runtime state. You should have
one per physical sensor.

```c
typedef struct {
    uint8_t             i2c_addr;       // I2C address (default 0x30)
    mmc5983ma_bw_t      bandwidth;      // Active bandwidth setting
    bool                mode_18bit;     // true = 18-bit, false = 16-bit
    mmc5983ma_offset_t  offset;         // Stored bridge offset counts
    bool                offset_valid;   // Has calibration been run?
} mmc5983ma_t;
```

**Bandwidth enum** — controls measurement time and noise floor:

| Value                  | Meas. Time | Bandwidth | RMS Noise |
|------------------------|------------|-----------|-----------|
| `MMC5983MA_BW_100HZ`  | 8 ms       | 100 Hz    | 0.4 mG    |
| `MMC5983MA_BW_200HZ`  | 4 ms       | 200 Hz    | 0.6 mG    |
| `MMC5983MA_BW_400HZ`  | 2 ms       | 400 Hz    | 0.8 mG    |
| `MMC5983MA_BW_800HZ`  | 0.5 ms     | 800 Hz    | 1.2 mG    |

**Continuous mode frequency enum:**

| Value                   | ODR      | Notes               |
|-------------------------|----------|---------------------|
| `MMC5983MA_CM_OFF`     | Disabled |                     |
| `MMC5983MA_CM_1HZ`     | 1 Hz     |                     |
| `MMC5983MA_CM_10HZ`    | 10 Hz    |                     |
| `MMC5983MA_CM_20HZ`    | 20 Hz    |                     |
| `MMC5983MA_CM_50HZ`    | 50 Hz    |                     |
| `MMC5983MA_CM_100HZ`   | 100 Hz   |                     |
| `MMC5983MA_CM_200HZ`   | 200 Hz   | Requires BW_200HZ   |
| `MMC5983MA_CM_1000HZ`  | 1000 Hz  | Requires BW_800HZ   |

---

### 4.2 Initialization

#### `mmc5983ma_init_handle`

```c
void mmc5983ma_init_handle(mmc5983ma_t *dev, bool mode_18bit);
```

Sets up the handle in memory. **Call this first, before anything else.**

- `mode_18bit` — pass `true` for 18-bit output (0.0625 mG/LSB),
  `false` for 16-bit (0.25 mG/LSB)
- Does not communicate with the sensor

```c
mmc5983ma_t mag;
mmc5983ma_init_handle(&mag, true);  // 18-bit mode
```

---

#### `mmc5983ma_init`

```c
int mmc5983ma_init(mmc5983ma_t *dev);
```

Performs the full startup sequence over I2C:
1. Software reset (clears all registers, re-reads OTP)
2. Waits 15 ms for reset to complete
3. Reads product ID register — returns `-1` if it doesn't read back `0x30`
4. Writes the bandwidth from the handle to CTRL1

**Returns** 0 on success, -1 on any error. Always check this return value.

```c
if (mmc5983ma_init(&mag) != 0) {
    // Sensor not found or I2C error
}
```

---

#### `mmc5983ma_reset`

```c
int mmc5983ma_reset(mmc5983ma_t *dev);
```

Triggers a software reset by writing `SW_RST` to CTRL1. The sensor takes
~10 ms to recover. You usually don't need to call this manually since
`mmc5983ma_init()` calls it for you.

---

#### `mmc5983ma_read_product_id`

```c
int mmc5983ma_read_product_id(mmc5983ma_t *dev, uint8_t *id);
```

Reads register `0x2F`. Expected value is `0x30`. Useful for confirming the
sensor is present on the bus before doing anything else.

---

### 4.3 SET / RESET Degaussing

The AMR sensing elements can become permanently polarized by strong external
fields (>10 G). SET and RESET apply a brief high-current pulse through an
on-chip coil to restore the sensor to a known state.

#### `mmc5983ma_set`

```c
int mmc5983ma_set(mmc5983ma_t *dev);
```

Applies the SET pulse (500 ns), then waits 1 ms. After a SET, the sensor's
magnetic domains point in the SET direction. Use before measurements when
you want a known baseline.

#### `mmc5983ma_reset_coil`

```c
int mmc5983ma_reset_coil(mmc5983ma_t *dev);
```

Applies the RESET pulse (500 ns), then waits 1 ms. Magnetizes elements in
the opposite direction to SET. Used in combination with SET for offset
calibration (see section 4.5).

> **Note:** `mmc5983ma_reset_coil()` controls the magnetic coil, not the
> chip's electronics. It is different from `mmc5983ma_reset()` which does a
> software register reset.

---

### 4.4 Single-Shot Measurements

#### `mmc5983ma_read_raw`

```c
int mmc5983ma_read_raw(mmc5983ma_t *dev, mmc5983ma_raw_t *raw);
```

Triggers one measurement, waits for `Meas_M_Done`, then burst-reads all
7 output registers (0x00–0x06) in a single I2C transaction.

Output struct:
```c
typedef struct {
    uint32_t x;   // Raw unsigned counts
    uint32_t y;
    uint32_t z;
} mmc5983ma_raw_t;
```

In 18-bit mode, values range from 0 to 262143. In 16-bit mode, 0 to 65535.
Zero-field output is at the midpoint (131072 / 32768 respectively).

**Timeout:** 20 ms. Returns -1 if `Meas_M_Done` never sets.

---

#### `mmc5983ma_read_gauss`

```c
int mmc5983ma_read_gauss(mmc5983ma_t *dev, mmc5983ma_data_t *data);
```

Calls `read_raw` internally, then converts to signed Gauss using:

```
field_G = (raw_count - offset) / sensitivity
```

Where `offset` is from `calibrate_offset()` if run, otherwise the fixed
null-field midpoint. `sensitivity` is 16384 counts/G (18-bit) or
4096 counts/G (16-bit).

Output struct:
```c
typedef struct {
    float x;   // Gauss
    float y;
    float z;
} mmc5983ma_data_t;
```

This is the function you will use most of the time.

---

#### `mmc5983ma_read_temperature`

```c
int mmc5983ma_read_temperature(mmc5983ma_t *dev, float *temp_c);
```

Triggers a temperature measurement (separate from magnetic), waits for
`Meas_T_Done`, reads the `TOUT` register, and converts:

```
T(°C) = -75 + count × 0.8
```

Range: -75 °C to +125 °C. Resolution: ~0.8 °C/LSB.

> Temperature and magnetic measurements **cannot** run simultaneously.
> The driver handles this by only ever setting one trigger bit at a time.

---

### 4.5 Offset Calibration

#### `mmc5983ma_calibrate_offset`

```c
int mmc5603nj_calibrate_offset(mmc5603nj_t *dev);
```

Runs the SET/RESET subtraction method to measure and store the bridge offset.
This removes the error caused by the sensor's own null-field output drifting
with temperature.

**What it does internally:**
1. SET → measure → store as `raw_set`
2. RESET → measure → store as `raw_reset`
3. Calculates: `offset = (raw_set + raw_reset) / 2`
4. Stores result in `dev->offset`
5. Performs a final SET to restore the sensor

**After calibration**, `mmc5983ma_read_gauss()` automatically uses the stored
offset instead of the fixed null-field constant.

**When to call it:**
- Once at startup after `init()`
- Again if operating temperature changes significantly (>10–20 °C shift)
- After exposure to a strong external magnetic field

```c
mmc5983ma_calibrate_offset(&mag);
// dev->offset.x / .y / .z now hold calibrated counts
// dev->offset_valid == true
```

---

### 4.6 Continuous Measurement Mode

#### `mmc5983ma_start_continuous`

```c
int mmc5983ma_start_continuous(mmc5983ma_t *dev,
                                mmc5983ma_cm_freq_t freq,
                                mmc5983ma_bw_t bw,
                                bool auto_sr);
```

Configures and starts continuous measurement mode. The sensor will take
measurements at the specified rate without needing a trigger each time.

- `freq`    — output data rate (see enum table in 4.1)
- `bw`      — bandwidth setting written to CTRL1
- `auto_sr` — if `true`, the chip automatically performs SET/RESET before
              each measurement (recommended; adds ~1 ms per measurement)

**In continuous mode**, poll `mmc5983ma_is_data_ready()` or use the INT pin
to know when data is available, then call `mmc5983ma_read_gauss()`.

```c
mmc5983ma_start_continuous(&mag, MMC5983MA_CM_100HZ, MMC5983MA_BW_200HZ, true);
```

---

#### `mmc5983ma_stop_continuous`

```c
int mmc5983ma_stop_continuous(mmc5983ma_t *dev);
```

Clears `Cmm_en` in CTRL2, returning the sensor to on-demand mode.

---

#### `mmc5983ma_is_data_ready`

```c
int mmc5983ma_is_data_ready(mmc5983ma_t *dev, bool *ready);
```

Reads the Status register and sets `*ready = true` if `Meas_M_Done` (bit 0)
is set. Use this to poll in continuous mode.

```c
bool ready = false;
while (!ready) {
    mmc5983ma_is_data_ready(&mag, &ready);
    delay_ms(1);
}
mmc5983ma_read_gauss(&mag, &field);
```

---

### 4.7 Configuration Helpers

#### `mmc5983ma_set_auto_sr`

```c
int mmc5983ma_set_auto_sr(mmc5983ma_t *dev, bool enable);
```

Writes the `Auto_SR_en` bit in CTRL0. When enabled, the chip automatically
performs a SET before each measurement. Recommended for most applications.

---

#### `mmc5983ma_set_bandwidth`

```c
int mmc5983ma_set_bandwidth(mmc5983ma_t *dev, mmc5983ma_bw_t bw);
```

Updates the BW bits in CTRL1 and stores the new value in the handle.
Wider bandwidth = faster measurement, more noise.

---

#### `mmc5983ma_enable_interrupt`

```c
int mmc5983ma_enable_interrupt(mmc5983ma_t *dev, bool enable);
```

Sets `INT_meas_done_en` in CTRL0. When enabled, the INT pin goes high after
each completed measurement (magnetic or temperature). The INT pin is
active-high and held hi-Z when interrupts are disabled.

---

#### `mmc5983ma_enable_periodic_set`

```c
int mmc5983ma_enable_periodic_set(mmc5983ma_t *dev,
                                  mmc5983ma_prd_set_t interval);
```

In continuous mode, automatically performs a SET every N measurements.
Helps correct for slow thermal drift without manually calling SET.

Requires both `Auto_SR_en` (CTRL0) and `Cmm_en` (CTRL2) to be active first.

| Value                     | SET interval       |
|---------------------------|--------------------|
| `MMC5983MA_PRD_SET_1`    | Every measurement  |
| `MMC5983MA_PRD_SET_25`   | Every 25           |
| `MMC5983MA_PRD_SET_75`   | Every 75           |
| `MMC5983MA_PRD_SET_100`  | Every 100          |
| `MMC5983MA_PRD_SET_250`  | Every 250          |
| `MMC5983MA_PRD_SET_500`  | Every 500          |
| `MMC5983MA_PRD_SET_1000` | Every 1000         |
| `MMC5983MA_PRD_SET_2000` | Every 2000         |

---

### 4.8 Raw Register Access

```c
int mmc5983ma_read_register(mmc5983ma_t *dev, uint8_t reg, uint8_t *val);
int mmc5983ma_write_register(mmc5983ma_t *dev, uint8_t reg, uint8_t val);
```

Direct register read/write for debugging or features not exposed by the API.
Register addresses are defined as `MMC5983MA_REG_*` macros in the header.

---

## 5. MMC5603NJ Driver Reference

The MMC5603NJ driver follows the exact same pattern as the MMC5983MA driver.
This section covers the differences and MMC5603NJ-specific features.

### 5.1 Device Handle & Enums

```c
typedef struct {
    uint8_t              i2c_addr;      // I2C address (default 0x30)
    mmc5603nj_bw_t       bandwidth;     // Active bandwidth setting
    mmc5603nj_res_t      resolution;    // Output resolution (16/18/20-bit)
    mmc5603nj_offset_t   offset;        // Stored bridge offset counts
    bool                 offset_valid;  // Has calibration been run?
} mmc5603nj_t;
```

**Resolution enum** (unique to MMC5603NJ — the MMC5983MA only supports 16/18):

| Value                   | Bits used    | Sensitivity     | Null-field count |
|-------------------------|--------------|-----------------|------------------|
| `MMC5603NJ_RES_16BIT`  | X[19:4]      | 1024 counts/G   | 32768            |
| `MMC5603NJ_RES_18BIT`  | X[19:2]      | 4096 counts/G   | 131072           |
| `MMC5603NJ_RES_20BIT`  | X[19:0]      | 16384 counts/G  | 524288           |

**Bandwidth enum:**

| Value                   | Meas. Time | Max ODR  |
|-------------------------|------------|----------|
| `MMC5603NJ_BW_6_6MS`  | 6.6 ms     | 75 Hz    |
| `MMC5603NJ_BW_3_5MS`  | 3.5 ms     | 150 Hz   |
| `MMC5603NJ_BW_2_0MS`  | 2.0 ms     | 255 Hz   |
| `MMC5603NJ_BW_1_2MS`  | 1.2 ms     | 1000 Hz  |

---

### 5.2 Initialization

#### `mmc5603nj_init_handle`

```c
void mmc5603nj_init_handle(mmc5603nj_t *dev,
                            mmc5603nj_res_t resolution,
                            mmc5603nj_bw_t bw);
```

Same as the MMC5983MA version but takes both a resolution and bandwidth
instead of just a mode flag. **Call this first.**

```c
mmc5603nj_t mag;
mmc5603nj_init_handle(&mag, MMC5603NJ_RES_20BIT, MMC5603NJ_BW_6_6MS);
```

---

#### `mmc5603nj_init`

```c
int mmc5603nj_init(mmc5603nj_t *dev);
```

Same flow as the MMC5983MA: reset → wait (25 ms for MMC5603NJ) → check
product ID (expects `0x10` at register `0x39`) → set bandwidth.

---

### 5.3 SET / RESET Degaussing

```c
int mmc5603nj_set(mmc5603nj_t *dev);
int mmc5603nj_reset_coil(mmc5603nj_t *dev);
```

Functionally identical to the MMC5983MA versions. Both apply a 1 ms wait
after the pulse (tSR timing requirement).

---

### 5.4 Single-Shot Measurements

#### `mmc5603nj_read_raw`

```c
int mmc5603nj_read_raw(mmc5603nj_t *dev, mmc5603nj_raw_t *raw);
```

Triggers a measurement with `Auto_SR_en | Take_meas_M`, waits for
`Meas_m_done` (bit 6 of Status1), then burst-reads 9 registers (0x00–0x08).

The 9 registers break down as:

```
Reg 0x00 (Xout0): X bits [19:12]
Reg 0x01 (Xout1): X bits [11:4]
Reg 0x02 (Yout0): Y bits [19:12]
Reg 0x03 (Yout1): Y bits [11:4]
Reg 0x04 (Zout0): Z bits [19:12]
Reg 0x05 (Zout1): Z bits [11:4]
Reg 0x06 (Xout2): X bits [3:0] in upper nibble [7:4]
Reg 0x07 (Yout2): Y bits [3:0] in upper nibble [7:4]
Reg 0x08 (Zout2): Z bits [3:0] in upper nibble [7:4]
```

The driver assembles the correct number of bits based on the resolution in
the handle. In 16-bit mode only Xout0/1 are used; in 20-bit mode all 9
registers contribute.

> **Important:** Reading any output register (0x00–0x08) automatically clears
> `Meas_m_done`. You do not need to clear it manually.

---

#### `mmc5603nj_read_gauss`

```c
int mmc5603nj_read_gauss(mmc5603nj_t *dev, mmc5603nj_data_t *data);
```

Identical in behaviour to the MMC5983MA version — calls `read_raw` then
converts using the null-field offset and sensitivity for the active resolution.

---

#### `mmc5603nj_read_temperature`

```c
int mmc5603nj_read_temperature(mmc5603nj_t *dev, float *temp_c);
```

Same formula as MMC5983MA: `T(°C) = -75 + count × 0.8`. Same restriction
applies — cannot run simultaneously with a magnetic measurement.

---

### 5.5 Offset Calibration

#### `mmc5603nj_calibrate_offset`

```c
int mmc5603nj_calibrate_offset(mmc5603nj_t *dev);
```

Identical procedure to the MMC5983MA version. SET → measure → RESET →
measure → store average as offset. After calling this, `read_gauss()` uses
the stored offset automatically.

---

### 5.6 Continuous Measurement Mode

#### `mmc5603nj_start_continuous`

```c
int mmc5603nj_start_continuous(mmc5603nj_t *dev,
                                uint8_t odr,
                                mmc5603nj_bw_t bw,
                                bool auto_sr,
                                bool hpower);
```

The MMC5603NJ ODR is a **direct Hz value** (1–255) written to a dedicated
register, rather than a fixed preset enum. The strict startup sequence
required by the datasheet is:

```
1. Write ODR value  → register 0x1A
2. Write Cmm_freq_en = 1  → CTRL0 (triggers internal period calculation)
3. Write Cmm_en = 1  → CTRL2
```

The driver enforces this order. Do not write these registers manually while
in continuous mode.

- `odr`     — desired rate in Hz (1–255). For 1000 Hz, use 255 + `hpower=true`
- `bw`      — must be compatible with ODR (e.g., BW_1_2MS for >255 Hz)
- `auto_sr` — auto SET/RESET per measurement
- `hpower`  — set `true` only for 1000 Hz mode; also requires `bw=BW_1_2MS`

```c
// 50 Hz, 2ms measurement time, auto SET/RESET, normal power
mmc5603nj_start_continuous(&mag, 50, MMC5603NJ_BW_2_0MS, true, false);

// 1000 Hz high-power mode
mmc5603nj_start_continuous(&mag, 255, MMC5603NJ_BW_1_2MS, true, true);
```

---

#### `mmc5603nj_stop_continuous`

```c
int mmc5603nj_stop_continuous(mmc5603nj_t *dev);
```

Clears CTRL2, stopping the continuous mode engine.

---

#### `mmc5603nj_is_data_ready`

```c
int mmc5603nj_is_data_ready(mmc5603nj_t *dev, bool *ready);
```

Reads Status1 and checks `Meas_m_done` (bit 6). Same usage pattern as the
MMC5983MA version.

---

### 5.7 Self-Test

The MMC5603NJ has an on-chip self-test not present on the MMC5983MA.

#### `mmc5603nj_selftest`

```c
int mmc5603nj_selftest(mmc5603nj_t *dev, bool *passed);
```

**What it does:**
1. Reads factory self-test reference values from registers `ST_X`, `ST_Y`, `ST_Z`
2. Writes thresholds at 80% of those values to `ST_X_TH`, `ST_Y_TH`, `ST_Z_TH`
3. Triggers a measurement with `Auto_st_en | Take_meas_M`
4. Checks the `Sat_sensor` bit in Status1 after measurement completes

- `*passed = true` means `Sat_sensor = 0` — the sensor response exceeded 80%
  of its factory value, confirming the sensing elements are functioning
- `*passed = false` means the sensor response was too low — possible
  contamination, damage, or strong permanent magnetization

Run this at startup before any measurements, ideally before `calibrate_offset()`.

```c
bool ok = false;
mmc5603nj_selftest(&mag, &ok);
if (!ok) {
    // Sensor may be damaged or saturated — perform SET/RESET and retry
    mmc5603nj_set(&mag);
    mmc5603nj_selftest(&mag, &ok);
}
```

---

### 5.8 Configuration Helpers

```c
int mmc5603nj_set_auto_sr(mmc5603nj_t *dev, bool enable);
int mmc5603nj_set_bandwidth(mmc5603nj_t *dev, mmc5603nj_bw_t bw);
int mmc5603nj_enable_periodic_set(mmc5603nj_t *dev, mmc5603nj_prd_set_t interval);
```

All three work identically to the MMC5983MA counterparts. Periodic SET
requires continuous mode and Auto_SR_en to already be active.

---

### 5.9 Raw Register Access

```c
int mmc5603nj_read_register(mmc5603nj_t *dev, uint8_t reg, uint8_t *val);
int mmc5603nj_write_register(mmc5603nj_t *dev, uint8_t reg, uint8_t val);
```

Direct register access for debugging. All register addresses are defined as
`MMC5603NJ_REG_*` macros in the header.

---

## 6. Return Values & Error Handling

Every function that communicates with the sensor returns an `int`:

| Return value | Meaning                                              |
|--------------|------------------------------------------------------|
| `0`          | Success                                              |
| `-1`         | I2C error, product ID mismatch, or measurement timeout |

The driver propagates HAL return values, so any non-zero value from your
`hal_i2c_read` / `hal_i2c_write` will bubble up as a non-zero return.

**Recommended pattern — always check init:**

```c
if (mmc5983ma_init(&mag) != 0) {
    // log error, halt, or retry
}
```

**Recommended pattern — tolerate measurement failures gracefully:**

```c
if (mmc5983ma_read_gauss(&mag, &field) == 0) {
    use_data(field);
} else {
    // log and continue — don't halt for a single missed sample
}
```

---

## 7. Typical Usage Flows

### Flow A — Simple one-shot polling (MMC5983MA)

```c
mmc5983ma_t mag;

// 1. Set up handle
mmc5983ma_init_handle(&mag, true);   // 18-bit mode

// 2. Start sensor
mmc5983ma_init(&mag);

// 3. One-time calibration
mmc5983ma_calibrate_offset(&mag);

// 4. Read loop
while (1) {
    mmc5983ma_data_t field;
    mmc5983ma_read_gauss(&mag, &field);
    printf("X=%.3f Y=%.3f Z=%.3f G\n", field.x, field.y, field.z);
    delay_ms(100);
}
```

---

### Flow B — Continuous mode with polling (MMC5603NJ)

```c
mmc5603nj_t mag;

mmc5603nj_init_handle(&mag, MMC5603NJ_RES_20BIT, MMC5603NJ_BW_2_0MS);
mmc5603nj_init(&mag);
mmc5603nj_selftest(&mag, &ok);
mmc5603nj_calibrate_offset(&mag);

// Start at 50 Hz
mmc5603nj_start_continuous(&mag, 50, MMC5603NJ_BW_2_0MS, true, false);

while (1) {
    bool ready = false;
    mmc5603nj_is_data_ready(&mag, &ready);
    if (ready) {
        mmc5603nj_data_t field;
        mmc5603nj_read_gauss(&mag, &field);
        // process field
    }
}
```

---

### Flow C — Interrupt-driven continuous mode (either sensor)

```c
// In your GPIO interrupt handler:
void EXTI_IRQHandler(void) {
    data_ready_flag = true;
}

// In your main loop or task:
if (data_ready_flag) {
    data_ready_flag = false;
    mmc5983ma_read_gauss(&mag, &field);
}
```

Enable the interrupt with:
```c
mmc5983ma_enable_interrupt(&mag, true);
```

---

### Flow D — High-accuracy heading (SET/RESET offset removal per sample)

```c
mmc5983ma_raw_t raw_set, raw_reset;
mmc5983ma_data_t field;

mmc5983ma_set(&mag);
mmc5983ma_read_raw(&mag, &raw_set);

mmc5983ma_reset_coil(&mag);
mmc5983ma_read_raw(&mag, &raw_reset);

// field = (SET - RESET) / 2  in counts, then convert
float sens = 16384.0f;
field.x = ((float)raw_set.x - (float)raw_reset.x) / 2.0f / sens;
field.y = ((float)raw_set.y - (float)raw_reset.y) / 2.0f / sens;
field.z = ((float)raw_set.z - (float)raw_reset.z) / 2.0f / sens;
```

This eliminates offset entirely on every sample pair, at the cost of halving
your effective sample rate.

---

## 8. Key Differences Between the Two Sensors

| Aspect                     | MMC5983MA                              | MMC5603NJ                                    |
|----------------------------|----------------------------------------|----------------------------------------------|
| **Resolution config**      | `bool mode_18bit` in init_handle       | `mmc5603nj_res_t` enum (16/18/20-bit)        |
| **Output registers**       | 7 regs (0x00–0x06), shared XYZout2    | 9 regs (0x00–0x08), separate Xout2/Yout2/Zout2 |
| **Meas_done bit**          | Bit 0 of Status (0x08)                 | Bit 6 of Status1 (0x18)                      |
| **Meas_done clear**        | Write 1 to clear manually              | Auto-clears on any output register read       |
| **Continuous ODR**         | Enum with fixed presets                | Direct Hz value 1–255 in ODR register         |
| **CMM start sequence**     | BW → auto_sr → Cmm_en                 | BW → ODR → Cmm_freq_en → Cmm_en (strict)     |
| **1000 Hz mode**           | `MMC5983MA_CM_1000HZ` enum value       | `odr=255` + `hpower=true`                    |
| **Self-test**              | Not available                          | `mmc5603nj_selftest()` with threshold regs   |
| **Product ID register**    | 0x2F, expects 0x30                     | 0x39, expects 0x10                           |
| **Reset settle time**      | 15 ms (driver uses)                    | 25 ms (driver uses)                          |
| **Channels measured**      | X, Y, Z in parallel                    | X, Y, Z sequentially                         |
