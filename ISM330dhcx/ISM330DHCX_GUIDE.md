# ISM330DHCX Driver — Complete Guide
### For STM32 + HAL + I2C
*Written for humans. No PhD required.*

---

## What Are These Three Files?

| File | What it is |
|---|---|
| `ism330dhcx.h` | The menu — all register addresses, enums, structs, function names |
| `ism330dhcx.c` | The kitchen — actual implementation of every function |
| `ism330dhcx_example.c` | Examples — you can delete this once you understand the driver |

You only ever touch `main.c`. The driver files just sit there and do their job.

---

## Project Setup (CubeIDE)

```
MyProject/
├── Core/
│   ├── Inc/
│   │   ├── main.h
│   │   └── ism330dhcx.h     ← drag here
│   └── Src/
│       ├── main.c
│       └── ism330dhcx.c     ← drag here
```

Delete `ism330dhcx_example.c` — you don't need it.

---

## One-Time Fix in `ism330dhcx.c`

Find this line and change it so the driver uses HAL delays properly:

```c
// FIND THIS:
#define ISM330DHCX_DELAY_MS(ms) _delay_ms_default(ms)

// CHANGE TO THIS:
#define ISM330DHCX_DELAY_MS(ms) HAL_Delay(ms)
```

That's the only edit you ever make to the driver files.

---

## Wiring

```
ISM330DHCX          STM32
──────────          ─────
VDD         →       3.3V
GND         →       GND
SDA         →       I2C SDA (e.g. PB7)
SCL         →       I2C SCL (e.g. PB6)
SDO/SA0     →       GND   (I2C address = 0x6A)
            or      3.3V  (I2C address = 0x6B)
```

**SA0 pin decides the I2C address — remember which one you used.**

---

## main.c — Full Template

Copy this entire block. Change the two `#define` lines at the top to match your setup.

```c
#include "main.h"
#include "ism330dhcx.h"

// ── CHANGE THESE TWO LINES TO MATCH YOUR SETUP ──────────────────────────────
#define IMU_I2C_HANDLE   hi2c1            // whatever CubeMX generated
#define IMU_I2C_ADDR    (0x6B << 1)       // 0x6B if SA0→VDD, 0x6A if SA0→GND
// ────────────────────────────────────────────────────────────────────────────

extern I2C_HandleTypeDef IMU_I2C_HANDLE;

// I2C read callback — don't touch this
static int imu_read_cb(void *handle, uint8_t reg, uint8_t *buf, size_t len)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)handle;
    return HAL_I2C_Mem_Read(hi2c, IMU_I2C_ADDR, reg,
                             I2C_MEMADD_SIZE_8BIT,
                             buf, len, HAL_MAX_DELAY) == HAL_OK ? 0 : -1;
}

// I2C write callback — don't touch this
static int imu_write_cb(void *handle, uint8_t reg,
                         const uint8_t *buf, size_t len)
{
    I2C_HandleTypeDef *hi2c = (I2C_HandleTypeDef *)handle;
    uint8_t tmp[32];
    for (size_t i = 0; i < len; i++) tmp[i] = buf[i];
    return HAL_I2C_Mem_Write(hi2c, IMU_I2C_ADDR, reg,
                              I2C_MEMADD_SIZE_8BIT,
                              tmp, len, HAL_MAX_DELAY) == HAL_OK ? 0 : -1;
}

// ── Your device handle — one global is fine ──────────────────────────────────
ism330dhcx_dev_t imu;

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_I2C1_Init();               // must run before imu_init

    // ── Init ────────────────────────────────────────────────────────────────
    if (ism330dhcx_init(&imu, imu_read_cb, imu_write_cb,
                         &IMU_I2C_HANDLE) != ISM330DHCX_OK)
    {
        Error_Handler();           // not found — check wiring and SA0 pin
    }

    ism330dhcx_enable_bdu(&imu);  // always call this after init

    // ── Configure ───────────────────────────────────────────────────────────
    ism330dhcx_config_accel(&imu, ISM330DHCX_XL_ODR_104HZ, ISM330DHCX_XL_FS_4G);
    ism330dhcx_config_gyro (&imu, ISM330DHCX_GY_ODR_104HZ, ISM330DHCX_GY_FS_2000DPS);

    // ── Loop ────────────────────────────────────────────────────────────────
    ism330dhcx_accel_t accel;
    ism330dhcx_gyro_t  gyro;
    float temp;

    while (1)
    {
        ism330dhcx_read_all(&imu, &accel, &gyro);
        ism330dhcx_read_temp(&imu, &temp);

        // accel.x / .y / .z  →  in mg   (1000 mg = 1 g)
        // gyro.x  / .y / .z  →  in mdps (1000 mdps = 1 degree/sec)
        // temp               →  in °C

        HAL_Delay(10);
    }
}
```

---

## Every Function Explained

### `ism330dhcx_init()`
```c
ism330dhcx_init(&imu, imu_read_cb, imu_write_cb, &hi2c1);
```
Connects the driver to your hardware. Checks WHO_AM_I register (must return 0x6B).
If it returns `ISM330DHCX_ENODEV` your I2C address is wrong or wiring is bad.
**Call this first, before anything else.**

---

### `ism330dhcx_enable_bdu()`
```c
ism330dhcx_enable_bdu(&imu);
```
Prevents reading corrupted data when a new sample arrives mid-read.
**Always call this right after init. One line, never skip it.**

---

### `ism330dhcx_config_accel()`
```c
ism330dhcx_config_accel(&imu, ISM330DHCX_XL_ODR_104HZ, ISM330DHCX_XL_FS_4G);
//                             ↑ sample rate             ↑ measurement range
```

**ODR options (how often it samples):**

| Option | Rate |
|---|---|
| `ISM330DHCX_XL_ODR_OFF` | Off (power down) |
| `ISM330DHCX_XL_ODR_12_5HZ` | 12.5 Hz |
| `ISM330DHCX_XL_ODR_26HZ` | 26 Hz |
| `ISM330DHCX_XL_ODR_52HZ` | 52 Hz |
| `ISM330DHCX_XL_ODR_104HZ` | 104 Hz ← good default |
| `ISM330DHCX_XL_ODR_208HZ` | 208 Hz |
| `ISM330DHCX_XL_ODR_416HZ` | 416 Hz |
| `ISM330DHCX_XL_ODR_833HZ` | 833 Hz |
| `ISM330DHCX_XL_ODR_1660HZ` | 1.66 kHz |
| `ISM330DHCX_XL_ODR_3330HZ` | 3.33 kHz |
| `ISM330DHCX_XL_ODR_6660HZ` | 6.66 kHz |

**Full-scale options (max measurable range):**

| Option | Range | Sensitivity |
|---|---|---|
| `ISM330DHCX_XL_FS_2G` | ±2 g | 0.061 mg/LSB — most precise |
| `ISM330DHCX_XL_FS_4G` | ±4 g | 0.122 mg/LSB ← good default |
| `ISM330DHCX_XL_FS_8G` | ±8 g | 0.244 mg/LSB |
| `ISM330DHCX_XL_FS_16G` | ±16 g | 0.488 mg/LSB — most range |

*Rule of thumb: pick the smallest range that won't clip your signal.*

---

### `ism330dhcx_config_gyro()`
```c
ism330dhcx_config_gyro(&imu, ISM330DHCX_GY_ODR_104HZ, ISM330DHCX_GY_FS_2000DPS);
```

**ODR options:** same rates as accel above, just replace `XL` with `GY`.

**Full-scale options:**

| Option | Range | Sensitivity |
|---|---|---|
| `ISM330DHCX_GY_FS_125DPS` | ±125 dps | 4.375 mdps/LSB — most precise |
| `ISM330DHCX_GY_FS_250DPS` | ±250 dps | 8.75 mdps/LSB |
| `ISM330DHCX_GY_FS_500DPS` | ±500 dps | 17.5 mdps/LSB |
| `ISM330DHCX_GY_FS_1000DPS` | ±1000 dps | 35 mdps/LSB |
| `ISM330DHCX_GY_FS_2000DPS` | ±2000 dps | 70 mdps/LSB ← good default |
| `ISM330DHCX_GY_FS_4000DPS` | ±4000 dps | 140 mdps/LSB — most range |

---

### `ism330dhcx_read_all()`
```c
ism330dhcx_accel_t accel;
ism330dhcx_gyro_t  gyro;

ism330dhcx_read_all(&imu, &accel, &gyro);

// Access values like this:
float ax = accel.x;   // mg
float ay = accel.y;   // mg
float az = accel.z;   // mg

float gx = gyro.x;    // mdps
float gy = gyro.y;    // mdps
float gz = gyro.z;    // mdps
```

Single I2C burst — reads all 6 axes in one transaction. Faster than calling accel and gyro separately.

---

### `ism330dhcx_read_accel()` / `ism330dhcx_read_gyro()`
```c
ism330dhcx_accel_t accel;
ism330dhcx_read_accel(&imu, &accel);   // just accel

ism330dhcx_gyro_t gyro;
ism330dhcx_read_gyro(&imu, &gyro);     // just gyro
```

Use these if you only need one sensor and want to save a few bytes of I2C traffic.

---

### `ism330dhcx_read_temp()`
```c
float temp;
ism330dhcx_read_temp(&imu, &temp);
// temp is in °C, e.g. 27.34
```

This is the chip's internal temperature, not ambient room temperature.
Useful for thermal compensation if you need very precise measurements.

---

### `ism330dhcx_get_status()`
```c
uint8_t status;
ism330dhcx_get_status(&imu, &status);

if (status & ISM330DHCX_STATUS_XLDA)  // accel data ready
if (status & ISM330DHCX_STATUS_GDA)   // gyro data ready
if (status & ISM330DHCX_STATUS_TDA)   // temperature data ready
```

Tells you if fresh data is available. Useful if you want to read exactly when new data arrives instead of polling on a timer.

---

### `ism330dhcx_reset()`
```c
ism330dhcx_reset(&imu);
```

Software reset — puts all registers back to factory defaults.
Useful if the sensor gets into a weird state.
**Must re-configure accel and gyro after calling this.**

---

### `ism330dhcx_set_int1()` / `ism330dhcx_set_int2()`
```c
// Route data-ready signal to INT1 pin
ism330dhcx_set_int1(&imu, ISM330DHCX_INT_DRDY_XL | ISM330DHCX_INT_DRDY_G);

// Available flags:
// ISM330DHCX_INT_DRDY_XL    — accel data ready
// ISM330DHCX_INT_DRDY_G     — gyro data ready
// ISM330DHCX_INT_DRDY_TEMP  — temperature data ready
// ISM330DHCX_INT_FIFO_TH    — FIFO watermark reached
// ISM330DHCX_INT_FIFO_FULL  — FIFO full
// ISM330DHCX_INT_FIFO_OVR   — FIFO overrun
```

Wire INT1/INT2 to a GPIO EXTI pin on your STM32 and read data inside the interrupt callback instead of polling. More efficient than `HAL_Delay`.

---

### `ism330dhcx_reg_read()` / `ism330dhcx_reg_write()` / `ism330dhcx_reg_update()`
```c
// Read any register directly
uint8_t val;
ism330dhcx_reg_read(&imu, ISM330DHCX_REG_CTRL1_XL, &val);

// Write any register directly
ism330dhcx_reg_write(&imu, ISM330DHCX_REG_CTRL1_XL, 0x40);

// Change specific bits without touching others
// (register, mask of bits to change, new values for those bits)
ism330dhcx_reg_update(&imu, ISM330DHCX_REG_CTRL1_XL, 0x02, 0x02);
```

Escape hatch for anything not covered by the high-level functions —
Machine Learning Core, sensor hub, OIS chain, etc.

---

## Units — Quick Reference

| Value | Unit | To convert |
|---|---|---|
| `accel.x/y/z` | mg | divide by 1000 → g |
| `gyro.x/y/z` | mdps | divide by 1000 → dps |
| `temp` | °C | already °C |

```c
// Example conversions
float accel_g   = accel.x / 1000.0f;     // mg → g
float gyro_dps  = gyro.x  / 1000.0f;     // mdps → dps
float gyro_rads = gyro.x  / 1000.0f * 0.01745f;  // mdps → rad/s
```

---

## Common Patterns

### Just read data every 10ms
```c
while (1)
{
    ism330dhcx_accel_t accel;
    ism330dhcx_gyro_t  gyro;
    ism330dhcx_read_all(&imu, &accel, &gyro);
    HAL_Delay(10);
}
```

### Wait for fresh data before reading
```c
uint8_t status = 0;
while (!(status & ISM330DHCX_STATUS_XLDA))
    ism330dhcx_get_status(&imu, &status);

ism330dhcx_read_all(&imu, &accel, &gyro);
```

### Only need accelerometer (save power, turn gyro off)
```c
ism330dhcx_config_accel(&imu, ISM330DHCX_XL_ODR_104HZ, ISM330DHCX_XL_FS_4G);
ism330dhcx_config_gyro (&imu, ISM330DHCX_GY_ODR_OFF,   ISM330DHCX_GY_FS_250DPS);
```

### Power everything down
```c
ism330dhcx_config_accel(&imu, ISM330DHCX_XL_ODR_OFF, ISM330DHCX_XL_FS_2G);
ism330dhcx_config_gyro (&imu, ISM330DHCX_GY_ODR_OFF, ISM330DHCX_GY_FS_250DPS);
```

---

## Troubleshooting

| Problem | Likely cause | Fix |
|---|---|---|
| `init` returns `ENODEV` | Wrong I2C address | Check SA0 pin, swap `ADDR_HIGH` ↔ `ADDR_LOW` |
| `init` returns `ERR` | I2C not working | Check `MX_I2C1_Init()` runs before init, check wiring |
| Values are always 0 | BDU not enabled | Call `ism330dhcx_enable_bdu()` after init |
| Values look wrong | Wrong full-scale | Make sure FS matches what you configured |
| Readings are jumpy | No BDU | Call `ism330dhcx_enable_bdu()` |
| HAL_Delay not working in driver | Delay not set | Change delay define as shown above |

---

## What You Can Safely Ignore / Delete

| Thing | Safe to delete? |
|---|---|
| `ism330dhcx_example.c` | Yes, entirely |
| `ism330dhcx_self_test()` | Yes, production code doesn't need it |
| `ism330dhcx_fifo_*` functions | Yes, unless you specifically need FIFO |
| FIFO register defines in `.h` | Yes if you deleted FIFO functions |
| `_delay_ms_default()` in `.c` | Yes after you add `HAL_Delay` define |

---

*Now go moonwalk — aaou! 🕺*
