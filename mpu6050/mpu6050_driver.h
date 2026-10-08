#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>

/* ------------------------------------------------------------------ */
/*  I2C address (7-bit). AD0 low = 0x68, AD0 high = 0x69.              */
/*  Same convention as the MMC5603 driver: whatever your HAL wrapper   */
/*  expects (7-bit, or already shifted <<1) must be applied there.     */
/* ------------------------------------------------------------------ */
#define MPU6050_I2C_ADDR            0x68

/* ------------------------------------------------------------------ */
/*  Registers (datasheet section 3)                                    */
/* ------------------------------------------------------------------ */
#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_INT_PIN_CFG     0x37
#define MPU6050_REG_INT_ENABLE      0x38
#define MPU6050_REG_INT_STATUS      0x3A
#define MPU6050_REG_ACCEL_XOUT_H    0x3B   /* 14 bytes: ACCEL XYZ, TEMP, GYRO XYZ */
#define MPU6050_REG_SIGNAL_PATH_RST 0x68
#define MPU6050_REG_USER_CTRL       0x6A
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_PWR_MGMT_2      0x6C
#define MPU6050_REG_WHO_AM_I        0x75

#define MPU6050_WHO_AM_I_VAL        0x68   /* upper 6 bits of address, NOT affected by AD0 */

/* PWR_MGMT_1 bits */
#define MPU6050_PWR1_DEVICE_RESET   0x80
#define MPU6050_PWR1_SLEEP          0x40
#define MPU6050_PWR1_CYCLE          0x20
#define MPU6050_PWR1_TEMP_DIS       0x08
#define MPU6050_PWR1_CLKSEL_MASK    0x07

/* INT_PIN_CFG bits */
#define MPU6050_INTCFG_INT_LEVEL    0x80   /* 1 = active low  */
#define MPU6050_INTCFG_INT_OPEN     0x40   /* 1 = open drain  */
#define MPU6050_INTCFG_LATCH_INT_EN 0x20   /* 1 = held until cleared */
#define MPU6050_INTCFG_INT_RD_CLEAR 0x10   /* 1 = cleared on any read */

/* INT_ENABLE / INT_STATUS bits */
#define MPU6050_INT_DATA_RDY        0x01
#define MPU6050_INT_FIFO_OFLOW      0x10
#define MPU6050_INT_MOT             0x40

/* GYRO_CONFIG / ACCEL_CONFIG: FS select lives in bits [4:3] */
#define MPU6050_FS_SHIFT            3
#define MPU6050_FS_MASK             (0x03 << MPU6050_FS_SHIFT)

/* ------------------------------------------------------------------ */
/*  Configuration enums                                                */
/* ------------------------------------------------------------------ */
typedef enum {
    MPU6050_ACCEL_FS_2G  = 0,   /* 16384 LSB/g */
    MPU6050_ACCEL_FS_4G  = 1,   /*  8192 LSB/g */
    MPU6050_ACCEL_FS_8G  = 2,   /*  4096 LSB/g */
    MPU6050_ACCEL_FS_16G = 3    /*  2048 LSB/g */
} mpu6050_accel_fs_t;

typedef enum {
    MPU6050_GYRO_FS_250DPS  = 0,   /* 131.0 LSB/dps */
    MPU6050_GYRO_FS_500DPS  = 1,   /*  65.5 LSB/dps */
    MPU6050_GYRO_FS_1000DPS = 2,   /*  32.8 LSB/dps */
    MPU6050_GYRO_FS_2000DPS = 3    /*  16.4 LSB/dps */
} mpu6050_gyro_fs_t;

/* DLPF_CFG. 0 => gyro output rate 8 kHz (accel BW 260 Hz / gyro 256 Hz),
 * 1..6 => gyro output rate 1 kHz. 7 is RESERVED. */
typedef enum {
    MPU6050_DLPF_260HZ = 0,
    MPU6050_DLPF_184HZ = 1,
    MPU6050_DLPF_94HZ  = 2,
    MPU6050_DLPF_44HZ  = 3,
    MPU6050_DLPF_21HZ  = 4,
    MPU6050_DLPF_10HZ  = 5,
    MPU6050_DLPF_5HZ   = 6
} mpu6050_dlpf_t;

/* CLKSEL. Datasheet recommends a gyro PLL over the internal 8 MHz osc. */
typedef enum {
    MPU6050_CLK_INTERNAL_8MHZ = 0,
    MPU6050_CLK_PLL_XGYRO     = 1,
    MPU6050_CLK_PLL_YGYRO     = 2,
    MPU6050_CLK_PLL_ZGYRO     = 3,
    MPU6050_CLK_PLL_EXT_32K   = 4,
    MPU6050_CLK_PLL_EXT_19M   = 5,
    MPU6050_CLK_STOP          = 7
} mpu6050_clk_t;

/* ------------------------------------------------------------------ */
/*  Data types                                                         */
/* ------------------------------------------------------------------ */
typedef struct { int16_t x, y, z; } mpu6050_vec3i16_t;
typedef struct { int32_t x, y, z; } mpu6050_vec3i32_t;
typedef struct { float   x, y, z; } mpu6050_vec3f_t;

/* Raw sensor words (signed 16-bit, 2's complement) */
typedef struct {
    mpu6050_vec3i16_t accel;
    int16_t           temp;
    mpu6050_vec3i16_t gyro;
} mpu6050_raw_t;

/* Scaled data */
typedef struct {
    mpu6050_vec3f_t accel_g;     /* g     */
    mpu6050_vec3f_t gyro_dps;    /* deg/s */
    float           temp_c;      /* deg C */
} mpu6050_data_t;

typedef struct {
    mpu6050_accel_fs_t accel_fs;
    mpu6050_gyro_fs_t  gyro_fs;
    mpu6050_dlpf_t     dlpf;
    mpu6050_clk_t      clk;
    uint8_t            smplrt_div;     /* last value written to SMPLRT_DIV */
    uint8_t            pwr1_shadow;    /* last value written to PWR_MGMT_1 */
    uint8_t            int_en_shadow;  /* last value written to INT_ENABLE */

    /* Offsets in RAW LSB at the full-scale ranges active when
     * mpu6050_calibrate_offset() was run. Re-calibrate if you change FS. */
    mpu6050_vec3i32_t  accel_offset;
    mpu6050_vec3i32_t  gyro_offset;
} mpu6050_t;

/* ------------------------------------------------------------------ */
/*  HAL hooks — implement these in your port layer (same contract as   */
/*  the mmc5603nj_hal_i2c_* functions: return 0 on success).           */
/* ------------------------------------------------------------------ */
int mpu6050_hal_i2c_write(uint8_t dev_addr, uint8_t reg, const uint8_t *buf, uint16_t len);
int mpu6050_hal_i2c_read (uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint16_t len);

/* ------------------------------------------------------------------ */
/*  API                                                                */
/* ------------------------------------------------------------------ */
void mpu6050_init_handle(mpu6050_t *dev, mpu6050_accel_fs_t afs,
                         mpu6050_gyro_fs_t gfs, mpu6050_dlpf_t dlpf);

int  mpu6050_reset(mpu6050_t *dev);
int  mpu6050_init(mpu6050_t *dev);                 /* WHO_AM_I check, wake, apply config */
int  mpu6050_configure(mpu6050_t *dev);            /* write DLPF + FS ranges */
int  mpu6050_set_sample_rate(mpu6050_t *dev, uint16_t hz);
int  mpu6050_enable_data_ready_int(mpu6050_t *dev, bool active_low, bool open_drain,
                                   bool latch);
int  mpu6050_set_sleep(mpu6050_t *dev, bool sleep);
int  mpu6050_calibrate_offset(mpu6050_t *dev, uint16_t samples);

bool mpu6050_is_data_ready(mpu6050_t *dev);        /* NOTE: reading INT_STATUS clears the flag */
int  mpu6050_read_raw_burst(mpu6050_t *dev, uint8_t *buf14);
void mpu6050_parse_raw(const uint8_t *buf14, mpu6050_raw_t *raw);
int  mpu6050_read_raw(mpu6050_t *dev, mpu6050_raw_t *raw);
void mpu6050_convert(const mpu6050_t *dev, const mpu6050_raw_t *raw, mpu6050_data_t *out);
int  mpu6050_read(mpu6050_t *dev, mpu6050_data_t *out);   /* burst read + offset + scale */

#endif /* MPU6050_H */