/**
 * @file    app_mpu6050.c
 * @brief   自写 MPU6050 / MPU6500 / MPU9250 寄存器驱动（评分点 3，禁用第三方库与 DMP）
 *
 * 三款芯片寄存器大体兼容，靠 WHO_AM_I 自动识别（0x68 / 0x70 / 0x71），
 * 差异只有两处，都在下面按型号分支处理：
 *   1) 加速度数字低通：6500/9250 在 ACCEL_CONFIG2(0x1D)，6050 的 0x1D 是 FF_THR；
 *   2) 温度换算：6050 是 raw/340+36.53，6500/9250 是 raw/333.87+21。
 *
 * 初始化顺序（RM-MPU-6000A / RM-MPU-6500A 推荐）：
 *   1. 读 WHO_AM_I 识别型号与地址（0x68 / 0x69 轮流试）
 *   2. PWR_MGMT_1 = 0x80 软复位，等 100ms
 *   3. PWR_MGMT_1 = 0x01 退出睡眠，时钟源选 PLL(X 轴陀螺)
 *   4. SIGNAL_PATH_RESET = 0x07 复位陀螺/加速度/温度信号链，等 100ms
 *   5. CONFIG(DLPF) [+ ACCEL_CONFIG2] -> SMPLRT_DIV -> GYRO_CONFIG -> ACCEL_CONFIG
 *   6. INT_PIN_CFG：latch + 读清除 + 高有效，配合 EXTI 上升沿
 */
#include "main.h"
#include "board_config.h"
#include "app_mpu6050.h"
#include "app_mpu6050_reg.h"
#include "app_i2c_bus.h"

static uint8_t    s_addr = IMU_ADDR_PRIMARY;
static uint8_t    s_inited;
static mpu_chip_t s_chip = MPU_CHIP_NONE;
static float      s_acc_lsb_per_g = 8192.0f;    /* AFS_SEL = 1 (±4g) */
static float      s_gyro_lsb_dps  = 65.5f;      /* FS_SEL  = 1 (±500°/s) */
static float      s_temp_sens     = MPU6050_TEMP_SENSITIVITY;
static float      s_temp_off      = MPU6050_TEMP_OFFSET_C;

/* DMA 突发读使用 */
static uint8_t           s_dma_buf[14];
static volatile uint8_t  s_dma_state;         /* 0=进行中 1=完成 2=出错 */

/* ==================================================================
 * 寄存器级读写
 * ================================================================== */
static mpu_status_t mpu_reg_write(uint8_t reg, uint8_t val)
{
    if (i2c_bus_write(s_addr, reg, &val, 1U) != HAL_OK)
    {
        return MPU_ERR_I2C;
    }
    return MPU_OK;
}

static mpu_status_t mpu_reg_read(uint8_t reg, uint8_t *buf, uint16_t len)
{
    if (i2c_bus_read(s_addr, reg, buf, len) != HAL_OK)
    {
        return MPU_ERR_I2C;
    }
    return MPU_OK;
}

/** @brief 把 14 字节大端原始数据拆成有符号数 */
static void mpu_parse_raw(const uint8_t *b, mpu_raw_t *r)
{
    r->ax   = (int16_t)(((uint16_t)b[0]  << 8) | (uint16_t)b[1]);
    r->ay   = (int16_t)(((uint16_t)b[2]  << 8) | (uint16_t)b[3]);
    r->az   = (int16_t)(((uint16_t)b[4]  << 8) | (uint16_t)b[5]);
    r->temp = (int16_t)(((uint16_t)b[6]  << 8) | (uint16_t)b[7]);
    r->gx   = (int16_t)(((uint16_t)b[8]  << 8) | (uint16_t)b[9]);
    r->gy   = (int16_t)(((uint16_t)b[10] << 8) | (uint16_t)b[11]);
    r->gz   = (int16_t)(((uint16_t)b[12] << 8) | (uint16_t)b[13]);
}

/** @brief 原始值 -> 物理量 */
static void mpu_to_sample(const mpu_raw_t *r, imu_sample_t *s)
{
    s->raw    = *r;
    s->ax_g   = (float)r->ax / s_acc_lsb_per_g;
    s->ay_g   = (float)r->ay / s_acc_lsb_per_g;
    s->az_g   = (float)r->az / s_acc_lsb_per_g;
    s->gx_dps = (float)r->gx / s_gyro_lsb_dps;
    s->gy_dps = (float)r->gy / s_gyro_lsb_dps;
    s->gz_dps = (float)r->gz / s_gyro_lsb_dps;
    s->temp_c = (float)r->temp / s_temp_sens + s_temp_off;
    s->tick_ms = HAL_GetTick();
}

/* ==================================================================
 * 公共接口
 * ================================================================== */
uint8_t mpu_whoami(void)
{
    uint8_t v = 0U;

    if (i2c_bus_read(s_addr, MPU_REG_WHO_AM_I, &v, 1U) != HAL_OK)
    {
        return 0U;
    }
    return v;
}

uint8_t mpu_addr(void)
{
    return s_addr;
}

mpu_chip_t mpu_chip(void)
{
    return s_chip;
}

const char *mpu_chip_name(void)
{
    switch (s_chip)
    {
        case MPU_CHIP_MPU6050: return "MPU6050";
        case MPU_CHIP_MPU6500: return "MPU6500";
        case MPU_CHIP_MPU9250: return "MPU9250";
        default:               return "unknown";
    }
}

/** @brief WHO_AM_I 值 -> 芯片型号 */
static mpu_chip_t mpu_chip_from_id(uint8_t id)
{
    switch (id)
    {
        case MPU_WHOAMI_MPU6050: return MPU_CHIP_MPU6050;
        case MPU_WHOAMI_MPU6500: return MPU_CHIP_MPU6500;
        case MPU_WHOAMI_MPU9250: return MPU_CHIP_MPU9250;
        default:                 return MPU_CHIP_NONE;
    }
}

/** @brief 按型号装载温度换算系数 */
static void mpu_load_temp_const(mpu_chip_t chip)
{
    if (chip == MPU_CHIP_MPU6050)
    {
        s_temp_sens = MPU6050_TEMP_SENSITIVITY;
        s_temp_off  = MPU6050_TEMP_OFFSET_C;
    }
    else
    {
        /* MPU6500 / MPU9250 */
        s_temp_sens = MPU6500_TEMP_SENSITIVITY;
        s_temp_off  = MPU6500_TEMP_OFFSET_C;
    }
}

const char *mpu_status_str(mpu_status_t st)
{
    switch (st)
    {
        case MPU_OK:            return "OK";
        case MPU_ERR_NOT_INIT:  return "ERR:NOT_INIT";
        case MPU_ERR_I2C:       return "ERR:I2C(NACK/TIMEOUT)";
        case MPU_ERR_ID:        return "ERR:WHO_AM_I";
        case MPU_ERR_PARAM:     return "ERR:PARAM";
        case MPU_ERR_BUSY:      return "ERR:BUSY";
        case MPU_ERR_SELFTEST:  return "ERR:SELFTEST";
        default:                return "ERR:UNKNOWN";
    }
}

mpu_status_t mpu_init(void)
{
    uint8_t id = 0U;
    mpu_status_t st;

    /* ---- 1. 探测地址 + 识别型号：先试 0x68，再试 0x69 ---- */
    if (i2c_bus_read(IMU_ADDR_PRIMARY, MPU_REG_WHO_AM_I, &id, 1U) == HAL_OK)
    {
        s_addr = IMU_ADDR_PRIMARY;
    }
    else if (i2c_bus_read(IMU_ADDR_SECONDARY, MPU_REG_WHO_AM_I, &id, 1U) == HAL_OK)
    {
        s_addr = IMU_ADDR_SECONDARY;
    }
    else
    {
        return MPU_ERR_I2C;
    }

    s_chip = mpu_chip_from_id(id);
    if (s_chip == MPU_CHIP_NONE)
    {
        /* 地址通了但型号不认识：INFO 里会把这个 WHO_AM_I 原值打出来 */
        return MPU_ERR_ID;
    }
    mpu_load_temp_const(s_chip);

    /* ---- 2. 软复位 ---- */
    st = mpu_reg_write(MPU_REG_PWR_MGMT_1, MPU_PWR1_DEVICE_RESET);
    if (st != MPU_OK)
    {
        return st;
    }
    HAL_Delay(100);

    /* ---- 3. 退出睡眠，时钟源 = PLL(X 陀螺) ---- */
    st = mpu_reg_write(MPU_REG_PWR_MGMT_1, MPU_PWR1_CLKSEL_PLL_GX);
    if (st != MPU_OK)
    {
        return st;
    }
    HAL_Delay(20);

    /* ---- 4. 复位信号链 ---- */
    st = mpu_reg_write(MPU_REG_SIGNAL_PATH_RESET, 0x07U);
    if (st != MPU_OK)
    {
        return st;
    }
    HAL_Delay(100);

    /* ---- 5. DLPF：陀螺带宽取 3；MPU6500/9250 的加速度滤波器要单独配 ---- */
    st = mpu_reg_write(MPU_REG_CONFIG, 0x03U);
    if (st != MPU_OK)
    {
        return st;
    }
    if (s_chip != MPU_CHIP_MPU6050)
    {
        /* ACCEL_CONFIG2：ACCEL_FCHOICE_B = 0（启用 DLPF），A_DLPF_CFG = 3 → 41Hz。
           6050 的这个地址(0x1D)是 FF_THR 自由落体阈值，绝不能写。 */
        st = mpu_reg_write(MPU_REG_ACCEL_CONFIG2, 0x03U);
        if (st != MPU_OK)
        {
            return st;
        }
    }

    /* ---- 6. 采样率 100Hz（DLPF 使能时陀螺输出 1kHz） ---- */
    st = mpu_set_sample_rate(SAMPLE_RATE_DEFAULT_HZ);
    if (st != MPU_OK)
    {
        return st;
    }

    /* ---- 7. 量程：陀螺 ±500°/s，加速度 ±4g ---- */
    st = mpu_set_gyro_fs(1U);
    if (st != MPU_OK)
    {
        return st;
    }
    st = mpu_set_accel_fs(1U);
    if (st != MPU_OK)
    {
        return st;
    }

    /* ---- 8. 中断引脚：latch + 读清除 + 高有效（对应 PB5 上升沿） ---- */
    st = mpu_reg_write(MPU_REG_INT_PIN_CFG,
                       MPU_INT_CFG_LATCH_EN | MPU_INT_CFG_RD_CLEAR | MPU_INT_CFG_ACTIVE_HIGH);
    if (st != MPU_OK)
    {
        return st;
    }

    /* ---- 9. 默认关闭 FIFO 与数据就绪中断，由命令按需打开 ---- */
    (void)mpu_reg_write(MPU_REG_FIFO_EN, 0x00U);
    (void)mpu_reg_write(MPU_REG_USER_CTRL, 0x00U);
    (void)mpu_reg_write(MPU_REG_INT_ENABLE, 0x00U);

    s_inited = 1U;
    return MPU_OK;
}

mpu_status_t mpu_read_raw(mpu_raw_t *raw)
{
    uint8_t buf[14];
    mpu_status_t st;

    if (raw == 0)
    {
        return MPU_ERR_PARAM;
    }
    if (s_inited == 0U)
    {
        return MPU_ERR_NOT_INIT;
    }
    st = mpu_reg_read(MPU_REG_ACCEL_XOUT_H, buf, 14U);
    if (st != MPU_OK)
    {
        return st;
    }
    mpu_parse_raw(buf, raw);
    return MPU_OK;
}

mpu_status_t mpu_read(imu_sample_t *sample)
{
    uint8_t buf[14];
    mpu_raw_t raw;
    mpu_status_t st;

    if (sample == 0)
    {
        return MPU_ERR_PARAM;
    }
    if (s_inited == 0U)
    {
        return MPU_ERR_NOT_INIT;
    }
    st = mpu_reg_read(MPU_REG_ACCEL_XOUT_H, buf, 14U);
    if (st != MPU_OK)
    {
        return st;
    }
    mpu_parse_raw(buf, &raw);
    mpu_to_sample(&raw, sample);
    return MPU_OK;
}

mpu_status_t mpu_set_sample_rate(uint16_t hz)
{
    uint16_t div;

    if (hz == 0U || hz > 1000U)
    {
        return MPU_ERR_PARAM;
    }
    /* Rate = 1000Hz / (1 + SMPLRT_DIV)  ->  DIV = 1000/rate - 1 */
    div = (uint16_t)(1000U / hz);
    if (div == 0U)
    {
        div = 1U;
    }
    return mpu_reg_write(MPU_REG_SMPLRT_DIV, (uint8_t)(div - 1U));
}

mpu_status_t mpu_set_dlpf(uint8_t cfg)
{
    if (cfg > 6U)
    {
        return MPU_ERR_PARAM;
    }
    return mpu_reg_write(MPU_REG_CONFIG, cfg);
}

mpu_status_t mpu_set_gyro_fs(uint8_t fs_sel)
{
    static const float lsb[4] = {131.0f, 65.5f, 32.8f, 16.4f};

    if (fs_sel > 3U)
    {
        return MPU_ERR_PARAM;
    }
    s_gyro_lsb_dps = lsb[fs_sel];
    return mpu_reg_write(MPU_REG_GYRO_CONFIG, (uint8_t)(fs_sel << 3));
}

mpu_status_t mpu_set_accel_fs(uint8_t afs_sel)
{
    static const float lsb[4] = {16384.0f, 8192.0f, 4096.0f, 2048.0f};

    if (afs_sel > 3U)
    {
        return MPU_ERR_PARAM;
    }
    s_acc_lsb_per_g = lsb[afs_sel];
    return mpu_reg_write(MPU_REG_ACCEL_CONFIG, (uint8_t)(afs_sel << 3));
}

mpu_status_t mpu_int_enable(uint8_t enable)
{
    return mpu_reg_write(MPU_REG_INT_ENABLE, (enable != 0U) ? MPU_INT_EN_DATA_RDY : 0x00U);
}

mpu_status_t mpu_fifo_enable(uint8_t enable)
{
    mpu_status_t st;

    if (enable != 0U)
    {
        /* 加速度 + 陀螺 6 轴入 FIFO，每帧 8 字节 */
        st = mpu_reg_write(MPU_REG_FIFO_EN, 0x78U);
        if (st != MPU_OK)
        {
            return st;
        }
        return mpu_reg_write(MPU_REG_USER_CTRL, MPU_USERCTRL_FIFO_EN);
    }
    st = mpu_reg_write(MPU_REG_USER_CTRL, 0x00U);
    if (st != MPU_OK)
    {
        return st;
    }
    return mpu_reg_write(MPU_REG_FIFO_EN, 0x00U);
}

mpu_status_t mpu_fifo_reset(void)
{
    mpu_status_t st;

    st = mpu_reg_write(MPU_REG_USER_CTRL, MPU_USERCTRL_FIFO_RESET | MPU_USERCTRL_FIFO_EN);
    HAL_Delay(2);
    return st;
}

uint16_t mpu_fifo_count(void)
{
    uint8_t b[2] = {0U, 0U};

    if (mpu_reg_read(MPU_REG_FIFO_COUNTH, b, 2U) != MPU_OK)
    {
        return 0U;
    }
    return (uint16_t)(((uint16_t)b[0] << 8) | (uint16_t)b[1]);
}

mpu_status_t mpu_fifo_read(uint8_t *buf, uint16_t len)
{
    if (buf == 0 || len == 0U)
    {
        return MPU_ERR_PARAM;
    }
    return mpu_reg_read(MPU_REG_FIFO_R_W, buf, len);
}

/* ==================================================================
 * DMA 突发读（发挥项：与轮询对比采样抖动 / CPU 占用）
 * ================================================================== */
mpu_status_t mpu_start_dma_read(void)
{
    s_dma_state = 0U;
    if (i2c_bus_read_dma(s_addr, MPU_REG_ACCEL_XOUT_H, s_dma_buf, 14U) != HAL_OK)
    {
        s_dma_state = 2U;
        return MPU_ERR_I2C;
    }
    return MPU_OK;
}

uint8_t mpu_dma_state(void)
{
    return s_dma_state;
}

mpu_status_t mpu_finish_dma_read(imu_sample_t *sample)
{
    mpu_raw_t raw;

    if (sample == 0)
    {
        return MPU_ERR_PARAM;
    }
    if (s_dma_state == 0U)
    {
        return MPU_ERR_BUSY;
    }
    if (s_dma_state == 2U)
    {
        return MPU_ERR_I2C;
    }
    s_dma_state = 0U;
    mpu_parse_raw(s_dma_buf, &raw);
    mpu_to_sample(&raw, sample);
    return MPU_OK;
}

/**
 * @brief HAL 的 I2C 接收 DMA 完成回调
 */
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        s_dma_state = 1U;
    }
}

/**
 * @brief HAL 的 I2C 错误回调（DMA 出错时置错误标志，主循环会降级处理）
 */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    {
        s_dma_state = 2U;
    }
}

/* ==================================================================
 * 自检
 * ================================================================== */
mpu_status_t mpu_selftest(float *acc_norm_g)
{
    imu_sample_t s;
    float        sum = 0.0f;
    uint8_t      i;
    mpu_status_t st;

    if (mpu_chip_from_id(mpu_whoami()) == MPU_CHIP_NONE)
    {
        return MPU_ERR_ID;
    }

    for (i = 0U; i < 10U; i++)
    {
        st = mpu_read(&s);
        if (st != MPU_OK)
        {
            return st;
        }
        sum += (s.ax_g * s.ax_g + s.ay_g * s.ay_g + s.az_g * s.az_g);
        HAL_Delay(2);
    }
    sum = sum / 10.0f;

    if (acc_norm_g != 0)
    {
        /* 这里用 sqrt 的近似：先返回平方和，调用方需要模长时自己开方 */
        *acc_norm_g = sum;
    }

    /* 静止时三轴合成应接近 1g²（0.6..1.7 之间都算合理，含安装误差） */
    if (sum < 0.6f || sum > 1.7f)
    {
        return MPU_ERR_SELFTEST;
    }
    return MPU_OK;
}
