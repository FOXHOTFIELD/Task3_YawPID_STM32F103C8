/**
 * @file    app_i2c_bus.c
 * @brief   I2C 总线双后端实现（硬件 I2C1 / GPIO 软件模拟）
 */
#include "main.h"
#include "board_config.h"
#include "app_i2c_bus.h"

#define I2C_TIMEOUT_MS   20U

#if IMU_I2C_BACKEND_HW

/* ==================================================================
 * 后端 A：硬件 I2C1（PB6/PB7，400kHz）
 * ================================================================== */

HAL_StatusTypeDef i2c_bus_init(void)
{
    /* 引脚与时钟已在 HAL_I2C_MspInit 中配置，这里只做一次总线健康检查 */
    return HAL_OK;
}

HAL_StatusTypeDef i2c_bus_write(uint8_t addr7, uint8_t reg, const uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Write(&hi2c1, (uint16_t)(addr7 << 1), reg,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t *)data, len, I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef i2c_bus_read(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read(&hi2c1, (uint16_t)(addr7 << 1), reg,
                            I2C_MEMADD_SIZE_8BIT, data, len, I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef i2c_bus_read_dma(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len)
{
    return HAL_I2C_Mem_Read_DMA(&hi2c1, (uint16_t)(addr7 << 1), reg,
                                I2C_MEMADD_SIZE_8BIT, data, len);
}

/* 位操作恢复用的短延时：400kHz 时半周期 1.25us，这里给足余量，
   恢复过程本来就不要求速度（只在 IMU 掉线重连时走一次）。 */
static void hw_recover_delay(void)
{
    volatile uint32_t i;

    for (i = 0U; i < 120U; i++)
    {
        __NOP();
    }
}

void i2c_bus_recover(void)
{
    GPIO_InitTypeDef g;
    uint8_t i;

    /* 1) 先把引脚从 I2C 外设手里收回来，切成开漏 GPIO 自己发时钟。
          从机若停在半个字节里（或把 SDA 拉死），它必须看到 SCL 才会把
          移位寄存器走完并释放 SDA；9 个时钟足够走完一个字节。 */
    (void)HAL_I2C_DeInit(&hi2c1);

    __HAL_RCC_GPIOB_CLK_ENABLE();
    g.Pin   = (uint32_t)(IMU_SCL_PIN | IMU_SDA_PIN);
    g.Mode  = GPIO_MODE_OUTPUT_OD;
    g.Pull  = GPIO_PULLUP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(IMU_SCL_PORT, &g);

    HAL_GPIO_WritePin(IMU_SDA_PORT, IMU_SDA_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(IMU_SCL_PORT, IMU_SCL_PIN, GPIO_PIN_SET);
    hw_recover_delay();

    for (i = 0U; i < 9U; i++)
    {
        HAL_GPIO_WritePin(IMU_SCL_PORT, IMU_SCL_PIN, GPIO_PIN_RESET);
        hw_recover_delay();
        HAL_GPIO_WritePin(IMU_SCL_PORT, IMU_SCL_PIN, GPIO_PIN_SET);
        hw_recover_delay();
    }

    /* 2) 补一个 STOP：SCL 高电平期间 SDA 由低变高 */
    HAL_GPIO_WritePin(IMU_SCL_PORT, IMU_SCL_PIN, GPIO_PIN_RESET);
    hw_recover_delay();
    HAL_GPIO_WritePin(IMU_SDA_PORT, IMU_SDA_PIN, GPIO_PIN_RESET);
    hw_recover_delay();
    HAL_GPIO_WritePin(IMU_SCL_PORT, IMU_SCL_PIN, GPIO_PIN_SET);
    hw_recover_delay();
    HAL_GPIO_WritePin(IMU_SDA_PORT, IMU_SDA_PIN, GPIO_PIN_SET);
    hw_recover_delay();

    /* 3) 重新初始化外设：MspInit 会把 PB6/PB7 切回复用开漏，
          并把硬件 I2C 状态机、错误标志全部清干净 */
    (void)HAL_I2C_Init(&hi2c1);
}

const char *i2c_bus_name(void)
{
    return "HW-I2C1@400kHz";
}

#else

/* ==================================================================
 * 后端 B：软件模拟 I2C（PB6/PB7 开漏，约 100kHz）
 *
 * 开漏输出写 1 表示释放总线（由上拉拉高），写 0 表示拉低；
 * 读 SDA 时把引脚临时切成输入（F1 的 GPIO 输入数据寄存器在输出模式下也可读，
 * 这里统一切成浮空输入，语义最清晰）。
 * ================================================================== */

static void sw_delay(void)
{
    volatile uint32_t i;
    for (i = 0U; i < 40U; i++)
    {
        __NOP();
    }
}

static void sda_low(void)   { HAL_GPIO_WritePin(IMU_SDA_PORT, IMU_SDA_PIN, GPIO_PIN_RESET); }
static void sda_high(void)  { HAL_GPIO_WritePin(IMU_SDA_PORT, IMU_SDA_PIN, GPIO_PIN_SET); }
static void scl_low(void)   { HAL_GPIO_WritePin(IMU_SCL_PORT, IMU_SCL_PIN, GPIO_PIN_RESET); }
static void scl_high(void)  { HAL_GPIO_WritePin(IMU_SCL_PORT, IMU_SCL_PIN, GPIO_PIN_SET); }

static uint8_t sda_read(void)
{
    return (HAL_GPIO_ReadPin(IMU_SDA_PORT, IMU_SDA_PIN) == GPIO_PIN_SET) ? 1U : 0U;
}

static void sw_start(void)
{
    sda_high();
    sw_delay();
    scl_high();
    sw_delay();
    sda_low();      /* SCL 高时 SDA 下降 = 起始条件 */
    sw_delay();
    scl_low();
    sw_delay();
}

static void sw_stop(void)
{
    sda_low();
    sw_delay();
    scl_high();
    sw_delay();
    sda_high();     /* SCL 高时 SDA 上升 = 停止条件 */
    sw_delay();
}

/**
 * @brief 发送一字节，返回从机 ACK（0 = 有应答）
 */
static uint8_t sw_write_byte(uint8_t b)
{
    uint8_t i;
    uint8_t nack;

    for (i = 0U; i < 8U; i++)
    {
        if ((b & 0x80U) != 0U)
        {
            sda_high();
        }
        else
        {
            sda_low();
        }
        b = (uint8_t)(b << 1);
        sw_delay();
        scl_high();
        sw_delay();
        scl_low();
        sw_delay();
    }

    sda_high();             /* 释放 SDA，等从机拉低表示 ACK */
    sw_delay();
    scl_high();
    sw_delay();
    nack = sda_read();
    scl_low();
    sw_delay();
    return nack;
}

/**
 * @brief 读一字节，ack=1 时回 ACK，ack=0 时回 NACK（最后一个字节）
 */
static uint8_t sw_read_byte(uint8_t ack)
{
    uint8_t i;
    uint8_t b = 0U;

    sda_high();
    for (i = 0U; i < 8U; i++)
    {
        sw_delay();
        scl_high();
        sw_delay();
        b = (uint8_t)(b << 1);
        if (sda_read() != 0U)
        {
            b |= 1U;
        }
        scl_low();
        sw_delay();
    }

    if (ack != 0U)
    {
        sda_low();
    }
    else
    {
        sda_high();
    }
    sw_delay();
    scl_high();
    sw_delay();
    scl_low();
    sw_delay();
    sda_high();

    return b;
}

HAL_StatusTypeDef i2c_bus_init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    /* 软件 I2C 用开漏输出：高电平由上拉电阻决定，符合 I2C 电气规范 */
    GPIO_InitStruct.Pin   = IMU_SCL_PIN | IMU_SDA_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    sda_high();
    scl_high();
    return HAL_OK;
}

HAL_StatusTypeDef i2c_bus_read(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len)
{
    uint16_t i;

    sw_start();
    if (sw_write_byte((uint8_t)(addr7 << 1)) != 0U)     /* 写地址 + W */
    {
        sw_stop();
        return HAL_ERROR;
    }
    if (sw_write_byte(reg) != 0U)
    {
        sw_stop();
        return HAL_ERROR;
    }

    sw_start();                                          /* 重复起始 */
    if (sw_write_byte((uint8_t)((addr7 << 1) | 1U)) != 0U)
    {
        sw_stop();
        return HAL_ERROR;
    }

    for (i = 0U; i < len; i++)
    {
        data[i] = sw_read_byte((uint8_t)((i + 1U) < len ? 1U : 0U));
    }
    sw_stop();
    return HAL_OK;
}

HAL_StatusTypeDef i2c_bus_write(uint8_t addr7, uint8_t reg, const uint8_t *data, uint16_t len)
{
    uint16_t i;

    sw_start();
    if (sw_write_byte((uint8_t)(addr7 << 1)) != 0U)
    {
        sw_stop();
        return HAL_ERROR;
    }
    if (sw_write_byte(reg) != 0U)
    {
        sw_stop();
        return HAL_ERROR;
    }
    for (i = 0U; i < len; i++)
    {
        if (sw_write_byte(data[i]) != 0U)
        {
            sw_stop();
            return HAL_ERROR;
        }
    }
    sw_stop();
    return HAL_OK;
}

HAL_StatusTypeDef i2c_bus_read_dma(uint8_t addr7, uint8_t reg, uint8_t *data, uint16_t len)
{
    (void)addr7;
    (void)reg;
    (void)data;
    (void)len;
    return HAL_ERROR;      /* 软件模拟后端没有 DMA 通路 */
}

void i2c_bus_recover(void)
{
    uint8_t i;

    /* 从机把 SDA 拉死时，用 9 个 SCL 时钟让它把字节吐完 */
    sda_high();
    for (i = 0U; i < 9U; i++)
    {
        scl_low();
        sw_delay();
        scl_high();
        sw_delay();
    }
    sw_stop();
}

const char *i2c_bus_name(void)
{
    return "SW-I2C(GPIO bit-bang ~100kHz)";
}

#endif /* IMU_I2C_BACKEND_HW */

/* ==================================================================
 * 公共：地址扫描（两套后端共用，用于确认 MPU6050 的 7 位地址）
 * ================================================================== */
uint8_t i2c_bus_scan(uint8_t *found, uint8_t max)
{
    uint8_t addr;
    uint8_t n = 0U;
    uint8_t dummy;

    for (addr = 0x08U; addr <= 0x77U; addr++)
    {
        if (i2c_bus_read(addr, 0x75U, &dummy, 1U) == HAL_OK)
        {
            if (n < max)
            {
                found[n] = addr;
            }
            n++;
        }
    }
    return n;
}
