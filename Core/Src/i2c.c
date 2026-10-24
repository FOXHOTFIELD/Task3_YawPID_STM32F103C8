/**
 * @file    i2c.c
 * @brief   硬件 I2C1 初始化：PB6 = SCL，PB7 = SDA，400kHz（Fast Mode）
 *
 * 为什么用 400kHz：MPU6050 支持 400kHz；一次 14 字节突发读约 0.5ms，
 * 100Hz 采样下占用总线时间不到 5%，留给命令回显和状态输出足够。
 * 若用软件模拟 I2C（board_config.h 里 IMU_I2C_BACKEND=0），本外设不会被使用，
 * 引脚由 app_i2c_bus.c 重新配置成开漏 GPIO。
 */
#include "main.h"
#include "peripherals.h"
#include "board_config.h"

I2C_HandleTypeDef hi2c1;

void MX_I2C1_Init(void)
{
    hi2c1.Instance             = I2C1;
    hi2c1.Init.ClockSpeed      = 400000;
    hi2c1.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1     = 0;
    hi2c1.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2     = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

    if (HAL_I2C_Init(&hi2c1) != HAL_OK)
    {
        Error_Handler();
    }
}
