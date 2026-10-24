/**
 * @file    stm32f1xx_hal_msp.c
 * @brief   HAL 底层初始化：时钟、引脚复用、NVIC
 *
 * 本工程串口与 I2C 都用轮询方式，所以这里没有 DMA 与 I2C 中断，
 * 唯一的硬件中断是 TIM4（1ms 节拍）、USART1（接收 1 字节）和 EXTI0（按键）。
 */
#include "main.h"
#include "board_config.h"

extern I2C_HandleTypeDef hi2c1;

void HAL_MspInit(void)
{
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    /* 只用 SWD 调试，关掉 JTAG：
       - 释放 PA15/PB3/PB4，其中 PA15/PB3 正好给 TIM2 编码器用；
       - PA13/PA14 的 SWD 口继续保留，烧写调试不受影响。 */
    __HAL_AFIO_REMAP_SWJ_NOJTAG();
}

/**
 * @brief USART1 底层：PA9 = TX（复用推挽），PA10 = RX（上拉输入）
 */
void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (huart->Instance == USART1)
    {
        __HAL_RCC_USART1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin   = GPIO_PIN_9;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin  = GPIO_PIN_10;
        GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        HAL_NVIC_SetPriority(USART1_IRQn, 3, 0);
        HAL_NVIC_EnableIRQ(USART1_IRQn);
    }
}

/**
 * @brief TIM4（时基）底层：只需要开时钟和更新中断
 */
void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim_base)
{
    if (htim_base->Instance == TIM4)
    {
        __HAL_RCC_TIM4_CLK_ENABLE();
        HAL_NVIC_SetPriority(TIM4_IRQn, 2, 0);
        HAL_NVIC_EnableIRQ(TIM4_IRQn);
    }
}

/**
 * @brief TIM3（PWM）底层：PA6 复用推挽输出
 * @note  F1 的 HAL 没有 HAL_TIM_PWM_MspPostInit 钩子，
 *        复用配置必须写在 HAL_TIM_PWM_MspInit 里，漏了就没有波形。
 */
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (htim->Instance == TIM3)
    {
        __HAL_RCC_TIM3_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        GPIO_InitStruct.Pin   = TB_PWM_PIN;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull  = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(TB_PWM_PORT, &GPIO_InitStruct);
    }
}

/**
 * @brief TIM2（编码器）底层：PA15 = CH1、PB3 = CH2，需要 TIM2 部分重映射
 * @note  重映射必须在这里做（AFIO 时钟已在 HAL_MspInit 里打开）。
 *        两个脚都是 5V 容忍脚；输入用上拉，兼容开漏输出的霍尔编码器。
 */
void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef *htim)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (htim->Instance == TIM2)
    {
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();

        /* TIM2_REMAP = 01：CH1/ETR -> PA15，CH2 -> PB3 */
        __HAL_AFIO_REMAP_TIM2_PARTIAL_1();

        GPIO_InitStruct.Pin   = GPIO_PIN_15;          /* PA15 = ENC_A */
        GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
        GPIO_InitStruct.Pull  = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin   = GPIO_PIN_3;           /* PB3 = ENC_B */
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}

/**
 * @brief I2C1 底层：PB6/PB7 开漏复用
 * @note  I2C 是开漏总线，必须用 AF_OD 模式；GY-521 模块板载 4.7k 上拉，
 *        自制板需要外接。本工程用轮询收发，不挂 DMA、不开 I2C 中断。
 */
void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    if (hi2c->Instance == I2C1)
    {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_I2C1_CLK_ENABLE();

        GPIO_InitStruct.Pin   = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode  = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull  = GPIO_PULLUP;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    }
}
