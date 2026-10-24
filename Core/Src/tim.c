/**
 * @file    tim.c
 * @brief   三个定时器各司其职
 *
 *  TIM3  -> 电机 PWM（CH1 = PA6，20kHz，3600 级占空比）
 *  TIM4  -> 1ms 控制节拍（更新中断，中断里只累加计数/置标志）
 *  TIM2  -> 编码器接口（CH1 = PA15、CH2 = PB3，TIM2 部分重映射，4 倍频计数）
 *
 * 为什么编码器用 TIM2 + 部分重映射：
 *   TIM3 已经拿去做 PWM，TIM4 的 CH1/CH2 又正好是 I2C1 的 PB6/PB7，
 *   只有 TIM2 还能做编码器接口；直连的 PA0/PA1 已经被按键占用，
 *   而部分重映射的 PA15/PB3 恰好是 JTAG 脚（本工程只保留 SWD），
 *   且这两个脚都是 5V 容忍脚，编码器用 5V 供电时也安全。
 */
#include "main.h"
#include "peripherals.h"
#include "board_config.h"

TIM_HandleTypeDef htim2;   /* 编码器接口 */
TIM_HandleTypeDef htim3;   /* 电机 PWM */
TIM_HandleTypeDef htim4;   /* 1ms 节拍 */

/* ==================================================================
 * TIM3：电机 PWM
 * ================================================================== */
void MX_TIM3_Init(void)
{
    TIM_OC_InitTypeDef sConfigOC = {0};

    htim3.Instance               = TIM3;
    htim3.Init.Prescaler         = 0;                    /* 72MHz 直接计数 */
    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim3.Init.Period            = TB_PWM_ARR;           /* 72MHz/3600 = 20kHz */
    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
    {
        Error_Handler();
    }

    sConfigOC.OCMode     = TIM_OCMODE_PWM1;
    sConfigOC.Pulse      = 0;                            /* 上电先给 0 占空比 */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ==================================================================
 * TIM4：1ms 控制节拍
 * ================================================================== */
void MX_TIM4_Init(void)
{
    htim4.Instance               = TIM4;
    htim4.Init.Prescaler         = 71;      /* 72MHz / 72 = 1MHz */
    htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim4.Init.Period            = 999;     /* 1000 个计数 = 1ms */
    htim4.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
    {
        Error_Handler();
    }
}

/* ==================================================================
 * TIM2：正交编码器接口
 * ================================================================== */
void MX_TIM2_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig = {0};

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 0;
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 0xFFFFU;   /* 16 位满量程，软件处理回绕 */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    /* TI1 与 TI2 都参与计数 -> 四倍频：A、B 每个边沿都计一个数。
       输入滤波 6（约 0.4us @72MHz）用来压掉电机换向的电刷噪声毛刺。 */
    sConfig.EncoderMode  = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter    = 6;
    sConfig.IC2Polarity  = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter    = 6;

    if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }
}
