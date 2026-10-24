/**
 * @file    stm32f1xx_it.c
 * @brief   中断服务函数：只转交给 HAL，具体处理放在回调里
 */
#include "main.h"
#include "stm32f1xx_it.h"
#include "board_config.h"

extern UART_HandleTypeDef huart1;

void NMI_Handler(void)
{
    while (1)
    {
    }
}

void HardFault_Handler(void)
{
    while (1)
    {
    }
}

void MemManage_Handler(void)
{
    while (1)
    {
    }
}

void BusFault_Handler(void)
{
    while (1)
    {
    }
}

void UsageFault_Handler(void)
{
    while (1)
    {
    }
}

void SVC_Handler(void)
{
}

void DebugMon_Handler(void)
{
}

void PendSV_Handler(void)
{
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}

/**
 * @brief 1ms 控制节拍：HAL 回调里再分频出 200Hz 控制周期
 */
void TIM4_IRQHandler(void)
{
    HAL_TIM_IRQHandler(TICK_TIM);
}

/**
 * @brief 按键 PA0（EXTI0）：HAL 会转到 HAL_GPIO_EXTI_Callback（在 app_key.c）
 */
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(KEY_PIN);
}

void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}
