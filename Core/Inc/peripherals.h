/**
 * @file    peripherals.h
 * @brief   外设初始化函数声明（CubeMX 命名风格）
 */
#ifndef __PERIPHERALS_H
#define __PERIPHERALS_H

#ifdef __cplusplus
extern "C" {
#endif

void MX_GPIO_Init(void);
void MX_USART1_UART_Init(void);
void MX_I2C1_Init(void);    /* 硬件 I2C1：PB6=SCL / PB7=SDA，400kHz */
void MX_TIM2_Init(void);    /* 编码器：PA15/PB3，TIM2 部分重映射 */
void MX_TIM3_Init(void);    /* 电机 PWM：TIM3_CH1 = PA6，20kHz */
void MX_TIM4_Init(void);    /* 1ms 控制节拍 */

#ifdef __cplusplus
}
#endif

#endif /* __PERIPHERALS_H */
