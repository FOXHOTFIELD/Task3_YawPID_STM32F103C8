/**
 * @file    stm32f1xx_it.h
 * @brief   中断服务函数声明
 */
#ifndef __STM32F1xx_IT_H
#define __STM32F1xx_IT_H

#ifdef __cplusplus
extern "C" {
#endif

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void);
void EXTI9_5_IRQHandler(void);        /* MPU6050 数据就绪（PB5） */
void TIM4_IRQHandler(void);           /* 1ms 采样节拍 */
void USART1_IRQHandler(void);         /* 命令行接收 */
void I2C1_EV_IRQHandler(void);        /* 硬件 I2C 事件 */
void I2C1_ER_IRQHandler(void);        /* 硬件 I2C 错误 */
void DMA1_Channel6_IRQHandler(void);  /* I2C1_TX DMA */
void DMA1_Channel7_IRQHandler(void);  /* I2C1_RX DMA */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_IT_H */
