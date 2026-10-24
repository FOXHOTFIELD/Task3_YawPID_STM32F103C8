/**
 * @file    gpio.c
 * @brief   GPIO 初始化：状态灯 PC13、TB6612 方向/使能、按键 PA0
 *
 * 注意：TB6612 的三根控制线（AIN1/AIN2/STBY）在初始化里先全部写成低电平，
 *       这样从复位到 app_init() 之间电机一定是"待机 + 滑行"，
 *       不会因为引脚浮空而自己转起来。
 */
#include "main.h"
#include "peripherals.h"
#include "board_config.h"

void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_AFIO_CLK_ENABLE();

    /* ---------------- 状态灯 PC13（低电平点亮） ---------------- */
    HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, LED1_OFF_LEVEL);
    GPIO_InitStruct.Pin   = LED1_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED1_PORT, &GPIO_InitStruct);

    /* ---------------- TB6612 AIN1 / AIN2 / STBY ---------------- */
    HAL_GPIO_WritePin(TB_AIN1_PORT, TB_AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TB_AIN2_PORT, TB_AIN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TB_STBY_PORT, TB_STBY_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

    GPIO_InitStruct.Pin = TB_AIN1_PIN;
    HAL_GPIO_Init(TB_AIN1_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = TB_AIN2_PIN;
    HAL_GPIO_Init(TB_AIN2_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin   = TB_STBY_PIN;
    HAL_GPIO_Init(TB_STBY_PORT, &GPIO_InitStruct);

    /* ---------------- 按键 PA0：下降沿中断 + 内部上拉 ----------------
     * 按键一端接 PA0，另一端接 GND；按下产生下降沿。
     * 消抖与短按/长按判定放在 app_key.c 里用节拍软件完成，中断里只记时间戳。
     */
    GPIO_InitStruct.Pin  = KEY_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    HAL_GPIO_Init(KEY_PORT, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(KEY_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(KEY_IRQn);
}
