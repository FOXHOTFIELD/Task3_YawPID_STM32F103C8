/**
 * @file    main.c
 * @brief   26级第三期考核任务：TB6612 电机驱动与水平转盘偏航角闭环控制
 * @target  STM32F103C8T6 @ 72MHz（HSE 8MHz，PLL x9）
 *
 * 外设分工：
 *   TIM3_CH1/PA6  -> 电机 PWM（20kHz）
 *   TIM4          -> 1ms 控制节拍（200Hz 控制周期）
 *   TIM2          -> 正交编码器（PA15/PB3，部分重映射，4 倍频）
 *   I2C1/PB6,PB7  -> MPU6050（400kHz）
 *   USART1/PA9,PA10 -> 115200，命令 + VOFA+ 数据流
 *   EXTI0/PA0     -> 按键（短按启停、长按重新标定）
 */
#include "main.h"
#include "peripherals.h"
#include "board_config.h"
#include "app.h"

static void SystemClock_Config(void);

int main(void)
{
    HAL_Init();             /* 内部会调用 HAL_MspInit：关 JTAG、只留 SWD */
    SystemClock_Config();

    MX_GPIO_Init();         /* 灯、TB6612 方向/使能、按键 */
    MX_USART1_UART_Init();
    MX_I2C1_Init();         /* 硬件 I2C1，软件模拟后端会自己接管引脚 */
    MX_TIM3_Init();         /* 电机 PWM */
    MX_TIM2_Init();         /* 编码器接口 */
    MX_TIM4_Init();         /* 1ms 节拍 */

    app_init();             /* 探测 IMU、启动 PWM/编码器、进入零偏标定 */

    HAL_TIM_Base_Start_IT(TICK_TIM);   /* 节拍最后启动，保证初始化期间不打断 */

    while (1)
    {
        app_loop();
    }
}

/**
 * @brief 系统时钟：HSE 8MHz x PLL9 = 72MHz
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                     | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;    /* TIM2/3/4 仍是 72MHz */
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

void Error_Handler(void)
{
    __disable_irq();
    while (1)
    {
    }
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
    (void)file;
    (void)line;
    Error_Handler();
}
#endif
