/**
 * @file    app_led.c
 * @brief   状态灯闪烁（1ms 节拍驱动，不占用任何延时）
 */
#include "main.h"
#include "board_config.h"
#include "app_led.h"

static led_mode_t s_mode = LED_MODE_OFF;
static uint32_t   s_ms;

static void led_write(uint8_t on)
{
    HAL_GPIO_WritePin(LED1_PORT, LED1_PIN, (on != 0U) ? LED1_ON_LEVEL : LED1_OFF_LEVEL);
}

void led_init(void)
{
    s_mode = LED_MODE_OFF;
    s_ms   = 0;
    led_write(0U);
}

void led_set_mode(led_mode_t m)
{
    if (m != s_mode)
    {
        s_mode = m;
        s_ms   = 0;
        led_write(0U);
    }
}

led_mode_t led_mode(void)
{
    return s_mode;
}

void led_task(void)
{
    uint32_t t;

    s_ms++;

    switch (s_mode)
    {
    case LED_MODE_OFF:
        led_write(0U);
        break;

    case LED_MODE_SOLID:
        led_write(1U);
        break;

    case LED_MODE_SLOW:
        led_write((uint8_t)(((s_ms % 1000U) < 500U) ? 1U : 0U));
        break;

    case LED_MODE_FAST:
        led_write((uint8_t)(((s_ms % 100U) < 50U) ? 1U : 0U));
        break;

    case LED_MODE_DOUBLE:
        t = s_ms % 1000U;
        if ((t < 100U) || ((t >= 200U) && (t < 300U)))
        {
            led_write(1U);
        }
        else
        {
            led_write(0U);
        }
        break;

    default:
        led_write(0U);
        break;
    }
}
