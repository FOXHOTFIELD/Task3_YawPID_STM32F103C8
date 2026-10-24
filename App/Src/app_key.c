/**
 * @file    app_key.c
 * @brief   按键消抖与长短按判定（EXTI0 + 1ms 轮询，非阻塞）
 */
#include "main.h"
#include "board_config.h"
#include "app_key.h"

static volatile uint8_t s_edge;          /* 中断置位：来了一个下降沿 */
static uint32_t s_last_edge_ms;          /* 上一次有效边沿（消抖） */
static uint8_t  s_pressed;               /* 1 = 正在按下 */
static uint32_t s_down_ms;               /* 按下时刻 */
static uint32_t s_release_ms;            /* 释放稳定计时 */
static uint8_t  s_long_fired;
static key_evt_t s_event;

void key_init(void)
{
    s_edge        = 0;
    s_last_edge_ms = 0;
    s_pressed     = 0;
    s_down_ms     = 0;
    s_release_ms  = 0;
    s_long_fired  = 0;
    s_event       = KEY_EVT_NONE;
}

void key_exti_cb(void)
{
    s_edge = 1;
}

void key_task(void)
{
    uint32_t now = HAL_GetTick();

    /* ---- 1. 处理下降沿（消抖窗口内重复的边沿直接丢弃） ---- */
    if (s_edge != 0U)
    {
        s_edge = 0;
        if ((now - s_last_edge_ms) >= KEY_DEBOUNCE_MS)
        {
            s_last_edge_ms = now;
            if (s_pressed == 0U)
            {
                s_pressed    = 1;
                s_down_ms    = now;
                s_release_ms = 0;
                s_long_fired = 0;
            }
        }
    }

    if (s_pressed == 0U)
    {
        return;
    }

    /* ---- 2. 长按：按住期间只报一次 ---- */
    if ((s_long_fired == 0U) && ((now - s_down_ms) >= KEY_LONG_MS))
    {
        s_long_fired = 1;
        s_event      = KEY_EVT_LONG;
    }

    /* ---- 3. 释放：引脚被内部上拉回高电平，且稳定 KEY_DEBOUNCE_MS ---- */
    if (HAL_GPIO_ReadPin(KEY_PORT, KEY_PIN) == GPIO_PIN_SET)
    {
        if (s_release_ms == 0U)
        {
            s_release_ms = now;
        }
        else if ((now - s_release_ms) >= KEY_DEBOUNCE_MS)
        {
            if ((s_long_fired == 0U) && (s_event == KEY_EVT_NONE))
            {
                s_event = KEY_EVT_SHORT;
            }
            s_pressed    = 0;
            s_release_ms = 0;
        }
    }
    else
    {
        s_release_ms = 0;
    }
}

key_evt_t key_take_event(void)
{
    key_evt_t e = s_event;

    s_event = KEY_EVT_NONE;
    return e;
}

/**
 * @brief HAL 的 EXTI 回调（EXTI0 触发后由 HAL_GPIO_EXTI_IRQHandler 调用）
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == KEY_PIN)
    {
        key_exti_cb();
    }
}
