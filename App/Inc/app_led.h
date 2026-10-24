/**
 * @file    app_led.h
 * @brief   板载状态灯（PC13）的非阻塞闪烁
 *
 * 灯语：
 *   常亮       —— 闭环运行中（RUN）
 *   1Hz 慢闪   —— 就绪待机（IDLE）
 *   10Hz 快闪  —— 正在标定零偏（CALIB）
 *   双闪       —— 故障（FAULT），配合串口 FAULT:xxx 一起看
 *   熄灭       —— 上电初始化 / IMU 未就绪（BOOT）
 */
#ifndef APP_LED_H
#define APP_LED_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    LED_MODE_OFF   = 0,
    LED_MODE_SLOW  = 1,   /**< 1Hz */
    LED_MODE_FAST  = 2,   /**< 10Hz */
    LED_MODE_SOLID = 3,   /**< 常亮 */
    LED_MODE_DOUBLE = 4   /**< 每 1s 闪两下（故障） */
} led_mode_t;

void led_init(void);
void led_set_mode(led_mode_t m);
led_mode_t led_mode(void);

/** @brief 每 1ms 调一次 */
void led_task(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_LED_H */
