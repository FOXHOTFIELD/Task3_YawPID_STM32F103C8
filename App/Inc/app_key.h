/**
 * @file    app_key.h
 * @brief   按键 PA0：短按暂停/继续、长按重新标定（全部非阻塞）
 *
 * 中断里只记一个时间戳，消抖与长短按判定在主循环里按 1ms 节拍完成。
 */
#ifndef APP_KEY_H
#define APP_KEY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    KEY_EVT_NONE  = 0,
    KEY_EVT_SHORT = 1,   /**< 短按（< KEY_LONG_MS） */
    KEY_EVT_LONG  = 2    /**< 长按（>= KEY_LONG_MS） */
} key_evt_t;

void key_init(void);

/** @brief EXTI0 中断里调用（HAL_GPIO_EXTI_Callback 转发） */
void key_exti_cb(void);

/** @brief 每 1ms 调一次：消抖 + 判定 */
void key_task(void);

/** @brief 取走一个事件（取走后清零） */
key_evt_t key_take_event(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_KEY_H */
