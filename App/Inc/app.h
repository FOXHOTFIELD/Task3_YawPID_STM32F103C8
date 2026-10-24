/**
 * @file    app.h
 * @brief   业务层总入口：初始化、节拍、主循环、状态输出
 */
#ifndef APP_H
#define APP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void app_init(void);

/** @brief TIM4 每 1ms 调一次（中断上下文，只累加计数、置标志） */
void app_tick_1ms(void);

/** @brief 主循环：所有耗时动作都在这里按标志执行，没有阻塞延时 */
void app_loop(void);

/* ---------------- VOFA+ 数据流 ---------------- */
uint8_t app_stream_enabled(void);
void    app_set_stream(uint8_t en);
uint8_t app_raw_stream(void);        /**< 1 = 第三通道附带角速度 */
void    app_set_raw_stream(uint8_t en);
uint32_t app_stream_frames(void);

/* ---------------- 周期性状态行 ---------------- */
uint8_t app_stat_enabled(void);
void    app_set_stat(uint8_t en);
void    app_print_status(void);      /**< 单行状态（STAT/S 命令打印一次） */
void    app_print_info(void);        /**< 详细转储（INFO 命令） */

uint32_t app_uptime_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_H */
