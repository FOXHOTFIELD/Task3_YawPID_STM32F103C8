/**
 * @file    app_enc.h
 * @brief   正交编码器接口（TIM2 硬件编码器模式，A/B 四倍频）
 *
 * 引脚：A -> PA15（TIM2_CH1）、B -> PB3（TIM2_CH2），走 TIM2 部分重映射。
 * 这两个脚是 JTAG 脚，工程只保留 SWD，所以可以安全释放；同时它们都是
 * 5V 容忍脚，编码器用 5V 供电时直接接也不会打坏 MCU。
 *
 * 编码器在本工程里的用途（主反馈仍然是陀螺 Yaw，编码器只做辅助）：
 *   1) 堵转/断线检测：给了足够占空比但编码器不动 -> 判故障停机；
 *   2) 速度观测：给 STAT/INFO 输出转速，便于判断 PID 是否在"空转使劲"；
 *   3) 一致性自检：ECHO 命令把计数与 Yaw 一起打印，用来核对方向与减速比。
 */
#ifndef APP_ENC_H
#define APP_ENC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief 启动 TIM2 编码器接口并把计数清零 */
void enc_init(void);

/** @brief 计数清零（ZERO 时一并调用） */
void enc_reset(void);

/**
 * @brief 每控制周期调用一次：读 CNT、算增量与速度
 * @param dt_s 距上一拍的秒数
 */
void enc_update(float dt_s);

int32_t  enc_count(void);      /**< 累计计数（四倍频，含方向） */
int16_t  enc_delta(void);      /**< 最近一拍的增量 */
float    enc_speed_cps(void);  /**< 低通后的速度，单位：计数/秒 */
uint16_t enc_raw(void);        /**< TIM2 计数器原始值（调试用） */

#ifdef __cplusplus
}
#endif

#endif /* APP_ENC_H */
