/**
 * @file    app_yaw.h
 * @brief   Yaw（偏航角）积分与角度工具
 *
 * MPU6050 的陀螺 Z 轴与电机轴平行安装，绕竖直轴的角速度积分就是转盘的偏航角。
 * 这里维护两个量：
 *   cont —— 连续累计角（不折返），用来做限位判断：从 +170° 走最短路径到 -170°
 *           时它会走到 190°，排线的实际扭转量对应的是它，不是显示角；
 *   yaw  —— 折返到 [-180, 180) 的显示角，VOFA+ 画的就是这个。
 */
#ifndef APP_YAW_H
#define APP_YAW_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef struct
{
    float   cont;       /**< 连续累计角（度），可能超出 ±180 */
    float   yaw;        /**< 折返显示角（度），[-180, 180) */
    float   bias_dps;   /**< 陀螺零偏（°/s），积分前减掉 */
    float   rate_dps;   /**< 最近一次去偏后的角速度（°/s） */
    uint8_t started;
} yaw_t;

/** @brief 初始化，bias 用标定结果（未标定时给 0） */
void yaw_init(yaw_t *y, float bias_dps);

/** @brief 把当前角度直接置成 deg（ZERO 命令用） */
void yaw_reset(yaw_t *y, float deg);

/** @brief 更新零偏（重新标定后调用） */
void yaw_set_bias(yaw_t *y, float bias_dps);

/**
 * @brief 积分一拍
 * @param gz_dps 陀螺原始角速度（°/s，含零偏）
 * @param dt_s   距离上一拍的秒数；传 0 表示"冻结积分"（静止保持）
 */
void yaw_update(yaw_t *y, float gz_dps, float dt_s);

/** @brief 折返到 [-180, 180) */
float yaw_wrap180(float deg);

/** @brief 从 actual 转到 target 的最短路径误差，结果在 [-180, 180) */
float yaw_shortest_error(float target, float actual);

#ifdef __cplusplus
}
#endif

#endif /* APP_YAW_H */
