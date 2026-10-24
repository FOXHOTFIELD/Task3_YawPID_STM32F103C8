/**
 * @file    app_pid.h
 * @brief   PID 控制器：位置式 + 增量式，带输出限幅与积分抗饱和
 *
 * 评分点 3 要求"编写 PID 结构体和计算函数，了解位置式以及增量式，
 * 包含输出限幅与积分抗饱和，能现场说明 P、I、D 参数对系统响应的影响"。
 *
 * 两个重要的工程细节：
 * 1) 微分先行（derivative on measurement）：
 *    微分项对"测量值的变化"求导，而不是对"误差的变化"求导。
 *    这样改目标角时不会因为误差阶跃而产生一个巨大的微分冲击（setpoint kick）。
 * 2) 条件积分抗饱和（conditional integration）：
 *    输出已经饱和、且误差还在往饱和方向推时，停止积分累加；
 *    再叠加积分项自身的上下限，双重保险，防止长时间大误差把积分撑爆后回不来。
 */
#ifndef APP_PID_H
#define APP_PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    PID_MODE_POSITION    = 0,   /**< 位置式：u = Kp*e + ∫Ki*e + Kd*de/dt */
    PID_MODE_INCREMENTAL = 1    /**< 增量式：Δu = Kp*Δe + Ki*e*dt + Kd*Δ²e/dt */
} pid_mode_t;

typedef struct
{
    /* ---- 参数 ---- */
    float kp;
    float ki;
    float kd;
    float out_min;      /**< 输出下限（千分比，通常 -1000） */
    float out_max;      /**< 输出上限 */
    float integ_min;    /**< 积分项限幅下限（抗饱和） */
    float integ_max;    /**< 积分项限幅上限 */
    float d_lpf;        /**< 微分低通系数 0..1，越大越平滑；1 = 完全旁路 */
    pid_mode_t mode;

    /* ---- 状态 ---- */
    float   integ;      /**< 积分累加值（位置式） */
    float   out;        /**< 最近一次输出 */
    float   d_filt;     /**< 低通后的微分值 */
    float   e_prev;     /**< 上一次误差（增量式用） */
    float   e_prev2;    /**< 上上次误差（增量式二阶差分用） */
    float   meas_prev;  /**< 上一次测量值（微分先行用） */
    float   ext_rate;   /**< 外部提供的测量速度（°/s），见 pid_set_rate_source */
    uint8_t d_ext;      /**< 1 = 微分直接用 ext_rate；0 = 用 meas 差分 */
    uint8_t started;    /**< 0 = 还没跑过第一拍 */
    uint8_t saturated;  /**< 1 = 上一拍输出被限幅 */
} pid_t;

/** @brief 初始化参数与状态 */
void pid_init(pid_t *p, float kp, float ki, float kd,
              float out_min, float out_max, pid_mode_t mode);

/** @brief 清空积分与历史（切换目标、重新启动时调用） */
void pid_reset(pid_t *p);

/** @brief 在线修改三项增益（串口调参用），不清状态 */
void pid_set_gains(pid_t *p, float kp, float ki, float kd);

/** @brief 微分低通系数 0..1 */
void pid_set_d_lpf(pid_t *p, float alpha);

/** @brief 积分项限幅 */
void pid_set_integ_limit(pid_t *p, float lo, float hi);

/**
 * @brief 微分改用外部测速（编码器 / 陀螺）
 *
 * 位置差分在 5ms 周期上的量化是很粗的：编码器 1 个计数就是 14.7°/s 的台阶，
 * Kd 一放大，微分项就会被量化噪声打得乱跳。所以有干净的速度观测时，
 * 用 pid_set_rate 把速度喂进来，微分项直接用这个值。
 * 默认关闭（d_ext=0），单元测试仍走原来的差分路径。
 */
void pid_set_rate_source(pid_t *p, uint8_t use_external);

/** @brief 喂入测量速度（°/s），只有 pid_set_rate_source(p, 1) 后才被采用 */
void pid_set_rate(pid_t *p, float rate_dps);

/**
 * @brief 计算一拍
 * @param p    控制器
 * @param sp   目标值（度）
 * @param meas 测量值（度）
 * @param dt   采样周期（秒），必须 > 0
 * @return 限幅后的输出（千分比）
 */
float pid_update(pid_t *p, float sp, float meas, float dt);

/** @brief 模式名："POSITION" / "INCREMENTAL" */
const char *pid_mode_str(pid_mode_t m);

#ifdef __cplusplus
}
#endif

#endif /* APP_PID_H */
