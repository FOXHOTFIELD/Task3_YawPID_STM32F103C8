/**
 * @file    app_ctl.h
 * @brief   闭环控制核心：状态机 + 陀螺采样 + Yaw 积分 + PID + 电机输出
 *
 * 状态机：
 *   BOOT  —— 上电后探测 MPU6050（失败每秒重试一次），成功即进 CALIB
 *   CALIB —— 静止 1.5s 采陀螺零偏（非阻塞，靠控制节拍累加），完成后 Yaw 归零
 *   IDLE  —— 电机断电滑行，Yaw 仍然实时积分（好看、也能随时 START）
 *   RUN   —— 200Hz 闭环：读陀螺 -> 积分 Yaw -> 最短路径误差 -> PID -> PWM
 *   FAULT —— 越限 / 堵转 / 传感器失效，电机刹车，必须 STOP(或 CLEAR) 才能解除
 *
 * 任务要求"主循环不得使用阻塞延时"：所有的时间推进都由 TIM4 的 1ms 中断
 * 置标志，主循环只在标志到来时做一次计算，标定、日志、闪烁全是非阻塞状态机。
 */
#ifndef APP_CTL_H
#define APP_CTL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "app_pid.h"

typedef enum
{
    CTL_STATE_BOOT  = 0,   /**< 等待/重试 IMU 初始化 */
    CTL_STATE_CALIB = 1,   /**< 陀螺零偏标定中 */
    CTL_STATE_IDLE  = 2,   /**< 就绪但未闭环，电机断电 */
    CTL_STATE_RUN   = 3,   /**< 闭环运行 */
    CTL_STATE_FAULT = 4    /**< 故障停机 */
} ctl_state_t;

typedef enum
{
    CTL_FAULT_NONE   = 0,
    CTL_FAULT_LIMIT  = 1,   /**< 软件限位触发（排线保护） */
    CTL_FAULT_STALL  = 2,   /**< 堵转：给力了但编码器不动 */
    CTL_FAULT_IMU    = 3    /**< IMU 通讯失败 */
} ctl_fault_t;

/**
 * @brief 反馈源：闭环用的角度从哪来
 *
 * 编码器装在电机轴（减速箱之前），折算后约 0.07°/count，不漂；
 * 陀螺 Yaw 平滑但零偏会漂。先用 ENC 把机械和增益调对，再切 IMU/FUSE。
 */
typedef enum
{
    CTL_FB_IMU  = 0,   /**< 只用陀螺积分 Yaw（任务书要求的主反馈） */
    CTL_FB_ENC  = 1,   /**< 只用编码器角度（不漂，调参用） */
    CTL_FB_FUSE = 2    /**< 互补滤波：陀螺高频 + 编码器低频修正 */
} ctl_fb_t;

/* ---------------- 生命周期 ---------------- */
void ctl_init(void);        /**< 初始化电机/编码器/IMU，进入标定 */
void ctl_step(void);        /**< 200Hz 控制节拍，主循环调用 */

/* ---------------- 命令 ---------------- */
void    ctl_request_calib(void);          /**< 重新标定零偏（会先停机） */
void    ctl_zero_yaw(void);               /**< 当前角置 0，目标角同步平移 */
uint8_t ctl_set_target(float deg);        /**< SET=<角度>；0 = 超出限位被拒绝 */
uint8_t ctl_start(void);                  /**< START；0 = 条件不满足 */
void    ctl_stop(void);                   /**< STOP：退出闭环，电机滑行 */
uint8_t ctl_clear_fault(void);            /**< 清故障回到 IDLE；0 = 当前无故障 */
uint8_t ctl_manual(int16_t pm);           /**< 开环点动（仅 IDLE），用于验证正反转/刹车 */
void    ctl_manual_release(void);         /**< 退出开环 */

/* ---------------- 参数 ---------------- */
void        ctl_set_gains(float kp, float ki, float kd);
void        ctl_get_gains(float *kp, float *ki, float *kd);
pid_mode_t  ctl_pid_mode(void);
void        ctl_set_pid_mode(pid_mode_t m);
uint8_t     ctl_set_limit(float deg);     /**< 软件限位（度）；0 = 越界 */
float       ctl_limit(void);
void        ctl_set_zupt(uint8_t en);
uint8_t     ctl_zupt(void);               /**< 静止保持功能是否开启 */
uint8_t     ctl_zupt_active(void);        /**< 1 = 当前真的在冻结积分 */
void        ctl_set_bias_auto(uint8_t en);/**< 怠速零偏在线跟踪开关 */
uint8_t     ctl_bias_auto(void);
void        ctl_set_out_max(int16_t pm);   /**< 输出限幅 OMAX（‰） */
int16_t     ctl_out_max(void);
void        ctl_set_stall(uint8_t en);    /**< 堵转检测开关（默认关，接好编码器再开） */
uint8_t     ctl_stall(void);

/* ---------------- 反馈源 / 编码器标定 ---------------- */
uint8_t         ctl_set_fb(ctl_fb_t fb);     /**< 切反馈源；0 = 当前状态不允许 */
ctl_fb_t        ctl_fb(void);
const char     *ctl_fb_str(void);
const char     *ctl_fb_str_of(ctl_fb_t fb);
void            ctl_set_cpd(float cpd);      /**< 编码器每转盘度的计数 */
float           ctl_cpd(void);
void            ctl_set_enc_sign(float s);   /**< ±1，编码器方向 */
float           ctl_enc_sign(void);
float           ctl_enc_deg(void);           /**< 编码器连续角（度） */
float           ctl_enc_rate_dps(void);      /**< 编码器测得的角速度 */
float           ctl_fb_deg(void);            /**< 当前闭环真正用的连续角 */
float           ctl_imu_enc_diff(void);      /**< 陀螺角 - 编码器角，看漂移用 */

/* ---------------- 观测 ---------------- */
ctl_state_t     ctl_state(void);
const char     *ctl_state_str(void);
ctl_fault_t     ctl_fault(void);
const char     *ctl_fault_str(void);
float           ctl_target(void);
float           ctl_yaw(void);
float           ctl_yaw_cont(void);
float           ctl_rate_dps(void);
float           ctl_bias_dps(void);
float           ctl_error(void);
int16_t         ctl_output_pm(void);
int16_t         ctl_duty_pm(void);
uint32_t        ctl_ctrl_count(void);
uint8_t         ctl_saturated(void);
uint8_t         ctl_ready(void);          /**< 标定完成、可 START */
uint8_t         ctl_imu_ok(void);
uint8_t         ctl_calib_quality(void);  /**< 标定期间静止质量 0..100 */
uint32_t        ctl_imu_errors(void);
const char     *ctl_imu_status_str(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CTL_H */
