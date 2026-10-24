/**
 * @file    app_pid.c
 * @brief   PID 实现（位置式 / 增量式，限幅 + 条件积分抗饱和 + 微分先行）
 */
#include "app_pid.h"

static float pid_clamp(float v, float lo, float hi)
{
    if (v < lo)
    {
        return lo;
    }
    if (v > hi)
    {
        return hi;
    }
    return v;
}

void pid_init(pid_t *p, float kp, float ki, float kd,
              float out_min, float out_max, pid_mode_t mode)
{
    if (p == 0)
    {
        return;
    }
    p->kp        = kp;
    p->ki        = ki;
    p->kd        = kd;
    p->out_min   = out_min;
    p->out_max   = out_max;
    p->integ_min = -out_max;
    p->integ_max = out_max;
    p->d_lpf     = 1.0f;
    p->mode      = mode;
    pid_reset(p);
}

void pid_reset(pid_t *p)
{
    if (p == 0)
    {
        return;
    }
    p->integ    = 0.0f;
    p->out      = 0.0f;
    p->d_filt   = 0.0f;
    p->e_prev   = 0.0f;
    p->e_prev2  = 0.0f;
    p->meas_prev = 0.0f;
    p->ext_rate  = 0.0f;
    p->started  = 0U;
    p->saturated = 0U;
}

void pid_set_gains(pid_t *p, float kp, float ki, float kd)
{
    if (p == 0)
    {
        return;
    }
    p->kp = kp;
    p->ki = ki;
    p->kd = kd;
}

void pid_set_d_lpf(pid_t *p, float alpha)
{
    if (p == 0)
    {
        return;
    }
    p->d_lpf = pid_clamp(alpha, 0.0f, 1.0f);
}

void pid_set_integ_limit(pid_t *p, float lo, float hi)
{
    if (p == 0)
    {
        return;
    }
    p->integ_min = lo;
    p->integ_max = hi;
}

void pid_set_rate_source(pid_t *p, uint8_t use_external)
{
    if (p == 0)
    {
        return;
    }
    p->d_ext = (use_external != 0U) ? 1U : 0U;
    p->d_filt = 0.0f;
}

void pid_set_rate(pid_t *p, float rate_dps)
{
    if (p == 0)
    {
        return;
    }
    p->ext_rate = rate_dps;
}

/**
 * @brief 位置式：u = Kp*e + I + Kd*d(meas)/dt（微分先行）
 *
 * 抗饱和用"条件积分"：
 *   算出候选积分后再算输出，如果输出已经顶到限幅、而且误差方向还在
 *   继续把输出往饱和方向推，就放弃这一拍的积分累加（保留旧积分），
 *   于是误差反向时能立刻退出饱和，不会出现"积分撑爆后长时间回不来"。
 */
static float pid_update_position(pid_t *p, float err, float meas, float dt)
{
    float p_term;
    float i_cand;
    float d_raw;
    float out;

    if (p->started == 0U)
    {
        p->meas_prev = meas;
        p->started   = 1U;
    }

    /* 微分先行：对测量值求导并取负号，等价于 -Kd * d(meas)/dt
       有干净的速度观测（编码器/陀螺）时直接用，避免位置差分的量化噪声 */
    if (p->d_ext != 0U)
    {
        d_raw = -p->ext_rate;
        p->meas_prev = meas;
    }
    else
    {
        d_raw        = -(meas - p->meas_prev) / dt;
        p->meas_prev = meas;
    }
    p->d_filt   = (p->d_lpf * p->d_filt) + ((1.0f - p->d_lpf) * d_raw);

    p_term = p->kp * err;
    i_cand = pid_clamp(p->integ + (p->ki * err * dt), p->integ_min, p->integ_max);
    out    = p_term + i_cand + (p->kd * p->d_filt);

    if (out > p->out_max)
    {
        if (err <= 0.0f)            /* 误差在往回拉，允许积分（防止"卡死"在饱和） */
        {
            p->integ = i_cand;
        }
        out          = p->out_max;
        p->saturated = 1U;
    }
    else if (out < p->out_min)
    {
        if (err >= 0.0f)
        {
            p->integ = i_cand;
        }
        out          = p->out_min;
        p->saturated = 1U;
    }
    else
    {
        p->integ     = i_cand;
        p->saturated = 0U;
    }

    p->out = out;
    return out;
}

/**
 * @brief 增量式：Δu = Kp*Δe + Ki*e*dt + Kd*Δ²e/dt
 *
 * 增量式输出的是"每一步要加多少"，天然带积分作用（输出本身累加），
 * 不需要单独的积分寄存器，也就没有积分饱和问题；代价是
 * 输出限幅会丢掉一部分增量信息，且切换目标时更容易有冲击。
 */
static float pid_update_incremental(pid_t *p, float err, float dt)
{
    float d_err = err - p->e_prev;
    float d2    = err - (2.0f * p->e_prev) + p->e_prev2;
    float inc   = (p->kp * d_err) + (p->ki * err * dt) + (p->kd * d2 / dt);

    p->e_prev2 = p->e_prev;
    p->e_prev  = err;

    p->out = pid_clamp(p->out + inc, p->out_min, p->out_max);
    p->saturated = ((p->out >= p->out_max) || (p->out <= p->out_min)) ? 1U : 0U;
    return p->out;
}

float pid_update(pid_t *p, float sp, float meas, float dt)
{
    float err;

    if (p == 0)
    {
        return 0.0f;
    }
    if (dt <= 0.0f)
    {
        dt = 0.005f;    /* 兜底，正常情况下不会是 0 */
    }
    err = sp - meas;

    if (p->mode == PID_MODE_INCREMENTAL)
    {
        return pid_update_incremental(p, err, dt);
    }
    return pid_update_position(p, err, meas, dt);
}

const char *pid_mode_str(pid_mode_t m)
{
    return (m == PID_MODE_INCREMENTAL) ? "INC" : "POS";
}
