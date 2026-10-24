/**
 * @file    app_yaw.c
 * @brief   Yaw 积分与角度工具（纯算法，可脱离硬件在 PC 上做单元测试）
 */
#include "app_yaw.h"

static float yaw_absf(float v)
{
    return (v < 0.0f) ? -v : v;
}

float yaw_wrap180(float deg)
{
    /* 正常情况累计角不会超过几百度，这里加一个上限保护，
       万一积分跑飞也不会陷入死循环 */
    if (deg > 1000000.0f)
    {
        deg = 1000000.0f;
    }
    if (deg < -1000000.0f)
    {
        deg = -1000000.0f;
    }
    while (deg >= 180.0f)
    {
        deg -= 360.0f;
    }
    while (deg < -180.0f)
    {
        deg += 360.0f;
    }
    return deg;
}

float yaw_shortest_error(float target, float actual)
{
    return yaw_wrap180(target - actual);
}

void yaw_init(yaw_t *y, float bias_dps)
{
    if (y == 0)
    {
        return;
    }
    y->cont     = 0.0f;
    y->yaw      = 0.0f;
    y->bias_dps = bias_dps;
    y->rate_dps = 0.0f;
    y->started  = 0U;
}

void yaw_reset(yaw_t *y, float deg)
{
    if (y == 0)
    {
        return;
    }
    y->cont = deg;
    y->yaw  = yaw_wrap180(deg);
}

void yaw_set_bias(yaw_t *y, float bias_dps)
{
    if (y == 0)
    {
        return;
    }
    y->bias_dps = bias_dps;
}

void yaw_update(yaw_t *y, float gz_dps, float dt_s)
{
    if (y == 0)
    {
        return;
    }

    /* 角速度先去掉零偏；不管有没有冻结积分，测量值都要更新 */
    y->rate_dps = gz_dps - y->bias_dps;

    if (dt_s > 0.0f)
    {
        if (y->started == 0U)
        {
            y->started = 1U;    /* 第一拍只建立基准，不积分 */
        }
        else
        {
            y->cont += (y->rate_dps * dt_s);
            y->yaw   = yaw_wrap180(y->cont);
        }
    }
}

/* 供 app_ctl 计算"静止质量"时使用，避免额外引入 libm */
float yaw_abs(float v)
{
    return yaw_absf(v);
}
