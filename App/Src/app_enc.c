/**
 * @file    app_enc.c
 * @brief   TIM2 正交编码器接口（4 倍频计数 + 速度观测）
 */
#include "main.h"
#include "board_config.h"
#include "app_enc.h"

static int32_t s_count;
static int16_t s_delta;
static float   s_speed_cps;
static int16_t s_raw_prev;

void enc_init(void)
{
    s_count     = 0;
    s_delta     = 0;
    s_speed_cps = 0.0f;
    s_raw_prev  = 0;

    __HAL_TIM_SET_COUNTER(ENC_TIM, 0U);

    if (HAL_TIM_Encoder_Start(ENC_TIM, TIM_CHANNEL_ALL) != HAL_OK)
    {
        Error_Handler();
    }
}

void enc_reset(void)
{
    s_count     = 0;
    s_delta     = 0;
    s_speed_cps = 0.0f;
    s_raw_prev  = 0;
    __HAL_TIM_SET_COUNTER(ENC_TIM, 0U);
}

void enc_update(float dt_s)
{
    /* 16 位计数器会回绕：先转成有符号再相减，回绕自动正确 */
    int16_t raw = (int16_t)__HAL_TIM_GET_COUNTER(ENC_TIM);
    int16_t d   = (int16_t)(raw - s_raw_prev);

    s_raw_prev = raw;
    s_delta    = d;
    s_count   += (int32_t)d;

    if (dt_s > 0.0f)
    {
        float inst = (float)d / dt_s;
        /* 一阶低通（系数 0.5），把计数的量化噪声压下去 */
        s_speed_cps = (0.5f * s_speed_cps) + (0.5f * inst);
    }
}

int32_t enc_count(void)
{
    return s_count;
}

int16_t enc_delta(void)
{
    return s_delta;
}

float enc_speed_cps(void)
{
    return s_speed_cps;
}

uint16_t enc_raw(void)
{
    return (uint16_t)__HAL_TIM_GET_COUNTER(ENC_TIM);
}
