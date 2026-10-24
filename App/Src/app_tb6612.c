/**
 * @file    app_tb6612.c
 * @brief   TB6612FNG 驱动实现（含死区补偿与可选启动"踢一脚"）
 */
#include "main.h"
#include "board_config.h"
#include "app_tb6612.h"

static int16_t    s_cmd;
static int16_t    s_duty;
static tb_state_t s_state = TB_STATE_STANDBY;
static int16_t    s_dz    = MOTOR_DEADZONE_PM;
static uint8_t    s_inv   = MOTOR_DIR_INVERT;
static int16_t    s_kick_pm = MOTOR_KICK_PM;
static uint16_t   s_kick_ms = MOTOR_KICK_MS;
static uint32_t   s_kick_until;
static int8_t     s_last_dir;

/* 千分比 -> CCR 比较值（四舍五入，0..ARR） */
static uint16_t pm_to_ccr(int16_t pm)
{
    uint32_t p = (pm < 0) ? (uint32_t)(-(int32_t)pm) : (uint32_t)pm;

    if (p > 1000U)
    {
        p = 1000U;
    }
    return (uint16_t)((p * (uint32_t)TB_PWM_ARR + 500U) / 1000U);
}

/* 一次写齐方向脚与占空比 */
static void tb_write_pins(uint8_t in1, uint8_t in2, uint16_t ccr)
{
    HAL_GPIO_WritePin(TB_AIN1_PORT, TB_AIN1_PIN, (in1 != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(TB_AIN2_PORT, TB_AIN2_PIN, (in2 != 0U) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(TB_PWM_TIM, TB_PWM_TIM_CH, ccr);
}

static uint8_t tb_kick_active(void)
{
    if (s_kick_ms == 0U || s_kick_pm <= 0)
    {
        return 0U;
    }
    /* 用有符号差值比较，避免 HAL_GetTick() 回绕出问题 */
    return ((int32_t)(HAL_GetTick() - s_kick_until) < 0) ? 1U : 0U;
}

void tb_init(void)
{
    s_cmd      = 0;
    s_duty     = 0;
    s_state    = TB_STATE_STANDBY;
    s_last_dir = 0;

    tb_write_pins(0U, 0U, 0U);
    HAL_GPIO_WritePin(TB_STBY_PORT, TB_STBY_PIN, GPIO_PIN_RESET);

    if (HAL_TIM_PWM_Start(TB_PWM_TIM, TB_PWM_TIM_CH) != HAL_OK)
    {
        Error_Handler();
    }
}

void tb_standby(void)
{
    tb_write_pins(0U, 0U, 0U);
    HAL_GPIO_WritePin(TB_STBY_PORT, TB_STBY_PIN, GPIO_PIN_RESET);
    s_cmd   = 0;
    s_duty  = 0;
    s_state = TB_STATE_STANDBY;
}

void tb_coast(void)
{
    HAL_GPIO_WritePin(TB_STBY_PORT, TB_STBY_PIN, GPIO_PIN_SET);
    tb_write_pins(0U, 0U, 0U);
    s_cmd   = 0;
    s_duty  = 0;
    s_state = TB_STATE_COAST;
}

void tb_brake(void)
{
    HAL_GPIO_WritePin(TB_STBY_PORT, TB_STBY_PIN, GPIO_PIN_SET);
    tb_write_pins(1U, 1U, (uint16_t)TB_PWM_ARR);   /* AIN1=AIN2=1 短路刹车 */
    s_cmd   = 0;
    s_duty  = 1000;
    s_state = TB_STATE_BRAKE;
}

void tb_set_cmd(int16_t cmd_pm)
{
    int16_t  mag;
    int16_t  duty;
    uint8_t  forward;
    int8_t   dir;

    if (cmd_pm > 1000)
    {
        cmd_pm = 1000;
    }
    if (cmd_pm < -1000)
    {
        cmd_pm = -1000;
    }
    if (s_inv != 0U)
    {
        cmd_pm = (int16_t)(-cmd_pm);
    }

    s_cmd = cmd_pm;
    mag   = (cmd_pm >= 0) ? cmd_pm : (int16_t)(-cmd_pm);
    dir   = (cmd_pm > 0) ? (int8_t)1 : ((cmd_pm < 0) ? (int8_t)-1 : (int8_t)0);

    /* 力度太小：直接滑行，避免零附近反复换向抖动 */
    if (mag <= MOTOR_CMD_MIN_PM)
    {
        tb_coast();
        s_last_dir = 0;
        return;
    }

    /* ---- 死区补偿：把 (0..1000] 线性映射到 [死区..1000] ----
     * 这样 PID 给出任何非零力度时，实际占空比都高于启动阈值，
     * 电机一定能转。缺点是小力度对应更大的实际占空比，
     * 所以 PID 的 Kp 不要调得太大，否则会"一跳一跳"。 */
    duty = (int16_t)(s_dz + (int16_t)(((int32_t)mag * (int32_t)(1000 - s_dz)) / 1000));
    if (duty > 1000)
    {
        duty = 1000;
    }

    /* ---- 可选：换向瞬间踢一脚，帮电机越过静摩擦 ---- */
    if ((s_kick_ms > 0U) && (dir != s_last_dir) && (dir != 0))
    {
        s_kick_until = HAL_GetTick() + (uint32_t)s_kick_ms;
    }
    s_last_dir = dir;
    if (tb_kick_active() != 0U && duty < s_kick_pm)
    {
        duty = s_kick_pm;
    }

    forward = (dir > 0) ? 1U : 0U;
    HAL_GPIO_WritePin(TB_STBY_PORT, TB_STBY_PIN, GPIO_PIN_SET);
    if (forward != 0U)
    {
        tb_write_pins(1U, 0U, pm_to_ccr(duty));
        s_state = TB_STATE_FWD;
    }
    else
    {
        tb_write_pins(0U, 1U, pm_to_ccr(duty));
        s_state = TB_STATE_REV;
    }
    s_duty = duty;
}

int16_t tb_cmd(void)
{
    return s_cmd;
}

int16_t tb_duty_pm(void)
{
    return s_duty;
}

tb_state_t tb_state(void)
{
    return s_state;
}

const char *tb_state_str(void)
{
    switch (s_state)
    {
    case TB_STATE_STANDBY: return "STANDBY";
    case TB_STATE_COAST:   return "COAST";
    case TB_STATE_FWD:     return "FWD";
    case TB_STATE_REV:     return "REV";
    case TB_STATE_BRAKE:   return "BRAKE";
    default:               return "?";
    }
}

void tb_set_deadzone(int16_t pm)
{
    if (pm < 0)
    {
        pm = 0;
    }
    if (pm > 600)
    {
        pm = 600;   /* 再大就没法调速了，给出硬上限 */
    }
    s_dz = pm;
}

int16_t tb_deadzone(void)
{
    return s_dz;
}

void tb_set_invert(uint8_t inv)
{
    s_inv = (inv != 0U) ? 1U : 0U;
}

uint8_t tb_invert(void)
{
    return s_inv;
}

void tb_set_kick(int16_t pm, uint16_t ms)
{
    if (pm < 0)
    {
        pm = 0;
    }
    if (pm > 1000)
    {
        pm = 1000;
    }
    s_kick_pm = pm;
    s_kick_ms = ms;
}

int16_t tb_kick_pm(void)
{
    return s_kick_pm;
}

uint16_t tb_kick_ms(void)
{
    return s_kick_ms;
}
