/**
 * @file    app.c
 * @brief   调度层：1ms 节拍 -> 200Hz 控制 -> VOFA 上报 / 状态行 / 按键 / 灯 / 命令
 *
 * 时间驱动的部分全部由 TIM4 的 1ms 中断置标志（app_tick_1ms），
 * 主循环看到标志才做一件事，全程没有 HAL_Delay()，
 * 满足"主循环不得使用阻塞延时"的要求。
 */
#include "main.h"
#include "board_config.h"
#include "app.h"
#include "app_cli.h"
#include "app_ctl.h"
#include "app_enc.h"
#include "app_key.h"
#include "app_led.h"
#include "app_log.h"
#include "app_tb6612.h"
#include "app_util.h"
#include "app_vofa.h"

static volatile uint32_t s_tick;
static volatile uint8_t  s_ctrl_due;
static uint32_t s_ctrl_ticks;

static uint8_t  s_stream_on = 1U;
static uint8_t  s_raw_on;
static uint8_t  s_stat_on;
static uint32_t s_stream_frames;
static uint32_t s_stream_div_cnt;
static uint32_t s_last_stat_ms;

/* ==================================================================
 * 节拍
 * ================================================================== */
void app_tick_1ms(void)
{
    s_tick++;
    s_ctrl_ticks++;
    if (s_ctrl_ticks >= CTRL_PERIOD_MS)
    {
        s_ctrl_ticks = 0U;
        s_ctrl_due   = 1U;
    }
}

/**
 * @brief HAL 的定时器溢出回调（TIM4 的 1ms 节拍从这里进来）
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        app_tick_1ms();
    }
}

/* ==================================================================
 * 输出
 * ================================================================== */
void app_print_status(void)
{
    char b_tgt[16];
    char b_yaw[16];
    char b_err[16];
    char b_rate[16];
    char b_cont[16];
    char b_edeg[16];
    char b_iyaw[16];
    float left;

    vofa_format_float(b_tgt, ctl_target(), 3U);
    vofa_format_float(b_yaw, ctl_yaw(), 3U);
    vofa_format_float(b_err, ctl_error(), 3U);
    vofa_format_float(b_rate, ctl_rate_dps(), 3U);
    vofa_format_float(b_cont, ctl_yaw_cont(), 3U);
    vofa_format_float(b_edeg, ctl_enc_deg(), 3U);
    vofa_format_float(b_iyaw, ctl_imu_enc_diff(), 3U);

    /* left = 距离限位还剩多少度：调试时先看这个，别把排线拧到 0 */
    left = ctl_limit() - util_absf(ctl_yaw_cont());
    if (left < 0.0f)
    {
        left = 0.0f;
    }

    logln("STAT st=%s fb=%s tgt=%s yaw=%s cont=%s err=%s left=%d out=%d duty=%d rate=%s zt=%u enc=%ld edeg=%s dIE=%s",
          ctl_state_str(), ctl_fb_str(), b_tgt, b_yaw, b_cont, b_err,
          (int)left, (int)ctl_output_pm(), (int)ctl_duty_pm(), b_rate,
          (unsigned)ctl_zupt_active(), (long)enc_count(), b_edeg, b_iyaw);
}

void app_print_info(void)
{
    char b[16];

    logln("---- INFO ----");
    logln("fw   : Task3_YawPID %s", FW_VERSION_STR);
    logln("board: STM32F103C8T6 @72MHz, TB6612 + MPU6050 yaw turntable");
    logln("pins : PWM=PA6 AIN1=PA4 AIN2=PA5 STBY=PB0 ENC=PA15/PB3");
    logln("       SCL=PB6 SDA=PB7 TX=PA9 RX=PA10 LED=PC13 KEY=PA0");
    logln("pwm  : %uHz, ARR=%u", (unsigned)TB_PWM_FREQ_HZ, (unsigned)TB_PWM_ARR);
    logln("ctrl : %uHz (TIM4 1ms), stream=%uHz", (unsigned)CTRL_HZ, (unsigned)STREAM_HZ_DEFAULT);
    logln("state: %s fault=%s uptime=%lums", ctl_state_str(), ctl_fault_str(),
          (unsigned long)app_uptime_ms());

    {
        float kp;
        float ki;
        float kd;
        ctl_get_gains(&kp, &ki, &kd);
        vofa_format_float(b, kp, 3U);
        logln("pid  : mode=%s Kp=%s", pid_mode_str(ctl_pid_mode()), b);
        vofa_format_float(b, ki, 3U);
        logln("       Ki=%s", b);
        vofa_format_float(b, kd, 3U);
        logln("       Kd=%s", b);
    }

    vofa_format_float(b, ctl_limit(), 1U);
    logln("limit: +-%s deg (soft, margin %d deg)", b, (int)SOFT_LIMIT_MARGIN_DEG);
    logln("motor: deadzone=%dpm kick=%dpm/%ums invert=%u state=%s",
          (int)tb_deadzone(), (int)tb_kick_pm(), (unsigned)tb_kick_ms(),
          (unsigned)tb_invert(), tb_state_str());

    vofa_format_float(b, ctl_bias_dps(), 3U);
    logln("imu  : %s bias=%s dps cal_quality=%u errs=%lu", ctl_imu_status_str(), b,
          (unsigned)ctl_calib_quality(), (unsigned long)ctl_imu_errors());

    {
        char b_cpd[16];
        char b_edeg[16];
        char b_d[16];

        vofa_format_float(b_cpd, ctl_cpd(), 3U);
        vofa_format_float(b_edeg, ctl_enc_deg(), 3U);
        vofa_format_float(b_d, ctl_imu_enc_diff(), 3U);
        logln("fb   : %s cpd=%s sign=%d enc_deg=%s (imu-enc)=%s",
              ctl_fb_str(), b_cpd, (int)ctl_enc_sign(), b_edeg, b_d);
    }
    logln("stream: %s proto=%s raw=%u frames=%lu",
          (s_stream_on != 0U) ? "ON" : "OFF", vofa_mode_str(),
          (unsigned)s_raw_on, (unsigned long)s_stream_frames);
    logln("zupt enable=%u active=%u stall=%u enc_count=%ld cps=%ld",
          (unsigned)ctl_zupt(), (unsigned)ctl_zupt_active(), (unsigned)ctl_stall(),
          (long)enc_count(), (long)enc_speed_cps());
}

/* ==================================================================
 * 按键 / 状态灯
 * ================================================================== */
static void app_key_handle(void)
{
    key_evt_t evt = key_take_event();

    if (evt == KEY_EVT_SHORT)
    {
        if (ctl_state() == CTL_STATE_RUN)
        {
            ctl_stop();
            logln("KEY: STOP");
        }
        else if (ctl_state() == CTL_STATE_FAULT)
        {
            if (ctl_clear_fault() != 0U)
            {
                logln("KEY: fault cleared");
            }
            else
            {
                logln("KEY: still over limit, move it back first");
            }
        }
        else if (ctl_start() != 0U)
        {
            logln("KEY: START");
        }
        else
        {
            logln("KEY: not ready (calibrating or no IMU)");
        }
    }
    else if (evt == KEY_EVT_LONG)
    {
        ctl_request_calib();
        logln("KEY: recalibrate");
    }
    else
    {
        /* 没有事件 */
    }
}

static void app_led_refresh(void)
{
    switch (ctl_state())
    {
    case CTL_STATE_BOOT:
        led_set_mode(LED_MODE_OFF);
        break;
    case CTL_STATE_CALIB:
        led_set_mode(LED_MODE_FAST);
        break;
    case CTL_STATE_IDLE:
        led_set_mode(LED_MODE_SLOW);
        break;
    case CTL_STATE_RUN:
        led_set_mode(LED_MODE_SOLID);
        break;
    case CTL_STATE_FAULT:
    default:
        led_set_mode(LED_MODE_DOUBLE);
        break;
    }
}

/* ==================================================================
 * 初始化与主循环
 * ================================================================== */
void app_init(void)
{
    log_init();
    vofa_init();
    key_init();
    led_init();
    ctl_init();
    cli_init();

    logln("");
    logln("== 26级第三期：TB6612 + MPU6050 Yaw 闭环转盘 ==");
    logln(" PWM  PA6 (TIM3_CH1, %uHz)  AIN1 PA4  AIN2 PA5  STBY PB0",
          (unsigned)TB_PWM_FREQ_HZ);
    logln(" ENC  PA15/PB3 (TIM2 x4)     IMU  PB6/PB7 (400kHz)");
    logln(" UART PA9/PA10 115200 8N1    LED PC13   KEY PA0");
    logln(" VM=6~12V, VCC=3V3, GND common, 100uF+100nF at VM");
    logln(" type HELP for the command list");
    logln("");
}

void app_loop(void)
{
    uint32_t now;

    /* ---- 1. 200Hz 控制 + 100Hz 上报 ---- */
    if (s_ctrl_due != 0U)
    {
        s_ctrl_due = 0U;
        ctl_step();

        if (s_stream_on != 0U)
        {
            s_stream_div_cnt++;
            if (s_stream_div_cnt >= STREAM_DIV)
            {
                s_stream_div_cnt = 0U;
                if (s_raw_on != 0U)
                {
                    vofa_send_yaw3(ctl_target(), ctl_yaw(), ctl_rate_dps());
                }
                else
                {
                    vofa_send_yaw(ctl_target(), ctl_yaw());
                }
                s_stream_frames++;
            }
        }
    }

    /* ---- 2. 人机接口 ---- */
    led_task();
    key_task();
    app_key_handle();
    cli_poll();
    app_led_refresh();

    /* ---- 3. 1s 状态行 ---- */
    now = HAL_GetTick();
    if ((s_stat_on != 0U) && ((now - s_last_stat_ms) >= 1000U))
    {
        s_last_stat_ms = now;
        app_print_status();
    }
}

/* ==================================================================
 * 供 CLI 使用的小接口
 * ================================================================== */
uint8_t app_stream_enabled(void)
{
    return s_stream_on;
}

void app_set_stream(uint8_t en)
{
    s_stream_on = (en != 0U) ? 1U : 0U;
}

uint8_t app_raw_stream(void)
{
    return s_raw_on;
}

void app_set_raw_stream(uint8_t en)
{
    s_raw_on = (en != 0U) ? 1U : 0U;
}

uint32_t app_stream_frames(void)
{
    return s_stream_frames;
}

uint8_t app_stat_enabled(void)
{
    return s_stat_on;
}

void app_set_stat(uint8_t en)
{
    s_stat_on = (en != 0U) ? 1U : 0U;
    if (en != 0U)
    {
        s_last_stat_ms = HAL_GetTick();
    }
}

uint32_t app_uptime_ms(void)
{
    return s_tick;
}
