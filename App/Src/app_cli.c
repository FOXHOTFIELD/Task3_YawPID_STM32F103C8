/**
 * @file    app_cli.c
 * @brief   串口命令行实现（环形缓冲 + 逐行解析 + 命令分发）
 *
 * 评分点 4 要求完成 SET=<角度>、START、STOP 的解析；
 * 发挥项 1 要求能通过串口调 PID 三个值并回显。这里把两类都做全：
 *   SET/START/STOP/CAL/ZERO/STREAM/PROTO/RAW/STAT/INFO/HELP
 *   KP=/KI=/KD=/PID/MODE=/LIMIT=/ZUPT=/STALL=/INV=/DZ=/KICK=
 *   FWD/REV/BRK/COAST/DUTY=（开环验证正反转与刹车）
 *   所有非法输入统一回 "ERR:<原因>"，方便上位机脚本判定。
 */
#include "main.h"
#include "board_config.h"
#include "app.h"
#include "app_cli.h"
#include "app_ctl.h"
#include "app_enc.h"
#include "app_led.h"
#include "app_log.h"
#include "app_tb6612.h"
#include "app_util.h"
#include "app_vofa.h"

#include <string.h>

#define CLI_RING_SIZE    256U
#define CLI_LINE_MAX     96U
#define CLI_DEFAULT_PM   400    /* FWD/REV 默认力度 */

static volatile uint8_t  s_ring[CLI_RING_SIZE];
static volatile uint16_t s_head;
static volatile uint16_t s_tail;

static uint8_t s_rx_byte;
static char    s_line[CLI_LINE_MAX];
static uint8_t s_line_len;
static uint8_t s_discard;      /* 1 = 本行已超长，丢弃到换行为止 */
static uint8_t s_overflow;

/* ==================================================================
 * 接收：中断里只入队
 * ================================================================== */
void cli_rx_byte(uint8_t b)
{
    uint16_t next = (uint16_t)((s_head + 1U) % CLI_RING_SIZE);

    if (next != s_tail)
    {
        s_ring[s_head] = b;
        s_head = next;
    }
    else
    {
        s_overflow = 1U;
    }
}

uint8_t cli_overflow(void)
{
    uint8_t v = s_overflow;

    s_overflow = 0U;
    return v;
}

void cli_init(void)
{
    s_head     = 0U;
    s_tail     = 0U;
    s_line_len = 0U;
    s_discard  = 0U;
    s_overflow = 0U;

    (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
}

/**
 * @brief HAL 的接收完成回调：把字节丢进环形缓冲并重新武装
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        cli_rx_byte(s_rx_byte);
        (void)HAL_UART_Receive_IT(&huart1, &s_rx_byte, 1U);
    }
}

/* ==================================================================
 * 回复工具
 * ================================================================== */
static void reply_err(const char *why)
{
    logln("ERR:%s", why);
}

static void reply_ok_float(const char *key, float v)
{
    char b[16];

    vofa_format_float(b, v, 3U);
    logln("OK:%s=%s", key, b);
}

/* ==================================================================
 * 命令处理
 * ================================================================== */
static void cmd_onoff(const char *key, const char *val)
{
    uint8_t on;
    uint8_t ok = 1U;

    if (util_str_eq_ci(val, "ON") || util_str_eq_ci(val, "1"))
    {
        on = 1U;
    }
    else if (util_str_eq_ci(val, "OFF") || util_str_eq_ci(val, "0"))
    {
        on = 0U;
    }
    else
    {
        reply_err("SYNTAX");
        return;
    }

    if (util_str_eq_ci(key, "STREAM"))
    {
        app_set_stream(on);
    }
    else if (util_str_eq_ci(key, "RAW"))
    {
        app_set_raw_stream(on);
    }
    else if (util_str_eq_ci(key, "STAT"))
    {
        app_set_stat(on);
    }
    else if (util_str_eq_ci(key, "ZUPT"))
    {
        ctl_set_zupt(on);
    }
    else if (util_str_eq_ci(key, "STALL"))
    {
        ctl_set_stall(on);
    }
    else if (util_str_eq_ci(key, "INV"))
    {
        tb_set_invert(on);
    }
    else if (util_str_eq_ci(key, "BIASAUTO"))
    {
        ctl_set_bias_auto(on);
    }
    else if (util_str_eq_ci(key, "IMU"))
    {
        ctl_set_imu_dump((uint8_t)((on != 0U) ? 2U : 0U));
    }
    else
    {
        ok = 0U;
    }

    if (ok == 0U)
    {
        reply_err("UNKNOWN_CMD");
        return;
    }
    logln("OK:%s=%s", key, (on != 0U) ? "ON" : "OFF");
}

static void cmd_set(const char *val)
{
    float deg;

    if (util_parse_float(val, &deg) == 0U)
    {
        reply_err("SYNTAX");
        return;
    }
    if (ctl_set_target(deg) == 0U)
    {
        reply_err("LIMIT");
        return;
    }
    reply_ok_float("SET", ctl_target());
}

static void cmd_gain(const char *key, const char *val)
{
    float kp;
    float ki;
    float kd;
    float v;

    if (util_parse_float(val, &v) == 0U)
    {
        reply_err("SYNTAX");
        return;
    }
    if ((v < 0.0f) || (v > 1000.0f))
    {
        reply_err("RANGE");
        return;
    }

    ctl_get_gains(&kp, &ki, &kd);
    if (util_str_eq_ci(key, "KP"))
    {
        kp = v;
    }
    else if (util_str_eq_ci(key, "KI"))
    {
        ki = v;
    }
    else
    {
        kd = v;
    }
    ctl_set_gains(kp, ki, kd);
    reply_ok_float(key, v);
}

static void cmd_limit(const char *val)
{
    float deg;

    if (util_parse_float(val, &deg) == 0U)
    {
        reply_err("SYNTAX");
        return;
    }
    if (ctl_set_limit(deg) == 0U)
    {
        reply_err("RANGE");
        return;
    }
    reply_ok_float("LIMIT", ctl_limit());
}

static void cmd_duty(const char *val)
{
    uint32_t pm;

    if (util_parse_u32(val, &pm) == 0U)
    {
        reply_err("SYNTAX");
        return;
    }
    if (pm > 1000U)
    {
        reply_err("RANGE");
        return;
    }
    if (ctl_state() != CTL_STATE_IDLE)
    {
        reply_err("STATE");     /* 闭环中不允许开环点动 */
        return;
    }
    if (ctl_manual((int16_t)pm) == 0U)
    {
        reply_err("LIMIT");     /* 剩余角度不足且方向朝外 */
        return;
    }
    logln("OK:DUTY=%u", (unsigned)pm);
}

static void cmd_kick(const char *val)
{
    uint32_t pm;
    uint32_t ms;
    const char *comma = strchr(val, ',');

    if (comma == 0)
    {
        reply_err("SYNTAX");
        return;
    }
    {
        char head[8];
        uint8_t n = (uint8_t)(comma - val);

        if ((n == 0U) || (n >= sizeof(head)))
        {
            reply_err("SYNTAX");
            return;
        }
        memcpy(head, val, n);
        head[n] = '\0';
        if (util_parse_u32(head, &pm) == 0U)
        {
            reply_err("SYNTAX");
            return;
        }
    }
    if (util_parse_u32(comma + 1, &ms) == 0U)
    {
        reply_err("SYNTAX");
        return;
    }
    if ((pm > 1000U) || (ms > 2000U))
    {
        reply_err("RANGE");
        return;
    }
    tb_set_kick((int16_t)pm, (uint16_t)ms);
    logln("OK:KICK=%u,%u", (unsigned)pm, (unsigned)ms);
}

static void cmd_mode(const char *val)
{
    if (util_str_eq_ci(val, "POS") || util_str_eq_ci(val, "POSITION"))
    {
        ctl_set_pid_mode(PID_MODE_POSITION);
    }
    else if (util_str_eq_ci(val, "INC") || util_str_eq_ci(val, "INCREMENTAL"))
    {
        ctl_set_pid_mode(PID_MODE_INCREMENTAL);
    }
    else
    {
        reply_err("SYNTAX");
        return;
    }
    logln("OK:MODE=%s", pid_mode_str(ctl_pid_mode()));
}

static void cmd_proto(const char *val)
{
    if (util_str_eq_ci(val, "FIRE") || util_str_eq_ci(val, "FIREWATER"))
    {
        vofa_set_mode(VOFA_MODE_FIREWATER);
    }
    else if (util_str_eq_ci(val, "JUST") || util_str_eq_ci(val, "JUSTFLOAT"))
    {
        vofa_set_mode(VOFA_MODE_JUSTFLOAT);
    }
    else
    {
        reply_err("SYNTAX");
        return;
    }
    logln("OK:PROTO=%s", vofa_mode_str());
}

static void cmd_print_pid(void)
{
    char b_kp[16];
    char b_ki[16];
    char b_kd[16];
    float kp;
    float ki;
    float kd;

    ctl_get_gains(&kp, &ki, &kd);
    vofa_format_float(b_kp, kp, 3U);
    vofa_format_float(b_ki, ki, 3U);
    vofa_format_float(b_kd, kd, 3U);
    logln("PID mode=%s Kp=%s Ki=%s Kd=%s", pid_mode_str(ctl_pid_mode()), b_kp, b_ki, b_kd);
}

static void cmd_pid_all(const char *val)
{
    float  v[3];
    uint8_t i;
    const char *p = val;
    char   num[16];

    for (i = 0U; i < 3U; i++)
    {
        uint8_t k = 0U;

        while ((*p != '\0') && (*p != ',') && (k < (uint8_t)(sizeof(num) - 1U)))
        {
            num[k++] = *p++;
        }
        num[k] = '\0';
        if (util_parse_float(num, &v[i]) == 0U)
        {
            reply_err("SYNTAX");
            return;
        }
        if (*p == ',')
        {
            p++;
        }
        else if (i < 2U)
        {
            reply_err("SYNTAX");
            return;
        }
    }
    if ((v[0] < 0.0f) || (v[1] < 0.0f) || (v[2] < 0.0f) ||
        (v[0] > 1000.0f) || (v[1] > 1000.0f) || (v[2] > 1000.0f))
    {
        reply_err("RANGE");
        return;
    }
    ctl_set_gains(v[0], v[1], v[2]);
    cmd_print_pid();
}

static void cmd_openloop(const char *key)
{
    if (ctl_state() != CTL_STATE_IDLE)
    {
        reply_err("STATE");
        return;
    }

    if (util_str_eq_ci(key, "FWD"))
    {
        (void)ctl_manual(CLI_DEFAULT_PM);
        logln("OK:FWD=%d", (int)CLI_DEFAULT_PM);
    }
    else if (util_str_eq_ci(key, "REV"))
    {
        (void)ctl_manual((int16_t)(-CLI_DEFAULT_PM));
        logln("OK:REV=%d", (int)(-CLI_DEFAULT_PM));
    }
    else if (util_str_eq_ci(key, "BRK"))
    {
        tb_brake();
        logln("OK:BRK");
    }
    else /* COAST */
    {
        ctl_manual_release();
        logln("OK:COAST");
    }
}

static void cmd_enc(void)
{
    char b_spd[16];
    char b_yaw[16];
    char b_deg[16];
    char b_erc[16];
    char b_cpd[16];

    vofa_format_float(b_spd, enc_speed_cps(), 1U);
    vofa_format_float(b_yaw, ctl_yaw(), 3U);
    vofa_format_float(b_deg, ctl_enc_deg(), 3U);
    vofa_format_float(b_erc, ctl_enc_rate_dps(), 2U);
    vofa_format_float(b_cpd, ctl_cpd(), 3U);
    logln("ENC count=%ld delta=%d raw=%u cps=%s | edeg=%s erate=%s cpd=%s sig=%d | fb=%s yaw=%s",
          (long)enc_count(), (int)enc_delta(), (unsigned)enc_raw(), b_spd,
          b_deg, b_erc, b_cpd, (int)ctl_enc_sign(), ctl_fb_str(), b_yaw);
}

static void cmd_bare(const char *key)
{
    if (util_str_eq_ci(key, "START"))
    {
        if (ctl_start() == 0U)
        {
            reply_err("STATE");     /* 未标定完 / 有故障 / 已在限位边 */
            return;
        }
        reply_ok_float("TARGET", ctl_target());
    }
    else if (util_str_eq_ci(key, "STOP"))
    {
        ctl_stop();
        logln("OK:STOP");
    }
    else if (util_str_eq_ci(key, "CAL"))
    {
        if (ctl_state() == CTL_STATE_BOOT)
        {
            reply_err("STATE");
            return;
        }
        ctl_request_calib();
        logln("OK:CAL");
    }
    else if (util_str_eq_ci(key, "ZERO"))
    {
        ctl_zero_yaw();
        logln("OK:ZERO");
    }
    else if (util_str_eq_ci(key, "CLEAR"))
    {
        if (ctl_clear_fault() == 0U)
        {
            reply_err("STATE");
            return;
        }
        logln("OK:CLEAR");
    }
    else if (util_str_eq_ci(key, "PID"))
    {
        cmd_print_pid();
    }
    else if (util_str_eq_ci(key, "ENC"))
    {
        cmd_enc();
    }
    else if (util_str_eq_ci(key, "BIAS"))
    {
        char b_bias[16];
        vofa_format_float(b_bias, ctl_bias_dps(), 3U);
        logln("BIAS=%s dps auto=%u", b_bias, (unsigned)ctl_bias_auto());
    }
    else if (util_str_eq_ci(key, "INFO"))
    {
        app_print_info();
    }
    else if (util_str_eq_ci(key, "IMUINIT"))
    {
        /* 线松过/掉电后不用重启板子：重新探测 IMU，成功则自动重做零偏标定 */
        if (ctl_imu_reinit() == 0U)
        {
            reply_err(ctl_imu_ok() != 0U ? "STATE" : "IMU");
            return;
        }
        logln("OK:IMUINIT");
    }
    else if (util_str_eq_ci(key, "IMU"))
    {
        ctl_set_imu_dump(1U);       /* 打一条六轴原始数据 */
        logln("OK:IMU (one sample follows)");
    }
    else if (util_str_eq_ci(key, "STAT") || util_str_eq_ci(key, "S"))
    {
        app_print_status();
    }
    else if (util_str_eq_ci(key, "FWD") || util_str_eq_ci(key, "REV") ||
             util_str_eq_ci(key, "BRK") || util_str_eq_ci(key, "COAST"))
    {
        cmd_openloop(key);
    }
    else if (util_str_eq_ci(key, "HELP") || util_str_eq_ci(key, "?"))
    {
        logln("---- commands ----");
        logln(" SET=<deg>       target angle, e.g. SET=-45.5 (limit +-%d)", (int)ctl_limit());
        logln(" START / STOP    closed loop on / off");
        logln(" CAL             re-run gyro bias calibration (hold still)");
        logln(" ZERO            set current angle to 0");
        logln(" KP= / KI= / KD= tune one gain, PID=<kp>,<ki>,<kd> set all");
        logln(" MODE=POS|INC    position / incremental PID");
        logln(" LIMIT=<deg>     software limit protecting the cable");
        logln(" DZ=<pm>         motor dead-zone (permille), KICK=<pm>,<ms>");
        logln(" INV=ON|OFF      invert motor direction");
        logln(" ZUPT=ON|OFF     freeze yaw integration while still");
        logln(" STALL=ON|OFF    stall detection (encoder needed)");
        logln(" FB=IMU|ENC|FUSE feedback source (ENC for tuning, IMU for acceptance)");
        logln(" CPD=<n>         encoder counts per degree (calibrate against gyro)");
        logln(" SIG=1|-1        encoder counting direction vs yaw");
        logln(" GSIG=1|-1       gyro Z mount direction (+1 chip up, -1 chip down)");
        logln(" FWD/REV/BRK/COAST/DUTY=<pm>  open-loop test (IDLE only)");
        logln(" STREAM=ON|OFF / RAW=ON|OFF / PROTO=FIRE|JUST");
        logln(" STAT=ON|OFF / INFO / ENC / RESET / IMU[=ON|OFF]");
        logln(" IMUINIT         re-probe the MPU6050 (after a loose wire), recalibrates");
    }
    else if (util_str_eq_ci(key, "RESET"))
    {
        ctl_stop();
        ctl_set_gains(PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT);
        ctl_set_pid_mode((PID_MODE_DEFAULT_POS != 0U) ? PID_MODE_POSITION : PID_MODE_INCREMENTAL);
        (void)ctl_set_limit(SOFT_LIMIT_DEG_DEFAULT);
        ctl_set_zupt((uint8_t)ZUPT_DEFAULT);
        ctl_set_stall(0U);
        (void)ctl_set_fb((ctl_fb_t)FB_SOURCE_DEFAULT);
        ctl_set_cpd(ENC_CPD_DEFAULT);
        ctl_set_enc_sign(ENC_SIGN_DEFAULT);
        ctl_set_gyro_sign(GYRO_SIGN_DEFAULT);
        tb_set_deadzone(MOTOR_DEADZONE_PM);
        tb_set_kick(MOTOR_KICK_PM, MOTOR_KICK_MS);
        tb_set_invert(MOTOR_DIR_INVERT);
        app_set_stream(1U);
        app_set_raw_stream(0U);
        app_set_stat(0U);
        ctl_zero_yaw();
        logln("OK:RESET");
    }
    else
    {
        reply_err("UNKNOWN_CMD");
    }
}

/**
 * @brief 执行一整行命令
 */
static void cli_exec(char *line)
{
    char *eq;
    char *key;
    char *val;

    line = util_trim(line);
    if (*line == '\0')
    {
        return;
    }

    eq = strchr(line, '=');
    if (eq != 0)
    {
        *eq = '\0';
        key = util_trim(line);
        val = util_trim(eq + 1);
    }
    else
    {
        /* 也允许 "SET 90" 这种空格分隔写法，方便手动敲命令 */
        char *sp;

        key = util_trim(line);
        sp  = strchr(key, ' ');
        if (sp != 0)
        {
            *sp = '\0';
            val = util_trim(sp + 1);
            key = util_trim(key);
        }
        else
        {
            val = 0;
        }
    }

    if (*key == '\0')
    {
        reply_err("SYNTAX");
        return;
    }

    /* ---- 带 = 的命令 ---- */
    if (val != 0)
    {
        if (util_str_eq_ci(key, "SET"))
        {
            cmd_set(val);
        }
        else if (util_str_eq_ci(key, "KP") || util_str_eq_ci(key, "KI") || util_str_eq_ci(key, "KD"))
        {
            cmd_gain(key, val);
        }
        else if (util_str_eq_ci(key, "PID"))
        {
            cmd_pid_all(val);
        }
        else if (util_str_eq_ci(key, "MODE"))
        {
            cmd_mode(val);
        }
        else if (util_str_eq_ci(key, "LIMIT"))
        {
            cmd_limit(val);
        }
        else if (util_str_eq_ci(key, "DZ"))
        {
            uint32_t pm;

            if (util_parse_u32(val, &pm) == 0U)
            {
                reply_err("SYNTAX");
            }
            else if (pm > 600U)
            {
                reply_err("RANGE");
            }
            else
            {
                tb_set_deadzone((int16_t)pm);
                logln("OK:DZ=%u", (unsigned)pm);
            }
        }
        else if (util_str_eq_ci(key, "OMAX"))
        {
            uint32_t pm;

            if (util_parse_u32(val, &pm) == 0U)
            {
                reply_err("SYNTAX");
            }
            else if ((pm < 20U) || (pm > 1000U))
            {
                reply_err("RANGE");
            }
            else
            {
                ctl_set_out_max((int16_t)pm);
                logln("OK:OMAX=%u", (unsigned)pm);
            }
        }
        else if (util_str_eq_ci(key, "KICK"))
        {
            cmd_kick(val);
        }
        else if (util_str_eq_ci(key, "FB"))
        {
            ctl_fb_t fb;

            if (util_str_eq_ci(val, "ENC") || util_str_eq_ci(val, "ENCODER"))
            {
                fb = CTL_FB_ENC;
            }
            else if (util_str_eq_ci(val, "IMU") || util_str_eq_ci(val, "GYRO"))
            {
                fb = CTL_FB_IMU;
            }
            else if (util_str_eq_ci(val, "FUSE") || util_str_eq_ci(val, "FUSION"))
            {
                fb = CTL_FB_FUSE;
            }
            else
            {
                reply_err("SYNTAX");
                return;
            }
            if (ctl_set_fb(fb) == 0U)
            {
                reply_err("STATE");     /* 闭环运行中不允许切换 */
                return;
            }
            logln("OK:FB=%s", ctl_fb_str());
        }
        else if (util_str_eq_ci(key, "CPD"))
        {
            float v;

            if (util_parse_float(val, &v) == 0U)
            {
                reply_err("SYNTAX");
            }
            else if ((v < 0.5f) || (v > 100000.0f))
            {
                reply_err("RANGE");
            }
            else
            {
                char b[16];
                ctl_set_cpd(v);
                vofa_format_float(b, ctl_cpd(), 3U);
                logln("OK:CPD=%s", b);
            }
        }
        else if (util_str_eq_ci(key, "SIG") || util_str_eq_ci(key, "CSIGN"))
        {
            float v;

            if (util_parse_float(val, &v) == 0U)
            {
                reply_err("SYNTAX");
            }
            else if ((v != 1.0f) && (v != -1.0f))
            {
                reply_err("RANGE");
            }
            else
            {
                ctl_set_enc_sign(v);
                logln("OK:SIG=%d (encoder angle reset, send ZERO again)", (int)ctl_enc_sign());
            }
        }
        else if (util_str_eq_ci(key, "GSIG"))
        {
            float v;

            if (util_parse_float(val, &v) == 0U)
            {
                reply_err("SYNTAX");
            }
            else if ((v != 1.0f) && (v != -1.0f))
            {
                reply_err("RANGE");
            }
            else
            {
                ctl_set_gyro_sign(v);
                logln("OK:GSIG=%d (yaw reset, recalibrating)", (int)ctl_gyro_sign());
            }
        }
        else if (util_str_eq_ci(key, "DUTY"))
        {
            cmd_duty(val);
        }
        else if (util_str_eq_ci(key, "PROTO"))
        {
            cmd_proto(val);
        }
        else if (util_str_eq_ci(key, "STREAM") || util_str_eq_ci(key, "RAW") ||
                 util_str_eq_ci(key, "STAT") || util_str_eq_ci(key, "ZUPT") ||
                 util_str_eq_ci(key, "STALL") || util_str_eq_ci(key, "INV") ||
                 util_str_eq_ci(key, "BIASAUTO") || util_str_eq_ci(key, "IMU"))
        {
            cmd_onoff(key, val);
        }
        else
        {
            reply_err("UNKNOWN_CMD");
        }
        return;
    }

    /* ---- 不带 = 的命令 ---- */
    cmd_bare(key);
}

/* ==================================================================
 * 轮询
 * ================================================================== */
void cli_poll(void)
{
    while (s_tail != s_head)
    {
        uint8_t b = s_ring[s_tail];

        s_tail = (uint16_t)((s_tail + 1U) % CLI_RING_SIZE);

        if ((b == '\r') || (b == '\n'))
        {
            if (s_discard != 0U)
            {
                s_discard  = 0U;
                s_line_len = 0U;
                reply_err("LINE_TOO_LONG");
                continue;
            }
            if (s_line_len > 0U)
            {
                s_line[s_line_len] = '\0';
                s_line_len = 0U;
                cli_exec(s_line);
            }
            continue;
        }

        if (s_discard != 0U)
        {
            continue;
        }
        if (s_line_len >= (CLI_LINE_MAX - 1U))
        {
            s_discard = 1U;
            continue;
        }
        s_line[s_line_len++] = (char)b;
    }
}
