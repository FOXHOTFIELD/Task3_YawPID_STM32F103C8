/**
 * @file    app_ctl.c
 * @brief   闭环控制核心实现
 *
 * 一拍（200Hz，5ms）做的事情，顺序固定：
 *   量 dt -> 读 MPU6050 -> 读编码器 -> (标定) -> 静止判断 -> Yaw 积分
 *   -> 最短路径误差 -> PID -> 限位裁剪 -> 写 TB6612
 *
 * 时间基准用 DWT 周期计数器，不用 HAL_GetTick()：
 * 1ms 的 tick 分辨率在 5ms 周期上是 20% 的量化误差，直接积分会让 Yaw
 * 的速度刻度系统性偏大或偏小；DWT 是 72MHz 计数，精度足够。
 */
#include "main.h"
#include "board_config.h"
#include "app_ctl.h"
#include "app_enc.h"
#include "app_i2c_bus.h"
#include "app_log.h"
#include "app_mpu6050.h"
#include "app_pid.h"
#include "app_tb6612.h"
#include "app_util.h"
#include "app_vofa.h"
#include "app_yaw.h"

/* ---------------- DWT 周期计数器（Cortex-M3 自带） ---------------- */
#define DWT_CTRL_REG      (*(volatile uint32_t *)0xE0001000UL)
#define DWT_CYCCNT_REG    (*(volatile uint32_t *)0xE0001004UL)
#define DEMCR_REG         (*(volatile uint32_t *)0xE000EDFCUL)
#define DEMCR_TRCENA      (1UL << 24)

/* ---------------- 内部常量 ---------------- */
#define IMU_RETRY_MS        1000U   /* IMU 初始化失败后的重试周期 */
#define IMU_ERR_LIMIT       100U    /* 连续失败多少拍判 IMU 故障（200Hz 下 500ms） */
#define STALL_PM_MIN        400     /* 力度超过这个才检查堵转 */
/* 堵转超时取 2 秒：用手把转盘按住制造"30° 扰动"时，电机给了力却转不动，
   和真堵转在固件眼里完全一样。2 秒能把"手推一下"和"机构卡死"区分开，
   实测 600ms 会在扰动测试里误报 FAULT:STALL 而放弃回位。 */
#define STALL_TIMEOUT_MS    2000U
#define ZUPT_RUN_OUT_MIN    200     /* 闭环时输出小于它才允许判"静止" */
#define ZUPT_LEARN_MS       1000U   /* 静止超过这么久才开始在线学零偏 */

static ctl_state_t s_state = CTL_STATE_BOOT;
static ctl_fault_t s_fault = CTL_FAULT_NONE;

static yaw_t  s_yaw;
static pid_t  s_pid;

static float   s_target;
static float   s_error;
static float   s_limit;
static int16_t s_out_pm;
static uint32_t s_ctrl_count;

static uint32_t s_last_cyc;
static uint32_t s_boot_retry_ms;

static uint8_t  s_zupt_en;
static uint8_t  s_stall_en;
static uint8_t  s_zupt_active;
static uint32_t s_still_ms;

static uint8_t  s_imu_ok;
static uint32_t s_imu_err;
static mpu_status_t s_mpu_status = MPU_ERR_NOT_INIT;

/* 标定 */
static uint32_t s_cal_start_ms;
static float    s_cal_sum;
static float    s_cal_sqsum;
static uint32_t s_cal_n;
static float    s_cal_std;
static uint8_t  s_cal_quality;

/* 堵转 */
static uint32_t s_stall_ref_ms;
static int32_t  s_stall_ref_count;

/* 开环点动 */
static uint32_t s_manual_ms;   /* 最近一次开环点动的时刻，用于超时自动断电 */

/* 怠速零偏在线跟踪 */
static uint8_t s_bias_auto = (uint8_t)IDLE_BIAS_ENABLE;

/* 输出限幅（OMAX）：调参期间限制最大力度，防止算错把转盘甩到限位 */
static int16_t s_out_max = (int16_t)OUTPUT_LIMIT_DEFAULT_PM;

/* ---------------- 反馈源 / 编码器角度 ----------------
 * 编码器在电机轴上（减速箱之前）。减速比把回差和振动都缩到输出端可忽略，
 * 折算后 1 count ≈ 0.07°，比陀螺积分稳得多，所以先用它把 PID 调对，
 * 再把陀螺引进来（FB=FUSE 时编码器负责把陀螺的漂移慢慢拉回来）。 */
static ctl_fb_t s_fb = (ctl_fb_t)FB_SOURCE_DEFAULT;
static float    s_enc_deg;                                  /* 编码器连续角 */
static float    s_enc_rate;                                 /* 编码器角速度（低通后） */
static float    s_cpd      = ENC_CPD_DEFAULT;               /* counts per 转盘度 */
static float    s_enc_sign = ENC_SIGN_DEFAULT;              /* ±1 */
static float    s_fb_deg;                                   /* 闭环真正用的连续角 */
static float    s_fb_rate;                                  /* 闭环真正用的角速度 */
static float    s_fuse_deg;                                 /* 互补滤波状态 */
static int16_t  s_stall_pm_min = STALL_PM_MIN;
/* ==================================================================
 * 内部工具
 * ================================================================== */
static void ctl_dwt_init(void)
{
    DEMCR_REG     |= DEMCR_TRCENA;
    DWT_CYCCNT_REG = 0U;
    DWT_CTRL_REG  |= 1U;
}

static float ctl_calc_dt(uint32_t cyc)
{
    float    dt;
    uint32_t d;

    if (s_last_cyc == 0U)
    {
        s_last_cyc = (cyc != 0U) ? cyc : 1U;
        return (float)CTRL_PERIOD_MS / 1000.0f;
    }
    d          = cyc - s_last_cyc;          /* 无符号相减，回绕自动正确 */
    s_last_cyc = cyc;

    dt = (float)d / (float)BOARD_SYSCLK_HZ;
    if ((dt < 0.0005f) || (dt > 0.05f))
    {
        dt = (float)CTRL_PERIOD_MS / 1000.0f;   /* DWT 没跑或异常时用标称周期 */
    }
    return dt;
}

static void ctl_raise_fault(ctl_fault_t f)
{
    s_fault  = f;
    s_state  = CTL_STATE_FAULT;
    s_out_pm = 0;
    tb_brake();
    logln("FAULT:%s (send STOP to clear, see STAT)", ctl_fault_str());
}

/* 陀螺读失败但反馈源是编码器时，把这一拍当成"陀螺输出 0"继续跑 */
static void ctl_sample_zero(imu_sample_t *s)
{
    s->ax_g    = 0.0f;
    s->ay_g    = 0.0f;
    s->az_g    = 0.0f;
    s->gx_dps  = 0.0f;
    s->gy_dps  = 0.0f;
    s->gz_dps  = 0.0f;
    s->temp_c  = 0.0f;
}

/* ---------------- 编码器角度累加 ----------------
 * count -> 转盘度：inc = delta * sign / cpd。
 * cpd 是"每转盘度多少 count"，已经把减速比和编码器线数一起折进去了，
 * 用 CPD= 命令可以现场改（标定方法见 README）。
 * 速度用一阶低通：一拍里 1 个 count 就是 14.7dps 的台阶，不滤的话
 * 微分项会被量化噪声打得乱跳。 */
static void ctl_enc_update(float dt)
{
    float inc;

    if (s_cpd < 0.01f)
    {
        s_cpd = ENC_CPD_DEFAULT;        /* 兜底，防止除以 0 */
    }

    inc = ((float)enc_delta() * s_enc_sign) / s_cpd;
    s_enc_deg += inc;

    if (dt > 0.0f)
    {
        float inst = inc / dt;
        float a    = (float)CTRL_PERIOD_MS / (float)ENC_RATE_TAU_MS;

        if (a > 1.0f)
        {
            a = 1.0f;
        }
        s_enc_rate += (a * (inst - s_enc_rate));
    }
}

/* ---------------- 反馈源选择 ---------------- */
static void ctl_fb_update(float dt)
{
    switch (s_fb)
    {
    case CTL_FB_ENC:
        s_fb_deg  = s_enc_deg;
        s_fb_rate = s_enc_rate;
        break;

    case CTL_FB_FUSE:
    {
        float diff;     /* 编码器 - 融合：非零就是陀螺漂了 */

        s_fuse_deg += (s_yaw.rate_dps * dt);            /* 陀螺负责"快" */
        diff        = s_enc_deg - s_fuse_deg;

        if (util_absf(diff) > FUSE_RESYNC_DEG)
        {
            s_fuse_deg = s_enc_deg;                     /* 漂太多直接重同步 */
        }
        else
        {
            s_fuse_deg += (((float)CTRL_PERIOD_MS / (float)FUSE_TAU_MS) * diff);
        }
        s_fb_deg  = s_fuse_deg;
        s_fb_rate = s_yaw.rate_dps;
        break;
    }

    case CTL_FB_IMU:
    default:
        s_fb_deg  = s_yaw.cont;
        s_fb_rate = s_yaw.rate_dps;
        break;
    }
}

/* ==================================================================
 * 初始化与标定
 * ================================================================== */
static uint8_t ctl_imu_setup(void)
{
    mpu_status_t st;

    if (i2c_bus_init() != HAL_OK)
    {
        s_mpu_status = MPU_ERR_I2C;
        return 0U;
    }
    st = mpu_init();
    if (st != MPU_OK)
    {
        s_mpu_status = st;
        return 0U;
    }
    /* 陀螺输出率 1kHz（SMPLRT_DIV=0），控制端按 200Hz 读取最新值；
       DLPF=3（42Hz 带宽）压掉电机振动带来的高频噪声。 */
    (void)mpu_set_dlpf((uint8_t)IMU_DLPF_CFG);
    (void)mpu_set_sample_rate((uint16_t)IMU_OUTPUT_HZ);
    (void)mpu_set_gyro_fs(1U);      /* ±500dps，转盘用足够 */
    (void)mpu_set_accel_fs(1U);     /* ±4g */

    s_mpu_status = MPU_OK;
    s_imu_ok     = 1U;
    return 1U;
}

static void ctl_begin_calib(uint32_t now)
{
    s_cal_sum      = 0.0f;
    s_cal_sqsum    = 0.0f;
    s_cal_n        = 0U;
    s_cal_start_ms = now;
    s_cal_std      = 0.0f;
    s_cal_quality  = 0U;
    s_state        = CTL_STATE_CALIB;
    s_fault        = CTL_FAULT_NONE;
    s_out_pm       = 0;

    pid_reset(&s_pid);
    tb_coast();
    enc_reset();

    logln("CAL: hold the table still for %ums ...", (unsigned)CAL_DURATION_MS);
}

static void ctl_finish_calib(uint32_t now)
{
    float mean;
    float var;

    (void)now;

    if (s_cal_n == 0U)
    {
        ctl_raise_fault(CTL_FAULT_IMU);
        return;
    }

    mean = s_cal_sum / (float)s_cal_n;
    var  = (s_cal_sqsum / (float)s_cal_n) - (mean * mean);
    if (var < 0.0f)
    {
        var = 0.0f;
    }
    s_cal_std = util_sqrtf(var);

    /* 静止质量：标准差 0 -> 100 分；超过 CAL_STILL_LIMIT_DPS -> 0 分 */
    if (s_cal_std >= CAL_STILL_LIMIT_DPS)
    {
        s_cal_quality = 0U;
    }
    else
    {
        float q = 100.0f - (s_cal_std * (100.0f / CAL_STILL_LIMIT_DPS));
        s_cal_quality = (uint8_t)(q + 0.5f);
    }

    yaw_set_bias(&s_yaw, mean);
    yaw_reset(&s_yaw, 0.0f);
    s_target = 0.0f;
    s_state  = CTL_STATE_IDLE;

    {
        char b_bias[16];
        char b_std[16];

        vofa_format_float(b_bias, mean, 3U);
        vofa_format_float(b_std, s_cal_std, 3U);
        logln("CAL done: bias=%s std=%s quality=%u%s",
              b_bias, b_std, (unsigned)s_cal_quality,
              (s_cal_quality == 0U) ? " (WARN: moved during calibration!)" : "");
    }
}

void ctl_init(void)
{
    ctl_dwt_init();

    s_target   = 0.0f;
    s_error    = 0.0f;
    s_limit    = SOFT_LIMIT_DEG_DEFAULT;
    s_out_pm   = 0;
    s_ctrl_count = 0U;
    s_last_cyc   = 0U;
    s_zupt_en    = (uint8_t)ZUPT_DEFAULT;
    s_stall_en   = 0U;
    s_zupt_active = 0U;
    s_still_ms   = 0U;
    s_imu_ok     = 0U;
    s_imu_err    = 0U;
    s_fault      = CTL_FAULT_NONE;
    s_stall_ref_ms    = HAL_GetTick();
    s_stall_ref_count = 0;

    /* 反馈源：调参期默认编码器 */
    s_fb        = (ctl_fb_t)FB_SOURCE_DEFAULT;
    s_enc_deg   = 0.0f;
    s_enc_rate  = 0.0f;
    s_cpd       = ENC_CPD_DEFAULT;
    s_enc_sign  = ENC_SIGN_DEFAULT;
    s_fb_deg    = 0.0f;
    s_fb_rate   = 0.0f;
    s_fuse_deg  = 0.0f;
    s_stall_pm_min = (s_fb == CTL_FB_ENC) ? (int16_t)STALL_PM_MIN_ENC : (int16_t)STALL_PM_MIN;
    if (s_fb == CTL_FB_ENC)
    {
        s_stall_en = 1U;    /* 编码器是唯一反馈，断线必须能被发现 */
    }

    pid_init(&s_pid, PID_KP_DEFAULT, PID_KI_DEFAULT, PID_KD_DEFAULT,
             -PID_OUT_LIMIT_PM, PID_OUT_LIMIT_PM,
             (PID_MODE_DEFAULT_POS != 0U) ? PID_MODE_POSITION : PID_MODE_INCREMENTAL);
    pid_set_integ_limit(&s_pid, -PID_INTEG_LIMIT_PM, PID_INTEG_LIMIT_PM);
    pid_set_d_lpf(&s_pid, PID_D_LPF_DEFAULT);
    pid_set_rate_source(&s_pid, 1U);    /* 微分用编码器/陀螺测速，不用位置差分 */

    yaw_init(&s_yaw, 0.0f);

    tb_init();
    enc_init();

    if (ctl_imu_setup() != 0U)
    {
        ctl_begin_calib(HAL_GetTick());
    }
    else if (s_fb == CTL_FB_ENC)
    {
        /* 编码器模式不依赖 IMU：直接进 IDLE，可以立刻 START。
           （IMU 拆了/没接的时候，靠这条才能继续调参） */
        s_state = CTL_STATE_IDLE;
        logln("IMU init failed: %s -> running ENCODER-ONLY (fb=ENC), gyro features off",
              mpu_status_str(s_mpu_status));
    }
    else
    {
        s_state          = CTL_STATE_BOOT;
        s_boot_retry_ms  = HAL_GetTick();
        logln("IMU init failed: %s (retrying every %ums)",
              mpu_status_str(s_mpu_status), (unsigned)IMU_RETRY_MS);
    }
}

/* ==================================================================
 * 控制节拍
 * ================================================================== */
void ctl_step(void)
{
    imu_sample_t sample;
    mpu_status_t st;
    uint32_t     now = HAL_GetTick();
    uint32_t     cyc = DWT_CYCCNT_REG;
    float        dt;

    s_ctrl_count++;
    dt = ctl_calc_dt(cyc);

    /* 编码器每一拍都要更新：它既可能直接是反馈源，也是限位保护的依据，
       放最前面，后面任何提前 return（IMU 故障、标定中）都不会漏计。 */
    enc_update(dt);
    ctl_enc_update(dt);

    /* ---------------- BOOT：等 IMU 就绪 ---------------- */
    if (s_state == CTL_STATE_BOOT)
    {
        if ((now - s_boot_retry_ms) >= IMU_RETRY_MS)
        {
            s_boot_retry_ms = now;
            if (ctl_imu_setup() != 0U)
            {
                logln("IMU online: %s addr=0x%02X", mpu_chip_name(), (unsigned)mpu_addr());
                ctl_begin_calib(now);
            }
        }
        return;
    }

    /* ---------------- 读传感器 ---------------- */
    st = mpu_read(&sample);
    if (st != MPU_OK)
    {
        s_imu_err++;
        s_mpu_status = st;

        if (s_fb == CTL_FB_ENC)
        {
            /* 编码器闭环根本不看陀螺：拆了、没插、线松都不影响调参。
               这里只是把这一拍当成"陀螺输出 0"，继续走后面的编码器闭环。 */
            s_imu_ok = 0U;
            ctl_sample_zero(&sample);
        }
        else
        {
            if (s_imu_err >= IMU_ERR_LIMIT)
            {
                s_imu_ok = 0U;
                ctl_raise_fault(CTL_FAULT_IMU);
            }
            return;
        }
    }
    else
    {
        s_imu_err    = 0U;
        s_imu_ok     = 1U;
        s_mpu_status = MPU_OK;
    }

    /* ---------------- 标定：累计零偏统计 ---------------- */
    if (s_state == CTL_STATE_CALIB)
    {
        float gz = sample.gz_dps;
        s_cal_sum   += gz;
        s_cal_sqsum += (gz * gz);
        s_cal_n++;
        if ((now - s_cal_start_ms) >= CAL_DURATION_MS)
        {
            ctl_finish_calib(now);
        }
        return;
    }

    /* ---------------- 静止判断（ZUPT） ----------------
     * 静止的判据：
     *   1) 编码器没有动（有编码器时这是最可靠的判据：轮子不动才是真静止）；
     *   2) 去偏后的角速度很小（进入 1.0dps / 退出 2.0dps，迟滞）；
     *   3) 闭环时输出也很小（否则就是在慢慢爬，不能冻积分）；
     *   4) 连续满足 ZUPT_HOLD_MS。
     * 计数同时做两件事：
     *   - 封顶 ZUPT_STILL_CAP_MS：否则静止越久，攒的计数越多，
     *     退出时按拍泄漏根本回不到阈值以下，会"冻死"（v1.0.2 实测踩到：
     *     转盘在转、yaw 却不更新，连限位用的 cont 也一起冻住，排线保护失效）；
     *   - 非静止时按 4 拍泄漏：0.6s 内一定能解冻，又不会被单拍毛刺打回原形。 */
    {
        float   rate  = sample.gz_dps - s_yaw.bias_dps;
        float   thr   = (s_zupt_active != 0U) ? ZUPT_EXIT_DPS : ZUPT_RATE_DPS;
        uint8_t still = 0U;

        if (enc_delta() != 0)
        {
            still = 0U;             /* 编码器在动 -> 绝对不是静止 */
        }
        else if (util_absf(rate) < thr)
        {
            if ((s_state != CTL_STATE_RUN) || (util_absf((float)s_out_pm) < (float)ZUPT_RUN_OUT_MIN))
            {
                still = 1U;
            }
        }

        if (still != 0U)
        {
            if (s_still_ms < ZUPT_STILL_CAP_MS)
            {
                s_still_ms += (uint32_t)CTRL_PERIOD_MS;
            }
        }
        else if (s_still_ms > (4U * (uint32_t)CTRL_PERIOD_MS))
        {
            s_still_ms -= (4U * (uint32_t)CTRL_PERIOD_MS);   /* 快速泄漏，最多 0.6s 解冻 */
        }
        else
        {
            s_still_ms = 0U;
        }

        s_zupt_active = ((s_zupt_en != 0U) && (s_still_ms >= ZUPT_HOLD_MS)) ? 1U : 0U;

        /* 长时间静止时用很小的系数在线修正零偏，进一步压漂移 */
        if ((s_zupt_active != 0U) && (s_still_ms >= (ZUPT_HOLD_MS + ZUPT_LEARN_MS)))
        {
            float bias = s_yaw.bias_dps + (ZUPT_LEARN_ALPHA * (sample.gz_dps - s_yaw.bias_dps));
            yaw_set_bias(&s_yaw, bias);
        }
    }

    /* ---------------- Yaw 积分 ---------------- */
    /* ---------------- 怠速零偏在线跟踪 ----------------
       陀螺零偏会漂（实测从 -0.15 漂到 -3.5 dps），单次标定压不住。
       电机没出力 = 转盘默认静止，这时用 15s 时间常数把零偏拉向当前 gz：
       零偏漂移会在几十秒内被吃掉；手快转（>10dps）因门限不会被学进去。 */
    if ((IDLE_BIAS_ENABLE != 0U) && (s_bias_auto != 0U) &&
        (util_absf(sample.gz_dps - s_yaw.bias_dps) < IDLE_BIAS_GATE_DPS))
    {
        uint8_t idle = 0U;

        if ((s_state == CTL_STATE_IDLE) &&
            ((tb_state() == TB_STATE_COAST) || (tb_state() == TB_STATE_STANDBY)))
        {
            idle = 1U;
        }
        else if ((s_state == CTL_STATE_RUN) &&
                 (util_absf((float)s_out_pm) < (float)IDLE_BIAS_RUN_OUT_PM))
        {
            idle = 1U;
        }

        if (idle != 0U)
        {
            float alpha = (float)CTRL_PERIOD_MS / (float)IDLE_BIAS_TAU_MS;
            float bias  = s_yaw.bias_dps + (alpha * (sample.gz_dps - s_yaw.bias_dps));
            yaw_set_bias(&s_yaw, bias);
        }
    }

    yaw_update(&s_yaw, sample.gz_dps, (s_zupt_active != 0U) ? 0.0f : dt);

    /* 选出这一拍闭环真正用的角度：ENC（不漂）/ IMU（任务书主反馈）/ FUSE */
    ctl_fb_update(dt);

    /* 最近路径误差：目标 -170°、当前 +170° 时误差是 +20°，不是 -340°。
       IDLE 时也一直算，STAT/INFO 直接能看到离目标还有多少。 */
    s_error = yaw_shortest_error(s_target, yaw_wrap180(s_fb_deg));

    /* ---------------- 闭环 ---------------- */
    if (s_state == CTL_STATE_RUN)
    {
        float out;

        /* 用连续角做 PID 的输入输出，避免在 ±180 处跳变：
           等效目标 = 当前连续角 + 最短路径误差 */
        pid_set_rate(&s_pid, s_fb_rate);    /* 微分项用干净的速度观测 */
        out = pid_update(&s_pid, s_fb_deg + s_error, s_fb_deg, dt);

        /* 软限位保护排线：接近限位时只允许往回收方向出力 */
        if ((s_fb_deg > (s_limit - SOFT_LIMIT_MARGIN_DEG)) && (out > 0.0f))
        {
            out = 0.0f;
        }
        if ((s_fb_deg < -(s_limit - SOFT_LIMIT_MARGIN_DEG)) && (out < 0.0f))
        {
            out = 0.0f;
        }

        /* 制动增强（见 board_config）：
           只有"误差与速度反号"才是被外力推离目标，这时全力抵抗；
           朝目标收敛时（同号）保持正常限幅——那一阶段该由 Kd 做阻尼，
           再放大只会刹过头反向冲出去（实测过：反向超调 14°）。 */
        if ((util_absf(s_fb_rate) > BRAKE_RATE_DPS) &&
            ((s_error * s_fb_rate) < 0.0f))
        {
            out *= (float)BRAKE_BOOST_NUM;
            if (out > (float)OUTPUT_BRAKE_PM) { out = (float)OUTPUT_BRAKE_PM; }
            if (out < -(float)OUTPUT_BRAKE_PM) { out = -(float)OUTPUT_BRAKE_PM; }
        }
        else
        {
            if (out > (float)s_out_max) { out = (float)s_out_max; }
            if (out < -(float)s_out_max) { out = -(float)s_out_max; }
        }
        s_out_pm = (int16_t)out;

        /* 输出小到推不动电机时改成短路刹车：
           转盘几乎没有摩擦，滑行会一直漂过目标，静差压不住 */
        if (util_absf(out) < (float)BRAKE_OUT_PM)
        {
            tb_brake();
        }
        else
        {
            tb_set_cmd(s_out_pm);
        }

        /* 越限：立刻刹车并报故障，防止把排线拧断 */
        if (util_absf(s_fb_deg) > s_limit)
        {
            ctl_raise_fault(CTL_FAULT_LIMIT);
            return;
        }

        /* 堵转检测：给了足够力度，编码器却完全不动。
           编码器当反馈源时这个必须开着——它是唯一的闭环依据，
           线一松就只能靠这里发现（门限也降到了 STALL_PM_MIN_ENC）。 */
        if (s_stall_en != 0U)
        {
            if (util_absf((float)s_out_pm) >= (float)s_stall_pm_min)
            {
                if (enc_count() != s_stall_ref_count)
                {
                    s_stall_ref_count = enc_count();
                    s_stall_ref_ms    = now;
                }
                else if ((now - s_stall_ref_ms) >= STALL_TIMEOUT_MS)
                {
                    ctl_raise_fault(CTL_FAULT_STALL);
                    return;
                }
            }
            else
            {
                s_stall_ref_count = enc_count();
                s_stall_ref_ms    = now;
            }
        }
    }

    /* ---------------- 开环点动保护 ----------------
       开环没有 PID 兜底，必须在这里守住两条：
       1) 越限立刻刹车报故障；
       2) 接近限位且还在往外推，直接断电滑行；
       3) 点动超过 MANUAL_TIMEOUT_MS 自动断电——
          防止上位机/脚本异常退出后电机一直转（这个坑已经踩过一次）。 */
    if ((s_state == CTL_STATE_IDLE) &&
        ((tb_state() == TB_STATE_FWD) || (tb_state() == TB_STATE_REV)))
    {
        float m  = util_absf(s_fb_deg);
        int16_t c = tb_cmd();

        if (m > s_limit)
        {
            ctl_raise_fault(CTL_FAULT_LIMIT);
            return;
        }
        if ((m > (s_limit - SOFT_LIMIT_MARGIN_DEG)) && ((s_fb_deg * (float)c) > 0.0f))
        {
            tb_coast();
            logln("LIMIT: manual blocked, only %d deg left", (int)(s_limit - m));
            return;
        }
        if ((now - s_manual_ms) > MANUAL_TIMEOUT_MS)
        {
            tb_coast();
            logln("SAFE: manual motion timed out (%ums), motor off", (unsigned)MANUAL_TIMEOUT_MS);
            return;
        }
    }
}

/* ==================================================================
 * 命令接口
 * ================================================================== */
void ctl_request_calib(void)
{
    if (s_state == CTL_STATE_BOOT)
    {
        return;     /* 还没识别到 IMU，等它自己重试 */
    }
    ctl_begin_calib(HAL_GetTick());
}

void ctl_zero_yaw(void)
{
    float old = s_fb_deg;       /* 当前闭环角，目标要跟着一起平移 */

    yaw_reset(&s_yaw, 0.0f);
    /* 目标角跟着平移，保持当前实际角度相对目标的差值不变 */
    s_target = yaw_wrap180(s_target - old);
    pid_reset(&s_pid);
    enc_reset();

    /* 编码器角、融合角一起归零，否则切换反馈源会出现一次大跳变 */
    s_enc_deg  = 0.0f;
    s_enc_rate = 0.0f;
    s_fuse_deg = 0.0f;
    s_fb_deg   = 0.0f;
    s_fb_rate  = 0.0f;
}

uint8_t ctl_set_target(float deg)
{
    float d   = yaw_wrap180(deg);
    float lim = s_limit - TARGET_LIMIT_MARGIN_DEG;

    if ((d > lim) || (d < -lim))
    {
        return 0U;      /* 超出限位允许的目标范围，拒绝 */
    }
    s_target = d;
    return 1U;
}

uint8_t ctl_start(void)
{
    if (s_state != CTL_STATE_IDLE)
    {
        return 0U;      /* 只有标定完成、无故障时才能启动 */
    }
    if (util_absf(s_fb_deg) > (s_limit - SOFT_LIMIT_MARGIN_DEG))
    {
        return 0U;      /* 已经在限位边上，先用手转回来 */
    }

    pid_reset(&s_pid);
    s_error   = yaw_shortest_error(s_target, yaw_wrap180(s_fb_deg));
    s_out_pm  = 0;
    s_state   = CTL_STATE_RUN;

    /* 堵转检测的参考时刻必须在启动时重新打点。
       它只在 RUN 里更新，如果沿用上一次运行留下的旧时刻，
       第一拍 out 一大、编码器又还没来得及动，now-旧时刻 早就超过
       STALL_TIMEOUT_MS，会立刻误判 FAULT:STALL（实测踩到过：
       目标一改就报堵转，转盘纹丝不动）。 */
    s_stall_ref_ms    = HAL_GetTick();
    s_stall_ref_count = enc_count();

    tb_coast();         /* 先上使能、零占空比，再由 PID 接管 */
    return 1U;
}

void ctl_stop(void)
{
    if ((s_state == CTL_STATE_CALIB) || (s_state == CTL_STATE_BOOT))
    {
        return;
    }
    s_state   = CTL_STATE_IDLE;
    s_fault   = CTL_FAULT_NONE;
    s_out_pm  = 0;
    pid_reset(&s_pid);
    tb_coast();
}

uint8_t ctl_clear_fault(void)
{
    if (s_state != CTL_STATE_FAULT)
    {
        return 0U;
    }
    if (util_absf(s_fb_deg) > s_limit)
    {
        return 0U;      /* 还超着限位，先手动转回来 */
    }
    s_fault  = CTL_FAULT_NONE;
    s_state  = CTL_STATE_IDLE;
    s_out_pm = 0;
    pid_reset(&s_pid);
    tb_coast();
    return 1U;
}

uint8_t ctl_manual(int16_t pm)
{
    float   m;
    int16_t eff;    /* 取反之后真正作用到电机的方向 */

    if (s_state != CTL_STATE_IDLE)
    {
        return 0U;      /* 闭环时不允许开环点动 */
    }

    /* 限位守卫必须按"电机实际转向"判断：INV=ON 时正命令其实是反转。
       以前这里直接用 pm，接上 INV 后守卫方向是反的，反向脉冲会被全部
       拒绝，只剩单向累积——实测把转盘一路推到 +121° 撞限位。 */
    eff = (tb_invert() != 0U) ? (int16_t)(-pm) : pm;

    /* 开环点动也要尊重限位：剩余角度不足且方向朝外，直接拒绝 */
    m = util_absf(s_fb_deg);
    if ((eff != 0) && (m > (s_limit - SOFT_LIMIT_MARGIN_DEG)) && ((s_fb_deg * (float)eff) > 0.0f))
    {
        return 0U;
    }

    if (pm != 0)
    {
        s_manual_ms = HAL_GetTick();
    }
    tb_set_cmd(pm);
    return 1U;
}

void ctl_manual_release(void)
{
    s_manual_ms = 0U;
    if (s_state == CTL_STATE_IDLE)
    {
        tb_coast();
    }
}

/* ==================================================================
 * 参数
 * ================================================================== */
void ctl_set_gains(float kp, float ki, float kd)
{
    pid_set_gains(&s_pid, kp, ki, kd);
}

void ctl_get_gains(float *kp, float *ki, float *kd)
{
    if (kp != 0)
    {
        *kp = s_pid.kp;
    }
    if (ki != 0)
    {
        *ki = s_pid.ki;
    }
    if (kd != 0)
    {
        *kd = s_pid.kd;
    }
}

pid_mode_t ctl_pid_mode(void)
{
    return s_pid.mode;
}

void ctl_set_pid_mode(pid_mode_t m)
{
    s_pid.mode = m;
    pid_reset(&s_pid);
}

uint8_t ctl_set_limit(float deg)
{
    float d = util_absf(deg);

    if ((d < SOFT_LIMIT_MIN_DEG) || (d > SOFT_LIMIT_MAX_DEG))
    {
        return 0U;
    }
    s_limit = d;
    if (s_target > (s_limit - TARGET_LIMIT_MARGIN_DEG))
    {
        s_target = s_limit - TARGET_LIMIT_MARGIN_DEG;
    }
    if (s_target < -(s_limit - TARGET_LIMIT_MARGIN_DEG))
    {
        s_target = -(s_limit - TARGET_LIMIT_MARGIN_DEG);
    }
    return 1U;
}

float ctl_limit(void)
{
    return s_limit;
}

void ctl_set_zupt(uint8_t en)
{
    s_zupt_en = (en != 0U) ? 1U : 0U;
}

uint8_t ctl_zupt(void)
{
    return s_zupt_en;
}

uint8_t ctl_zupt_active(void)
{
    return s_zupt_active;
}

void ctl_set_bias_auto(uint8_t en)
{
    s_bias_auto = (en != 0U) ? 1U : 0U;
}

void ctl_set_out_max(int16_t pm)
{
    if (pm < 20) { pm = 20; }
    if (pm > 1000) { pm = 1000; }
    s_out_max = pm;

    /* 必须把限幅同步给 PID 自己，否则抗饱和的条件积分配的是 -1000..1000，
       而真正生效的是外面的 s_out_max（比如 250）——积分会在被外圈砍掉的
       那段时间里继续涨到 ±400，误差一反向立刻冲过头。实测 Kp=12 Ki=8 时
       30° 阶跃超调能到 3.6°，改成同步之后抗饱和才真正起作用。 */
    s_pid.out_max = (float)pm;
    s_pid.out_min = -(float)pm;
}

int16_t ctl_out_max(void)
{
    return s_out_max;
}

uint8_t ctl_bias_auto(void)
{
    return s_bias_auto;
}

void ctl_set_stall(uint8_t en)
{
    s_stall_en = (en != 0U) ? 1U : 0U;
}

uint8_t ctl_stall(void)
{
    return s_stall_en;
}

/* ==================================================================
 * 反馈源 / 编码器标定
 * ================================================================== */
const char *ctl_fb_str_of(ctl_fb_t fb)
{
    switch (fb)
    {
    case CTL_FB_ENC:  return "ENC";
    case CTL_FB_FUSE: return "FUSE";
    case CTL_FB_IMU:
    default:          return "IMU";
    }
}

const char *ctl_fb_str(void)
{
    return ctl_fb_str_of(s_fb);
}

ctl_fb_t ctl_fb(void)
{
    return s_fb;
}

/**
 * @brief 切换反馈源（只能在非 RUN 状态切换）
 *
 * 切到 FUSE 时把融合状态直接对到编码器角，否则第一拍会看到陀螺与编码器
 * 之间的历史差值，白白拉一个大误差出来。
 * 切到 ENC 时自动打开堵转检测：编码器是唯一反馈，线松了就没有别的保护。
 */
uint8_t ctl_set_fb(ctl_fb_t fb)
{
    if (s_state == CTL_STATE_RUN)
    {
        return 0U;
    }

    switch (fb)
    {
    case CTL_FB_IMU:
    case CTL_FB_ENC:
    case CTL_FB_FUSE:
        break;
    default:
        return 0U;
    }

    s_fb = fb;

    if (fb == CTL_FB_FUSE)
    {
        s_fuse_deg = s_enc_deg;
    }
    ctl_fb_update(0.0f);
    pid_reset(&s_pid);

    if (fb == CTL_FB_ENC)
    {
        s_stall_en     = 1U;
        s_stall_pm_min = (int16_t)STALL_PM_MIN_ENC;
    }
    else
    {
        s_stall_pm_min = (int16_t)STALL_PM_MIN;
    }
    return 1U;
}

void ctl_set_cpd(float cpd)
{
    if ((cpd >= 0.5f) && (cpd <= 100000.0f))
    {
        s_cpd = cpd;
    }
}

float ctl_cpd(void)
{
    return s_cpd;
}

/**
 * @brief 编码器方向：+1 与 Yaw 同向，-1 反向
 *
 * 改了方向就把累计角清零（当前物理位置成为新的 0°），
 * 所以顺序应该是：先定方向、再 ZERO、再 START。
 */
void ctl_set_enc_sign(float sign)
{
    float s = (sign < 0.0f) ? -1.0f : 1.0f;

    if (s != s_enc_sign)
    {
        s_enc_sign = s;
        s_enc_deg  = 0.0f;
        s_enc_rate = 0.0f;
        s_fuse_deg = 0.0f;
        s_fb_deg   = 0.0f;
        pid_reset(&s_pid);
    }
}

float ctl_enc_sign(void)
{
    return s_enc_sign;
}

float ctl_enc_deg(void)
{
    return s_enc_deg;
}

float ctl_enc_rate_dps(void)
{
    return s_enc_rate;
}

float ctl_fb_deg(void)
{
    return s_fb_deg;
}

float ctl_imu_enc_diff(void)
{
    return s_yaw.cont - s_enc_deg;
}

/* ==================================================================
 * 观测
 * ================================================================== */
ctl_state_t ctl_state(void)
{
    return s_state;
}

const char *ctl_state_str(void)
{
    switch (s_state)
    {
    case CTL_STATE_BOOT:  return "BOOT";
    case CTL_STATE_CALIB: return "CALIB";
    case CTL_STATE_IDLE:  return "IDLE";
    case CTL_STATE_RUN:   return "RUN";
    case CTL_STATE_FAULT: return "FAULT";
    default:              return "?";
    }
}

ctl_fault_t ctl_fault(void)
{
    return s_fault;
}

const char *ctl_fault_str(void)
{
    switch (s_fault)
    {
    case CTL_FAULT_LIMIT: return "LIMIT";
    case CTL_FAULT_STALL: return "STALL";
    case CTL_FAULT_IMU:   return "IMU";
    case CTL_FAULT_NONE:
    default:              return "NONE";
    }
}

float ctl_target(void)
{
    return s_target;
}

float ctl_yaw(void)
{
    return yaw_wrap180(s_fb_deg);
}

float ctl_yaw_cont(void)
{
    return s_fb_deg;
}

float ctl_rate_dps(void)
{
    return s_fb_rate;
}

float ctl_bias_dps(void)
{
    return s_yaw.bias_dps;
}

float ctl_error(void)
{
    return s_error;
}

int16_t ctl_output_pm(void)
{
    return s_out_pm;
}

int16_t ctl_duty_pm(void)
{
    return tb_duty_pm();
}

uint32_t ctl_ctrl_count(void)
{
    return s_ctrl_count;
}

uint8_t ctl_saturated(void)
{
    return s_pid.saturated;
}

uint8_t ctl_ready(void)
{
    return (s_state == CTL_STATE_IDLE) ? 1U : 0U;
}

uint8_t ctl_imu_ok(void)
{
    return s_imu_ok;
}

uint8_t ctl_calib_quality(void)
{
    return s_cal_quality;
}

uint32_t ctl_imu_errors(void)
{
    return s_imu_err;
}

const char *ctl_imu_status_str(void)
{
    return mpu_status_str(s_mpu_status);
}
