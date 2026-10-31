/**
 * @file    test_task3.c
 * @brief   任务三（TB6612 + Yaw PID）纯算法部分的主机端单元测试
 *
 * 只编译与硬件无关的模块：app_pid.c / app_yaw.c / app_util.c，
 * 因此不需要任何 HAL 桩，也不需要开发板。
 *
 * 覆盖：
 *   - 角度回绕与"最短路径"（发挥项 3：+170° 到 -170° 只转 20°）
 *   - Yaw 积分：去零偏、dt 精度、冻结、跨 ±180°
 *   - 位置式 PID：限幅、条件积分抗饱和、微分先行无冲击
 *   - 增量式 PID：累加特性、限幅
 *   - 工具函数：浮点/定点解析、Newton 开方
 *
 * 用法：build_and_run_task3.ps1
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "app_pid.h"
#include "app_util.h"
#include "app_yaw.h"

static int g_pass;
static int g_fail;

static void check(int cond, const char *what)
{
    if (cond)
    {
        g_pass++;
    }
    else
    {
        g_fail++;
        printf("  FAIL: %s\n", what);
    }
}

static int near(double a, double b, double eps)
{
    return fabs(a - b) <= eps;
}

static void test_util(void)
{
    float f;
    char  buf[16];

    printf("[util]\n");
    check(util_parse_float("0", &f) == 1 && near(f, 0.0, 1e-6), "parse 0");
    check(util_parse_float("12.5", &f) == 1 && near(f, 12.5, 1e-6), "parse 12.5");
    check(util_parse_float("-3.25", &f) == 1 && near(f, -3.25, 1e-6), "parse -3.25");
    check(util_parse_float("+0.5", &f) == 1 && near(f, 0.5, 1e-6), "parse +0.5");
    check(util_parse_float("  45 ", &f) == 1 && near(f, 45.0, 1e-6), "parse spaces");
    check(util_parse_float("abc", &f) == 0, "reject abc");
    check(util_parse_float("", &f) == 0, "reject empty");
    check(util_parse_float("-", &f) == 0, "reject lone sign");
    check(util_parse_float("1.2.3", &f) == 1 && near(f, 1.2, 1e-6), "parse 1.2.3 -> 1.2");

    check(near(util_sqrtf(16.0f), 4.0, 1e-3), "sqrt 16");
    check(near(util_sqrtf(2.0f), 1.41421356, 1e-3), "sqrt 2");
    check(near(util_sqrtf(0.0f), 0.0, 1e-6), "sqrt 0");
    check(near(util_sqrtf(-1.0f), 0.0, 1e-6), "sqrt negative -> 0");
    check(near(util_absf(-2.5f), 2.5, 1e-6), "absf");

    util_format_fixed(buf, 375U, 1U);
    check(strcmp(buf, "37.5") == 0, "format_fixed 37.5");
    util_format_fixed(buf, 5U, 2U);
    check(strcmp(buf, "0.05") == 0, "format_fixed 0.05");
}

static void test_yaw(void)
{
    yaw_t y;
    int   i;

    printf("[yaw]\n");
    check(near(yaw_wrap180(0.0f), 0.0, 1e-6), "wrap 0");
    check(near(yaw_wrap180(179.9f), 179.9, 1e-4), "wrap 179.9");
    check(near(yaw_wrap180(190.0f), -170.0, 1e-4), "wrap 190 -> -170");
    check(near(yaw_wrap180(-190.0f), 170.0, 1e-4), "wrap -190 -> 170");
    check(near(yaw_wrap180(540.0f), -180.0, 1e-4), "wrap 540 -> -180");
    check(near(yaw_wrap180(-540.0f), -180.0, 1e-4), "wrap -540 -> -180");

    /* 发挥项 3：从 +170° 转到 -170° 只转 20°（正方向） */
    check(near(yaw_shortest_error(-170.0f, 170.0f), 20.0, 1e-4), "shortest +170 -> -170 = +20");
    check(near(yaw_shortest_error(170.0f, -170.0f), -20.0, 1e-4), "shortest -170 -> +170 = -20");
    check(near(yaw_shortest_error(0.0f, 179.0f), -179.0, 1e-4), "shortest 179 -> 0 = -179");
    check(near(yaw_shortest_error(179.0f, 0.0f), 179.0, 1e-4), "shortest 0 -> 179 = +179");

    /* 积分：零偏 1dps，实测 11dps，200 拍 x 5ms -> 首拍只建基准，应积 9.95° */
    yaw_init(&y, 1.0f);
    for (i = 0; i < 200; i++)
    {
        yaw_update(&y, 11.0f, 0.005f);
    }
    check(near(y.rate_dps, 10.0, 1e-4), "yaw rate = gz - bias");
    check(near(y.cont, 9.95, 0.05), "yaw integration 9.95 deg");

    /* 冻结积分：dt = 0 时角度不变，但角速度仍更新 */
    {
        float before = y.cont;
        yaw_update(&y, 50.0f, 0.0f);
        check(near(y.cont, before, 1e-6), "frozen integration keeps angle");
        check(near(y.rate_dps, 49.0, 1e-4), "frozen still updates rate");
    }

    /* 跨 ±180：连续角 175° 再加 10° -> cont 185、显示 -175 */
    yaw_reset(&y, 175.0f);
    y.started = 1U;
    yaw_update(&y, 10.0f + y.bias_dps, 1.0f);
    check(near(y.cont, 185.0, 1e-3), "cont crosses 180");
    check(near(y.yaw, -175.0, 1e-3), "display wraps to -175");

    yaw_reset(&y, 200.0f);
    check(near(y.yaw, -160.0, 1e-3), "reset wraps immediately");
}

static void test_pid_position(void)
{
    pid_t p;

    printf("[pid position]\n");
    pid_init(&p, 10.0f, 0.0f, 0.0f, -1000.0f, 1000.0f, PID_MODE_POSITION);
    pid_set_d_lpf(&p, 0.0f);          /* 关掉低通，方便断言 */
    check(near(pid_update(&p, 10.0f, 0.0f, 0.01f), 100.0, 1e-3), "P only: 10 deg -> 100pm");

    /* 输出限幅 */
    pid_init(&p, 1000.0f, 0.0f, 0.0f, -100.0f, 100.0f, PID_MODE_POSITION);
    check(near(pid_update(&p, 10.0f, 0.0f, 0.01f), 100.0, 1e-3), "output clamped to max");
    check(p.saturated == 1U, "saturation flag set");

    /* 积分限幅（抗饱和的第二重保险） */
    pid_init(&p, 0.0f, 1000.0f, 0.0f, -1000.0f, 1000.0f, PID_MODE_POSITION);
    pid_set_integ_limit(&p, -50.0f, 50.0f);
    {
        int i;
        for (i = 0; i < 100; i++)
        {
            (void)pid_update(&p, 10.0f, 0.0f, 0.01f);
        }
    }
    check(near(p.out, 50.0, 1e-3), "integral clamped to 50");

    /* 条件积分：饱和后误差反向，输出要立刻退出饱和 */
    pid_init(&p, 100.0f, 50.0f, 0.0f, -100.0f, 100.0f, PID_MODE_POSITION);
    {
        int   i;
        float out;

        for (i = 0; i < 200; i++)
        {
            (void)pid_update(&p, 10.0f, 0.0f, 0.01f);   /* 长时间饱和 */
        }
        check(near(p.out, 100.0, 1e-3), "saturated at max after long error");
        out = pid_update(&p, 0.0f, 0.1f, 0.01f);        /* 误差 -0.1 */
        check(out < 100.0f, "leaves saturation in one sample (conditional integration)");
        check(p.integ <= 100.0f, "integral not wind up");
    }

    /* 微分先行：目标阶跃不产生微分冲击 */
    pid_init(&p, 0.0f, 0.0f, 100.0f, -1000.0f, 1000.0f, PID_MODE_POSITION);
    pid_set_d_lpf(&p, 0.0f);
    (void)pid_update(&p, 0.0f, 0.0f, 0.01f);            /* 建立基准 */
    check(near(pid_update(&p, 10.0f, 0.0f, 0.01f), 0.0, 1e-3), "no derivative kick on setpoint step");

    /* 微分项对测量值有效：测量以 10dps 上升 -> D 项 -1000 */
    check(near(pid_update(&p, 10.0f, 0.1f, 0.01f), -1000.0, 1.0), "derivative on measurement");

    /* 重置 */
    pid_reset(&p);
    check(near(p.out, 0.0, 1e-6) && near(p.integ, 0.0, 1e-6), "pid_reset clears state");

    /* 低通：alpha=0.9 时微分冲击应显著变小 */
    pid_init(&p, 0.0f, 0.0f, 100.0f, -1000.0f, 1000.0f, PID_MODE_POSITION);
    pid_set_d_lpf(&p, 0.9f);
    (void)pid_update(&p, 0.0f, 0.0f, 0.01f);
    check(fabs(pid_update(&p, 0.0f, 0.1f, 0.01f)) < 200.0, "derivative low-pass attenuates");
}

static void test_pid_incremental(void)
{
    pid_t p;
    int   i;

    printf("[pid incremental]\n");
    /* 增量式：Δu = Kp*Δe + Ki*e*dt，误差"不变"时只有 Ki 项在累加，
       这一条正是位置式与增量式最直观的区别，答辩常被问到。 */
    pid_init(&p, 1.0f, 100.0f, 0.0f, -1000.0f, 1000.0f, PID_MODE_INCREMENTAL);
    check(near(pid_update(&p, 1.0f, 0.0f, 0.01f), 2.0, 1e-4), "first step = Kp*de + Ki*e*dt");

    /* 误差恒定：Kp 项贡献一次就没了，Ki 项每拍加 Ki*e*dt = 1 */
    for (i = 0; i < 5; i++)
    {
        (void)pid_update(&p, 1.0f, 0.0f, 0.01f);
    }
    check(near(p.out, 7.0, 1e-3), "constant error accumulates through Ki only");

    /* 纯 P 的增量式：误差不再变化时输出保持不变 */
    pid_init(&p, 1.0f, 0.0f, 0.0f, -1000.0f, 1000.0f, PID_MODE_INCREMENTAL);
    (void)pid_update(&p, 1.0f, 0.0f, 0.01f);
    {
        float before = p.out;
        (void)pid_update(&p, 1.0f, 0.0f, 0.01f);   /* 误差没变，Δe = 0 */
        check(near(p.out, before, 1e-3), "P-only incremental keeps output when error is steady");
    }

    /* 限幅 */
    pid_init(&p, 100.0f, 0.0f, 0.0f, -10.0f, 10.0f, PID_MODE_INCREMENTAL);
    check(near(pid_update(&p, 1.0f, 0.0f, 0.01f), 10.0, 1e-3), "incremental clamps output");
    check(p.saturated == 1U, "incremental saturation flag");

    check(strcmp(pid_mode_str(PID_MODE_POSITION), "POS") == 0, "mode string POS");
    check(strcmp(pid_mode_str(PID_MODE_INCREMENTAL), "INC") == 0, "mode string INC");
}

/**
 * 闭环仿真：对象按 TB6612 的真实非线性搭
 *   duty = 死区 + |u|*(1000-死区)/1000        （与 app_tb6612.c 一致）
 *   速度 = (duty - 库仑摩擦) * 增益，摩擦以下不动
 *   速度再经过 30ms 一阶惯性（电机+转盘惯量）
 * 验证默认增益能收敛、抗扰动且不振荡。
 */
static float sim_step(pid_t *p, float sp, float theta, float *vel, float dt)
{
    const float dz    = 150.0f;
    const float fric  = 180.0f;
    const float gain  = 0.12f;      /* 千分比 -> dps */
    float u = pid_update(p, sp, theta, dt);
    float mag = fabsf(u);
    float duty = (mag <= 5.0f) ? 0.0f : (dz + (mag * (1000.0f - dz) / 1000.0f));
    float vt;

    if (u < 0.0f)
    {
        duty = -duty;
    }
    vt = (fabsf(duty) <= fric) ? 0.0f : ((duty > 0.0f ? duty - fric : duty + fric) * gain);
    *vel += (vt - *vel) * (dt / 0.03f);
    return theta + (*vel * dt);
}

static void test_closed_loop_sim(void)
{
    pid_t p;
    float theta = 0.0f;
    float vel   = 0.0f;
    float sp    = 30.0f;
    float dt    = 0.005f;
    float max_over = 0.0f;
    int   i;

    printf("[closed loop sim with friction]\n");
    pid_init(&p, 20.0f, 5.0f, 3.0f, -1000.0f, 1000.0f, PID_MODE_POSITION);
    pid_set_d_lpf(&p, 0.7f);
    pid_set_integ_limit(&p, -400.0f, 400.0f);

    for (i = 0; i < 4000; i++)      /* 20 s */
    {
        theta = sim_step(&p, sp, theta, &vel, dt);
    }
    check(fabsf(theta - sp) < 2.5f, "settles inside 2.5 deg (friction + dead zone)");

    /* 发挥项 2：外力把转盘拨开 30°，松手后应回到目标且不超调 */
    theta -= 30.0f;
    for (i = 0; i < 4000; i++)
    {
        theta = sim_step(&p, sp, theta, &vel, dt);
        if ((theta - sp) > max_over)
        {
            max_over = theta - sp;
        }
    }
    check(fabsf(theta - sp) < 2.5f, "recovers after 30 deg kick");
    check(max_over < 2.0f, "no overshoot after kick");

    /* 目标突变：最短路径 + 收敛 */
    sp = -30.0f;
    for (i = 0; i < 8000; i++)
    {
        theta = sim_step(&p, sp, theta, &vel, dt);
    }
    check(fabsf(theta - sp) < 2.5f, "retarget to -30 deg converges");
}

int main(void)
{
    printf("== task3 host unit tests ==\n");
    test_util();
    test_yaw();
    test_pid_position();
    test_pid_incremental();
    test_closed_loop_sim();

    printf("\n%d passed, %d failed\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
