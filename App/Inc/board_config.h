/**
 * @file    board_config.h
 * @brief   板级配置集中处：引脚、定时器、电机与 PID 参数都在这里改
 *
 * 接线表（STM32F103C8T6 最小系统板 + TB6612FNG + GY-521/MPU6050）
 * ------------------------------------------------------------------
 *  功能        引脚      模式                  接到
 *  PWMA        PA6       TIM3_CH1 PWM 20kHz    TB6612 PWMA
 *  AIN1        PA4       GPIO 推挽             TB6612 AIN1
 *  AIN2        PA5       GPIO 推挽             TB6612 AIN2
 *  STBY        PB0       GPIO 推挽（上电低）   TB6612 STBY
 *  PWMB        PA7       预留（TIM3_CH2）      第二路电机备用，本工程不发波
 *  I2C1_SCL    PB6       AF 开漏 400kHz        MPU6050 SCL
 *  I2C1_SDA    PB7       AF 开漏 400kHz        MPU6050 SDA
 *  USART1_TX   PA9       AF 推挽 115200        CH340 RX（VOFA+ / 命令）
 *  USART1_RX   PA10      上拉输入              CH340 TX
 *  状态灯      PC13      GPIO 推挽（板载）     心跳 / 标定 / 故障
 *  按键        PA0       EXTI0 下降沿 + 上拉   按钮另一端接 GND
 *  SWD         PA13/PA14 -                     烧写调试，占用不解
 * ------------------------------------------------------------------
 *  TB6612 电源（比引脚更容易接错，务必逐条核对）：
 *    VM  <- 电机电源 6~12V（独立电池/电源，绝不可从开发板 5V 取）
 *    VCC -> 3V3  ★必须 3.3V：VIH = 0.7×VCC，VCC=5V 时门限 3.5V，3.3V 逻辑推不动
 *    GND -> 与开发板、电机电源负极共地
 *    VM-GND 就近并 100uF 电解 + 100nF 瓷片，抑制换向火花干扰 I2C
 *    AO1/AO2 -> 电机两根线
 *
 *  注意：不要开 TIM3 部分重映射（TIM3_REMAP），它会抢走 PB5/PB0/PB1，
 *        与 MPU6050 的 INT 和这里 STBY 的 PB0 冲突。
 */
#ifndef BOARD_CONFIG_H
#define BOARD_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

/* ================= 固件版本 ================= */
#define FW_VERSION_STR            "v1.2.4"   /* 改固件时同步改这里，INFO 会打印 */

/* ================= 时钟 ================= */
#define BOARD_SYSCLK_HZ           72000000UL
#define BOARD_HSE_HZ              8000000UL
#define BOARD_TIM_CLK_HZ          72000000UL   /* TIM2/3/4 计数时钟 */

/* ================= TB6612 电机驱动 ================= */
#define TB_PWM_PORT               GPIOA
#define TB_PWM_PIN                GPIO_PIN_6
#define TB_PWM_TIM_CH             TIM_CHANNEL_1

#define TB_AIN1_PORT              GPIOA
#define TB_AIN1_PIN               GPIO_PIN_4
#define TB_AIN2_PORT              GPIOA
#define TB_AIN2_PIN               GPIO_PIN_5

#define TB_STBY_PORT              GPIOB
#define TB_STBY_PIN               GPIO_PIN_0

/* PWM：72MHz / (ARR+1) = 20kHz。20kHz 超出人耳可听范围，电机不会啸叫；
   ARR=3599 同时给出 3600 级占空比分辨率（0.028%），死区补偿足够细。 */
#define TB_PWM_ARR                3599U
#define TB_PWM_FREQ_HZ            20000UL

/* 方向取反：电机两根线接反了、或要让"正输出"对应顺时针，就置 1 不用重接线
   ★ 本机实测：TB6612 到电机的两根线是反的，必须置 1，否则正输出让编码器角
     减小，闭环变成正反馈会直接加速跑飞。 */
#define MOTOR_DIR_INVERT          1

/* 电机启动死区（千分比）：低于这个占空比电机不转，只会"嗡嗡"发热。
   PID 输出是"期望力度"（-1000..1000），驱动层把它映射到
   [死区..1000]，于是任何非零输出都一定能推动电机。
   实测（拆掉 MPU 后转盘自由）：60/90/120‰ 完全不转，150‰ 起转，
   180‰≈40dps，300‰≈100dps。取 145 让补偿刚好贴住起动门限。 */
#define MOTOR_DEADZONE_PM         145

/* 命令死区：PID 输出换算后小于这个千分比就当作 0（避免零附近抖振换向） */
#define MOTOR_CMD_MIN_PM          1

/* 阻尼刹车门限：闭环输出小到这个程度就进去短路刹车，而不是滑行。
   转盘断电后靠自身摩擦能停住，但带着速度冲过目标前先刹车能明显压超调。 */
#define BRAKE_OUT_PM              6

/* ================= 制动增强（非对称出力） =================
 * 实测发现"加速能力"和"制动能力"的需求完全相反：
 *   - 加速给太大 → 一冲就过，满力正反打，±25° 振荡（OMAX=600 实测）；
 *   - 制动给太小 → 手一推就顶不住（实测手推到 500dps，401‰ 占空比完全没辙）。
 * 所以出力上限做成非对称：正常出力受 OMAX 限制，**被外力推离目标时**放大
 * BRAKE_BOOST_NUM 倍（最多 OUTPUT_BRAKE_PM），用来顶住外推/快速拉回。
 *
 * 判据必须是"误差 × 速度 < 0"（正在远离目标），不能只看"输出与速度反向"：
 * 朝目标收敛减速时输出也是反向的，那样会被误放大 3 倍、刹过头反向冲出去
 * （实测反向超调 14°），改成误差判据后正常收敛不受影响。
 */
#define BRAKE_BOOST_NUM           3        /* 抗扰时出力放大倍数 */
#define BRAKE_RATE_DPS            20.0f    /* 角速度超过它才算"真的在动" */
#define OUTPUT_BRAKE_PM           1000     /* 增强方向的出力硬上限 */

/* ---- 陀螺反馈专用的两处"抗冲击"参数（v1.2.4） ----
 * 换陀螺做反馈后实测到粘滑极限环：转盘在 22~35° 之间以约 5Hz 来回撞，
 * 瞬时角速度峰值 ±400dps（编码器反馈时只有 ±80dps，因为编码器测的是
 * 电机轴、且带了 ENC_RATE_TAU_MS 滤波）。
 * 这两条通路会被冲击直接点燃、把环路变成 bang-bang：
 *   1) Kd 微分项：2×400 = 800pm，每拍都打到 OMAX；
 *   2) 制动增强：|rate|>20dps 就 ×3，每个滑动尖峰都触发。
 * 对策：控制用的角速度先低通到 CTL_RATE_TAU_IMU_MS，并把制动门限抬到
 * BRAKE_RATE_IMU_DPS。编码器模式不受影响（它本来就被 ENC_RATE_TAU_MS 滤过）。 */
#define CTL_RATE_TAU_IMU_MS       80U
#define BRAKE_RATE_IMU_DPS        60.0f

/* 启动"踢一脚"：换向瞬间额外给一段较强占空比，帮电机越过静摩擦。
   默认关闭（0），需要时用串口 KICK=<pm>,<ms> 打开。 */
#define MOTOR_KICK_PM             0
#define MOTOR_KICK_MS             0U

/* ================= MPU6050 ================= */
/* 1 = 硬件 I2C1（默认）；0 = 软件模拟 I2C（对比实验用） */
#define IMU_I2C_BACKEND_HW        1

#define IMU_SCL_PORT              GPIOB
#define IMU_SCL_PIN               GPIO_PIN_6
#define IMU_SDA_PORT              GPIOB
#define IMU_SDA_PIN               GPIO_PIN_7

#define IMU_ADDR_PRIMARY          0x68U   /* AD0 = GND */
#define IMU_ADDR_SECONDARY        0x69U   /* AD0 = 3V3 */

/* 传感器内部输出率：DLPF 打开时基准 1kHz，SMPLRT_DIV=0 -> 1kHz。
   我们按控制节拍 200Hz 去读，每次读到的都是最新值，不会丢样本也不会重复。 */
#define IMU_OUTPUT_HZ             1000U
#define IMU_DLPF_CFG              3U      /* 陀螺带宽 42Hz，噪声小、延迟 4.9ms */

/* 陀螺 Z 轴安装方向补偿：+1 = 芯片面朝上（Z 轴向上，逆时针为正）；
   -1 = 芯片面朝下 / 倒装（Z 轴向下，顺时针为正）。
 * 本机是**芯片面朝下**平贴在转盘上，所以取 -1。
 * 为什么必须处理：电机正输出 -> 编码器角增大（这一条已由 FB=ENC 闭环验证），
 * 而同一次动作陀螺 gz 却是减小的，于是 FB=IMU 直接上去就是**正反馈**，
 * 会一路加速到限位。翻符号只在读传感器之后做一次，零偏标定 / ZUPT /
 * 怠速零偏 / 积分全都共用同一套符号，不会出现"标定与积分符号不一致"。
 * 芯片翻过来装了（或换了新模块）就改回 +1，也可以串口 GSIG=1 现场切。 */
#define GYRO_SIGN_DEFAULT         (-1.0f)

/* ================= 控制节拍 ================= */
/* TIM4 每 1ms 进一次中断，中断里只累加计数并置标志（不读 I2C、不做 PID）。 */
#define TICK_PERIOD_MS            1U
/* 控制周期 5ms -> 200Hz：读传感器 -> Yaw 积分 -> PID -> 写 PWM */
#define CTRL_PERIOD_MS            5U
#define CTRL_HZ                   (1000U / CTRL_PERIOD_MS)

/* VOFA+ 上报频率：200Hz 每帧约 26 字节，115200 带宽够，
   但 100Hz 曲线已经足够平滑，留一半带宽给命令回显和日志。 */
#define STREAM_HZ_DEFAULT         100U
#define STREAM_DIV                (CTRL_HZ / STREAM_HZ_DEFAULT)

/* ================= 陀螺零偏标定 ================= */
#define CAL_DURATION_MS           1500U   /* 开机静止 1.5s 采平均 -> 零偏 */
#define CAL_STILL_LIMIT_DPS       2.5f    /* 标定期间角速度标准差上限，超了给出警告 */

/* ================= 静止保持（ZUPT） ================= */
/* 零偏不可能校准到绝对 0，残余 0.1~0.3dps 就是 6~18°/min 的 Yaw 漂移。
   检测到"陀螺角速度很小"持续一段时间就认为静止，此时冻结积分，曲线就平了。 */
#define ZUPT_RATE_DPS             1.0f
#define ZUPT_EXIT_DPS             2.0f    /* 退出冻结的阈值（迟滞，避免噪声一抖就解冻） */
#define ZUPT_HOLD_MS              500U
#define ZUPT_STILL_CAP_MS         2500U   /* 静止计数的上限：封顶后退出最多 0.6s，不会"越静越难解除" */
#define ZUPT_LEARN_ALPHA          0.002f  /* 静止时在线微调零偏的 IIR 系数 */
#define ZUPT_DEFAULT              1U

/* ================= 怠速零偏在线跟踪 =================
 * 实测这片的零偏会从 -0.15 漂到 -3.5 dps（单次标定压不住，yaw 会以
 * 200°/min 跑飞）。电机没出力时默认转盘静止，用很慢的一阶跟踪把零偏
 * 慢慢拉向当前 gz：
 *   时间常数 15s —— 零偏漂移几分钟内被吃掉，手转（几秒）基本不受影响；
 *   门限 10dps   —— 超过这个角速度一律不学，避免把手动转动当零偏。
 */
#define IDLE_BIAS_ENABLE          1U
#define IDLE_BIAS_TAU_MS          15000U
#define IDLE_BIAS_GATE_DPS        10.0f
#define IDLE_BIAS_RUN_OUT_PM      100     /* 闭环时输出小于它才算在保持、没在爬 */

/* ================= 反馈源（Yaw 从哪来） =================
 * 本机编码器装在**电机轴（减速箱之前）**，减速比把它折算到转盘上约
 * 13.6 counts/度（≈0.07°/count），分辨率远好于只靠陀螺积分，而且不漂。
 * 于是调参分两步走：
 *   FB=ENC   只用编码器闭环     —— 没有零偏、没有振动噪声，先把机械和增益调对
 *   FB=IMU   只用陀螺 Yaw       —— 任务书要求的主反馈
 *   FB=FUSE  陀螺高频 + 编码器低频修正（互补滤波）—— 兼顾平滑与不漂
 * 调参期默认 ENC；验收时改回 IMU 或 FUSE。
 */
#define FB_SOURCE_DEFAULT         1U      /* 0=IMU, 1=ENC, 2=FUSE */
#define FB_SOURCE_IMU             0U
#define FB_SOURCE_ENC             1U
#define FB_SOURCE_FUSE            2U

/* 编码器每"转盘度"的计数：减速比 + 编码器线数折算后的结果。
 * ★ 6.933 是机械真值，不是估的：13 线编码器 × 4 倍频 × 48:1 减速箱
 *   = 2496 count/转盘圈 = 2496/360 = 6.9333 count/度。
 *   实测核对（IMU 装回后做的，见 README 4.8 节）：
 *     手转 90°（陀螺 +87.0°）-> 编码器 44.34°（此时 cpd 还是 13.6）
 *     换算 cpd = 13.6 × 44.34/86.99 = 6.931，与机械值差 0.03%。
 *   ⚠ 旧值 13.6 整整大了一倍，等于把"30° 阶跃"当成 60° 在调，所有以角度
 *   为单位的旧结论（静差、限位、超调）都要按 ×2 重新理解。 */
#define ENC_CPD_DEFAULT           6.933f
/* 编码器计数方向：与 yaw 同向为 +1，反向填 -1（接线反了不用拆线） */
#define ENC_SIGN_DEFAULT          1.0f
/* 编码器测速低通时间常数：单拍量化误差 1/cpd/dt = 28.8dps（cpd 改对后
   从 14.7 翻了一倍，因为一个 count 现在代表 0.144°），必须压一压 */
#define ENC_RATE_TAU_MS           40U
/* 互补滤波时间常数：编码器把陀螺 Yaw 慢慢拉回来的快慢 */
#define FUSE_TAU_MS               2000U
/* FUSE 模式下编码器与陀螺差超过这个角度就认为陀螺跑偏了，直接重同步 */
#define FUSE_RESYNC_DEG           20.0f
/* 编码器闭环时把堵转门限降到这个值：编码器是唯一反馈，断线＝失控 */
#define STALL_PM_MIN_ENC          150

/* ================= PID 默认值 ================= */
/* 单位：Kp = 千分比占空比 / 度，Ki = 千分比 /(度·秒)，Kd = 千分比 /(度/秒)
 * ★ 这组是真机整定值（编码器反馈 FB=ENC，DZ=145，OMAX=300 + 制动增强 3×）：
 *     30° 阶跃：进入 ±1.5° 约 0.3s，零超调；
 *     手推 30° 扰动：被推开后单调回位、无反向超调。
 *
 *   调参踩过的两头：
 *     - Kp 小（30/4）：响应 0.58s，太慢；
 *     - Kp 大 + OMAX 大（100/3/600）：阶跃 0.26s 很漂亮，但加速度太大，
 *       一旦带着速度越过目标就满力正反打，±25° 振荡；
 *     - 正解是 OMAX 限加速度 + 制动增强给力矩，两者分开。
 *
 *   ⚠ CPD 从 13.6 改回真值 6.933 之后，同样一个物理误差换算出的"度数"
 *   翻了一倍，所以这组增益整体除了 2（60/3/4 -> 30/1.5/2）才能保持原来
 *   那段物理响应不变。别以为"增益变了就是重新调过"。
 *   切到 FB=IMU 后陀螺噪声与 42Hz 低通相位滞后会要求再降 Kp/Kd。 */
#define PID_KP_DEFAULT            30.0f
#define PID_KI_DEFAULT            1.5f
#define PID_KD_DEFAULT            2.0f
#define PID_D_LPF_DEFAULT         0.70f   /* 微分项低通，越大越平滑 */
#define PID_OUT_LIMIT_PM          1000.0f
#define OUTPUT_LIMIT_DEFAULT_PM   300     /* 正常出力上限；制动时有 BRAKE_BOOST_NUM 倍 */
#define PID_INTEG_LIMIT_PM        400.0f
#define PID_MODE_DEFAULT_POS      1U      /* 1 = 位置式（默认），0 = 增量式 */

/* ================= 软件限位（保护排线） =================
 * 限位作用在"连续累计角"上，不是显示角。
 * 本机实测排线余量约 ±180°（从中间位算），所以默认收到 165°：
 *   165° 是"必须停"的硬线，预留 15° 给惯性、齿隙和控制超调。
 * 想做"±180° 最短路径"演示时，先手动把转盘往负方向转 40° 再 ZERO，
 * 然后临时 LIMIT=200——此时 cont=190 对应的物理位置只有 150°，仍在余量内。
 */
#define SOFT_LIMIT_DEG_DEFAULT    165.0f
#define SOFT_LIMIT_MIN_DEG        30.0f
#define SOFT_LIMIT_MAX_DEG        300.0f
#define SOFT_LIMIT_MARGIN_DEG     10.0f   /* 进入这个带宽后只允许往回收 */
#define TARGET_LIMIT_MARGIN_DEG   5.0f    /* 目标角允许的最大绝对值 = 限位 - 5° */

/* 开环点动超时：上位机/脚本异常退出后，电机最多再转这么久就自动断电。
   正常调参按 200ms 一个脉冲发，每次都会重新计时。 */
#define MANUAL_TIMEOUT_MS         1500U

/* ================= 按键 ================= */
#define KEY_PORT                  GPIOA
#define KEY_PIN                   GPIO_PIN_0
#define KEY_IRQn                  EXTI0_IRQn
#define KEY_DEBOUNCE_MS           20U
#define KEY_LONG_MS               1500U

/* ================= 状态灯 ================= */
#define LED1_PORT                 GPIOC
#define LED1_PIN                  GPIO_PIN_13
#define LED1_ON_LEVEL             GPIO_PIN_RESET   /* 板载 LED 低电平点亮 */
#define LED1_OFF_LEVEL            GPIO_PIN_SET

/* ================= 外设句柄（定义在 Core/Src） ================= */
extern UART_HandleTypeDef huart1;
extern I2C_HandleTypeDef  hi2c1;
extern TIM_HandleTypeDef  htim2;   /* 编码器接口 */
extern TIM_HandleTypeDef  htim3;   /* 电机 PWM */
extern TIM_HandleTypeDef  htim4;   /* 1ms 控制节拍 */

#define ENC_TIM                    (&htim2)
#define TB_PWM_TIM                 (&htim3)
#define TICK_TIM                   (&htim4)

/* MPU 驱动内部初始化时用的默认采样率（本工程会再显式改成 IMU_OUTPUT_HZ） */
#define SAMPLE_RATE_DEFAULT_HZ     IMU_OUTPUT_HZ

#ifdef __cplusplus
}
#endif

#endif /* BOARD_CONFIG_H */
