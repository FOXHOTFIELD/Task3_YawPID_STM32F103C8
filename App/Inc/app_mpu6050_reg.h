/**
 * @file    app_mpu6050_reg.h
 * @brief   MPU6050 寄存器与位域定义（依据 RM-MPU-6000A / PS-MPU-6000A）
 *
 * 注意：考核明确禁止直接调用第三方 MPU6050 驱动和 DMP 库，
 *       因此这里只放"寄存器地址 + 位域"这类芯片手册事实，
 *       读写流程、量程换算、校准与解算全部在 app_mpu6050.c 里自己实现。
 */
#ifndef APP_MPU6050_REG_H
#define APP_MPU6050_REG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------- 采样率与滤波 ---------------- */
#define MPU_REG_SMPLRT_DIV        0x19U   /* 采样率分频：Rate = 1kHz / (1 + DIV)（DLPF 使能时） */
#define MPU_REG_CONFIG            0x1AU   /* bit[2:0] DLPF_CFG，bit[5:3] EXT_SYNC_SET */
#define MPU_REG_GYRO_CONFIG       0x1BU   /* bit[4:3] FS_SEL，bit[7:5] 自检使能 */
#define MPU_REG_ACCEL_CONFIG      0x1CU   /* bit[4:3] AFS_SEL，bit[7:5] 自检使能 */

/* ---------------- 中断 ---------------- */
#define MPU_REG_INT_PIN_CFG       0x37U   /* bit7 INT_LEVEL, bit6 INT_OPEN, bit5 LATCH_INT_EN,
                                             bit4 INT_RD_CLEAR, bit3 FSYNC_INT_LEVEL,
                                             bit2 FSYNC_INT_EN, bit1 I2C_BYPASS_EN */
#define MPU_REG_INT_ENABLE        0x38U   /* bit0 DATA_RDY_EN, bit4 FIFO_OFLOW_EN, bit6 WOM_EN */
#define MPU_REG_INT_STATUS        0x3AU   /* bit0 DATA_RDY_INT, bit4 FIFO_OFLOW_INT */

/* ---------------- 数据寄存器（14 字节突发读的起点） ---------------- */
#define MPU_REG_ACCEL_XOUT_H      0x3BU   /* 0x3B..0x40：AX/AY/AZ 高字节在前 */
#define MPU_REG_TEMP_OUT_H        0x41U   /* 0x41..0x42：温度 */
#define MPU_REG_GYRO_XOUT_H       0x43U   /* 0x43..0x48：GX/GY/GZ */

/* ---------------- FIFO ---------------- */
#define MPU_REG_FIFO_EN           0x23U   /* bit3 ACCEL, bit4 ZG, bit5 YG, bit6 XG, bit7 TEMP */
#define MPU_REG_FIFO_COUNTH       0x72U   /* 0x72..0x73：FIFO 中字节数（大端） */
#define MPU_REG_FIFO_R_W          0x74U   /* FIFO 读出/写入窗口 */

/* ---------------- 电源与复位 ---------------- */
#define MPU_REG_SIGNAL_PATH_RESET 0x68U   /* bit2 GYRO_RESET, bit1 ACCEL_RESET, bit0 TEMP_RESET */
#define MPU_REG_USER_CTRL         0x6AU   /* bit6 FIFO_EN, bit2 FIFO_RESET, bit7 DMP_EN */
#define MPU_REG_PWR_MGMT_1        0x6BU   /* bit7 DEVICE_RESET, bit6 SLEEP, bit[2:0] CLKSEL */
#define MPU_REG_PWR_MGMT_2        0x6CU   /* bit[5:0] 各轴待机 */
#define MPU_REG_WHO_AM_I          0x75U   /* 默认 0x68 */

/* ---------------- 关键位域 ---------------- */
#define MPU_WHOAMI_MASK           0x7EU
#define MPU_WHOAMI_VALUE          0x68U

#define MPU_PWR1_DEVICE_RESET     0x80U
#define MPU_PWR1_SLEEP            0x40U
#define MPU_PWR1_CLKSEL_PLL_GX    0x01U   /* CLKSEL = 1：PLL with X axis gyroscope reference */

#define MPU_INT_CFG_LATCH_EN      0x20U   /* 中断保持到清除，避免 50us 窄脉冲丢失 */
#define MPU_INT_CFG_RD_CLEAR      0x10U   /* 读 INT_STATUS 即清中断 */
#define MPU_INT_CFG_ACTIVE_HIGH   0x80U

#define MPU_INT_EN_DATA_RDY       0x01U

#define MPU_USERCTRL_FIFO_EN      0x40U
#define MPU_USERCTRL_FIFO_RESET   0x04U

/* ---------------- 加速度数字低通（只有 MPU6500/9250 有） ----------------
 * 注意：MPU6050 的 0x1D 是 FF_THR（自由落体阈值），写进滤波器配置会出错，
 *       所以这个寄存器必须按芯片型号区分对待（见 app_mpu6050.c 的 mpu_init）。
 */
#define MPU_REG_ACCEL_CONFIG2     0x1DU   /* bit3 ACCEL_FCHOICE_B, bit[2:0] A_DLPF_CFG */

/* ---------------- 芯片识别（WHO_AM_I） ----------------
 * MPU6050 : 0x68（bit0/bit7 保留为 0，手册要求用 0x7E 掩码比较）
 * MPU6500 : 0x70
 * MPU9250 : 0x71（寄存器与 MPU6500 基本兼容，只是多一个 AK8963 磁力计）
 */
#define MPU_WHOAMI_MPU6050        0x68U
#define MPU_WHOAMI_MPU6500        0x70U
#define MPU_WHOAMI_MPU9250        0x71U

/* ---------------- DLPF 配置表（DLPF_CFG -> 陀螺带宽 / 延时；加速度带宽） ----------------
 * MPU6050 的 CONFIG(0x1A) 同时管加速度与陀螺：
 *   0: 260 / 0ms      1: 184 / 2.0ms    2: 94 / 3.0ms     3: 44 / 4.9ms  <-- 默认
 *   4:  21 / 8.5ms    5:  10 / 13.8ms   6:  5 / 19.0ms
 * MPU6500/9250 的 CONFIG(0x1A) 只管陀螺（加速度在 ACCEL_CONFIG2 的 A_DLPF_CFG）：
 *   陀螺 DLPF_CFG: 0: 250Hz(8kHz Fs) 1: 184 2: 92 3: 41 4: 20 5: 10 6: 5
 *   加计 A_DLPF_CFG: 0: 460 1: 184 2: 92 3: 41 4: 20 5: 10 6: 5
 *   两者都取 3 时：陀螺 41Hz、加速度 41Hz，带宽匹配，正好配合 100Hz 采样。
 */

/* ---------------- 量程与灵敏度（三款芯片相同） ----------------
 * GYRO_CONFIG  FS_SEL: 0=±250(131.0) 1=±500(65.5) 2=±1000(32.8) 3=±2000(16.4)  LSB/(°/s)
 * ACCEL_CONFIG AFS_SEL: 0=±2g(16384) 1=±4g(8192)  2=±8g(4096)   3=±16g(2048)     LSB/g
 * 温度换算公式三款不同，必须按芯片区分：
 *   MPU6050 : T(°C) = raw / 340.0  + 36.53
 *   MPU6500 : T(°C) = raw / 333.87 + 21.00   （MPU9250 同）
 */
#define MPU6050_TEMP_SENSITIVITY  340.0f
#define MPU6050_TEMP_OFFSET_C     36.53f
#define MPU6500_TEMP_SENSITIVITY  333.87f
#define MPU6500_TEMP_OFFSET_C     21.00f

#ifdef __cplusplus
}
#endif

#endif /* APP_MPU6050_REG_H */
