/**
 * @file    app_mpu6050.h
 * @brief   自写 MPU6050 寄存器驱动接口（评分点 3）
 */
#ifndef APP_MPU6050_H
#define APP_MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    MPU_OK = 0,
    MPU_ERR_NOT_INIT,     /**< 未初始化 */
    MPU_ERR_I2C,          /**< 总线无应答 / 超时 */
    MPU_ERR_ID,           /**< WHO_AM_I 不匹配 */
    MPU_ERR_PARAM,        /**< 参数越界 */
    MPU_ERR_BUSY,         /**< DMA 尚未完成 */
    MPU_ERR_SELFTEST      /**< 自检不通过（例如未放平、量程异常） */
} mpu_status_t;

/**
 * @brief 芯片型号：靠 WHO_AM_I 自动识别
 * @note  MPU6500 与 MPU6050 寄存器大体兼容，但有两处必须区分：
 *        1) 加速度数字低通在 MPU6500 的 ACCEL_CONFIG2(0x1D)，6050 那个地址是 FF_THR；
 *        2) 温度换算公式不同（6500: raw/333.87+21，6050: raw/340+36.53）。
 */
typedef enum
{
    MPU_CHIP_NONE = 0,
    MPU_CHIP_MPU6050,     /**< WHO_AM_I = 0x68 */
    MPU_CHIP_MPU6500,     /**< WHO_AM_I = 0x70 */
    MPU_CHIP_MPU9250      /**< WHO_AM_I = 0x71，寄存器与 6500 兼容（多一个磁力计） */
} mpu_chip_t;

/** @brief 一次突发读得到的原始值（寄存器原样，未换算） */
typedef struct
{
    int16_t ax;
    int16_t ay;
    int16_t az;
    int16_t temp;
    int16_t gx;
    int16_t gy;
    int16_t gz;
} mpu_raw_t;

/** @brief 换算成物理量后的一帧数据 */
typedef struct
{
    mpu_raw_t raw;
    float ax_g;
    float ay_g;
    float az_g;
    float gx_dps;
    float gy_dps;
    float gz_dps;
    float temp_c;
    uint32_t tick_ms;
} imu_sample_t;

/** @brief 初始化：探测地址 -> 软复位 -> 时钟源 -> DLPF -> 采样率 -> 量程 -> 中断引脚 */
mpu_status_t mpu_init(void);

/** @brief 读 14 字节原始数据（0x3B 起，AX..GZ 外加温度） */
mpu_status_t mpu_read_raw(mpu_raw_t *raw);

/** @brief 读一帧并换算为 g / °/s / °C */
mpu_status_t mpu_read(imu_sample_t *sample);

/** @brief 设置采样率（10..1000Hz，DLPF 使能时基准 1kHz） */
mpu_status_t mpu_set_sample_rate(uint16_t hz);

/** @brief 设置数字低通（DLPF_CFG = 0..6） */
mpu_status_t mpu_set_dlpf(uint8_t cfg);

/** @brief 设置陀螺量程 FS_SEL = 0..3（±250/500/1000/2000 °/s） */
mpu_status_t mpu_set_gyro_fs(uint8_t fs_sel);

/** @brief 设置加速度量程 AFS_SEL = 0..3（±2/4/8/16 g） */
mpu_status_t mpu_set_accel_fs(uint8_t afs_sel);

/** @brief 数据就绪中断开关（接 PB5/EXTI5） */
mpu_status_t mpu_int_enable(uint8_t enable);

/** @brief FIFO 开关与复位 */
mpu_status_t mpu_fifo_enable(uint8_t enable);
mpu_status_t mpu_fifo_reset(void);

/** @brief 当前 FIFO 中可读字节数（每帧 8 字节：AX..GZ） */
uint16_t mpu_fifo_count(void);

/** @brief 从 FIFO 读出 len 字节 */
mpu_status_t mpu_fifo_read(uint8_t *buf, uint16_t len);

/** @brief 启动一次 DMA 突发读（仅硬件 I2C 后端有效） */
mpu_status_t mpu_start_dma_read(void);

/** @brief DMA 状态：0 = 进行中，1 = 完成，2 = 出错 */
uint8_t mpu_dma_state(void);

/** @brief DMA 完成后，把收到的 14 字节换算成物理量 */
mpu_status_t mpu_finish_dma_read(imu_sample_t *sample);

/** @brief 读 WHO_AM_I（不做初始化也能读，用于探测） */
uint8_t mpu_whoami(void);

/** @brief 实际使用的 7 位地址（0x68 或 0x69） */
uint8_t mpu_addr(void);

/** @brief 已识别到的芯片型号（mpu_init 之后有效） */
mpu_chip_t mpu_chip(void);

/** @brief 芯片型号字符串："MPU6050" / "MPU6500" / "MPU9250" / "unknown" */
const char *mpu_chip_name(void);

/** @brief 自检：WHO_AM_I + 静止时加速度模长是否接近 1g */
mpu_status_t mpu_selftest(float *acc_norm_g);

/** @brief 状态码转字符串 */
const char *mpu_status_str(mpu_status_t st);

#ifdef __cplusplus
}
#endif

#endif /* APP_MPU6050_H */
