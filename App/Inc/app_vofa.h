/**
 * @file    app_vofa.h
 * @brief   VOFA+ 上位机数据流（FireWater 文本 / JustFloat 二进制）
 *
 * 评分点 5 要求把 Target（目标）与 Yaw（实际）两组数据送进 VOFA+ 画动态波形。
 * 默认两通道：ch0 = Target(deg)，ch1 = Yaw(deg)；串口 RAW=ON 时追加
 * ch2 = 实测角速度(deg/s)，调 P/D 参数时看这个更快。
 *
 * FireWater 帧格式：  yaw:<v0>,<v1>[,<v2>]\r\n
 * JustFloat 帧格式：  小端 float × N + 帧尾 00 00 80 7F
 */
#ifndef APP_VOFA_H
#define APP_VOFA_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    VOFA_MODE_FIREWATER = 0,   /**< 文本，默认 */
    VOFA_MODE_JUSTFLOAT = 1    /**< 二进制 */
} vofa_mode_t;

void        vofa_init(void);
void        vofa_set_mode(vofa_mode_t m);
vofa_mode_t vofa_mode(void);
const char *vofa_mode_str(void);

/** @brief 两通道：Target / Yaw */
void vofa_send_yaw(float target, float actual);

/** @brief 三通道：Target / Yaw / 角速度 */
void vofa_send_yaw3(float target, float actual, float rate_dps);

/** @brief 原样发送一行文本（日志、命令回显） */
void vofa_send_text(const char *s);

/** @brief 浮点转定点字符串（四舍五入，不用 printf 的 %f） */
void vofa_format_float(char *buf, float v, uint8_t dec);

#ifdef __cplusplus
}
#endif

#endif /* APP_VOFA_H */
