/**
 * @file    app_tb6612.h
 * @brief   TB6612FNG 电机驱动（正转 / 反转 / 刹车 / 滑行 / 待机 + 死区补偿）
 *
 * 评分点 1：硬件 PWM 调速、方向控制、刹车，并处理电机启动死区。
 * 约定的"力度"单位是千分比（-1000..1000），与 PID 输出同一量纲：
 *   0        -> 滑行（AIN1=AIN2=0，电机不受力）
 *   正数     -> 正转
 *   负数     -> 反转
 *   驱动层负责把力度映射到 [死区..1000] 的占空比，所以任何非零力度都推得动电机。
 */
#ifndef APP_TB6612_H
#define APP_TB6612_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum
{
    TB_STATE_STANDBY = 0,   /**< STBY=0：整片待机，输出高阻 */
    TB_STATE_COAST   = 1,   /**< AIN1=AIN2=0：滑行（惯性自由转） */
    TB_STATE_FWD     = 2,   /**< AIN1=1, AIN2=0 */
    TB_STATE_REV     = 3,   /**< AIN1=0, AIN2=1 */
    TB_STATE_BRAKE   = 4    /**< AIN1=AIN2=1：短路刹车 */
} tb_state_t;

/** @brief 初始化：STBY 拉低、占空比清零、启动 PWM 通道 */
void tb_init(void);

/** @brief 设定力度（千分比，-1000..1000）；0 = 滑行 */
void tb_set_cmd(int16_t cmd_pm);

/** @brief 短路刹车（AIN1=AIN2=1，占空比 100%） */
void tb_brake(void);

/** @brief 滑行（STBY=1，AIN1=AIN2=0，占空比 0） */
void tb_coast(void);

/** @brief 待机（STBY=0），电机彻底断开 */
void tb_standby(void);

int16_t     tb_cmd(void);        /**< 最近一次设定的力度 */
int16_t     tb_duty_pm(void);    /**< 实际写入的占空比（已含死区补偿） */
tb_state_t  tb_state(void);
const char *tb_state_str(void);

/** @brief 死区补偿量（千分比），默认 MOTOR_DEADZONE_PM */
void    tb_set_deadzone(int16_t pm);
int16_t tb_deadzone(void);

/** @brief 方向取反（电机线接反时用，不必重新焊线） */
void    tb_set_invert(uint8_t inv);
uint8_t tb_invert(void);

/** @brief 启动"踢一脚"：换向瞬间给 kick_pm 力度、持续 kick_ms 毫秒（0 = 关闭） */
void     tb_set_kick(int16_t pm, uint16_t ms);
int16_t  tb_kick_pm(void);
uint16_t tb_kick_ms(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_TB6612_H */
