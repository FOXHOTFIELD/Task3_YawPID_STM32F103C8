/**
 * @file    app_cli.h
 * @brief   串口命令行：接收环形缓冲 + 逐行解析
 *
 * 接收用"1 字节中断 + 环形缓冲"，主循环里非阻塞地取字节拼行，
 * 绝不在中断里解析命令；行缓冲满了丢弃并回 ERR:LINE_TOO_LONG。
 *
 * 命令与 VOFA+ 数据流共用同一个串口（115200 8N1）：
 *   数据帧形如 yaw:12.345,-3.210\r\n（带冒号标签、全小写）；
 *   命令形如 SET=90、START、KP=18.5（大写关键字）。
 * 解析器只把"整行都是合法命令"的行当命令，其余静默忽略，
 * 所以两条链路互不干扰。
 */
#ifndef APP_CLI_H
#define APP_CLI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void cli_init(void);

/** @brief 主循环里反复调用，非阻塞 */
void cli_poll(void);

/** @brief 串口收到一个字节时调用（HAL_UART_RxCpltCallback 里） */
void cli_rx_byte(uint8_t b);

/** @brief 接收环形缓冲是否溢出过（STAT 行显示） */
uint8_t cli_overflow(void);

#ifdef __cplusplus
}
#endif

#endif /* APP_CLI_H */
