/**
 * @file    app_log.h
 * @brief   串口日志：printf 重定向 + 行缓冲输出
 *
 * 说明：
 *   1) logln() 内部用 vsnprintf 拼好整行再一次性发送，避免多任务/中断交错；
 *   2) 同时提供 fputc() 重定向，使标准库 printf() 也能直接从 USART1 输出
 *      （满足考核「完成 printf 重定向」的要求，工程已开启 MicroLIB）。
 */
#ifndef APP_LOG_H
#define APP_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 初始化：发送一行启动信息，确认串口链路正常 */
void log_init(void);

/** @brief 原样发送字符串（不加换行） */
void log_raw(const char *s);

/** @brief 格式化并发送一行（自动补 \r\n），不支持 %f */
void logln(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif /* APP_LOG_H */
