/**
 * @file    app_log.c
 * @brief   串口日志与 printf 重定向（评分点 2）
 *
 * 实现要点：
 *   - logln() 先 vsnprintf 到栈上缓冲，再一次性 HAL_UART_Transmit，
 *     一行日志不会被 10ms 中断或其它输出打断；
 *   - 同时实现 fputc()，让标准库 printf() 也能输出到 USART1，
 *     这是考核要求的"printf 重定向"（Keil 工程已打开 MicroLIB）。
 */
#include "main.h"
#include "board_config.h"
#include "app_log.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define LOG_LINE_MAX   192U
#define LOG_TX_TIMEOUT 100U

static void uart_send(const char *s, uint16_t len)
{
    if (len == 0U)
    {
        return;
    }
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)s, len, LOG_TX_TIMEOUT);
}

void log_raw(const char *s)
{
    if (s == 0)
    {
        return;
    }
    uart_send(s, (uint16_t)strlen(s));
}

void logln(const char *fmt, ...)
{
    char    buf[LOG_LINE_MAX];
    int     n;
    va_list ap;

    if (fmt == 0)
    {
        return;
    }

    va_start(ap, fmt);
    n = vsnprintf(buf, sizeof(buf) - 2U, fmt, ap);
    va_end(ap);

    if (n < 0)
    {
        return;
    }
    if (n > (int)(sizeof(buf) - 3U))
    {
        n = (int)(sizeof(buf) - 3U);   /* 截断保护 */
    }
    buf[n]     = '\r';
    buf[n + 1] = '\n';
    buf[n + 2] = '\0';
    uart_send(buf, (uint16_t)(n + 2));
}

/* ------------------------------------------------------------------
 * printf 重定向：标准库把字符输出交给 fputc，这里转到 USART1
 * ------------------------------------------------------------------ */
int fputc(int ch, FILE *f)
{
    char c = (char)ch;
    (void)f;
    uart_send(&c, 1U);
    return ch;
}

void log_init(void)
{
    /* 这行故意用标准库 printf，验证重定向链路是通的 */
    printf("[BOOT] printf redirect OK, USART1=115200-8N1\r\n");
}
