/**
 * @file    app_vofa.c
 * @brief   VOFA+ 组帧与发送（FireWater 文本 / JustFloat 二进制）
 */
#include "main.h"
#include "board_config.h"
#include "app_vofa.h"

#include <string.h>

#define VOFA_BUF_MAX   128U

static vofa_mode_t s_mode = VOFA_MODE_FIREWATER;

/**
 * @brief 浮点转定点字符串（四舍五入，不用 libc 的 %f）
 * @note  Keil MicroLIB 的 printf 不支持 %f，串口日志与命令回显全部走这里。
 */
void vofa_format_float(char *buf, float v, uint8_t dec)
{
    uint32_t mul = 1U;
    uint32_t scaled;
    uint32_t whole;
    uint32_t frac;
    uint8_t  i;
    uint8_t  n = 0U;
    uint8_t  sign = 0U;
    char     tmp[12];

    if (buf == 0)
    {
        return;
    }
    if (dec > 3U)
    {
        dec = 3U;
    }
    for (i = 0U; i < dec; i++)
    {
        mul *= 10U;
    }

    if (v < 0.0f)
    {
        v    = -v;
        sign = 1U;
    }
    if (v > 1.0e6f)          /* 上限保护，避免写穿缓冲 */
    {
        v = 1.0e6f;
    }

    scaled = (uint32_t)((v * (float)mul) + 0.5f);
    whole  = scaled / mul;
    frac   = scaled % mul;

    /* 先取整再决定符号：-0.0004 舍成 0 时不要输出 "-0.000" */
    if ((sign != 0U) && (scaled != 0U))
    {
        buf[n++] = '-';
    }

    if (whole == 0U)
    {
        tmp[0] = '0';
        i = 1U;
    }
    else
    {
        i = 0U;
        while ((whole > 0U) && (i < (uint8_t)sizeof(tmp)))
        {
            tmp[i++] = (char)('0' + (whole % 10U));
            whole /= 10U;
        }
    }
    while (i > 0U)
    {
        buf[n++] = tmp[--i];
    }

    if (dec > 0U)
    {
        uint32_t d = mul / 10U;
        buf[n++] = '.';
        for (i = 0U; i < dec; i++)
        {
            buf[n++] = (char)('0' + ((frac / d) % 10U));
            if (d >= 10U)
            {
                d /= 10U;
            }
        }
    }
    buf[n] = '\0';
}

void vofa_init(void)
{
    s_mode = VOFA_MODE_FIREWATER;
    vofa_send_text("# ch0=Target(deg) ch1=Yaw(deg) ch2=Rate(dps, RAW=ON)\r\n");
}

void vofa_set_mode(vofa_mode_t m)
{
    s_mode = (m == VOFA_MODE_JUSTFLOAT) ? VOFA_MODE_JUSTFLOAT : VOFA_MODE_FIREWATER;
}

vofa_mode_t vofa_mode(void)
{
    return s_mode;
}

const char *vofa_mode_str(void)
{
    return (s_mode == VOFA_MODE_JUSTFLOAT) ? "JUSTFLOAT" : "FIREWATER";
}

static void vofa_send_firewater(const float *vals, uint8_t n)
{
    char     buf[VOFA_BUF_MAX];
    char     num[16];
    uint8_t  i;
    uint8_t  k;
    uint16_t pos = 0U;

    /* 通道标签必须是字母开头、以冒号结尾，VOFA+ 会当作通道名 */
    buf[pos++] = 'y';
    buf[pos++] = 'a';
    buf[pos++] = 'w';
    buf[pos++] = ':';

    for (i = 0U; i < n; i++)
    {
        vofa_format_float(num, vals[i], 3U);
        k = 0U;
        while ((num[k] != '\0') && ((uint16_t)(pos + 1U) < VOFA_BUF_MAX))
        {
            buf[pos++] = num[k++];
        }
        if ((uint8_t)(i + 1U) < n)
        {
            buf[pos++] = ',';
        }
    }
    buf[pos++] = '\r';
    buf[pos++] = '\n';
    buf[pos]   = '\0';

    (void)HAL_UART_Transmit(&huart1, (uint8_t *)buf, pos, 50U);
}

static void vofa_send_justfloat(const float *vals, uint8_t n)
{
    uint8_t  buf[VOFA_BUF_MAX];
    uint16_t pos = 0U;
    uint8_t  i;
    uint8_t  k;

    for (i = 0U; i < n; i++)
    {
        float v = vals[i];
        const uint8_t *p = (const uint8_t *)&v;   /* Cortex-M 小端，直接按字节拷 */
        for (k = 0U; k < 4U; k++)
        {
            buf[pos++] = p[k];
        }
    }
    buf[pos++] = 0x00U;
    buf[pos++] = 0x00U;
    buf[pos++] = 0x80U;
    buf[pos++] = 0x7FU;                          /* 帧尾，VOFA+ 靠它对齐 */

    (void)HAL_UART_Transmit(&huart1, buf, pos, 50U);
}

static void vofa_send_vals(const float *vals, uint8_t n)
{
    if ((vals == 0) || (n == 0U))
    {
        return;
    }
    if (s_mode == VOFA_MODE_JUSTFLOAT)
    {
        vofa_send_justfloat(vals, n);
    }
    else
    {
        vofa_send_firewater(vals, n);
    }
}

void vofa_send_yaw(float target, float actual)
{
    float v[2];

    v[0] = target;
    v[1] = actual;
    vofa_send_vals(v, 2U);
}

void vofa_send_yaw3(float target, float actual, float rate_dps)
{
    float v[3];

    v[0] = target;
    v[1] = actual;
    v[2] = rate_dps;
    vofa_send_vals(v, 3U);
}

void vofa_send_text(const char *s)
{
    if (s == 0)
    {
        return;
    }
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)s, (uint16_t)strlen(s), 50U);
}
