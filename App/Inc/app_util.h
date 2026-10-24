/**
 * @file    app_util.h
 * @brief   无依赖的小工具：字符串比较/数值解析/定点格式化
 * @note    刻意不使用 atof/strtod/printf("%f")：Keil microlib 下浮点 printf 体积大
 *          且行为受编译选项影响，这里用纯整数运算保证确定性。
 */
#ifndef APP_UTIL_H
#define APP_UTIL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/** @brief 忽略大小写的字符串相等比较；a 为输入，b 为期望（全大写） */
uint8_t util_str_eq_ci(const char *a, const char *b);

/** @brief 去掉首尾空格（原地修改），返回入参指针 */
char *util_trim(char *s);

/** @brief 跳过前导空格 */
const char *util_skip_spaces(const char *s);

/**
 * @brief 解析十进制无符号整数，遇到非数字即停止
 * @param s      输入字符串
 * @param out    输出值
 * @return 1 = 至少解析到一位数字；0 = 不是数字
 */
uint8_t util_parse_u32(const char *s, uint32_t *out);

/**
 * @brief 解析定点小数（如 "37.5"），最多 max_dec 位小数
 * @param s        输入
 * @param out_x10  以 10^max_dec 为单位的整数（max_dec=1 时 37.5 -> 375）
 * @param max_dec  允许的小数位数（1 或 2）
 * @return 1 = 成功；0 = 格式非法
 */
uint8_t util_parse_fixed(const char *s, uint32_t *out_x10, uint8_t max_dec);

/**
 * @brief 把定点整数格式化成字符串（如 375 + dec=1 -> "37.5"）
 * @param buf 输出缓冲，建议 >= 16 字节
 * @param x10 定点值
 * @param dec 小数位数
 */
void util_format_fixed(char *buf, uint32_t x10, uint8_t dec);

/** @brief 解析浮点数（不用 atof/strtod）：[+-]整数[.小数]，最多 4 位小数 */
uint8_t util_parse_float(const char *s, float *out);

/** @brief 绝对值（不引 libm，避免把整个数学库拖进固件） */
float util_absf(float x);

/** @brief 平方根（Newton 迭代，不引 libm；x<=0 返回 0） */
float util_sqrtf(float x);

#ifdef __cplusplus
}
#endif

#endif /* APP_UTIL_H */
