#ifndef APP_TEST_H
#define APP_TEST_H

/* ── BSP 驱动逐个测试框架 ──
 *
 * 使用方式：在 app_test.h 中改 #define TEST_SELECT 的值，编译烧录。
 *   0 = 不测试（正常运行 App）
 *   1 = LED 测试
 *   2 = 按键测试
 *   3 = 蜂鸣器测试
 *   4 = 电机测试
 *   5 = 舵机测试
 *   6 = 编码器测试
 *   7 = GPIO 传感器测试（循迹 + 超声波）
 *   8 = UART 回环测试
 *
 * 测试结果反馈：
 *   通过 → 蜂鸣器 OK 提示音 + STATE_LED 长亮
 *   失败 → 蜂鸣器 ERROR 提示音 + STATE_LED 快闪
 *   运行中 → LED1~3 根据测试状态变化
 * ──────────────────────────────────────────────────────────── */

#define TEST_NONE       0
#define TEST_LED        1
#define TEST_KEY        2
#define TEST_BUZZER     3
#define TEST_MOTOR      4
#define TEST_SERVO      5
#define TEST_ENCODER    6
#define TEST_SENSOR     7
#define TEST_UART       8

/* 选择当前要运行的测试 */
#define TEST_SELECT  0

void AppTest_Init(void);
void AppTest_Run(void);

#endif /* APP_TEST_H */
