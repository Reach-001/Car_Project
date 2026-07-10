#include "app_test.h"

#include "main.h"              /* HAL_GetTick */
#include "bsp_buzzer.h"
#include "bsp_encoder.h"
#include "bsp_gpio_sensor.h"
#include "bsp_key.h"
#include "bsp_led.h"
#include "bsp_motor.h"
#include "bsp_servo.h"
#include "bsp_uart.h"
#include "scheduler.h"
#include "tim.h"

#include <string.h>

/* ══════════════════════════════════════════════════════════════
 * 测试 1：LED
 *
 * 预期：LED1→LED2→LED3→STATE_LED 依次亮 500ms，重复 3 轮。
 *       完成后 STATE_LED 常亮。
 * ══════════════════════════════════════════════════════════════ */

#if TEST_SELECT == TEST_LED

static bool s_done;

void AppTest_Init(void) { BspLed_Init(); s_done = false; }

void AppTest_Run(void)
{
    static const BspLedId seq[4] = {BSP_LED_1, BSP_LED_2, BSP_LED_3, BSP_LED_STATE};
    static int round, led;
    static uint32_t t0;
    static bool inited;

    if (s_done) return;

    if (!inited) {
        inited = true;
        t0 = HAL_GetTick();
        BspLed_Set(seq[0], true);   /* 先点亮第一个 */
    }

    if ((uint32_t)(HAL_GetTick() - t0) >= 500U) {
        t0 = HAL_GetTick();

        BspLed_Set(seq[led], false);  /* 灭当前 */
        ++led;

        if (led >= 4) {
            led = 0;
            if (++round >= 3) {
                s_done = true;
                BspLed_Set(BSP_LED_STATE, true);
                return;
            }
        }
        BspLed_Set(seq[led], true);   /* 亮下一个 */
    }
}

#endif /* TEST_LED */

/* ══════════════════════════════════════════════════════════════
 * 测试 2：按键
 *
 * 预期：按 KEY1→LED1 亮，KEY2→LED2 亮，KEY3→LED3 亮，KEY4→STATE 亮。
 *       全按过 STATE 常亮 = 通过。
 * ══════════════════════════════════════════════════════════════ */

#if TEST_SELECT == TEST_KEY

static bool s_key_done[4];

void AppTest_Init(void)
{
    BspLed_Init();
    BspKey_Init();
    for (int i = 0; i < 4; ++i) s_key_done[i] = false;
}

void AppTest_Run(void)
{
    static uint32_t last;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last) < 10U) return;
    last = now;

    if (s_key_done[0] && s_key_done[1] && s_key_done[2] && s_key_done[3]) return;

    BspKey_Task10ms();
    if (BspKey_TakeClickedEvent(BSP_KEY_1)) { BspLed_Set(BSP_LED_1, true); s_key_done[0] = true; }
    if (BspKey_TakeClickedEvent(BSP_KEY_2)) { BspLed_Set(BSP_LED_2, true); s_key_done[1] = true; }
    if (BspKey_TakeClickedEvent(BSP_KEY_3)) { BspLed_Set(BSP_LED_3, true); s_key_done[2] = true; }
    if (BspKey_TakeClickedEvent(BSP_KEY_4)) { BspLed_Set(BSP_LED_STATE, true); s_key_done[3] = true; }
}

#endif /* TEST_KEY */

/* ══════════════════════════════════════════════════════════════
 * 测试 3：蜂鸣器
 *
 * 预期：2000→3000→4000→1000→500Hz 各 2s，间隔 1s 静音。
 *       LED1 亮 = 发声阶段，LED2 亮 = 静音阶段。
 *       完成后 STATE_LED 常亮。
 * ══════════════════════════════════════════════════════════════ */

#if TEST_SELECT == TEST_BUZZER

void AppTest_Init(void)
{
    BspLed_Init();
    BspBuzzer_Init();
}

void AppTest_Run(void)
{
    static const uint16_t freqs[] = {2000, 3000, 4000, 1000, 500};
    static const int N = sizeof(freqs) / sizeof(freqs[0]);
    static int step;               /* 0..N-1=发声, N..2N-1=静音 */
    static uint32_t t0;
    static bool inited;

    uint32_t now = HAL_GetTick();

    if (!inited) {
        inited = true; t0 = now;
        BspBuzzer_SetFrequency(freqs[0]);
        BspBuzzer_Set(true);
        BspLed_Set(BSP_LED_1, true);
    }

    /* 发声 2s，静音 1s */
    uint32_t dur = (step < N) ? 2000U : 1000U;

    if ((uint32_t)(now - t0) >= dur) {
        t0 = now; ++step;

        if (step < N) {
            BspBuzzer_SetFrequency(freqs[step]);
            BspBuzzer_Set(true);
        } else if (step < N * 2) {
            BspBuzzer_Set(false);
        }

        BspLed_Set(BSP_LED_1, step < N);
        BspLed_Set(BSP_LED_2, step >= N && step < N * 2);
    }

    if (step >= N * 2) {
        BspBuzzer_Set(false);
        BspLed_Set(BSP_LED_1, false);
        BspLed_Set(BSP_LED_2, false);
        BspLed_Set(BSP_LED_STATE, true);
    }
}

#endif /* TEST_BUZZER */

/* ══════════════════════════════════════════════════════════════
 * 测试 4：电机
 *
 * ⚠ 车轮需悬空！
 *
 * 预期：只测试前进差速转弯：
 *       实车标定结果：
 *       左 90%、右 60% → 左转；
 *       左 60%、右 90% → 右转；
 *       每段之间 0/0 滑行停止，循环执行。
 * ══════════════════════════════════════════════════════════════ */

#if TEST_SELECT == TEST_MOTOR

void AppTest_Init(void) { BspLed_Init(); BspMotor_Init(); }

void AppTest_Run(void)
{
    struct { int16_t L; int16_t R; uint32_t ms; int led; } seq[] = {
        /* Left turn: calibrated on the vehicle, left motor faster than right. */
        {  900,   600, 2500, 1}, {    0,     0,  700, 0},

        /* Right turn: calibrated on the vehicle, right motor faster than left. */
        {  600,   900, 2500, 2}, {    0,     0,  700, 0},
    };
    static const int N = sizeof(seq) / sizeof(seq[0]);
    static int step;
    static uint32_t t0;
    uint32_t now = HAL_GetTick();

    if (step == 0 && t0 == 0) { t0 = now; BspLed_Set(BSP_LED_STATE, false); }

    if (step < N) {
        BspMotor_SetDuty(BSP_MOTOR_LEFT,  seq[step].L);
        BspMotor_SetDuty(BSP_MOTOR_RIGHT, seq[step].R);

        BspLed_Set(BSP_LED_1, seq[step].led == 1);
        BspLed_Set(BSP_LED_2, seq[step].led == 2);
        BspLed_Set(BSP_LED_3, false);

        if ((uint32_t)(now - t0) >= seq[step].ms) {
            step = (step + 1) % N;
            t0 = now;
        }
    }
}

#endif /* TEST_MOTOR */

/* ══════════════════════════════════════════════════════════════
 * 测试 5：舵机
 *
 * ⚠ 确认舵机机械行程无阻碍！
 *
 * 预期：中→右小(600‰)→中→左小(-600‰)→中，各 1s。STATE_LED 常亮 = 通过。
 * ══════════════════════════════════════════════════════════════ */

#if TEST_SELECT == TEST_SERVO

void AppTest_Init(void) { BspLed_Init(); BspServo_Init(); }

void AppTest_Run(void)
{
    static const int16_t targets[] = {0, 600, 0, -600, 0};
    static int step;
    static uint32_t t0;
    uint32_t now = HAL_GetTick();

    if (step == 0 && t0 == 0) { t0 = now; BspServo_SetSteerPermille(targets[0]); BspLed_Set(BSP_LED_2, true); }

    if (step < 5 && (uint32_t)(now - t0) >= 1000U) {
        ++step; t0 = now;
        if (step < 5) {
            BspServo_SetSteerPermille(targets[step]);
            BspLed_Set(BSP_LED_1, targets[step] < 0);
            BspLed_Set(BSP_LED_2, targets[step] == 0);
            BspLed_Set(BSP_LED_3, targets[step] > 0);
        }
    }

    if (step >= 5) {
        BspLed_Set(BSP_LED_1, false); BspLed_Set(BSP_LED_2, false); BspLed_Set(BSP_LED_3, false);
        BspLed_Set(BSP_LED_STATE, true);
    }
}

#endif /* TEST_SERVO */

/* ══════════════════════════════════════════════════════════════
 * 测试 6：编码器
 *
 * 预期：手转轮子，LED1~3 按增量大小点亮。按 KEY1 退出 → STATE 常亮。
 * ══════════════════════════════════════════════════════════════ */

#if TEST_SELECT == TEST_ENCODER

void AppTest_Init(void)
{
    BspLed_Init();
    BspEncoder_Init();
    BspKey_Init();
}

void AppTest_Run(void)
{
    static bool done;
    static uint32_t last;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last) < 10U) return;
    last = now;

    if (done) return;

    BspKey_Task10ms();
    if (BspKey_TakeClickedEvent(BSP_KEY_1)) { done = true; }

    if (!done) {
        BspEncoderSample enc = BspEncoder_Read();
        int32_t aL = enc.left_delta;   if (aL < 0) aL = -aL;
        int32_t aR = enc.right_delta;  if (aR < 0) aR = -aR;
        int32_t maxd = (aL > aR) ? aL : aR;
        BspLed_Set(BSP_LED_1, maxd > 10);
        BspLed_Set(BSP_LED_2, maxd > 50);
        BspLed_Set(BSP_LED_3, maxd > 200);
    } else {
        BspLed_Set(BSP_LED_1, false); BspLed_Set(BSP_LED_2, false); BspLed_Set(BSP_LED_3, false);
        BspLed_Set(BSP_LED_STATE, true);
    }
}

#endif /* TEST_ENCODER */

/* ══════════════════════════════════════════════════════════════
 * 测试 7：传感器（循迹 + 超声波）
 *
 * 预期：遮循迹探头 → 对应 LED 亮。
 *       障碍物 < 30cm → 蜂鸣器响 + LED1~3 全亮。
 *       按 KEY1 退出 → STATE 常亮。
 * ══════════════════════════════════════════════════════════════ */

#if TEST_SELECT == TEST_SENSOR

#include "ultrasonic.h"

void AppTest_Init(void)
{
    BspLed_Init();
    BspGpioSensor_Init();
    BspKey_Init();
    BspBuzzer_Init();
    Ultrasonic_Init();
}

void AppTest_Run(void)
{
    static bool done;
    static uint32_t last;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last) < 10U) return;
    last = now;

    BspKey_Task10ms();
    if (BspKey_TakeClickedEvent(BSP_KEY_1)) done = true;

    if (done) {
        BspBuzzer_Set(false);
        BspLed_Set(BSP_LED_1, false); BspLed_Set(BSP_LED_2, false);
        BspLed_Set(BSP_LED_3, false); BspLed_Set(BSP_LED_STATE, true);
        return;
    }

    /* ── 循迹：5 路映射 4 LED ── */
    BspTrackRaw track = BspGpioSensor_ReadTrack();
    BspLed_Set(BSP_LED_1,     track.sensor[0]);
    BspLed_Set(BSP_LED_2,     track.sensor[1]);
    BspLed_Set(BSP_LED_3,     track.sensor[2]);
    BspLed_Set(BSP_LED_STATE, track.sensor[3] || track.sensor[4]);

    /* ── 超声波：由模块 Task10ms 周期驱动 ── */
    Ultrasonic_Task10ms();
    UltrasonicSample us = Ultrasonic_GetSample();
    bool near = us.valid && us.distance_mm < 300U;   /* 30cm 内 = 有障碍 */

    BspBuzzer_Set(near);
    BspBuzzer_Task10ms();
    if (near) {
        BspLed_Set(BSP_LED_1, true);
        BspLed_Set(BSP_LED_2, true);
        BspLed_Set(BSP_LED_3, true);
    }
}

#endif /* TEST_SENSOR */

/* ══════════════════════════════════════════════════════════════
 * 测试 8：UART DMA
 *
 *  方案A（短接 USART2 TX → USART3 RX）：发 "TEST\r\n" → 检查收回 → OK
 *  方案B（串口助手）：向 USART2 发任意数据，LED 显示收到字节数
 *  KEY1 切换 A/B，KEY4 重发。
 * ══════════════════════════════════════════════════════════════ */

#if TEST_SELECT == TEST_UART

void AppTest_Init(void)
{
    BspLed_Init();
    BspKey_Init();
    BspUart_Init(BSP_UART_K230);
    BspUart_Init(BSP_UART_BT);
}

void AppTest_Run(void)
{
    static bool done, mode_echo = true, test_sent;
    static char rx_buf[64];
    static int  rx_idx;
    static uint32_t last, t_sent;
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last) < 10U) return;
    last = now;

    if (done) return;

    BspKey_Task10ms();
    if (BspKey_TakeClickedEvent(BSP_KEY_1)) { mode_echo = !mode_echo; }
    if (BspKey_TakeClickedEvent(BSP_KEY_4)) { test_sent = false; rx_idx = 0; }

    if (mode_echo) {
        if (!test_sent) {
            BspUart_WriteString(BSP_UART_K230, "TEST\r\n");
            test_sent = true; t_sent = now; rx_idx = 0;
            BspLed_Set(BSP_LED_2, true);
        }
        uint8_t b;
        while (BspUart_ReadByte(BSP_UART_BT, &b) && rx_idx < 63) rx_buf[rx_idx++] = (char)b;
        rx_buf[rx_idx] = '\0';

        if (rx_idx > 0 && strstr(rx_buf, "TEST")) {
            BspLed_Set(BSP_LED_1, false); BspLed_Set(BSP_LED_2, false); BspLed_Set(BSP_LED_3, false);
            BspLed_Set(BSP_LED_STATE, true); done = true;
        }
        /* 超时 5s → LED1 亮（需重新短接 TX-RX） */
        if (!done && rx_idx == 0 && (uint32_t)(now - t_sent) > 5000U) {
            BspLed_Set(BSP_LED_1, true);
            test_sent = false;
        }
    } else {
        uint8_t b; uint16_t n = 0;
        while (BspUart_ReadByte(BSP_UART_K230, &b)) ++n;
        BspLed_Set(BSP_LED_1, n > 0);
        BspLed_Set(BSP_LED_2, n > 5);
        BspLed_Set(BSP_LED_3, n > 20);
    }
}

#endif /* TEST_UART */

/* ══════════════════════════════════════════════════════════════
 * 默认：不测试
 * ══════════════════════════════════════════════════════════════ */

#if TEST_SELECT == TEST_NONE
void AppTest_Init(void) {}
void AppTest_Run(void)  {}
#endif
