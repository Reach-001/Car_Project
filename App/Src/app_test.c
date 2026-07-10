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
#include "estimation.h"
#include "motion.h"
#include "scheduler.h"
#include "sensor_domain.h"
#include "system_state_pool.h"
#include "tim.h"

#include <string.h>

#if (TEST_SELECT == TEST_MOTOR) || (TEST_SELECT == TEST_ENCODER) || (TEST_SELECT == TEST_SENSOR)
static SystemStatePool s_test_pool;
#endif

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
 * 预期：通过 V2 Motion 链路测试前进转弯：
 *       target speed/steer → Estimation → Motion → 电机/舵机。
 *       左转、停车、右转、停车循环执行。
 * ══════════════════════════════════════════════════════════════ */

#if TEST_SELECT == TEST_MOTOR

void AppTest_Init(void)
{
    BspLed_Init();
    BspKey_Init();
    BspMotor_Init();
    (void)BspServo_Init();
    BspEncoder_Init();

    SystemStatePool_Init(&s_test_pool);
    Estimation_Init();
    Motion_Init();
}

void AppTest_Run(void)
{
    struct { float speed_mps; float steer_rad; uint32_t ms; int led; } seq[] = {
        /* V2 Motion demo: negative steer = left turn, positive steer = right turn. */
        {  0.30f, -0.35f, 2500, 1}, {  0.0f, 0.0f,  700, 0},
        {  0.30f,  0.35f, 2500, 2}, {  0.0f, 0.0f,  700, 0},
    };
    static const int N = sizeof(seq) / sizeof(seq[0]);
    static int step;
    static uint32_t t0;
    static uint32_t last;
    uint32_t now = HAL_GetTick();

    if (step == 0 && t0 == 0) { t0 = now; BspLed_Set(BSP_LED_STATE, false); }
    if ((uint32_t)(now - last) < 10U) return;
    last = now;

    BspKey_Task10ms();
    if (BspKey_TakeClickedEvent(BSP_KEY_1))
    {
        s_test_pool.target.speed_mps = 0.0f;
        s_test_pool.target.steer_angle_rad = 0.0f;
        s_test_pool.target.valid = true;
        Motion_Task10ms(&s_test_pool);
        BspLed_Set(BSP_LED_STATE, true);
        return;
    }

    if (step < N) {
        s_test_pool.tick_ms = now;
        s_test_pool.target.speed_mps = seq[step].speed_mps;
        s_test_pool.target.steer_angle_rad = seq[step].steer_rad;
        s_test_pool.target.valid = true;

        Estimation_Task10ms(&s_test_pool);
        Motion_Task10ms(&s_test_pool);

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
    SystemStatePool_Init(&s_test_pool);
    Estimation_Init();
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
        Estimation_Task10ms(&s_test_pool);
        int32_t aL = s_test_pool.estimation.left_encoder_delta;
        int32_t aR = s_test_pool.estimation.right_encoder_delta;
        if (aL < 0) aL = -aL;
        if (aR < 0) aR = -aR;
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

void AppTest_Init(void)
{
    BspLed_Init();
    BspGpioSensor_Init();
    BspKey_Init();
    BspBuzzer_Init();
    SystemStatePool_Init(&s_test_pool);
    Sensor_Init();
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

    Sensor_Task20ms(&s_test_pool);

    BspLed_Set(BSP_LED_1, (s_test_pool.sensor.track_bits & (1U << 0)) != 0U);
    BspLed_Set(BSP_LED_2, (s_test_pool.sensor.track_bits & (1U << 1)) != 0U);
    BspLed_Set(BSP_LED_3, (s_test_pool.sensor.track_bits & (1U << 2)) != 0U);
    BspLed_Set(BSP_LED_STATE, (s_test_pool.sensor.track_bits & ((1U << 3) | (1U << 4))) != 0U);

    bool near = s_test_pool.sensor.obstacle_near;

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
 *  桥接测试：
 *    USART2_RX(PA3)  收到 → USART3_TX(PC10) 发出，LED1 闪
 *    USART3_RX(PC11) 收到 → USART2_TX(PA2)  发出，LED2 闪
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
    static uint32_t last;
    uint8_t bridge_buf[64];
    uint32_t now = HAL_GetTick();
    if ((uint32_t)(now - last) < 10U) return;
    last = now;

    BspKey_Task10ms();
    if (BspKey_TakeClickedEvent(BSP_KEY_1)) {
        BspLed_Set(BSP_LED_STATE, true);
    }

    uint16_t n = 0U;
    while ((n < sizeof(bridge_buf)) && BspUart_ReadByte(BSP_UART_K230, &bridge_buf[n])) {
        ++n;
    }
    bool forwarded_2_to_3 = (n > 0U) && BspUart_TxDone(BSP_UART_BT) &&
                            BspUart_WriteBuffer(BSP_UART_BT, bridge_buf, n);

    n = 0U;
    while ((n < sizeof(bridge_buf)) && BspUart_ReadByte(BSP_UART_BT, &bridge_buf[n])) {
        ++n;
    }
    bool forwarded_3_to_2 = (n > 0U) && BspUart_TxDone(BSP_UART_K230) &&
                            BspUart_WriteBuffer(BSP_UART_K230, bridge_buf, n);

    BspLed_Set(BSP_LED_1, forwarded_2_to_3);
    BspLed_Set(BSP_LED_2, forwarded_3_to_2);
    BspLed_Set(BSP_LED_3, !forwarded_2_to_3 && !forwarded_3_to_2);
}

#endif /* TEST_UART */

/* ══════════════════════════════════════════════════════════════
 * 默认：不测试
 * ══════════════════════════════════════════════════════════════ */

#if TEST_SELECT == TEST_NONE
void AppTest_Init(void) {}
void AppTest_Run(void)  {}
#endif
