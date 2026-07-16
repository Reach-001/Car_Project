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
#include "decision.h"
#include "estimation.h"
#include "motion.h"
#include "scheduler.h"
#include "sensor_domain.h"
#include "system_state_pool.h"
#include "tim.h"
#include "debug_trace.h"
#include "vehicle_config.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#define SPEED_PID_KEY_REPEAT_MS 250U
#define TEST_SERVO_SWEEP_DEG   20.0f

#if (TEST_SELECT == TEST_MOTOR) || (TEST_SELECT == TEST_ENCODER) || \
    (TEST_SELECT == TEST_SENSOR) || (TEST_SELECT == TEST_SPEED_PID) || \
    (TEST_SELECT == TEST_PARK_REVERSE) || (TEST_SELECT == TEST_PARK_PARALLEL)
static SystemStatePool s_test_pool;
#endif

#if (TEST_SELECT == TEST_SERVO) || (TEST_SELECT == TEST_NONE)
static int16_t servo_test_permille_from_wheel_deg(float target_deg)
{
    float limited_deg;
    float servo_deg;
    float permille;

    limited_deg = target_deg;
    if (limited_deg < VEHICLE_STEER_LEFT_MAX_DEG)
    {
        limited_deg = VEHICLE_STEER_LEFT_MAX_DEG;
    }
    else if (limited_deg > VEHICLE_STEER_RIGHT_MAX_DEG)
    {
        limited_deg = VEHICLE_STEER_RIGHT_MAX_DEG;
    }

    /* 测试输入使用“轮子相对中位角度”，输出前叠加实车居中修正。 */
    servo_deg = VEHICLE_STEER_CENTER_DEG + limited_deg;
    permille = (servo_deg / VEHICLE_MANUAL_INPUT_MAX_DEG) * 1000.0f;

    if (permille > 1000.0f) permille = 1000.0f;
    if (permille < -1000.0f) permille = -1000.0f;
    return (int16_t)permille;
}
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
 * 预期：通过 V2 Motion 链路测试电机闭环：
 *       target speed → Estimation → Motion → 电机。
 *       前进、停车、后退、停车循环执行，舵机保持中位。
 * ══════════════════════════════════════════════════════════════ */

#if TEST_SELECT == TEST_MOTOR

void AppTest_Init(void)
{
    BspLed_Init();
    BspKey_Init();
    BspMotor_Init();
    BspEncoder_Init();

    SystemStatePool_Init(&s_test_pool);
    Estimation_Init();
    Motion_Init();
}

void AppTest_Run(void)
{
    struct { float speed_mps; float steer_rad; uint32_t ms; int led; } seq[] = {
        {  0.30f, 0.0f, 2500, 1}, {  0.0f, 0.0f,  700, 0},
        { -0.30f, 0.0f, 2500, 2}, {  0.0f, 0.0f,  700, 0},
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
 * 预期：中→右 20°→中→左 20°→中，各 1s。STATE_LED 常亮 = 通过。
 * ══════════════════════════════════════════════════════════════ */

#if TEST_SELECT == TEST_SERVO

void AppTest_Init(void) { BspLed_Init(); BspServo_Init(); }

void AppTest_Run(void)
{
    static const float targets_deg[] = {0.0f, TEST_SERVO_SWEEP_DEG, 0.0f, -TEST_SERVO_SWEEP_DEG, 0.0f};
    static int step;
    static uint32_t t0;
    uint32_t now = HAL_GetTick();

    if (step == 0 && t0 == 0) {
        t0 = now;
        BspServo_SetSteerPermille(servo_test_permille_from_wheel_deg(targets_deg[0]));
        BspLed_Set(BSP_LED_2, true);
    }

    if (step < 5 && (uint32_t)(now - t0) >= 1000U) {
        ++step; t0 = now;
        if (step < 5) {
            BspServo_SetSteerPermille(servo_test_permille_from_wheel_deg(targets_deg[step]));
            BspLed_Set(BSP_LED_1, targets_deg[step] < 0.0f);
            BspLed_Set(BSP_LED_2, targets_deg[step] == 0.0f);
            BspLed_Set(BSP_LED_3, targets_deg[step] > 0.0f);
        }
    }

    if (step >= 5) {
        BspServo_SetSteerPermille(servo_test_permille_from_wheel_deg(0.0f));
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
 * 测试 9：速度环 PID 标定
 *
 * ⚠ 先悬空车轮，确认方向正确后再落地低速测试。
 *
 * 蓝牙输出 Debug 曲线帧；按键直接调整测试目标，不进入主任务框架。
 * 目标速度经过 Motion 速度 PI 闭环输出 PWM，用于标定速度环参数。
 *   KEY1 → 速度清零
 *   KEY2 → 减速一档，最低到 0，不进入倒车
 *   KEY3 → 加速一档
 *   KEY4 → 转向回中
 * ══════════════════════════════════════════════════════════════ */

#if TEST_SELECT == TEST_SPEED_PID

static float s_pid_target_speed_mps;
static float s_pid_angle_deg;
static uint32_t s_key_repeat_ms[4];

static float clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void update_speed_test_target(void)
{
    float steer = (float)s_pid_angle_deg * VEHICLE_DEG_TO_RAD;

    s_test_pool.target.speed_mps =
        clamp_float(s_pid_target_speed_mps, -VEHICLE_MAX_SPEED_MPS, VEHICLE_MAX_SPEED_MPS);
    s_test_pool.target.steer_angle_rad =
        clamp_float(steer, -VEHICLE_STEER_LEFT_CMD_LIMIT_RAD, VEHICLE_STEER_RIGHT_CMD_LIMIT_RAD);
    s_test_pool.target.valid = true;
    s_test_pool.mode = SYS_MODE_MANUAL;
}

static bool key_step_due(BspKeyId key, uint32_t now)
{
    uint32_t idx = (uint32_t)key;

    if (BspKey_TakePressedEvent(key) || BspKey_TakeClickedEvent(key))
    {
        s_key_repeat_ms[idx] = now;
        return true;
    }

    if (BspKey_IsPressed(key) &&
        ((uint32_t)(now - s_key_repeat_ms[idx]) >= SPEED_PID_KEY_REPEAT_MS))
    {
        s_key_repeat_ms[idx] = now;
        return true;
    }

    return false;
}

void AppTest_Init(void)
{
    BspLed_Init();
    BspKey_Init();
    BspMotor_Init();
    (void)BspServo_Init();
    BspEncoder_Init();
    BspUart_Init(BSP_UART_BT);

    SystemStatePool_Init(&s_test_pool);
    Estimation_Init();
    Motion_Init();
    DebugTrace_Init();
    s_test_pool.debug.enabled = true;

    s_pid_target_speed_mps = 0.0f;
    s_pid_angle_deg = 0.0f;
    for (uint32_t i = 0U; i < 4U; ++i)
    {
        s_key_repeat_ms[i] = 0U;
    }
    update_speed_test_target();
}

void AppTest_Run(void)
{
    static uint32_t last_10ms;
    static uint32_t last_100ms;
    uint32_t now = HAL_GetTick();

    s_test_pool.tick_ms = now;

    if ((uint32_t)(now - last_10ms) >= 10U)
    {
        last_10ms = now;

        BspKey_Task10ms();

        if (key_step_due(BSP_KEY_1, now))
        {
            s_pid_target_speed_mps = 0.0f;
            update_speed_test_target();
        }

        if (key_step_due(BSP_KEY_2, now))
        {
            s_pid_target_speed_mps =
                clamp_float(s_pid_target_speed_mps - VEHICLE_DEBUG_SPEED_STEP_MPS,
                            0.0f,
                            VEHICLE_MAX_SPEED_MPS);
            update_speed_test_target();
        }

        if (key_step_due(BSP_KEY_3, now))
        {
            s_pid_target_speed_mps =
                clamp_float(s_pid_target_speed_mps + VEHICLE_DEBUG_SPEED_STEP_MPS,
                            -VEHICLE_MAX_SPEED_MPS,
                            VEHICLE_MAX_SPEED_MPS);
            update_speed_test_target();
        }

        if (key_step_due(BSP_KEY_4, now))
        {
            s_pid_angle_deg = 0;
            update_speed_test_target();
        }

        Estimation_Task10ms(&s_test_pool);
        Motion_Task10ms(&s_test_pool);

        bool key1_pressed = BspKey_IsPressed(BSP_KEY_1);
        bool key2_pressed = BspKey_IsPressed(BSP_KEY_2);
        bool key3_pressed = BspKey_IsPressed(BSP_KEY_3);
        bool key4_pressed = BspKey_IsPressed(BSP_KEY_4);
        bool any_key_pressed = key1_pressed || key2_pressed || key3_pressed || key4_pressed;

        BspLed_Set(BSP_LED_1, any_key_pressed ? key1_pressed : (s_pid_target_speed_mps < 0.0f));
        BspLed_Set(BSP_LED_2, any_key_pressed ? key2_pressed : (s_pid_target_speed_mps == 0.0f));
        BspLed_Set(BSP_LED_3, any_key_pressed ? key3_pressed : (s_pid_target_speed_mps > 0.0f));
        BspLed_Set(BSP_LED_STATE,
                   any_key_pressed ||
                   (s_pid_target_speed_mps == 0.0f) ||
                   (s_test_pool.motion.left_pwm != 0) ||
                   (s_test_pool.motion.right_pwm != 0));
    }

    if ((uint32_t)(now - last_100ms) >= 100U)
    {
        last_100ms = now;
        DebugTrace_Task100ms(&s_test_pool);
    }
}

#endif /* TEST_SPEED_PID */

/* ══════════════════════════════════════════════════════════════
 * 测试 10/11：泊车动作
 *
 * TEST_PARK_REVERSE  = 倒车入库
 * TEST_PARK_PARALLEL = 侧方停车
 * 动作序列与 K230 P:1/P:2 共用 DecisionPark 状态机。
 * ══════════════════════════════════════════════════════════════ */

#if (TEST_SELECT == TEST_PARK_REVERSE) || (TEST_SELECT == TEST_PARK_PARALLEL)

void AppTest_Init(void)
{
    BspLed_Init();
    BspMotor_Init();
    (void)BspServo_Init();
    BspEncoder_Init();
    SystemStatePool_Init(&s_test_pool);
    Estimation_Init();
    Motion_Init();
    DecisionPark_Start((TEST_SELECT == TEST_PARK_REVERSE) ? DECISION_PARK_REVERSE
                                                          : DECISION_PARK_PARALLEL,
                       HAL_GetTick());
}

void AppTest_Run(void)
{
    static uint32_t last_10ms;
    uint32_t now = HAL_GetTick();
    float speed = 0.0f;
    float steer = 0.0f;
    bool active;

    if ((uint32_t)(now - last_10ms) < 10U) return;
    last_10ms = now;

    active = DecisionPark_ComputeTarget(now, &speed, &steer);
    if (!active && !DecisionPark_IsDone())
    {
        speed = 0.0f;
        steer = 0.0f;
    }

    s_test_pool.tick_ms = now;
    s_test_pool.target.speed_mps = speed;
    s_test_pool.target.steer_angle_rad = steer;
    s_test_pool.target.valid = true;

    Estimation_Task10ms(&s_test_pool);
    Motion_Task10ms(&s_test_pool);

    BspLed_Set(BSP_LED_1, TEST_SELECT == TEST_PARK_REVERSE);
    BspLed_Set(BSP_LED_2, TEST_SELECT == TEST_PARK_PARALLEL);
    BspLed_Set(BSP_LED_3, !DecisionPark_IsDone());
    BspLed_Set(BSP_LED_STATE, DecisionPark_IsDone());
}

#endif /* TEST_PARK_REVERSE / TEST_PARK_PARALLEL */

/* ══════════════════════════════════════════════════════════════
 * 默认：不测试
 * ══════════════════════════════════════════════════════════════ */

#if TEST_SELECT == TEST_NONE
void AppTest_Init(void) {}
void AppTest_Run(void)  {}
#endif

/* ══════════════════════════════════════════════════════════════
 * 运行时测试：正常 App 下通过蓝牙 TEST,n 进入
 * ══════════════════════════════════════════════════════════════ */

#if TEST_SELECT == TEST_NONE

typedef struct
{
    uint8_t test_id;
    uint8_t step;
    uint8_t round;
    uint32_t t0_ms;
    uint32_t last_10ms;
    uint32_t last_100ms;
    bool active;
    bool done;
    bool initialized;
    bool key_done[4];
    SystemStatePool pool;
    float pid_target_speed_mps;
    float pid_angle_deg;
    uint32_t key_repeat_ms[4];
} RuntimeTestState;

static RuntimeTestState s_runtime_test;

static float runtime_clamp_float(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

static void runtime_test_stop_outputs(void)
{
    DecisionPark_Stop();
    BspMotor_StopAll();
    if (BspServo_IsAvailable())
    {
        BspServo_SetSteerPermille(servo_test_permille_from_wheel_deg(0.0f));
    }
    BspBuzzer_Set(false);
    BspLed_Set(BSP_LED_1, false);
    BspLed_Set(BSP_LED_2, false);
    BspLed_Set(BSP_LED_3, false);
    BspLed_Set(BSP_LED_STATE, false);
}

static void runtime_update_speed_test_target(void)
{
    float steer = s_runtime_test.pid_angle_deg * VEHICLE_DEG_TO_RAD;

    s_runtime_test.pool.target.speed_mps =
        runtime_clamp_float(s_runtime_test.pid_target_speed_mps,
                            -VEHICLE_MAX_SPEED_MPS,
                            VEHICLE_MAX_SPEED_MPS);
    s_runtime_test.pool.target.steer_angle_rad =
        runtime_clamp_float(steer,
                            -VEHICLE_STEER_LEFT_CMD_LIMIT_RAD,
                            VEHICLE_STEER_RIGHT_CMD_LIMIT_RAD);
    s_runtime_test.pool.target.valid = true;
    s_runtime_test.pool.mode = SYS_MODE_MANUAL;
}

static bool runtime_key_step_due(BspKeyId key, uint32_t now)
{
    uint32_t idx = (uint32_t)key;

    if (BspKey_TakePressedEvent(key) || BspKey_TakeClickedEvent(key))
    {
        s_runtime_test.key_repeat_ms[idx] = now;
        return true;
    }

    if (BspKey_IsPressed(key) &&
        ((uint32_t)(now - s_runtime_test.key_repeat_ms[idx]) >= SPEED_PID_KEY_REPEAT_MS))
    {
        s_runtime_test.key_repeat_ms[idx] = now;
        return true;
    }

    return false;
}

static void runtime_test_common_init(uint8_t test_id)
{
    memset(&s_runtime_test, 0, sizeof(s_runtime_test));
    s_runtime_test.test_id = test_id;
    s_runtime_test.active = true;
    s_runtime_test.t0_ms = HAL_GetTick();

    BspLed_Init();
    BspKey_Init();
    runtime_test_stop_outputs();
}

static void runtime_test_init_by_id(uint8_t test_id)
{
    runtime_test_common_init(test_id);

    switch (test_id)
    {
    case TEST_LED:
        break;

    case TEST_KEY:
        break;

    case TEST_BUZZER:
        BspBuzzer_Init();
        BspBuzzer_SetFrequency(2000U);
        BspBuzzer_Set(true);
        BspLed_Set(BSP_LED_1, true);
        break;

    case TEST_MOTOR:
        BspMotor_Init();
        BspEncoder_Init();
        SystemStatePool_Init(&s_runtime_test.pool);
        Estimation_Init();
        Motion_Init();
        break;

    case TEST_SERVO:
        (void)BspServo_Init();
        BspServo_SetSteerPermille(servo_test_permille_from_wheel_deg(0.0f));
        BspLed_Set(BSP_LED_2, true);
        break;

    case TEST_ENCODER:
        BspEncoder_Init();
        SystemStatePool_Init(&s_runtime_test.pool);
        Estimation_Init();
        break;

    case TEST_SENSOR:
        BspGpioSensor_Init();
        BspBuzzer_Init();
        SystemStatePool_Init(&s_runtime_test.pool);
        Sensor_Init();
        break;

    case TEST_UART:
        BspUart_Init(BSP_UART_K230);
        BspUart_Init(BSP_UART_BT);
        break;

    case TEST_SPEED_PID:
        BspMotor_Init();
        (void)BspServo_Init();
        BspEncoder_Init();
        BspUart_Init(BSP_UART_BT);
        SystemStatePool_Init(&s_runtime_test.pool);
        Estimation_Init();
        Motion_Init();
        DebugTrace_Init();
        s_runtime_test.pool.debug.enabled = true;
        runtime_update_speed_test_target();
        break;

    case TEST_PARK_REVERSE:
    case TEST_PARK_PARALLEL:
        BspMotor_Init();
        (void)BspServo_Init();
        BspEncoder_Init();
        SystemStatePool_Init(&s_runtime_test.pool);
        Estimation_Init();
        Motion_Init();
        DecisionPark_Start((test_id == TEST_PARK_REVERSE) ? DECISION_PARK_REVERSE
                                                          : DECISION_PARK_PARALLEL,
                           HAL_GetTick());
        BspLed_Set((test_id == TEST_PARK_REVERSE) ? BSP_LED_1 : BSP_LED_2, true);
        break;

    default:
        AppTest_RuntimeStop();
        break;
    }
}

bool AppTest_RuntimeStart(uint8_t test_id)
{
    if ((test_id < TEST_LED) || (test_id > TEST_MAX_ID))
    {
        return false;
    }

    AppTest_RuntimeStop();
    runtime_test_init_by_id(test_id);
    return s_runtime_test.active;
}

void AppTest_RuntimeStop(void)
{
    runtime_test_stop_outputs();
    memset(&s_runtime_test, 0, sizeof(s_runtime_test));
}

bool AppTest_RuntimeActive(void)
{
    return s_runtime_test.active;
}

void AppTest_RuntimePollCommand(void)
{
    uint8_t byte;
    char line[32];
    static char rx_line[32];
    static uint8_t rx_len;

    while (BspUart_ReadByte(BSP_UART_BT, &byte))
    {
        if ((byte == '\r') || (byte == '\n'))
        {
            if (rx_len > 0U)
            {
                rx_line[rx_len] = '\0';
                (void)strncpy(line, rx_line, sizeof(line));
                line[sizeof(line) - 1U] = '\0';
                rx_len = 0U;

                if ((line[0] == 'T' || line[0] == 't') &&
                    (line[1] == 'E' || line[1] == 'e') &&
                    (line[2] == 'S' || line[2] == 's') &&
                    (line[3] == 'T' || line[3] == 't') &&
                    (line[4] == ','))
                {
                    char *end;
                    long id = strtol(&line[5], &end, 10);
                    while ((*end != '\0') && isspace((unsigned char)*end))
                    {
                        ++end;
                    }

                    if ((id == TEST_NONE) && (*end == '\0'))
                    {
                        AppTest_RuntimeStop();
                        return;
                    }
                    if ((id >= TEST_LED) && (id <= TEST_MAX_ID) && (*end == '\0'))
                    {
                        (void)AppTest_RuntimeStart((uint8_t)id);
                        return;
                    }
                }
            }
        }
        else if (rx_len < (uint8_t)(sizeof(rx_line) - 1U))
        {
            rx_line[rx_len++] = (char)byte;
        }
        else
        {
            rx_len = 0U;
        }
    }
}

static void runtime_run_led(uint32_t now)
{
    static const BspLedId seq[4] = {BSP_LED_1, BSP_LED_2, BSP_LED_3, BSP_LED_STATE};

    if (s_runtime_test.done) return;

    if (!s_runtime_test.initialized)
    {
        s_runtime_test.initialized = true;
        s_runtime_test.t0_ms = now;
        BspLed_Set(seq[0], true);
    }

    if ((uint32_t)(now - s_runtime_test.t0_ms) >= 500U)
    {
        s_runtime_test.t0_ms = now;
        BspLed_Set(seq[s_runtime_test.step], false);
        ++s_runtime_test.step;
        if (s_runtime_test.step >= 4U)
        {
            s_runtime_test.step = 0U;
            ++s_runtime_test.round;
            if (s_runtime_test.round >= 3U)
            {
                s_runtime_test.done = true;
                BspLed_Set(BSP_LED_STATE, true);
                return;
            }
        }
        BspLed_Set(seq[s_runtime_test.step], true);
    }
}

static void runtime_run_key(uint32_t now)
{
    if ((uint32_t)(now - s_runtime_test.last_10ms) < 10U) return;
    s_runtime_test.last_10ms = now;

    BspKey_Task10ms();
    if (BspKey_TakeClickedEvent(BSP_KEY_1)) { s_runtime_test.key_done[0] = true; BspLed_Set(BSP_LED_1, true); }
    if (BspKey_TakeClickedEvent(BSP_KEY_2)) { s_runtime_test.key_done[1] = true; BspLed_Set(BSP_LED_2, true); }
    if (BspKey_TakeClickedEvent(BSP_KEY_3)) { s_runtime_test.key_done[2] = true; BspLed_Set(BSP_LED_3, true); }
    if (BspKey_TakeClickedEvent(BSP_KEY_4)) { s_runtime_test.key_done[3] = true; BspLed_Set(BSP_LED_STATE, true); }
}

static void runtime_run_buzzer(uint32_t now)
{
    static const uint16_t freqs[] = {2000, 3000, 4000, 1000, 500};
    uint32_t n = (uint32_t)(sizeof(freqs) / sizeof(freqs[0]));
    uint32_t dur = (s_runtime_test.step < n) ? 2000U : 1000U;

    if ((uint32_t)(now - s_runtime_test.t0_ms) >= dur)
    {
        s_runtime_test.t0_ms = now;
        ++s_runtime_test.step;

        if (s_runtime_test.step < n)
        {
            BspBuzzer_SetFrequency(freqs[s_runtime_test.step]);
            BspBuzzer_Set(true);
        }
        else if (s_runtime_test.step < (n * 2U))
        {
            BspBuzzer_Set(false);
        }

        BspLed_Set(BSP_LED_1, s_runtime_test.step < n);
        BspLed_Set(BSP_LED_2, (s_runtime_test.step >= n) && (s_runtime_test.step < (n * 2U)));
    }

    if (s_runtime_test.step >= (n * 2U))
    {
        BspBuzzer_Set(false);
        BspLed_Set(BSP_LED_1, false);
        BspLed_Set(BSP_LED_2, false);
        BspLed_Set(BSP_LED_STATE, true);
    }
}

static void runtime_run_motor(uint32_t now)
{
    static const struct { float speed_mps; float steer_rad; uint32_t ms; int led; } seq[] = {
        {  0.30f, 0.0f, 2500U, 1}, {  0.0f, 0.0f,  700U, 0},
        { -0.30f, 0.0f, 2500U, 2}, {  0.0f, 0.0f,  700U, 0},
    };
    uint32_t n = (uint32_t)(sizeof(seq) / sizeof(seq[0]));

    if ((uint32_t)(now - s_runtime_test.last_10ms) < 10U) return;
    s_runtime_test.last_10ms = now;

    s_runtime_test.pool.tick_ms = now;
    s_runtime_test.pool.target.speed_mps = seq[s_runtime_test.step].speed_mps;
    s_runtime_test.pool.target.steer_angle_rad = seq[s_runtime_test.step].steer_rad;
    s_runtime_test.pool.target.valid = true;
    Estimation_Task10ms(&s_runtime_test.pool);
    Motion_Task10ms(&s_runtime_test.pool);

    BspLed_Set(BSP_LED_1, seq[s_runtime_test.step].led == 1);
    BspLed_Set(BSP_LED_2, seq[s_runtime_test.step].led == 2);
    BspLed_Set(BSP_LED_3, false);

    if ((uint32_t)(now - s_runtime_test.t0_ms) >= seq[s_runtime_test.step].ms)
    {
        s_runtime_test.step = (uint8_t)((s_runtime_test.step + 1U) % n);
        s_runtime_test.t0_ms = now;
    }
}

static void runtime_run_servo(uint32_t now)
{
    static const float targets_deg[] = {0.0f, TEST_SERVO_SWEEP_DEG, 0.0f, -TEST_SERVO_SWEEP_DEG, 0.0f};

    if ((s_runtime_test.step < 5U) &&
        ((uint32_t)(now - s_runtime_test.t0_ms) >= 1000U))
    {
        ++s_runtime_test.step;
        s_runtime_test.t0_ms = now;
        if (s_runtime_test.step < 5U)
        {
            float target_deg = targets_deg[s_runtime_test.step];
            BspServo_SetSteerPermille(servo_test_permille_from_wheel_deg(target_deg));
            BspLed_Set(BSP_LED_1, target_deg < 0.0f);
            BspLed_Set(BSP_LED_2, target_deg == 0.0f);
            BspLed_Set(BSP_LED_3, target_deg > 0.0f);
        }
    }

    if (s_runtime_test.step >= 5U)
    {
        BspServo_SetSteerPermille(servo_test_permille_from_wheel_deg(0.0f));
        BspLed_Set(BSP_LED_1, false);
        BspLed_Set(BSP_LED_2, false);
        BspLed_Set(BSP_LED_3, false);
        BspLed_Set(BSP_LED_STATE, true);
    }
}

static void runtime_run_encoder(uint32_t now)
{
    if ((uint32_t)(now - s_runtime_test.last_10ms) < 10U) return;
    s_runtime_test.last_10ms = now;

    Estimation_Task10ms(&s_runtime_test.pool);
    int32_t left = s_runtime_test.pool.estimation.left_encoder_delta;
    int32_t right = s_runtime_test.pool.estimation.right_encoder_delta;
    if (left < 0) left = -left;
    if (right < 0) right = -right;
    int32_t max_delta = (left > right) ? left : right;

    BspLed_Set(BSP_LED_1, max_delta > 10);
    BspLed_Set(BSP_LED_2, max_delta > 50);
    BspLed_Set(BSP_LED_3, max_delta > 200);
}

static void runtime_run_sensor(uint32_t now)
{
    if ((uint32_t)(now - s_runtime_test.last_10ms) < 20U) return;
    s_runtime_test.last_10ms = now;

    Sensor_Task20ms(&s_runtime_test.pool);
    BspLed_Set(BSP_LED_1, (s_runtime_test.pool.sensor.track_bits & (1U << 0)) != 0U);
    BspLed_Set(BSP_LED_2, (s_runtime_test.pool.sensor.track_bits & (1U << 1)) != 0U);
    BspLed_Set(BSP_LED_3, (s_runtime_test.pool.sensor.track_bits & (1U << 2)) != 0U);
    BspLed_Set(BSP_LED_STATE,
               (s_runtime_test.pool.sensor.track_bits & ((1U << 3) | (1U << 4))) != 0U);

    BspBuzzer_Set(s_runtime_test.pool.sensor.obstacle_near);
    BspBuzzer_Task10ms();
}

static void runtime_run_uart(uint32_t now)
{
    uint8_t bridge_buf[64];
    uint16_t n = 0U;

    if ((uint32_t)(now - s_runtime_test.last_10ms) < 10U) return;
    s_runtime_test.last_10ms = now;

    while ((n < sizeof(bridge_buf)) && BspUart_ReadByte(BSP_UART_K230, &bridge_buf[n]))
    {
        ++n;
    }
    bool forwarded_2_to_3 = (n > 0U) && BspUart_TxDone(BSP_UART_BT) &&
                            BspUart_WriteBuffer(BSP_UART_BT, bridge_buf, n);

    n = 0U;
    while ((n < sizeof(bridge_buf)) && BspUart_ReadByte(BSP_UART_BT, &bridge_buf[n]))
    {
        ++n;
    }
    bool forwarded_3_to_2 = (n > 0U) && BspUart_TxDone(BSP_UART_K230) &&
                            BspUart_WriteBuffer(BSP_UART_K230, bridge_buf, n);

    BspLed_Set(BSP_LED_1, forwarded_2_to_3);
    BspLed_Set(BSP_LED_2, forwarded_3_to_2);
    BspLed_Set(BSP_LED_3, !forwarded_2_to_3 && !forwarded_3_to_2);
}

static void runtime_run_speed_pid(uint32_t now)
{
    if ((uint32_t)(now - s_runtime_test.last_10ms) >= 10U)
    {
        s_runtime_test.last_10ms = now;
        s_runtime_test.pool.tick_ms = now;

        BspKey_Task10ms();
        if (runtime_key_step_due(BSP_KEY_1, now))
        {
            s_runtime_test.pid_target_speed_mps = 0.0f;
            runtime_update_speed_test_target();
        }
        if (runtime_key_step_due(BSP_KEY_2, now))
        {
            s_runtime_test.pid_target_speed_mps =
                runtime_clamp_float(s_runtime_test.pid_target_speed_mps - VEHICLE_DEBUG_SPEED_STEP_MPS,
                                    0.0f,
                                    VEHICLE_MAX_SPEED_MPS);
            runtime_update_speed_test_target();
        }
        if (runtime_key_step_due(BSP_KEY_3, now))
        {
            s_runtime_test.pid_target_speed_mps =
                runtime_clamp_float(s_runtime_test.pid_target_speed_mps + VEHICLE_DEBUG_SPEED_STEP_MPS,
                                    -VEHICLE_MAX_SPEED_MPS,
                                    VEHICLE_MAX_SPEED_MPS);
            runtime_update_speed_test_target();
        }
        if (runtime_key_step_due(BSP_KEY_4, now))
        {
            s_runtime_test.pid_angle_deg = 0.0f;
            runtime_update_speed_test_target();
        }

        Estimation_Task10ms(&s_runtime_test.pool);
        Motion_Task10ms(&s_runtime_test.pool);

        BspLed_Set(BSP_LED_1, s_runtime_test.pid_target_speed_mps < 0.0f);
        BspLed_Set(BSP_LED_2, s_runtime_test.pid_target_speed_mps == 0.0f);
        BspLed_Set(BSP_LED_3, s_runtime_test.pid_target_speed_mps > 0.0f);
        BspLed_Set(BSP_LED_STATE,
                   (s_runtime_test.pid_target_speed_mps == 0.0f) ||
                   (s_runtime_test.pool.motion.left_pwm != 0) ||
                   (s_runtime_test.pool.motion.right_pwm != 0));
    }

    if ((uint32_t)(now - s_runtime_test.last_100ms) >= 100U)
    {
        s_runtime_test.last_100ms = now;
        DebugTrace_Task100ms(&s_runtime_test.pool);
    }
}

static void runtime_run_park(uint32_t now)
{
    float speed = 0.0f;
    float steer = 0.0f;
    bool active;

    if ((uint32_t)(now - s_runtime_test.last_10ms) < 10U) return;
    s_runtime_test.last_10ms = now;
    s_runtime_test.pool.tick_ms = now;

    active = DecisionPark_ComputeTarget(now, &speed, &steer);
    if (!active && !DecisionPark_IsDone())
    {
        speed = 0.0f;
        steer = 0.0f;
        DecisionPark_Stop();
    }

    s_runtime_test.pool.target.speed_mps = speed;
    s_runtime_test.pool.target.steer_angle_rad = steer;
    s_runtime_test.pool.target.valid = true;

    Estimation_Task10ms(&s_runtime_test.pool);
    Motion_Task10ms(&s_runtime_test.pool);

    BspLed_Set(BSP_LED_1, s_runtime_test.test_id == TEST_PARK_REVERSE);
    BspLed_Set(BSP_LED_2, s_runtime_test.test_id == TEST_PARK_PARALLEL);
    BspLed_Set(BSP_LED_3, !DecisionPark_IsDone());
    BspLed_Set(BSP_LED_STATE, DecisionPark_IsDone());
}

void AppTest_RuntimeRun(void)
{
    uint32_t now;

    if (!s_runtime_test.active) return;

    now = HAL_GetTick();

    switch (s_runtime_test.test_id)
    {
    case TEST_LED:       runtime_run_led(now); break;
    case TEST_KEY:       runtime_run_key(now); break;
    case TEST_BUZZER:    runtime_run_buzzer(now); break;
    case TEST_MOTOR:     runtime_run_motor(now); break;
    case TEST_SERVO:     runtime_run_servo(now); break;
    case TEST_ENCODER:   runtime_run_encoder(now); break;
    case TEST_SENSOR:    runtime_run_sensor(now); break;
    case TEST_UART:      runtime_run_uart(now); break;
    case TEST_SPEED_PID: runtime_run_speed_pid(now); break;
    case TEST_PARK_REVERSE:
    case TEST_PARK_PARALLEL:
        runtime_run_park(now);
        break;
    default:             AppTest_RuntimeStop(); break;
    }
}

#endif /* TEST_SELECT == TEST_NONE */
