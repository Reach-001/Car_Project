/* ────────────────────────────────────────────────────────────
 * 电机 PWM 驱动实现
 *
 * 左电机：PA6(TIM3_CH1)=IN1 反转 / PA7(TIM3_CH2)=IN2 正转
 * 右电机：PB0(TIM3_CH3)=IN1 反转 / PB1(TIM3_CH4)=IN2 正转
 *
 * TIM3 PWM 参数（CubeMX IOC 配置）：
 *   时钟源    = APB1 × 2（TIM 时钟分频）= 170MHz
 *   Prescaler = 16
 *   Period    = 499
 *   频率      = 170MHz / (16+1) / (499+1) = 20kHz
 *   分辨率    = 500 级（0~500 对应 0%~100% 占空比）
 *
 * AT8236 控制逻辑（快衰减模式）：
 *   IN1=PWM, IN2=0  → 正转
 *   IN1=0, IN2=PWM  → 反转
 *   IN1=0, IN2=0    → 休眠（自由滑行）
 *   IN1=PWM, IN2=PWM → 刹车（能耗制动）
 *
 * CH1/CH3 接 IN1, CH2/CH4 接 IN2。
 * 正转 PWM 输出到 IN2（CH2/CH4），反转 PWM 输出到 IN1（CH1/CH3）。
 *
 * 参数说明：
 *   MOTOR_DUTY_MAX     = PWM 千分比上限。1000 = 100% 占空比
 *   MOTOR_TIM_PERIOD   = TIM3 ARR 值（CubeMX 配置）。改 Pre/Period 后同步更新
 *   MOTOR_TIM_COUNTS   = ARR + 1 = 500 级分辨率
 *                         CCR = duty_permille × COUNTS / DUTY_MAX
 *                         duty=500 → CCR=500×500/1000=250 → 50% 占空比
 * ──────────────────────────────────────────────────────────── */

#include "bsp_motor.h"

#include "tim.h"     /* htim3, TIM_CHANNEL_x 宏 */

/* ════════════════════════════════════════════════════════════
 * PWM 参数
 * ════════════════════════════════════════════════════════════ */

#define MOTOR_DUTY_MAX   1000            /* PWM 千分比上限（100% 占空比）               */
#define MOTOR_TIM_PERIOD 499U            /* TIM3 ARR 值（CubeMX: Presc=16, Period=499） */
#define MOTOR_TIM_COUNTS (MOTOR_TIM_PERIOD + 1U)  /* 等效计数单位 = 500                */

/* 千分比 duty → CCR 比较值 */
static uint32_t duty_to_compare(int16_t duty_permille)
{
    int32_t duty = duty_permille;
    if (duty < 0)                 { duty = -duty; }
    if (duty > MOTOR_DUTY_MAX)    { duty = MOTOR_DUTY_MAX; }
    return (uint32_t)((duty * (int32_t)MOTOR_TIM_COUNTS) / MOTOR_DUTY_MAX);
}

/* ── 初始化 ── */

void BspMotor_Init(void)
{
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);   /* PA6 = IN1_L = 反转 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);   /* PA7 = IN2_L = 正转 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);   /* PB0 = IN1_R = 反转 */
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);   /* PB1 = IN2_R = 正转 */
    BspMotor_StopAll();
}

/* ── 驱动接口 ── */

void BspMotor_SetDuty(BspMotorId motor, int16_t duty_permille)
{
    uint32_t forward = 0U;                       /* IN2（正转）通道 CCR */
    uint32_t reverse = 0U;                       /* IN1（反转）通道 CCR */
    uint32_t compare = duty_to_compare(duty_permille);

    /* 正数 → IN2 输出 PWM，IN1=0；负数 → IN1 输出 PWM，IN2=0 */
    if (duty_permille >= 0) { forward = compare; }
    else                    { reverse = compare; }

    if (motor == BSP_MOTOR_LEFT)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, reverse);   /* CH1=IN1 反转 */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, forward);   /* CH2=IN2 正转 */
    }
    else  /* BSP_MOTOR_RIGHT */
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, reverse);   /* CH3=IN1 反转 */
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, forward);   /* CH4=IN2 正转 */
    }
}

void BspMotor_StopAll(void)
{
    BspMotor_SetDuty(BSP_MOTOR_LEFT,  0);
    BspMotor_SetDuty(BSP_MOTOR_RIGHT, 0);
}
