/* ────────────────────────────────────────────────────────────
 * 阿克曼转向几何分配（Motion 域内部使用）
 *
 * 输入：车身速度 speed_mps（m/s）、前轮转角 angle_rad（rad）
 * 输出：左右轮目标速度 L_out / R_out（m/s）
 *
 * 简化模型（低速近似）：
 *   R_turn = wheelbase / tan(angle)
 *   L_speed = speed × (1 - track/(2×R_turn))
 *   R_speed = speed × (1 + track/(2×R_turn))
 *
 * 当 angle ≈ 0 时，tan≈0 → 直线行驶，L=R=speed。
 *
 * 标定参数在下层 static，换机械结构只改这里。
 * ──────────────────────────────────────────────────────────── */

#include "motion_internal.h"

#include <math.h>

static const float WHEELBASE_M = 0.18f;   /* 轴距（前后轮轴心距） m */
static const float TRACK_M    = 0.14f;    /* 轮距（左右轮间距）   m */

void Ackermann_Compute(float speed, float angle,
                       float *L_out, float *R_out)
{
    if (L_out == 0 || R_out == 0) return;

    /* 转角接近 0 → 直行 */
    float abs_angle = (angle < 0.0f) ? -angle : angle;
    if (abs_angle < 0.001f)
    {
        *L_out = speed;
        *R_out = speed;
        return;
    }

    /* R_turn = wheelbase / tan(angle) */
    float tan_a  = tanf(angle);
    if (tan_a == 0.0f) { *L_out = speed; *R_out = speed; return; }

    float R_turn = WHEELBASE_M / tan_a;

    /* 左右轮速度 */
    float ratio = TRACK_M / (2.0f * R_turn);
    *L_out = speed * (1.0f - ratio);
    *R_out = speed * (1.0f + ratio);
}
