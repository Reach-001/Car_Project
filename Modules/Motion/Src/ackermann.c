/* ────────────────────────────────────────────────────────────
 * 阿克曼转向几何分配（Motion 域内部使用）
 *
 * 输入：车身速度 speed_mps（m/s）、前轮转角 angle_rad（rad）
 * 输出：左右轮目标速度 L_out / R_out（m/s）
 *
 * 简化单轨模型（低速近似）：
 *   项目转向约定：angle < 0 左转，angle > 0 右转。
 *   转弯半径 R = wheelbase / tan(angle)
 *   左轮速度 = speed × (1 - track/2R)
 *   右轮速度 = speed × (1 + track/2R)
 * 角度 = 0 → 直线行驶，左右等速。
 *
 * 参数说明：
 *   VEHICLE_WHEELBASE_M           = 轴距
 *   VEHICLE_TRACK_WIDTH_M         = 轮距
 *   VEHICLE_ACKERMANN_RATIO_LIMIT = 左右轮速比例差限幅
 *
 * 换车/换结构 → 只改 vehicle_config.h 的车辆几何参数。
 * ──────────────────────────────────────────────────────────── */

#include "motion_internal.h"

#include "vehicle_config.h"

#include <math.h>

static float clampf(float value, float min_value, float max_value)
{
    if (value < min_value) return min_value;
    if (value > max_value) return max_value;
    return value;
}

void Ackermann_Compute(float speed, float angle,
                       float *L_out, float *R_out)
{
    if (L_out == 0 || R_out == 0) return;

    /* 转角极小 → 近似直行，避免除零 */
    float abs_angle = (angle < 0.0f) ? -angle : angle;
    if (abs_angle < 0.001f)
    {
        *L_out = speed;
        *R_out = speed;
        return;
    }

    float tan_a  = tanf(angle);
    if (tan_a == 0.0f) { *L_out = speed; *R_out = speed; return; }

    float R_turn = VEHICLE_WHEELBASE_M / tan_a;

    /* 几何公式默认正角度为左转；本项目协议约定负角度为左转。
     * 因此这里取反，让左转时左轮为内侧轮、右转时右轮为内侧轮。 */
    float ratio = -VEHICLE_TRACK_WIDTH_M / (2.0f * R_turn);
    ratio = clampf(ratio,
                   -VEHICLE_ACKERMANN_RATIO_LIMIT,
                   VEHICLE_ACKERMANN_RATIO_LIMIT);
    *L_out = speed * (1.0f - ratio);
    *R_out = speed * (1.0f + ratio);
}
