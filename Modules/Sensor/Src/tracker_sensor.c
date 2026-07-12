/* ────────────────────────────────────────────────────────────
 * 循迹传感器子模块（Sensor 域内部使用，不对外暴露头文件）
 *
 * 五路循迹探头（按车体前进方向看，从左到右为 TRACK_1~5）。
 * 实车模块高电平=白底，低电平=黑线。
 * 偏差 error = 加权平均数 ×1000（纯整数，方便 Decision 直接拿）。
 *
 *   error 为负 → 线偏左
 *   error 为正 → 线偏右
 *   Decision 根据实车安装方向决定是否取反。
 *
 * 参数说明：
 *   VEHICLE_TRACK_BLACK_LEVEL_HIGH = 黑线对应的 GPIO 电平。换传感模块时只改配置。
 *   VEHICLE_TRACK_WEIGHT_x = 五路权重。改动时注意对称且中央为 0。
 * ──────────────────────────────────────────────────────────── */

#include "bsp_gpio_sensor.h"
#include "sensor_internal.h"
#include "system_state_pool.h"
#include "vehicle_config.h"

/* 五路循迹权重从参数表取值，避免标定时改模块逻辑。 */
static const int16_t weights[5] = {
    VEHICLE_TRACK_WEIGHT_1,
    VEHICLE_TRACK_WEIGHT_2,
    VEHICLE_TRACK_WEIGHT_3,
    VEHICLE_TRACK_WEIGHT_4,
    VEHICLE_TRACK_WEIGHT_5
};

static uint8_t  s_bits;            /* 当前五路位图 bit0~4             */
static int16_t  s_error;           /* 循迹偏差（加权平均 ×1000）     */
static bool     s_valid;           /* 数据有效标志                    */

/* ── 10ms 周期读取 ── */

void Tracker_Task10ms(void)
{
    BspTrackRaw raw = BspGpioSensor_ReadTrack();

    int32_t weighted_sum = 0;
    uint8_t active_count = 0U;
    uint8_t bits         = 0U;

    for (uint32_t i = 0U; i < 5U; ++i)
    {
        bool active = (VEHICLE_TRACK_BLACK_LEVEL_HIGH != 0) ? raw.sensor[i] : !raw.sensor[i];
        if (active)
        {
            bits |= (uint8_t)(1U << i);
            weighted_sum += weights[i];
            ++active_count;
        }
    }

    s_bits  = bits;
    s_valid = active_count > 0U;

    if (active_count > 0U)
        s_error = (int16_t)(weighted_sum / active_count);    /* 加权平均，整数 */
    else
        s_error = 0;                                          /* 全灭=丢线    */
}

/* ── 写入状态池 ── */

void Tracker_WriteToPool(SystemStatePool *pool)
{
    if (pool == 0) return;
    pool->sensor.track_bits  = s_bits;
    pool->sensor.track_error = s_error;
    pool->sensor.track_valid = s_valid;
}
