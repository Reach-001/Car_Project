/* ────────────────────────────────────────────────────────────
 * 循迹传感器子模块（Sensor 域内部使用，不对外暴露头文件）
 *
 * 五路循迹探头（TRACK_1~5），黑线=高电平。
 * 偏差 error = 加权平均数 ×1000（纯整数，方便 Decision 直接拿）。
 *
 * 权重表：
 *   最左传感器(TRACK_1) 权重 = -2000
 *   左中   (TRACK_2)    权重 = -1000
 *   中央   (TRACK_3)    权重 = 0
 *   右中   (TRACK_4)    权重 = +1000
 *   最右   (TRACK_5)    权重 = +2000
 *
 *   error 为负 → 线偏左
 *   error 为正 → 线偏右
 *   Decision 根据实车安装方向决定是否取反。
 *
 * 参数说明：
 *   s_active_high = true（黑线高电平）。换传感模块时按需翻转。
 *   weights[] = 五路权重数组。改动时注意对称且中央为 0。
 * ──────────────────────────────────────────────────────────── */

#include "bsp_gpio_sensor.h"
#include "sensor_internal.h"
#include "system_state_pool.h"

/* 五路临界探头权重：中央=0，对称向两侧递增 */
static const int16_t weights[5] = {-2000, -1000, 0, 1000, 2000};

static uint8_t  s_bits;            /* 当前五路位图 bit0~4             */
static int16_t  s_error;           /* 循迹偏差（加权平均 ×1000）     */
static bool     s_valid;           /* 数据有效标志                    */
static bool     s_active_high = true; /* 黑线=高电平                 */

/* ── 10ms 周期读取 ── */

void Tracker_Task10ms(void)
{
    BspTrackRaw raw = BspGpioSensor_ReadTrack();

    int32_t weighted_sum = 0;
    uint8_t active_count = 0U;
    uint8_t bits         = 0U;

    for (uint32_t i = 0U; i < 5U; ++i)
    {
        bool active = s_active_high ? raw.sensor[i] : !raw.sensor[i];
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
