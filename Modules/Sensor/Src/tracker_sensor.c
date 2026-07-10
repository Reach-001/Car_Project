/* ────────────────────────────────────────────────────────────
 * 循迹传感器子模块（Sensor 域内部使用，不对外暴露头文件）
 *
 * 迁移自旧 Modules/Src/tracker.c
 * 依赖：BSP/bsp_gpio_sensor.h
 *
 * 五路循迹探头（TRACK_1~5），黑线=高电平。
 * 偏差 error 是加权平均数 ×1000，供循迹 PID 使用。
 * ──────────────────────────────────────────────────────────── */

#include "bsp_gpio_sensor.h"
#include "sensor_internal.h"
#include "system_state_pool.h"

static uint8_t s_bits;
static int16_t s_error;
static bool    s_valid;
static bool    s_active_high = true;   /* 黑线 = 高电平 */

/* ── 内部函数 ── */

void Tracker_Task10ms(void)
{
    static const int16_t weights[5] = {-2000, -1000, 0, 1000, 2000};

    BspTrackRaw raw = BspGpioSensor_ReadTrack();

    int32_t  weighted_sum = 0;
    uint8_t  active_count = 0U;
    uint8_t  bits         = 0U;

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
    {
        s_error = (int16_t)(weighted_sum / active_count);
    }
    else
    {
        s_error = 0;
    }
}

void Tracker_WriteToPool(SystemStatePool *pool)
{
    if (pool == 0) return;

    pool->sensor.track_bits  = s_bits;
    pool->sensor.track_error = s_error;
    pool->sensor.track_valid = s_valid;
}
