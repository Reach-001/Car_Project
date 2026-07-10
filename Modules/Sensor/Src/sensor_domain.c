#include "sensor_domain.h"
#include "sensor_internal.h"

/* ────────────────────────────────────────────────────────────
 * Sensor 域聚合入口
 *
 * 内部调用 tracker_sensor 和 ultrasonic_sensor 子模块，
 * 将结果统一写入 pool->sensor。
 * ──────────────────────────────────────────────────────────── */

static SensorState s_state;

/* ── 初始化 ── */

void Sensor_Init(void)
{
    s_state.track_bits       = 0U;
    s_state.track_error      = 0;
    s_state.track_valid      = false;
    s_state.ultrasonic_mm    = 0U;
    s_state.ultrasonic_valid = false;
    s_state.obstacle_near    = false;
}

/* ── 20ms 周期任务 ── */

void Sensor_Task20ms(SystemStatePool *pool)
{
    if (pool == 0) return;

    /* 调用各子模块的周期更新 */
    Tracker_Task10ms();
    Ultrasonic_Task10ms();

    /* 各子模块将结果写入 pool */
    Tracker_WriteToPool(pool);
    Ultrasonic_WriteToPool(pool);

    /* 传感器故障检测：track 和 ultrasonic 同时无效 → 故障 */
    pool->fault.sensor_invalid = !pool->sensor.track_valid && !pool->sensor.ultrasonic_valid;

    /* 更新本地快照 */
    s_state.track_bits       = pool->sensor.track_bits;
    s_state.track_error      = pool->sensor.track_error;
    s_state.track_valid      = pool->sensor.track_valid;
    s_state.ultrasonic_mm    = pool->sensor.ultrasonic_mm;
    s_state.ultrasonic_valid = pool->sensor.ultrasonic_valid;
    s_state.obstacle_near    = pool->sensor.obstacle_near;
}

/* ── 状态查询 ── */

SensorState Sensor_GetState(void)
{
    return s_state;
}
