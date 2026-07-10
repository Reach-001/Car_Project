#include "system_state_pool.h"

/* ────────────────────────────────────────────────────────────
 * 系统状态池实现
 *
 * Init：全部归零/无效，mode=SYS_MODE_STOP，进入安全状态
 * ClearCycleEvents：清零所有一次性 event 标志
 * ──────────────────────────────────────────────────────────── */

void SystemStatePool_Init(SystemStatePool *pool)
{
    if (pool == 0) return;

    pool->mode        = SYS_MODE_STOP;
    pool->target.speed_mps       = 0.0f;
    pool->target.steer_angle_rad = 0.0f;
    pool->target.valid           = false;
    pool->estimation.left_speed_mps       = 0.0f;
    pool->estimation.right_speed_mps      = 0.0f;
    pool->estimation.body_speed_mps       = 0.0f;
    pool->estimation.left_encoder_delta   = 0;
    pool->estimation.right_encoder_delta  = 0;
    pool->estimation.valid                = false;
    pool->motion.target_speed_mps         = 0.0f;
    pool->motion.target_steer_rad         = 0.0f;
    pool->motion.left_target_mps          = 0.0f;
    pool->motion.right_target_mps         = 0.0f;
    pool->motion.left_pwm                 = 0;
    pool->motion.right_pwm                = 0;
    pool->motion.servo_pulse_us           = 1500U;
    pool->motion.limited                  = false;
    pool->sensor.track_bits              = 0U;
    pool->sensor.track_error             = 0;
    pool->sensor.track_valid             = false;
    pool->sensor.ultrasonic_mm           = 0U;
    pool->sensor.ultrasonic_valid        = false;
    pool->sensor.obstacle_near           = false;
    pool->event.bt_command_ready         = false;
    pool->event.k230_result_ready        = false;
    pool->event.key_stop_clicked         = false;
    pool->event.key_mode_clicked         = false;
    pool->event.key_task_clicked         = false;
    pool->event.key_user_clicked         = false;
    pool->event.task_start_requested     = false;
    pool->event.task_pause_requested     = false;
    pool->fault.heartbeat_lost           = false;
    pool->fault.obstacle_too_close       = false;
    pool->fault.sensor_invalid           = false;
    pool->fault.motor_stall              = false;
    pool->fault.servo_limit              = false;
    pool->fault.emergency_stop           = false;
    pool->comm.bt_command.type            = BT_COMMAND_NONE;
    pool->comm.bt_command.valid           = false;
    pool->comm.k230_result.type           = K230_RESULT_NONE;
    pool->comm.k230_result.valid          = false;
    pool->tick_ms                        = 0U;
}

void SystemStatePool_ClearCycleEvents(SystemStatePool *pool)
{
    if (pool == 0) return;

    pool->event.bt_command_ready    = false;
    pool->event.k230_result_ready   = false;
    pool->event.key_stop_clicked    = false;
    pool->event.key_mode_clicked    = false;
    pool->event.key_task_clicked    = false;
    pool->event.key_user_clicked    = false;
    pool->event.task_start_requested = false;
    pool->event.task_pause_requested = false;
}
