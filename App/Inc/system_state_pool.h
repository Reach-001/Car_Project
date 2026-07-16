#ifndef SYSTEM_STATE_POOL_H
#define SYSTEM_STATE_POOL_H

#include <stdbool.h>
#include <stdint.h>

/* ────────────────────────────────────────────────────────────
 * 系统状态池 —— 唯一跨域数据交换中心
 *
 * 位于 App 层。所有模块域通过 pool 指针读写数据，模块之间不直
 * 接调用对方函数。
 *
 * 写入规则（各域只能写自己负责的字段）：
 *   Sensor     → pool->sensor / pool->fault.sensor_invalid
 *   Estimation → pool->estimation
 *   Comm       → pool->comm / pool->event.bt_command_ready / k230_result_ready
 *   HMI        → pool->event.key_* + 控制 BSP LED/蜂鸣器
 *   Decision   → pool->mode / pool->target（唯一能写 target 的域）
 *   Motion     → pool->motion / pool->fault.servo_limit
 *   Debug      → pool->debug
 *
 * event 是一次性的，本周期消费者消费后调用 ClearCycleEvents 清除。
 * fault 是持久的，由各域置位，Decision 决定如何处理。
 * ──────────────────────────────────────────────────────────── */

/* ── 系统模式 ── */

typedef enum
{
    SYS_MODE_STOP = 0,       /* 停车                */
    SYS_MODE_MANUAL,         /* 蓝牙手动遥控         */
    SYS_MODE_LINE_FOLLOW,    /* 循迹                 */
    SYS_MODE_INSPECTION,     /* 自动巡检             */
    SYS_MODE_AVOIDANCE,      /* 避障                 */
    SYS_MODE_ERROR           /* 故障                 */
} SystemMode;

/* ── 自动任务 ── */

typedef enum
{
    AUTO_TASK_NONE = 0,             /* 无自动任务           */
    AUTO_TASK_LINE_FOLLOW = 1,      /* 巡线                 */
    AUTO_TASK_LINE_FOLLOW_K230,     /* 巡线 + K230 指令控制   */
    AUTO_TASK_INSPECTION            /* 巡检预留             */
} SystemAutoTask;

/* ── 目标值（Decision 写 → Motion 读） ── */

typedef struct
{
    float speed_mps;         /* 目标线速度（m/s），正=前进       */
    float steer_angle_rad;   /* 目标前轮转角（rad），正=右转     */
    bool  valid;             /* 目标值是否有效                  */
} SystemTarget;

/* ── 速度估计（Estimation 写 → Motion/Decision 读） ── */

typedef struct
{
    float   left_speed_mps;       /* 左轮速度 m/s               */
    float   right_speed_mps;      /* 右轮速度 m/s               */
    float   body_speed_mps;       /* 车身中心速度 m/s           */
    int32_t left_encoder_delta;   /* 左编码器原始增量（调试用） */
    int32_t right_encoder_delta;  /* 右编码器原始增量（调试用） */
    bool    valid;                /* 估计值是否有效             */
} SystemEstimation;

/* ── 运动执行状态（Motion 写 → Debug/Decision 读） ── */

typedef struct
{
    float    target_speed_mps;  /* 限幅后的车身目标速度 m/s    */
    float    target_steer_rad;  /* 限幅后的前轮目标转角 rad     */
    float    left_target_mps;   /* 左轮目标速度 m/s             */
    float    right_target_mps;  /* 右轮目标速度 m/s             */
    int16_t  left_pwm;          /* 左轮 PWM 千分比              */
    int16_t  right_pwm;         /* 右轮 PWM 千分比              */
    uint16_t servo_pulse_us;    /* 舵机脉宽 us                  */
    bool     limited;           /* 输出是否触发限幅             */
} SystemMotion;

/* ── 传感器（Sensor 写 → Decision 读） ── */

typedef struct
{
    uint8_t  track_bits;     /* 五路循迹位图，bit0~4          */
    int16_t  track_error;    /* 循迹偏差（加权平均数 ×1000）   */
    bool     track_valid;    /* 循迹数据有效                   */
    uint16_t ultrasonic_mm;  /* 超声波距离 mm                  */
    bool     ultrasonic_valid; /* 超声波数据有效               */
    bool     obstacle_near;  /* 障碍物是否过近                 */
} SystemSensor;

/* ── 通信数据暂存（Comm 写 → Decision 读） ── */

/* 前向声明，避免模块头文件循环依赖 */
typedef enum
{
    BT_COMMAND_NONE = 0,
    BT_COMMAND_STOP,
    BT_COMMAND_SET_MODE,
    BT_COMMAND_MANUAL_MOVE,
    BT_COMMAND_START_TASK,
    BT_COMMAND_PAUSE_TASK,
    BT_COMMAND_DEBUG_OUTPUT,
    BT_COMMAND_START_TEST,
    BT_COMMAND_CUSTOM
} BtCommandType;

typedef struct
{
    BtCommandType type;
    int16_t       arg0;      /* 手动 speed(-100~100) 或 mode */
    int16_t       arg1;      /* 手动 angle_deg，轮子相对中位角度 */
    int16_t       arg2;      /* 参数2                      */
    uint32_t      timestamp_ms;
    bool          valid;
} BtCommand;

typedef enum
{
    K230_RESULT_NONE = 0,
    K230_RESULT_LINE,
    K230_RESULT_TARGET,
    K230_RESULT_MARKER,
    K230_RESULT_CUSTOM,
    K230_RESULT_CMD_STOP,       /* S:n，停车 n 秒              */
    K230_RESULT_CMD_SPEED,      /* V:n，视觉限速/恢复速度       */
    K230_RESULT_CMD_PARK,       /* P:n，1=倒车入库，2=侧方停车  */
    K230_RESULT_CMD_TURN        /* L/R:n，value0=-1左/+1右，value1=角度 */
} K230ResultType;

typedef struct
{
    K230ResultType type;
    int16_t        value0;
    int16_t        value1;
    int16_t        value2;
    uint32_t       timestamp_ms;
    bool           valid;
} K230Result;

typedef struct
{
    BtCommand  bt_command;       /* 普通控制命令：Decision 消费        */
    BtCommand  debug_command;    /* 最近一次调试命令记录              */
    K230Result k230_result;
} CommData;

/* ── 调试输出控制（Debug 域写入） ── */

typedef struct
{
    bool enabled;             /* true 时输出曲线帧 */
} DebugControl;

/* ── 一次性事件（HMI/Comm 写 → Decision 读后清除） ── */

typedef struct
{
    bool bt_command_ready;       /* 蓝牙有新命令              */
    bool k230_result_ready;      /* K230 有新识别结果         */
    bool key_stop_clicked;       /* KEY1 单击（紧急停车）     */
    bool key_mode_clicked;       /* KEY2 单击（模式切换）     */
    bool key_task_clicked;       /* KEY3 单击（任务启停）     */
    bool key_user_clicked;       /* User_Key 短按             */
    bool key_user_long_pressed;  /* User_Key 长按             */
    bool task_start_requested;   /* 蓝牙请求启动任务          */
    bool task_pause_requested;   /* 蓝牙请求暂停任务          */
} SystemEvent;

/* ── 故障标志（各域写 → Decision 读） ── */

typedef struct
{
    bool heartbeat_lost;         /* 通信心跳丢失              */
    bool obstacle_too_close;     /* 障碍物过近（紧急）        */
    bool sensor_invalid;         /* 传感器数据无效            */
    bool servo_limit;            /* 舵机限幅触发              */
    bool emergency_stop;         /* 急停请求                  */
} SystemFault;

/* ── 顶层状态池 ── */

typedef struct
{
    SystemMode       mode;        /* 当前系统模式              */
    SystemAutoTask   auto_task;   /* 当前自动任务              */
    SystemTarget     target;      /* 运动目标值               */
    SystemEstimation estimation;  /* 速度估计值               */
    SystemMotion     motion;      /* 运动执行状态             */
    SystemSensor    sensor;      /* 传感器物理量             */
    SystemEvent     event;       /* 一次性事件               */
    SystemFault     fault;       /* 故障标志                 */
    CommData        comm;        /* 通信数据暂存             */
    DebugControl    debug;       /* 调试输出控制             */
    uint32_t        tick_ms;     /* 当前系统时间（由 App 维护） */
} SystemStatePool;

/* ── 接口 ── */

/** 初始化状态池为安全默认值 */
void SystemStatePool_Init(SystemStatePool *pool);

/** 清除一次性事件标志（Decision 消费后调用） */
void SystemStatePool_ClearCycleEvents(SystemStatePool *pool);

#endif /* SYSTEM_STATE_POOL_H */
