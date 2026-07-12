#ifndef VEHICLE_CONFIG_H
#define VEHICLE_CONFIG_H

/* ────────────────────────────────────────────────────────────
 * 车辆级参数汇总 —— 所有标定常数集中在此文件
 *
 * 分类：
 *   1. 运动限幅  — 车身最大速度/转角
 *   2. 舵机标定  — 中位脉宽、行程范围、硬限幅
 *   3. 编码器    — 轮径、线数、减速比、方向修正
 *   4. 速度环    — 前馈+PI 参数、死区、积 分开关
 *   5. 其他      — 调试步进、通用换算
 *
 * 换机械结构 / 换电机 / 换舵机 → 改本文件即可，不动代码逻辑。
 * ──────────────────────────────────────────────────────────── */

/* ══════════════════════════════════════════════════════════════
 * 第 1 类：运动限幅（Decision + Motion 共用）
 * ══════════════════════════════════════════════════════════════ */

/* 度 → 弧度转换常数，1° = 0.01745329252 rad */
#define VEHICLE_DEG_TO_RAD              0.01745329252f

/* 车身最大允许线速度（m/s），前进和后退对称。实车先设 1.0，稳了再逐步提高 */
#define VEHICLE_MAX_SPEED_MPS           1.50f

/* 自动任务启动延时（ms）。切到非手动任务后先等待，避免模式切换瞬间小车立刻动作。 */
#define VEHICLE_AUTO_START_DELAY_MS     1000U

/* K230 限速指令的最低运行速度（m/s）。V:10 用于限速通行，不复用找线速度。 */
#define VEHICLE_K230_MIN_SPEED_MPS      0.20f

/* 前轮三点标定（单位：度）
 *   CENTER = 居中修正量。0° 对应舵机实际约 90°，改为 1° 表示在 90° 基础上加 1° 修正。
 *   LEFT   = 相对居中位置的最左机械角
 *   RIGHT  = 相对居中位置的最右机械角
 *
 * 蓝牙输入角度线性映射：
 *   蓝牙发  0° → 轮子居中
 *   蓝牙发 +INPUT_MAX° → 轮子打 RIGHT_MAX_DEG
 *   蓝牙发 -INPUT_MAX° → 轮子打 LEFT_MAX_DEG
 * 超出 INPUT_MAX 的角度被 clamp 到机械极限，不会顶死。
 *
 * 舵机实际输出角 = CENTER + 目标转向角。 */
#define VEHICLE_STEER_CENTER_DEG        0.9f           /* 居中修正量（度）     */
#define VEHICLE_STEER_LEFT_MAX_DEG     -30.0f          /* 相对居中的最左角     */
#define VEHICLE_STEER_RIGHT_MAX_DEG     34.0f          /* 相对居中的最右角     */

/* 蓝牙输入角度最大值（度）。与机械极限一致，保证线性映射到边界 */
#define VEHICLE_MANUAL_INPUT_MAX_DEG    45.0f           /* 蓝牙输入±45° → 轮子打满 */

/* 由相对居中角导出的控制限幅（弧度）。CENTER 只修正舵机输出，不改变控制命令范围。 */
#define VEHICLE_STEER_LEFT_CMD_LIMIT_RAD   ((0.0f - VEHICLE_STEER_LEFT_MAX_DEG) * VEHICLE_DEG_TO_RAD)
#define VEHICLE_STEER_RIGHT_CMD_LIMIT_RAD  (VEHICLE_STEER_RIGHT_MAX_DEG * VEHICLE_DEG_TO_RAD)

/* ══════════════════════════════════════════════════════════════
 * 第 2 类：车辆几何 / 阿克曼模型（Motion 域使用）
 * ══════════════════════════════════════════════════════════════ */

/* 轴距：前轮转向轴到后轮轴心的距离（m）。实测 14cm。 */
#define VEHICLE_WHEELBASE_M             0.14f

/* 轮距：左右驱动轮接地点中心距（m）。实测 12cm。 */
#define VEHICLE_TRACK_WIDTH_M           0.12f

/* 阿克曼模型允许的最大左右轮速比例差。
 * 例如 0.60 表示内外轮速度最多相差 ±60%，防止大角度时内侧轮目标速度过小或反向。 */
#define VEHICLE_ACKERMANN_RATIO_LIMIT   0.60f

/* 转弯内轮最小目标速度（m/s）。低于该值时实车内轮容易卡在静摩擦区。 */
#define VEHICLE_ACKERMANN_INNER_MIN_SPEED_MPS 0.25f

/* ══════════════════════════════════════════════════════════════
 * 第 3 类：巡线控制参数（Decision 域使用）
 * ══════════════════════════════════════════════════════════════ */

/* 循迹探头电平极性参数表：
 *   当前实车模块：全高 = 白底，黑线 = 低电平 → 填 0
 *   另一类模块：  白底 = 低电平，黑线 = 高电平 → 填 1
 *
 * 注意：上层 track_bits 始终表示“检测到黑线”，不直接表示 GPIO 原始电平。 */
#define VEHICLE_TRACK_BLACK_LEVEL_HIGH       0

/* 五路循迹权重（从左到右 TRACK_1~5）。
 * 权重表示黑线相对车体中心的位置误差，不是舵机角度；Decision 再用 KP/KD 换算成转向角。 */
#define VEHICLE_TRACK_WEIGHT_1              -1000
#define VEHICLE_TRACK_WEIGHT_2              -300
#define VEHICLE_TRACK_WEIGHT_3                  0
#define VEHICLE_TRACK_WEIGHT_4               300
#define VEHICLE_TRACK_WEIGHT_5               1000

/* 巡线基础速度（m/s）。实车标定阶段先低速，确认不跑出赛道后再逐步提高。 */
#define VEHICLE_LINE_FOLLOW_SPEED_MPS        0.45f

/* 巡线最低运动速度（m/s）。实车低于 0.2m/s 无法稳定克服静摩擦。 */
#define VEHICLE_LINE_FOLLOW_MIN_SPEED_MPS    0.35f

/* 循迹误差到转向角的比例。track_error 范围约 -2000~+2000。
 * STEER_SIGN 用于适配传感器安装方向：方向反了只改 +1/-1。 */
#define VEHICLE_LINE_FOLLOW_STEER_SIGN       1.0f
#define VEHICLE_LINE_FOLLOW_KP               0.0005f
#define VEHICLE_LINE_FOLLOW_KD               0.00005f

/* 中心探头锁线策略：
 *   CENTER_MASK 指向中间探头（TRACK_3）。
 *   中心探头已压线且误差很小时，缩小转向输出，减少左右摆动。
 *   只要中心探头未压线，就进入固定方向找线，直到 TRACK_3 重新检测到黑线。 */
#define VEHICLE_LINE_FOLLOW_CENTER_MASK      (1U << 2)
#define VEHICLE_LINE_FOLLOW_CENTER_DEADBAND  300
#define VEHICLE_LINE_FOLLOW_CENTER_STEER_SCALE 0.35f

/* 误差减速比例。0=不减速，1=最大误差时降到 MIN_SPEED。 */
#define VEHICLE_LINE_FOLLOW_SLOWDOWN_GAIN    0.85f

/* 中心找线处理：
 *   线较窄且探头间距较大时，黑线可能短暂落在两个探头中间，表现为全白。
 *   TRACK_3 未压线时，按锁存方向低速前进；方向不明确时随机选一侧。
 *   如果传感器依次扫过 TRACK_2→TRACK_1 或 TRACK_4→TRACK_5，判定为急转弯，舵机直接打满。
 *   只有 TRACK_3 重新检测到黑线，才退出找线并恢复正常循迹。 */
#define VEHICLE_LINE_FOLLOW_SEARCH_SPEED_MPS 0.25f
#define VEHICLE_LINE_FOLLOW_SEARCH_STEER_DEG 22.0f
#define VEHICLE_LINE_FOLLOW_LOST_MIN_ERROR   250

/* 方向不唯一兜底：
 *   例如从直线中间偏出时，可能出现两个探头对称在线、error=0 且没有历史偏差。
 *   此时随机生成一个等效误差，让舵机先打一边，避免卡在中位不响应。 */
#define VEHICLE_LINE_FOLLOW_AMBIGUOUS_ERROR  600

/* ══════════════════════════════════════════════════════════════
 * 第 4 类：舵机标定（bsp_servo + Motion 共用）
 * ══════════════════════════════════════════════════════════════ */

/* 舵机中位脉宽（us）。默认 1500 对应机械中位；若中位不准，轮子装正后量脉宽改这里 */
#define VEHICLE_SERVO_CENTER_US         1500U          /* 中位脉宽（微秒）       */

/* 舵机半幅行程（us）。-1000‰ = 1000us，+1000‰ = 2000us，中位 1500us。
 * 范围 500 意味着左右各 500us = 1000~2000us 区间。 */
#define VEHICLE_SERVO_RANGE_US          500U           /* 半幅范围（微秒）       */

/* 硬件绝对限幅 —— 即使上层代码算错，脉宽也不会超出此范围，保护舵机不顶死 */
#define VEHICLE_SERVO_MIN_US            (VEHICLE_SERVO_CENTER_US - VEHICLE_SERVO_RANGE_US) /* 最小 1000us */
#define VEHICLE_SERVO_MAX_US            (VEHICLE_SERVO_CENTER_US + VEHICLE_SERVO_RANGE_US) /* 最大 2000us */

/* ══════════════════════════════════════════════════════════════
 * 第 5 类：编码器 / 轮速估计（Estimation 域使用）
 * ══════════════════════════════════════════════════════════════ */

/* 轮径（m）。量一下轮子直径填真实值，影响速度计算 */
#define VEHICLE_WHEEL_DIAMETER_M        0.056f         /* 轮径 56mm */

/* 编码器线数（pulses per revolution）。电机轴上编码器码盘刻线数 */
#define VEHICLE_ENCODER_PPR             13.0f          /* 13 线/转 */

/* 减速比 = 电机端转速 / 轮端转速。实车速比以铭牌为准 */
#define VEHICLE_GEAR_RATIO              20.0f          /* 20:1 减速 */

/* 编码器方向符号：+1 表示前进时读数为正，-1 表示前进时读数为负。
 * 发 MANUAL + 60,0 前进，看 debug 曲线 CH5/CH8 actual 的符号：
 *   actual 为正 → 保持 +1；actual 为负 → 改为 -1 */
#define VEHICLE_LEFT_ENCODER_SIGN        1             /* 左轮方向符号  */
#define VEHICLE_RIGHT_ENCODER_SIGN      -1             /* 右轮方向符号  */

/* 左右编码器速度校准系数。
 * 用途：补偿左右轮编码器码盘、安装位置或轮径差异导致的反馈比例误差。
 * 标定方法：直行命令下如果实车左轮明显更快，但 CH5≈CH8，说明左轮反馈偏小，
 *          适当增大 LEFT_SCALE；反之减小。右轮同理。 */
#define VEHICLE_LEFT_ENCODER_SPEED_SCALE   1.0f        /* 左轮反馈比例 */
#define VEHICLE_RIGHT_ENCODER_SPEED_SCALE  1.0f        /* 右轮反馈比例 */

/* ══════════════════════════════════════════════════════════════
 * 第 6 类：速度环 PI 参数（Motion/ speed_pi.c 使用）
 *
 * 输出 = 前馈 + P 项 + I 项（可选），单位 PWM 千分比（-1000~1000）。
 *
 * 标定顺序（重要）：
 *   ① 先调 FF_MIN_PWM：发 20,0 低速前进，加大此值直到轮子刚好平滑转动
 *   ② 再调 FF_GAIN_PWM_PER_MPS：发 80,0 高速前进，调节使开环速度接近目标
 *   ③ 最后调 KP：发方波命令（交替 60 和 -60），看实际跟踪效果
 *   ④ I 项在 P+前馈稳定后按需打开（VEHICLE_SPEED_PI_I_ENABLE=1）
 * ══════════════════════════════════════════════════════════════ */

/* 比例增益 Kp。           单位：PWM‰ / (m/s)。P 太大 → 振荡；P 太小 → 响应慢 */
#define VEHICLE_SPEED_PI_KP              220.0f

/* 积分增益 Ki。           单位：PWM‰ / (m/s*秒)。默认关闭（I_ENABLE=0） */
#define VEHICLE_SPEED_PI_KI              30.0f

/* 是否启用积分项。        0 = 仅前馈+P；1 = 前馈+P+I。先跑稳 P 再开 I */
#define VEHICLE_SPEED_PI_I_ENABLE        0

/* 积分饱和上限。          防止积分量无限制累加导致超调。正负对称 */
#define VEHICLE_SPEED_PI_I_MAX           80.0f

/* 目标死区（m/s）。       目标速度绝对值 < 此值 → 直接停车（防零速抖动） */
#define VEHICLE_SPEED_PI_TARGET_DEADBAND 0.01f

/* PWM 斜率限制。每 10ms 最大变化量，抑制 PI 输出突跳。
 * 0,0 刹车不受该限制，仍会立即停车。 */
#define VEHICLE_MOTOR_PWM_SLEW_PER_10MS 30

/* 起步助推。目标从 0 进入非零时短时间给较高 PWM，越过静摩擦后回到速度环。
 * 不作为堵转保护使用，只解决低速命令下 PWM 逐步爬升过慢导致的起步失败。 */
#define VEHICLE_MOTOR_START_ACTUAL_MAX_MPS 0.08f
#define VEHICLE_MOTOR_START_BOOST_PWM      900
#define VEHICLE_MOTOR_START_BOOST_MS       160U

/* 前馈最小启动 PWM。      克服静摩擦所需的最小 PWM 占空比。
 * 标定方法：手动发逐步加大 PWM 直到轮子刚好平滑转动，记录该 PWM 值。 */
#define VEHICLE_SPEED_PI_FF_MIN_PWM      560.0f

/* 前馈速度斜率。           每增加 1m/s 目标速度，前馈 PWM 增加多少。
 * 标定方法：开环跑不同速度（关 KP、关 KI），记录 PWM 和实际速度，拟合斜率。 */
#define VEHICLE_SPEED_PI_FF_GAIN_PWM_PER_MPS 140.0f

/* ══════════════════════════════════════════════════════════════
 * 第 7 类：调试步进
 * ══════════════════════════════════════════════════════════════ */

/* 调试模式下每次按键步进的目标速度增量（m/s） */
#define VEHICLE_DEBUG_SPEED_STEP_MPS     0.10f

/* 开环最大速度测试参数。
 * 作用：把目标速度按比例映射到 PWM 千分比，0m/s -> 0，最大速度 -> 1000。
 * 用途：测车辆纯驱动的最高速度，不经过速度 PI 闭环。 */
#define VEHICLE_OPEN_LOOP_TEST_MAX_PWM_PERMILLE  1000.0f
#define VEHICLE_OPEN_LOOP_TEST_MAX_SPEED_MPS     VEHICLE_MAX_SPEED_MPS

#endif /* VEHICLE_CONFIG_H */
