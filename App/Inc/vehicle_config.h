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
#define VEHICLE_MAX_SPEED_MPS           2.0f

/* 前轮三点标定（单位：度）
 *   CENTER = 轮子居中时的机械角
 *   LEFT   = 最左机械角
 *   RIGHT  = 最右机械角
 *
 * 蓝牙输入角度线性映射：
 *   蓝牙发  0° → 轮子居中
 *   蓝牙发 +INPUT_MAX° → 轮子打 RIGHT_MAX_DEG
 *   蓝牙发 -INPUT_MAX° → 轮子打 LEFT_MAX_DEG
 * 超出 INPUT_MAX 的角度被 clamp 到机械极限，不会顶死。
 *
 * 改 LEFT/RIGHT_MAX_DEG 三个数即可完成转向标定。 */
#define VEHICLE_STEER_CENTER_DEG        0.0f           /* 居中角（度）        */
#define VEHICLE_STEER_LEFT_MAX_DEG     -28.0f          /* 最左机械角（度）     */
#define VEHICLE_STEER_RIGHT_MAX_DEG     28.0f          /* 最右机械角（度）     */

/* 蓝牙输入角度最大值（度）。与机械极限一致，保证线性映射到边界 */
#define VEHICLE_MANUAL_INPUT_MAX_DEG    45.0f           /* 蓝牙输入±45° → 轮子打满 */

/* 由标定导出的控制限幅（弧度） */
#define VEHICLE_STEER_LEFT_CMD_LIMIT_RAD   ((VEHICLE_STEER_CENTER_DEG - VEHICLE_STEER_LEFT_MAX_DEG) * VEHICLE_DEG_TO_RAD)
#define VEHICLE_STEER_RIGHT_CMD_LIMIT_RAD  ((VEHICLE_STEER_RIGHT_MAX_DEG - VEHICLE_STEER_CENTER_DEG) * VEHICLE_DEG_TO_RAD)

/* ══════════════════════════════════════════════════════════════
 * 第 2 类：车辆几何 / 阿克曼模型（Motion 域使用）
 * ══════════════════════════════════════════════════════════════ */

/* 轴距：前轮转向轴到后轮轴心的距离（m）。需要量实车。 */
#define VEHICLE_WHEELBASE_M             0.18f

/* 轮距：左右驱动轮接地点中心距（m）。需要量实车。 */
#define VEHICLE_TRACK_WIDTH_M           0.14f

/* 阿克曼模型允许的最大左右轮速比例差。
 * 例如 0.60 表示内外轮速度最多相差 ±60%，防止大角度时内侧轮目标速度过小或反向。 */
#define VEHICLE_ACKERMANN_RATIO_LIMIT   0.60f

/* ══════════════════════════════════════════════════════════════
 * 第 3 类：巡线控制参数（Decision 域使用）
 * ══════════════════════════════════════════════════════════════ */

/* 巡线模式固定速度（m/s）。第一版从低速开始，确认稳定后再提高 */
#define VEHICLE_LINE_FOLLOW_SPEED_MPS    0.30f

/* 循迹误差到转向角的比例。track_error 范围约 -2000~+2000。
 * 负误差 → 左转，正误差 → 右转。 */
#define VEHICLE_LINE_FOLLOW_KP           0.0005f

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
#define VEHICLE_SPEED_PI_KP              280.0f

/* 积分增益 Ki。           单位：PWM‰ / (m/s*秒)。默认关闭（I_ENABLE=0） */
#define VEHICLE_SPEED_PI_KI              30.0f

/* 是否启用积分项。        0 = 仅前馈+P；1 = 前馈+P+I。先跑稳 P 再开 I */
#define VEHICLE_SPEED_PI_I_ENABLE        1

/* 积分饱和上限。          防止积分量无限制累加导致超调。正负对称 */
#define VEHICLE_SPEED_PI_I_MAX           80.0f

/* 目标死区（m/s）。       目标速度绝对值 < 此值 → 直接停车（防零速抖动） */
#define VEHICLE_SPEED_PI_TARGET_DEADBAND 0.01f

/* 前馈最小启动 PWM。      克服静摩擦所需的最小 PWM 占空比。
 * 标定方法：手动发逐步加大 PWM 直到轮子刚好平滑转动，记录该 PWM 值。 */
#define VEHICLE_SPEED_PI_FF_MIN_PWM      480.0f

/* 前馈速度斜率。           每增加 1m/s 目标速度，前馈 PWM 增加多少。
 * 标定方法：开环跑不同速度（关 KP、关 KI），记录 PWM 和实际速度，拟合斜率。 */
#define VEHICLE_SPEED_PI_FF_GAIN_PWM_PER_MPS 100.0f

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
