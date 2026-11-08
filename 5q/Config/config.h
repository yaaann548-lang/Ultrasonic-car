#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H
/* ISP IRC setting must be 11.0592 MHz. These macros do not set the IRC. */
#define SYSCLK_HZ              11059200UL
#if SYSCLK_HZ != 11059200UL
#error Recalculate ultrasonic conversion before changing clock
#endif
/* 1 = four-channel AO line following, 0 = original ultrasonic mode. */
#define LINE_FOLLOW_ENABLE    1
/* 1 = OLED-guided bench calibration, 0 = fixed values for acceptance. */
#define LINE_CALIBRATE_ENABLE 1
/* 1 = stationary inner sensor test, 0 = normal line mode. */
#define LINE_CENTER_TEST_ENABLE 0
/* 1 = button-driven short moves with ADC readout, 0 = continuous tracking. */
#define LINE_STEP_TEST_ENABLE 0
/* ---- 转向增益: 分两段, 中间线性过渡 ---------------------------------------
 * 输出 = 增益 * error / 100  (单位: PWM 百分点)
 *
 * 直道上 |error| 小 -> 用低增益, 不会画龙;
 * 急弯里 |error| 大 -> 用高增益, 差速打得够猛, 才转得过来。
 *
 * 为什么要分两段: 原来只有一个 KP=3, 于是 |error|=200(单外侧压满黑)时
 * 转向量只有 6 个百分点, 差速 26/12, 曲率半径受限于约 13cm, 比急弯还大,
 * 所以车会直冲出去、红外跑到线外。把增益调高又会毁掉直道手感,
 * 所以做成"小偏差温柔、大偏差凶狠"的曲线。 */
#define LINE_GAIN_LOW        3     /* |error| <= LINE_GAIN_KNEE 时的增益 */
#define LINE_GAIN_HIGH       8     /* |error| >= LINE_ERROR_FULL 时的增益 */
#define LINE_GAIN_KNEE       80    /* 低增益区边界(误差单位, 约 8mm 偏移) */
#define LINE_ERROR_FULL     200    /* 满偏差: 单个外侧探头压满黑线时的 |error|
                                    * (若改了下面的权重, 这里要同步改) */
#define LINE_PID_KP          3     /* 兼容旧名: 等于 LINE_GAIN_LOW, 不生效 */
#define LINE_PID_KI          0     /* 积分: 一般不用, 左右轮速差请在 BASE 里补 */
#define LINE_PID_KD          3     /* 微分(预判): 0=关; 3~6 让转向提前半个拍子 */

/* 转向量硬上限(PWM 百分点)。原来写 18 但永远达不到(要 error=600 才行),
 * 属于死参数; 14 才是真正生效的帽子: 基准 20/18 配 14 -> 满舵 34/4,
 * 内侧基本停转, 等于原地转向, 这是这台车能给出的最大转向能力。 */
#define MAX_TRIM             14

/* ---- 弯道降速 -------------------------------------------------------------
 * 偏差越大, 两侧基准一起降, 给转向留出时间(不改变左右平衡)。
 * 4 = 每 100 误差单位降 4 个百分点; error=150 时基准 20/18 -> 14/12。 */
#define LINE_CURVE_SLOW       4
#define LINE_BASE_MIN_PCT    12    /* 降速下限, 别降到电机死区以下 */
/* Straight-run PWM, tuned independently using the BOTH WHEELS test. */
#define LINE_LEFT_BASE_PWM      20
#define LINE_RIGHT_BASE_PWM     18
/* Set to 0 if TB6612 channel B drives the left wheel. */
#define LINE_MOTOR_A_IS_LEFT  0
#define OLED_ENABLE           1
#define MOTOR_ENABLE          1
#define MOTOR_A_INVERT         1
#define MOTOR_B_INVERT         0
#define MOTOR_PWM_COUNTS       1000U
/* 电机斜坡: 每 MOTOR_STEP_MS 走 MOTOR_STEP_PERCENT 个百分点。
 * 原来 20ms 一步, 从直道到满舵要 3 步 = 60ms, 在 25% 占空比下等于
 * 先白跑 1~2cm 才开始转 —— 急弯来不及的第二个原因。改成 10ms 一步。 */
#define MOTOR_STEP_MS          10U
#define MOTOR_STEP_PERCENT     5
#define MOTOR_REVERSE_WAIT_MS  100U
/* Temporary bench correction; root cause of scale error is unresolved. */
/* Set to 1U to remove correction; 2U halves distance for display/control. */
#define US_DISTANCE_DIVISOR    1U//10.14 ȡ������/2 ԭ2
#if US_DISTANCE_DIVISOR != 1U && US_DISTANCE_DIVISOR != 2U
#error Unsupported temporary distance divisor
#endif
#define SAMPLE_PERIOD_MS       200U
#define DISPLAY_PERIOD_MS      200U
#define STARTUP_WAIT_MS         2000U
#define DIST_TARGET_MM          200U
#define DIST_NEAR_START_MM      150U
#define DIST_FAR_START_MM       250U
#define DIST_MAX_TRACK_MM       500U
/* Rotate in place when no target is found within 500 mm. */
#define SEARCH_PERCENT          15
#define LOST_SAMPLES_TO_SEARCH  2U
#if SEARCH_PERCENT < 1 || SEARCH_PERCENT > 100
#error Invalid search duty
#endif
#define VALID_SAMPLES_TO_MOVE   2U
#define DRIVE_MIN_PERCENT       25
#define DRIVE_MAX_PERCENT       25
#define REVERSE_MAX_PERCENT     20
#if DIST_NEAR_START_MM >= DIST_TARGET_MM || DIST_TARGET_MM >= DIST_FAR_START_MM
#error Invalid distance hysteresis thresholds
#endif
#if MOTOR_PWM_COUNTS > 65535U || MOTOR_PWM_COUNTS < 100U
#error Invalid PWM period
#endif
#endif
