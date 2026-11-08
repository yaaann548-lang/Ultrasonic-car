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
/* PWM points per 100 error units. Ki and Kd are per 10 ms tick. */
#define LINE_PID_KP             3
#define LINE_PID_KI             0
#define LINE_PID_KD             0
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
#define MOTOR_STEP_MS          20U
#define MOTOR_STEP_PERCENT     5
#define MOTOR_REVERSE_WAIT_MS  100U
/* Temporary bench correction; root cause of scale error is unresolved. */
/* Set to 1U to remove correction; 2U halves distance for display/control. */
#define US_DISTANCE_DIVISOR    1U//10.14 »°œ˚æ‡¿Î/2 ‘≠2
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
