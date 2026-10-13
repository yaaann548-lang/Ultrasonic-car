#ifndef CAR_CONFIG_H
#define CAR_CONFIG_H
/* ISP IRC setting must be 11.0592 MHz. These macros do not set the IRC. */
#define SYSCLK_HZ              11059200UL
#if SYSCLK_HZ != 11059200UL
#error Recalculate ultrasonic conversion before changing clock
#endif
#define OLED_ENABLE           1
#define MOTOR_ENABLE          1
#define MOTOR_A_INVERT         0
#define MOTOR_B_INVERT         1
#define MOTOR_PWM_COUNTS       1000U
#define MOTOR_STEP_MS          20U
#define MOTOR_STEP_PERCENT     5
#define MOTOR_REVERSE_WAIT_MS  100U
/* Temporary bench correction; root cause of scale error is unresolved. */
/* Set to 1U to remove correction; 2U halves distance for display/control. */
#define US_DISTANCE_DIVISOR    2U
#if US_DISTANCE_DIVISOR != 1U && US_DISTANCE_DIVISOR != 2U
#error Unsupported temporary distance divisor
#endif
#define SAMPLE_PERIOD_MS       200U
#define DISPLAY_PERIOD_MS      200U
#define STARTUP_WAIT_MS         2000U
#define DIST_TARGET_MM          250U
#define DIST_NEAR_START_MM      220U
#define DIST_FAR_START_MM       280U
#define DIST_MAX_TRACK_MM       1500U
#define VALID_SAMPLES_TO_MOVE   2U
#define DRIVE_MIN_PERCENT       30
#define DRIVE_MAX_PERCENT       55
#define REVERSE_MAX_PERCENT     35
#if DIST_NEAR_START_MM >= DIST_TARGET_MM || DIST_TARGET_MM >= DIST_FAR_START_MM
#error Invalid distance hysteresis thresholds
#endif
#if MOTOR_PWM_COUNTS > 65535U || MOTOR_PWM_COUNTS < 100U
#error Invalid PWM period
#endif
#endif
