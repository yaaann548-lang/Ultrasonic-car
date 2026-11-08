#include "line.h"
#include "board.h"
#include "config.h"
#include "motor.h"
#include "oled.h"
#include "timebase.h"

#if LINE_FOLLOW_ENABLE

#define SENSOR_COUNT 4U
/* Left outside, left inside, right inside, right outside. */
static const u8 code adc_channel[SENSOR_COUNT] = {9U, 8U, 10U, 11U};

#if !LINE_CENTER_TEST_ENABLE
#if LINE_STEP_TEST_ENABLE && !LINE_CALIBRATE_ENABLE
#error Line step test requires the button and calibration interface
#endif
#define SAMPLE_MS          10U
#define START_DELAY_MS     2000U
#define LINE_PRESENT_MIN   15U
#define INNER_PRESENT_MIN  25U
#define STEERING_DEADBAND  12
#define MAX_TRIM           18
#define PID_SCALE          100L
#define PID_INTEGRAL_LIMIT (6L * PID_SCALE)
#define FIND_TIMEOUT_MS    1000U
#define FIND_CONFIRM_FRAMES 3U

typedef signed int s16;
typedef signed long s32;

#if LINE_CALIBRATE_ENABLE
static u16 white_adc[SENSOR_COUNT];
static u16 black_adc[SENSOR_COUNT];
static u16 center_adc[SENSOR_COUNT];
#else
/* Bench measurements, from left outside to right outside. */
static const u16 code white_adc[SENSOR_COUNT] = {
    139U, 165U, 155U, 125U
};
static const u16 code black_adc[SENSOR_COUNT] = {
    1647U, 2194U, 2027U, 1566U
};
/* Only the inner two were measured in the older fixed calibration. */
static const u16 code center_adc[SENSOR_COUNT] = {139U, 385U, 618U, 125U};
#endif

enum { LINE_WAITING, LINE_RUNNING, LINE_FINDING, LINE_HALTED };

static s16 center_q;
#if LINE_STEP_TEST_ENABLE
static u16 xdata step_adc[SENSOR_COUNT];
#else
static u16 last_sample;
static u16 find_started;
static s8 last_turn;
static u8 missing_frames;
#endif
static u8 state;
static const s8 code weights[SENSOR_COUNT] = {-2, -1, 1, 2};
static void DriveTrim(s16 trim);
#if LINE_PID_KI
static s32 integral_term;
#endif
#if LINE_PID_KD
static s16 previous_error;
static u8 previous_valid;
#endif

static u8 Strength(u8 sensor, u16 value)
{
    s16 low = (s16)white_adc[sensor];
    s16 span = (s16)black_adc[sensor] - low;
    s32 strength = 100L * ((s16)value - low) / span;
    if (strength < 0) return 0;
    if (strength > 100) return 100;
    return (u8)strength;
}

static u8 LineSeen(u8 peak, u8 inner_sum)
{
    return peak >= LINE_PRESENT_MIN || inner_sum >= INNER_PRESENT_MIN;
}

static u8 SetCenterReference(void)
{
    u8 i, strength, peak = 0, inner_signal = 0;
    center_q = 0;
    for (i = 0; i < SENSOR_COUNT; i++) {
        strength = Strength(i, center_adc[i]);
        center_q += (s16)weights[i] * strength;
        if (strength > peak) peak = strength;
        if (i == 1U || i == 2U) inner_signal += strength;
    }
    return LineSeen(peak, inner_signal);
}
#endif

static u8 display_ready;

static void Show(char *first, char *second)
{
    if (!display_ready) return;
    OLED_Text(0, first);
    OLED_Text(2, second);
    display_ready = OLED_Ready();
}

static u8 ReadADC(u8 sensor, u16 *value)
{
    u16 started;
    ADC_CONTR = 0xC0 | adc_channel[sensor];
    started = Timebase_Now();
    while (!(ADC_CONTR & 0x20)) {
        if ((u16)(Timebase_Now() - started) >= 2U) {
            ADC_CONTR = 0x80;
            return 0;
        }
    }
    *value = ((u16)ADC_RES << 8) | ADC_RESL;
    ADC_CONTR &= 0xDF;
    return 1;
}

#if LINE_CALIBRATE_ENABLE || LINE_CENTER_TEST_ENABLE

#define CAL_SAMPLES 8U

static void WaitPress(void)
{
    do {
        while (!P33) { }
        Timebase_DelayMs(25);
    } while (!P33);
    do {
        while (P33) { }
        Timebase_DelayMs(25);
    } while (P33);
    do {
        while (!P33) { }
        Timebase_DelayMs(25);
    } while (!P33);
}

static u8 ConfirmPrompt(char *first, char *second, char *action)
{
    Show(first, second);
    if (!display_ready) return 0;
    OLED_Text(4, action);
    display_ready = OLED_Ready();
    if (!display_ready) return 0;
    WaitPress();
    return 1;
}

static u8 CapturePrompt(char *first, char *second)
{
    return ConfirmPrompt(first, second, "P3.3 TO SAVE");
}

static u8 ReadAverage(u8 sensor, u16 *value)
{
    u8 i;
    u16 reading, sum = 0;
    for (i = 0; i < CAL_SAMPLES; i++) {
        if (!ReadADC(sensor, &reading)) return 0;
        sum += reading;
        Timebase_DelayMs(5);
    }
    *value = (sum + CAL_SAMPLES / 2U) / CAL_SAMPLES;
    return 1;
}

static void WriteADC(char *text, u16 value)
{
    text[0] = '0' + value / 1000U;
    text[1] = '0' + value / 100U % 10U;
    text[2] = '0' + value / 10U % 10U;
    text[3] = '0' + value % 10U;
}

#endif

#if LINE_CENTER_TEST_ENABLE

static const char code center_prompt[3][18] = {
    "1 CENTER CAR", "2 CAR 5 MM LEFT", "3 CAR 5 MM RIGHT"
};
static const char code center_detail[3][19] = {
    "LINE BETWEEN IN", "FROM CENTER", "FROM CENTER"
};
static const char code center_label[3][6] = {"MID  ", "LEFT ", "RIGHT"};

static void CenterTest(void)
{
    u16 left[3], right[3];
    char row[] = "MID   L0000 R0000";
    u8 i, j;

    for (;;) {
        for (i = 0; i < 3U; i++) {
            if (!CapturePrompt(center_prompt[i], center_detail[i])) return;
            if (!ReadAverage(1U, &left[i]) || !ReadAverage(2U, &right[i])) {
                Show("ADC ERROR STOP", "RST TO REPEAT");
                return;
            }
        }

        for (i = 0; i < 3U; i++) {
            for (j = 0; j < 5U; j++) row[j] = center_label[i][j];
            WriteADC(row + 7, left[i]);
            WriteADC(row + 13, right[i]);
            OLED_Text(2U * i, row);
        }
        OLED_Text(5, "PHOTO RESULTS");
        OLED_Text(7, "P3.3 TO REPEAT");
        display_ready = OLED_Ready();
        if (!display_ready) return;
        WaitPress();
        OLED_Text(5, "");
        OLED_Text(7, "");
    }
}

#elif LINE_CALIBRATE_ENABLE

#define MIN_CAL_SPAN 120
#define MOTOR_MENU_MS 3000U
#define MOTOR_TEST_DUTY 25

enum { CAL_OK, CAL_BAD_SPAN, CAL_BAD_CENTER };

static const char code black_prompt[SENSOR_COUNT][12] = {
    "1 LEFT OUT", "2 LEFT IN", "3 RIGHT IN", "4 RIGHT OUT"
};

static void RunMotorFor(u16 duration)
{
    u16 start = Timebase_Now();
    while ((u16)(Timebase_Now() - start) < duration) Motor_Task();
    Motor_Stop();
}

static void RunMotorTest(s8 a, s8 b, u16 duration)
{
    Motor_SetSpeed(a, b);
    RunMotorFor(duration);
}

static u8 MotorTestMenu(void)
{
    u16 start;
    u8 requested = 0;
    Show("HOLD P3.3 TO TEST", "WAIT FOR CAL");
    if (!display_ready) return 0;
    start = Timebase_Now();
    while ((u16)(Timebase_Now() - start) < MOTOR_MENU_MS) {
        if (!P33) {
            Timebase_DelayMs(25);
            if (!P33) {
                requested = 1;
                break;
            }
        }
    }
    if (!requested) return 1;

    if (!ConfirmPrompt("LIFT WHEELS", "MOTOR A", "P3.3 TO START")) return 0;
    RunMotorTest(MOTOR_TEST_DUTY, 0, 750U);
    if (!ConfirmPrompt("LIFT WHEELS", "MOTOR B", "P3.3 TO START")) return 0;
    RunMotorTest(0, MOTOR_TEST_DUTY, 750U);
    if (!ConfirmPrompt("ON FLAT GROUND", "BOTH WHEELS", "P3.3 TO START")) return 0;
    DriveTrim(0);
    RunMotorFor(1200U);
    return 1;
}

static u8 CalibrateBlackWhite(void)
{
    u8 i;
    if (!CapturePrompt("ALL 4 ON WHITE", "HOLD CAR STILL")) return 0;
    for (i = 0; i < SENSOR_COUNT; i++) {
        if (!ReadAverage(i, &white_adc[i])) return 0;
    }
    for (i = 0; i < SENSOR_COUNT; i++) {
        if (!CapturePrompt(black_prompt[i], "ON BLACK CENTER")) return 0;
        if (!ReadAverage(i, &black_adc[i])) return 0;
    }
    return 1;
}

static u8 ReadCenter(void)
{
    u8 i;
    for (i = 0; i < SENSOR_COUNT; i++) {
        if (!ReadAverage(i, &center_adc[i])) return 0;
    }
    return 1;
}

static u8 ShowResults(void)
{
    char row[] = "1 W0000 B0000";
    char middle[] = "MID 1-0000 2-0000";
    s16 span;
    u8 i, result = CAL_OK;
    for (i = 0; i < SENSOR_COUNT; i++) {
        span = (s16)black_adc[i] - (s16)white_adc[i];
        if (span > -MIN_CAL_SPAN && span < MIN_CAL_SPAN) result = CAL_BAD_SPAN;
        row[0] = '1' + i;
        WriteADC(row + 3, white_adc[i]);
        WriteADC(row + 9, black_adc[i]);
        OLED_Text(i, row);
    }
    WriteADC(middle + 6, center_adc[0]);
    WriteADC(middle + 13, center_adc[1]);
    OLED_Text(4, middle);
    middle[4] = '3';
    middle[11] = '4';
    WriteADC(middle + 6, center_adc[2]);
    WriteADC(middle + 13, center_adc[3]);
    OLED_Text(5, middle);
    if (result == CAL_BAD_SPAN) {
        OLED_Text(6, "LOW ADC SPAN");
        OLED_Text(7, "P3.3 TO REPEAT");
    } else if (!SetCenterReference()) {
        result = CAL_BAD_CENTER;
        OLED_Text(6, "NO LINE AT CENTER");
        OLED_Text(7, "P3.3 TO SAVE MID");
    } else {
        OLED_Text(6, "CAL DONE");
#if LINE_STEP_TEST_ENABLE
        OLED_Text(7, "P3.3 TO VIEW");
#else
        OLED_Text(7, "P3.3 TO RUN");
#endif
    }
    display_ready = OLED_Ready();
    return result;
}

#endif

#if !LINE_CENTER_TEST_ENABLE
static void Halt(char *message)
{
    Motor_Stop();
    state = LINE_HALTED;
    Show("LINE FOLLOW", message);
}

static void ResetPID(void)
{
#if LINE_PID_KI
    integral_term = 0;
#endif
#if LINE_PID_KD
    previous_valid = 0;
#endif
}

static s16 SteeringTrim(s16 error)
{
    s32 output;
    if (error >= -STEERING_DEADBAND && error <= STEERING_DEADBAND) {
        ResetPID();
        return 0;
    }

    output = (s32)LINE_PID_KP * error;
#if LINE_PID_KI
    integral_term += (s32)LINE_PID_KI * error;
    if (integral_term > PID_INTEGRAL_LIMIT) integral_term = PID_INTEGRAL_LIMIT;
    else if (integral_term < -PID_INTEGRAL_LIMIT) integral_term = -PID_INTEGRAL_LIMIT;
    output += integral_term;
#endif
#if LINE_PID_KD
    if (previous_valid) output += (s32)LINE_PID_KD * (error - previous_error);
    previous_error = error;
    previous_valid = 1;
#endif
    if (output >= MAX_TRIM * PID_SCALE) return MAX_TRIM;
    if (output <= -MAX_TRIM * PID_SCALE) return -MAX_TRIM;
    return (s16)((output + (output < 0 ? -PID_SCALE / 2L : PID_SCALE / 2L)) / PID_SCALE);
}

static void Drive(s8 left, s8 right)
{
#if LINE_MOTOR_A_IS_LEFT
    Motor_SetSpeed(left, right);
#else
    Motor_SetSpeed(right, left);
#endif
}

static s8 ForwardPWM(s16 pwm)
{
    if (pwm < 0) return 0;
    if (pwm > 100) return 100;
    return (s8)pwm;
}

static void DriveTrim(s16 trim)
{
    Drive(ForwardPWM(LINE_LEFT_BASE_PWM + trim),
          ForwardPWM(LINE_RIGHT_BASE_PWM - trim));
}

enum { SAMPLE_OK, SAMPLE_NO_LINE, SAMPLE_ADC_ERROR };

static u8 ReadLineError(s16 *error)
{
    u16 value;
    s16 q = 0;
    u8 i, strength, peak = 0, inner_signal = 0;
    for (i = 0; i < SENSOR_COUNT; i++) {
        if (!ReadADC(i, &value)) return SAMPLE_ADC_ERROR;
#if LINE_STEP_TEST_ENABLE
        step_adc[i] = value;
#endif
        strength = Strength(i, value);
        if (strength > peak) peak = strength;
        if (i == 1U || i == 2U) inner_signal += strength;
        q += (s16)weights[i] * strength;
    }
    if (!LineSeen(peak, inner_signal)) return SAMPLE_NO_LINE;

    *error = q - center_q;
    return SAMPLE_OK;
}

#if LINE_STEP_TEST_ENABLE
#define STEP_RUN_MS 180U
#define STEP_SETTLE_MS 250U
#define STEP_NO_PWM 127

static void ShowStepADC(u8 index)
{
    char row[] = "1-0000 2-0000";
    row[0] = '1' + index;
    row[7] = '2' + index;
    WriteADC(row + 2, step_adc[index]);
    WriteADC(row + 9, step_adc[index + 1U]);
    OLED_Text(index / 2U, row);
}

static void ShowStepPWM(u8 page, s8 trim)
{
    char row[] = "PLAN L00 R00";
    u8 left, right;
    if (trim == STEP_NO_PWM) {
        OLED_Text(page, page == 4U ? "PLAN STOP" : "LAST NONE");
        return;
    }
    if (page == 5U) {
        row[0] = 'L'; row[1] = 'A'; row[2] = 'S'; row[3] = 'T';
    }
    left = (u8)ForwardPWM(LINE_LEFT_BASE_PWM + trim);
    right = (u8)ForwardPWM(LINE_RIGHT_BASE_PWM - trim);
    row[6] = '0' + left / 10U;
    row[7] = '0' + left % 10U;
    row[10] = '0' + right / 10U;
    row[11] = '0' + right % 10U;
    OLED_Text(page, row);
}

static void StepTest(void)
{
    char error_row[] = "ERR  0000";
    s16 error;
    s8 trim, last_trim = STEP_NO_PWM;
    u8 sample;
    for (;;) {
        sample = ReadLineError(&error);
        if (sample == SAMPLE_ADC_ERROR) break;
        ShowStepADC(0);
        ShowStepADC(2);
        if (sample == SAMPLE_OK) {
            error_row[4] = error < 0 ? '-' : ' ';
            WriteADC(error_row + 5, (u16)(error < 0 ? -error : error));
            OLED_Text(3, error_row);
            ResetPID();
            ShowStepPWM(4, (s8)SteeringTrim(error));
            OLED_Text(7, "P3.3 TO STEP");
        } else {
            OLED_Text(3, "NO LINE");
            ShowStepPWM(4, STEP_NO_PWM);
            OLED_Text(7, "P3.3 TO READ");
        }
        ShowStepPWM(5, last_trim);
        if (!OLED_Ready()) return;
        WaitPress();

        if (sample != SAMPLE_OK) continue;
        sample = ReadLineError(&error);
        if (sample == SAMPLE_ADC_ERROR) break;
        if (sample == SAMPLE_OK) {
            ResetPID();
            trim = (s8)SteeringTrim(error);
            DriveTrim(trim);
            RunMotorFor(STEP_RUN_MS);
            last_trim = trim;
            Timebase_DelayMs(STEP_SETTLE_MS);
        }
    }
    Halt("ADC ERROR STOP");
}
#else
static void FindLine(u16 now)
{
    if (state == LINE_WAITING) return;
    if (state == LINE_RUNNING) {
        state = LINE_FINDING;
        find_started = now;
        Show("LINE FOLLOW", "FIND LINE");
    } else if ((u16)(now - find_started) >= FIND_TIMEOUT_MS) {
        Halt("LINE LOST STOP");
        return;
    }
    /* Pivot on the inner wheel without reversing either motor. */
    if (last_turn < 0) Drive(0, LINE_RIGHT_BASE_PWM);
    else if (last_turn > 0) Drive(LINE_LEFT_BASE_PWM, 0);
    else DriveTrim(0);
}
#endif
#endif

void Line_Init(void)
{
#if LINE_CALIBRATE_ENABLE && !LINE_CENTER_TEST_ENABLE
    u8 i, result = CAL_BAD_SPAN;
#endif
    Motor_Stop();
    P0M1 |= 0x0F;
    P0M0 &= 0xF0;
    P0PU &= 0xF0;
    P0IE &= 0xF0;
    ADCTIM = 0x3F;
    ADCCFG = 0x2F; /* Right aligned, clock = SYSCLK / 32. */
    ADC_CONTR = 0x80;
    display_ready = 0;
#if OLED_ENABLE
    display_ready = OLED_Init();
#endif
#if LINE_CALIBRATE_ENABLE || LINE_CENTER_TEST_ENABLE
    P3M1 |= 0x08;
    P3M0 &= 0xF7;
    P3PU |= 0x08;
#endif
#if LINE_CENTER_TEST_ENABLE
    CenterTest();
#else
#if LINE_CALIBRATE_ENABLE
    if (!MotorTestMenu()) {
        Halt("OLED ERROR STOP");
        return;
    }
    do {
        if (result == CAL_BAD_SPAN &&
            (!CalibrateBlackWhite() || !CapturePrompt("CENTER CAR ON LINE", "LINE BETWEEN IN"))) {
            Halt("CAL ERROR STOP");
            return;
        }
        if (!ReadCenter()) {
            Halt("ADC ERROR STOP");
            return;
        }
        result = ShowResults();
        if (!display_ready) {
            Halt("OLED ERROR STOP");
            return;
        }
        WaitPress();
        if (result != CAL_BAD_CENTER) {
            for (i = 0; i < 8U; i++) OLED_Text(i, "");
        }
    } while (result != CAL_OK);
#else
    if (!SetCenterReference()) {
        Halt("NO LINE AT CENTER");
        return;
    }
#endif
    ResetPID();
#if LINE_STEP_TEST_ENABLE
    StepTest();
#else
    Show("LINE FOLLOW", "WAIT FOR LINE");
    Timebase_DelayMs(START_DELAY_MS);
    state = LINE_WAITING;
    last_turn = 0;
    missing_frames = 0;
    last_sample = Timebase_Now();
#endif
#endif
}

void Line_Task(void)
{
#if !LINE_CENTER_TEST_ENABLE && !LINE_STEP_TEST_ENABLE
    u16 now;
    s16 error, trim;
    u8 sample;

    if (state == LINE_HALTED) return;
    now = Timebase_Now();
    if ((u16)(now - last_sample) < SAMPLE_MS) return;
    last_sample = now;
    sample = ReadLineError(&error);
    if (sample == SAMPLE_ADC_ERROR) {
        Halt("ADC ERROR STOP");
        return;
    }
    if (sample == SAMPLE_NO_LINE) {
        if (state == LINE_RUNNING && ++missing_frames < FIND_CONFIRM_FRAMES)
            return;
        ResetPID();
        FindLine(now);
        return;
    }

    missing_frames = 0;
    if (state != LINE_RUNNING) {
        state = LINE_RUNNING;
        Show("LINE FOLLOW", "RUNNING");
    }
    if (error > STEERING_DEADBAND) last_turn = 1;
    else if (error < -STEERING_DEADBAND) last_turn = -1;
    trim = SteeringTrim(error);
    DriveTrim(trim);
#endif
}

#endif
