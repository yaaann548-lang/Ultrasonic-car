#include "car.h"
#include "config.h"
#include "ultrasonic.h"
#include "motor.h"
#include "timebase.h"
static u16 distance_mm, last_sample, boot_time;
static u8 sensor_status, state, good_count, startup_done;
static s8 Speed(u16 error, u8 reverse)
{
    u16 speed, limit;
    speed = DRIVE_MIN_PERCENT + error / 10U;
    limit = reverse ? REVERSE_MAX_PERCENT : DRIVE_MAX_PERCENT;
    if (speed > limit) speed = limit;
    return (s8)speed;
}
void Car_Init(void)
{
    distance_mm = 0; sensor_status = US_NO_ECHO;
    state = CAR_WAIT; good_count = 0; startup_done = 0;
    boot_time = Timebase_Now(); last_sample = boot_time;
    Motor_Stop();
}
void Car_Task(void)
{
    u16 now;
    s8 demand;
    now = Timebase_Now();
    if (!startup_done && (u16)(now - boot_time) >= STARTUP_WAIT_MS)
        startup_done = 1;
    if ((u16)(now - last_sample) < SAMPLE_PERIOD_MS) return;
    /* Schedule relative to completion: >=200ms between triggers. */
    sensor_status = Ultrasonic_Measure(&distance_mm);
    last_sample = Timebase_Now();
    if (sensor_status != US_OK) {
        good_count = 0; state = CAR_FAULT; Motor_Stop(); return;
    }
    if (distance_mm > DIST_MAX_TRACK_MM) {
        good_count = 0; state = CAR_FAR; Motor_Stop(); return;
    }
    if (good_count < VALID_SAMPLES_TO_MOVE) good_count++;
    if (!startup_done && (u16)(last_sample - boot_time) >= STARTUP_WAIT_MS)
        startup_done = 1;
    if (!startup_done || good_count < VALID_SAMPLES_TO_MOVE) {
        state = CAR_WAIT; Motor_Stop(); return;
    }
    /* Hysteresis: enter at 220/280, return to stop at 250mm. */
    if (distance_mm < DIST_NEAR_START_MM) state = CAR_REVERSE;
    else if (distance_mm > DIST_FAR_START_MM) state = CAR_FORWARD;
    else if (state == CAR_FORWARD && distance_mm > DIST_TARGET_MM) { }
    else if (state == CAR_REVERSE && distance_mm < DIST_TARGET_MM) { }
    else state = CAR_STOP;
    if (state == CAR_FORWARD) {
        demand = Speed(distance_mm - DIST_TARGET_MM, 0);
        Motor_SetSpeed(demand, demand);
    } else if (state == CAR_REVERSE) {
        demand = -Speed(DIST_TARGET_MM - distance_mm, 1);
        Motor_SetSpeed(demand, demand);
    } else Motor_Stop();
}
u16 Car_Distance(void) { return distance_mm; }
u8 Car_Status(void) { return sensor_status; }
u8 Car_State(void) { return state; }
