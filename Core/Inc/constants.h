
//values can be changed as needed

#ifndef __CONSTANTS_H
#define __CONSTANTS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

//the following constants determine the absolute upper and lower bounds that determine sensor failure

#define APPS1_UB    3900
#define APPS1_LB    100
#define APPS2_UB    3900
#define APPS2_LB    100

#define TPS1_UB     3900
#define TPS1_LB     100
#define TPS2_UB     3900
#define TPS2_LB     100

#define BS1_UB      3800
#define BS1_LB      100
#define BS2_UB      3800
#define BS2_LB      100

//the following constants determine the upper and lower bounds of normal measurable range

#define APPS1_OPERATION_LB  966    //opposite slope sensor
#define APPS1_OPERATION_UB  1935
#define APPS2_OPERATION_LB  2120    //positive slope sensor
#define APPS2_OPERATION_UB  2980

#define TPS1_OPERATION_LB   700     //positive slope sensor
#define TPS1_OPERATION_UB   3615
#define TPS2_OPERATION_LB   3140    //opposite slope sensor
#define TPS2_OPERATION_UB   225

#define BSE1_OPERATION_LB   405     //front brake circuit
#define BSE1_OPERATION_UB   1000    //pressed with hand as hard as possible, could be pressed harder
#define BSE2_OPERATION_LB   405     //back brake circuit
#define BSE2_OPERATION_UB   675     //pressed with hand as hard as possible, could be pressed harder

//the following are constants for tuning implausibility checks (dual sensor deviation, hard braking, open throttle)
// "percentage" is represented as a number from 0 to 1000, corresponding to 0% and 100%

#define MAX_GAS_PEDAL_SENSORS_DEVIATION             100
#define MAX_THROTTLE_SENSORS_DEVIATION              100

#define MAX_THROTTLE_INTENDED_VS_ACTUAL_DEVIATION   200

#define HARD_BRAKING_PERCENTAGE_THRESHOLD           160
#define OPEN_THROTTLE_PERCENTAGE_THRESHOLD          400


/* ===========================================================================
 *                        MEASURE ME  --  do not guess
 * ===========================================================================
 * Everything in this block is a physical property of the throttle body, the
 * motor, the driver, or the engine.  Each one has a pmbtune.py command that
 * measures it and prints the value to paste back in here.
 *
 * Values marked PLACEHOLDER are NOT derived.  They are deliberately inert
 * (no-ops that reproduce the previous firmware behaviour) or conservative,
 * so that flashing this build changes only the bug fixes.  The loop is not
 * actually tuned until these have been measured.
 * =========================================================================*/

/* --- soft travel limits -------------------------------- pmbtune.py limits --
 * THIS IS THE JAM FIX.
 *
 * The old code commanded position 0 on closure and the pedal map returned 1008
 * at full pedal, i.e. it drove the plate into both mechanical hard stops, and
 * the unbounded integral then pinned full PWM into the stop until the plate
 * wedged.  Every target in the firmware is now clamped into this window, so the
 * plate is never commanded against a stop.
 *
 * `pmbtune.py limits` creeps the plate to each stop under a PWM ceiling and
 * prints the measured endpoints plus a margin.  PLACEHOLDER until then. */
#define THROTTLE_SOFT_MIN       20      /* PLACEHOLDER  2.0% */
#define THROTTLE_SOFT_MAX       980     /* PLACEHOLDER 98.0% */

/* --- stiction / direction ---------------------------- pmbtune.py stiction --
 * A geared DC throttle body needs a minimum duty to break static friction.
 * Below it the motor does not move at all, which looks exactly like "the gains
 * do nothing".  0 = disabled, matching previous behaviour. */
#define THROTTLE_STICTION_PWM       0   /* PLACEHOLDER, output counts */

/* Minimum |output| worth reversing direction for.  Prevents the H-bridge
 * chattering back and forth over the last count of TPS noise. 0 = disabled. */
#define THROTTLE_DIR_HYSTERESIS     0   /* PLACEHOLDER, output counts */

/* --- return spring feedforward ----------------------------- pmbtune.py ff --
 * Holding the plate at angle x needs a roughly linear amount of motor effort
 * against the return spring:  u_ff = (FF_SLOPE * target)/1000 + FF_OFFSET.
 * Supplying it directly means the integral only has to correct the residual
 * instead of rediscovering the whole spring curve after every move.
 * Both 0 = disabled, matching previous behaviour. */
#define THROTTLE_FF_SLOPE           0   /* PLACEHOLDER, output counts per 100% */
#define THROTTLE_FF_OFFSET          0   /* PLACEHOLDER, output counts */

/* --- PID shaping ------------------------------------------------------------
 * Deadband: |error| below this holds position instead of hunting. 0 = disabled.
 * i_max:    integral clamp in output counts.  Full authority preserves the old
 *           (unbounded) behaviour while making the clamp exist at all.
 * d_lpf:    derivative EMA, tau ~= dt * 2^shift.  At dt = 1 ms, shift 3 is a
 *           ~20 Hz corner.  Inert today because both D gains are 0. */
#define THROTTLE_PID_DEADBAND       0   /* PLACEHOLDER, permille */
#define THROTTLE_PID_I_MAX          MAX_THROTTLE_MOTOR_PWM
#define THROTTLE_D_LPF_SHIFT        3

/* --- jam / stall guard ---------------------------------- pmbtune.py jam ----
 * Reasoned, not measured: full plate travel takes well under 100 ms, so output
 * saturated for 150 ms while position moves less than 1% is unambiguously a
 * stalled rotor, not a slow move.  A stalled brushed motor is also a thermal
 * event for the motor and the driver, so this guard protects the hardware as
 * much as it recovers the plate.
 *
 * On trip: PWM is dropped, a short pulse is driven the other way to break the
 * plate free, the integral is reset, and the move is retried.  After
 * UNJAM_RETRIES failures it escalates to the existing
 * throttle_not_at_intended_implausibility path (relay opens, spring closes). */
#define THROTTLE_STALL_TIMEOUT_MS   150
#define THROTTLE_STALL_MIN_MOTION   10  /* permille of travel in that window */
#define THROTTLE_UNJAM_PULSE_MS     60
#define THROTTLE_UNJAM_PWM          MAX_THROTTLE_MOTOR_PWM
#define THROTTLE_UNJAM_RETRIES      3

/* NOTE on how this interacts with THROTTLEBODY_FAILURE_TIMEOUT (200 ms below).
 * A jam also shows up as a large intended-vs-actual deviation, so the
 * implausibility task is counting at the same time.  With a 150 ms stall
 * timeout and a 200 ms failure timeout you get roughly ONE unjam attempt
 * before the safety path takes over, opens the throttle relay and lets the
 * spring close the plate.  That ordering is intentional -- the safety path
 * should win -- but it does mean UNJAM_RETRIES above 1 only has an effect if
 * you also raise THROTTLEBODY_FAILURE_TIMEOUT.  Decide deliberately; do not
 * raise the failure timeout just to let the unjam run. */

/* --- idle base opening --------------------------------- pmbtune.py idleff --
 * The idle loop was pure proportional with a zero base, so at target rpm it
 * commanded 0% throttle -- it could not hold an idle, only react after the idle
 * had already sagged, and then it closed the plate into the stop again.
 *
 * Idle is now  u = IDLE_FF + PI(rpm error), which is the standard form.  IDLE_FF
 * is the plate opening that holds the target idle with zero rpm error; it is
 * engine-specific and must be measured.
 *
 * WHILE THESE ARE 0, IDLE STILL WILL NOT HOLD.  Run pmbtune.py idleff. */
#define IDLE_FF_HOT                 0   /* PLACEHOLDER, permille */
#define IDLE_FF_COLD                0   /* PLACEHOLDER, permille */

/* ======================= end MEASURE ME block ============================ */


/* the following pid tuning values are for throttle flap control.
 *
 * All gains are stored multiplied by 1000: a stored 1900 means K = 1.9.
 *
 *   P  [output counts per permille of error]
 *   I  [output counts per permille per second]
 *   D  [output counts per (permille per millisecond)]
 *
 * NOTE on D: the rate is per millisecond, not per second.  Full travel in 50 ms
 * is 20 permille/ms, so useful D lands around 0.05 .. 5 and the x1000 integer
 * keeps real resolution.  See pid.h.
 *
 * These are the values carried over verbatim from the previous firmware.  They
 * were not derived from measurement; pmbtune.py derive replaces them. */

#define PID_LOW_BACKWARD_P  300
#define PID_LOW_BACKWARD_I  15
#define PID_FORWARD_P   1900
#define PID_BACKWARD_P  600
#define PID_FORWARD_I   35
#define PID_BACKWARD_I  30
#define PID_FORWARD_D   0
#define PID_BACKWARD_D  0

/* Nominal control period in ms.  The loop is now driven by TPS frame arrival
 * and uses the measured dt, so this is only the watchdog fallback period used
 * when CAN goes quiet.  Run pmbtune.py rate to find the real TPS interval. */
#define PID_DT          1
#define THROTTLE_CONTROL_TIMEOUT_MS 10

/* Largest dt the loop will believe.  Separate from the wake period above: the
 * loop wakes every 10 ms at worst, but if it was starved for longer than that
 * the real elapsed time is still the honest value to integrate over -- up to a
 * point.  Beyond this, assume something went wrong and do not hand the
 * integrator a huge step. */
#define THROTTLE_MAX_DT_MS          50

/* Thread flag readsensordata() sets to wake the throttle loop on a fresh TPS
 * frame.  The loop used to run at a fixed 1 kHz against a CAN value arriving
 * far slower, so most samples were stale repeats and the derivative saw
 * 0,0,0,spike.  Running on arrival also makes the measured dt meaningful. */
#define THROTTLE_TPS_EVENT_FLAG 0x01U

/* Below this plate position the closing direction uses the LOW_BACKWARD gains:
 * near the closed stop the return spring does most of the work, so the same
 * closing gain would slam it.  Replaces the inline literal 200. */
#define THROTTLE_LOW_REGION     200

// the following are constants for can frames

#define APPS1_CAN_ID        210
#define APPS1_CAN_OFFSET    0
#define APPS2_CAN_ID        215
#define APPS2_CAN_OFFSET    6
#define TPS1_CAN_ID         215
#define TPS1_CAN_OFFSET     4
#define TPS2_CAN_ID         215
#define TPS2_CAN_OFFSET     2
#define BS1_CAN_ID          215
#define BS1_CAN_OFFSET      0
#define BS2_CAN_ID          214
#define BS2_CAN_OFFSET      6

#define COOLANT_CAN_ID      1522
#define COOLANT_CAN_OFFSET  6
#define COOLANT_CAN_DIVIDE  10

#define RPM_CAN_ID          1520
#define RPM_CAN_OFFSET      6     //since this is 2 byte number, offset 3 means offset 6 bytes.

#define BSPD_CAN_ID         214
#define BSPD_CAN_OFFSET     4

#define SHIFT_ID            172
#define SHIFT_DIR_OFFSET    0
#define SHIFT_COUNT_OFFSET  1

#define SHIFT_DIR_UP        1
#define SHIFT_DIR_DOWN      2

//the following are timing constants

#define DISTANCE_BETWEEN_SHIFTS                 120

/* Downshift: solenoid on, wait SHUTDOWN_CUT_DELAY, blip for
 * DOWNSHIFT_SOLENOID_HOLD_TIME, wait DOWNSHIFT_SOLENOID_RELEASE_TIME, solenoid
 * off.  Values are the 10/160/40 that were hardcoded in paddleshift(). */
#define SHUTDOWN_CUT_DELAY                      10
#define DOWNSHIFT_SOLENOID_HOLD_TIME            160
#define DOWNSHIFT_SOLENOID_RELEASE_TIME         40

/* Upshift: solenoid on, wait LEAD, ignition cut for DURATION, wait RELEASE,
 * solenoid off.  Values are the 80/200/100 that were hardcoded. */
#define UPSHIFT_CUT_LEAD_TIME                   80
#define UPSHIFT_CUT_DURATION                    200
#define UPSHIFT_SOLENOID_RELEASE_TIME           100

/* Legacy, never referenced by any code path.  Left in place so nobody assumes
 * they were doing something; delete once you have confirmed they are dead. */
#define UPSHIFT_SOLENOID_HOLD_TIME              120
#define THROTTLE_BLIP_DELAY                     50

#define SENSOR_IMPLAUSIBILITY_TIMEOUT           100
#define SENSOR_BACK_TO_NORMAL_TIMEOUT           50

#define THROTTLEBODY_FAILURE_TIMEOUT            200
#define THROTTLEBODY_BACK_TO_NORMAL_TIMEOUT     200

#define OPEN_SHUTDOWN_CIRCUIT_TIMEOUT           600
#define CLOSE_SHUTDOWN_CIRCUIT_TIMEOUT          1200

#define IMPLAUSIBILITY_CHECK_INTERVAL           10

//the following determines gas pedal threshold for using idle pid

#define GAS_PEDAL_IDLE_THRESHOLD    100     //using PID idle control when gas pedal below 10%

/* Closed throttle.  Deliberately the soft minimum and not 0: commanding 0 drove
 * the plate into the closed hard stop, which is how it jammed.  On a real fault
 * the throttle relay opens anyway and the return spring closes the plate the
 * rest of the way mechanically. */
#define THROTTLE_CLOSURE_TARGET     THROTTLE_SOFT_MIN

/* --- downshift blip --------------------------------------------------------
 * Blip target is scheduled against engine speed: a downshift at 4000 rpm needs
 * a different blip than one at 8000.  Both endpoints default to the previous
 * fixed 800, so the schedule is flat (identical behaviour) until measured.
 *
 * A gearbox ratio table would let this be computed from the ratio step instead
 * of interpolated -- see the plan's deferred list. */
#define SHIFT_BLIP_TARGET           800     //80%
#define BLIP_RPM_LOW                4000
#define BLIP_RPM_HIGH               9000
#define SHIFT_BLIP_TARGET_LOW_RPM   SHIFT_BLIP_TARGET
#define SHIFT_BLIP_TARGET_HIGH_RPM  SHIFT_BLIP_TARGET

/* Hard ceiling on how long a blip may hold the throttle open, regardless of
 * what the shift task does.  The old code used a plain flag with no deadline:
 * if paddleshift was preempted or faulted mid-blip, the throttle stayed at 80%
 * indefinitely.  The throttle task now enforces this deadline itself. */
#define BLIP_MAX_DURATION_MS        250

/* Engine speed above which a downshift blip is suppressed.  Replaces the
 * inline literal 9000 in paddleshift(). */
#define DOWNSHIFT_MAX_RPM           9000

/* the following pid tuning values are for the idle rpm control.
 *
 * Same convention as the throttle gains: stored x1000, so 500 means K = 0.5.
 *   P  [permille of throttle per rpm of error]
 *   I  [permille of throttle per rpm per second]
 *
 * NOTE: the previous code divided the idle integral by 10000 while the throttle
 * loop divided by 1000, so the same "x1000" naming meant two different things.
 * Both now use the x1000 convention.  PID_RPM_I was 0, so nothing changes in
 * practice -- but a nonzero value here now means 10x what it used to. */

#define PID_RPM_P       500
#define PID_RPM_I       0
#define PID_RPM_D       0
#define PID_RPM_DT      10
#define PID_RPM_SAVE_P  800
#define PID_RPM_I_MAX   200     /* permille of throttle the idle integral may own */

//the following determines hot vs cold idle targets and thresholds for controlling idle

#define RPM_HOT_IDLE_MAX_THROTTLE       200     //40%
#define RPM_COLD_IDLE_MAX_THROTTLE      200     //40%
#define RPM_HOT_IDLE_TARGET             2000    //rpm
#define RPM_COLD_IDLE_TARGET            2000    //rpm
#define IDLE_TEMP_STEPDOWN_THRESHOLD    170     //170 degrees fahrenheit
#define RPM_TOO_LOW_THRESHOLD           1500     //rpm


//the following is the rpm that the starter motor spins at

#define STARTER_RPM     300


//the following determines maximum allowable current for running the throttle, it is a number from 0 to 1000 representing 0% to 100%

#define MAX_THROTTLE_MOTOR_PWM    1000

/* --- pedal to throttle map -------------------------------------------------
 * Progressive map, expressed so the endpoints are exact by construction:
 *
 *      f(x) = ((1000 - k) * x  +  k * x^2 / 1000) / 1000
 *
 *      f(0) = 0   and   f(1000) = 1000   for any k, algebraically.
 *
 * k is the progressivity in permille: 0 = straight linear, 1000 = pure square.
 *
 * The old form QUADRATIC(x, 100, 850, 0) returned 1008 at full pedal, i.e. it
 * commanded 0.8% past the open hard stop.  k = 984 reproduces the old curve to
 * within about one count across the travel (x=250: 64 vs 65, x=500: 254 vs 250,
 * x=750: 568 vs 565) while hitting 1000 exactly at full pedal.  The shape is
 * preserved; only the endpoint defect is gone. */
#define THROTTLE_MAP_PROGRESSIVITY  984

#define THROTTLE_MAP(x) \
  (((1000 - (THROTTLE_MAP_PROGRESSIVITY)) * (int32_t)(x) \
    + (THROTTLE_MAP_PROGRESSIVITY) * ((int32_t)(x) * (int32_t)(x)) / 1000) / 1000)

//the following slopes and intercepts are all calculated with equations, they shouldnt need to be modified, just change the OPERATION_UB and OPERATION_LB values above
// "percentages" are represented as a number from 0 to 1000, corresponding to 0% and 100%

#define SLOPE(y1, y2, x1, x2) (((y2) - (y1)) * 1000 / ((x2) - (x1)))                      //m = (y2 - y1) / (x2 - x1)
#define INTERCEPT(y1, y2, x1, x2) (((y1) * 1000) - (SLOPE(y1, y2, x1, x2) * (x1)))    //b = y2 - (m*x2)

//this computes a cubic polynomial (ax^3 + bx^2 + cx + d). Only works with coefficients that are multiplied by 100000
#define CUBIC(x, a, b, c, d) ((((a)*(x)*(x)*(x)) + ((b)*(x)*(x)) + ((c)*(x)) + (d)) / 1000000)

//this computes a quadratic polynomial (ax^2 + bx + c). Only works with coefficients that are multiplied by 10000

#define QUADRATIC(x, a, b, c) ((((a)*(x)*(x)) + ((b)*(x)) + (c)) / 100000)

//this computes a linear function (y = mx + b). Only works with slope and intercepts that are multiplied by 1000
#define LINEAR(x, m, b) ((((m)*(x)) + (b)) / 1000)

/* Clamp a sensor-derived permille value into range.  LINEAR truncates toward
 * zero and can return negatives just outside the calibrated span, and a
 * negative "position" has no meaning in the control law. */
#define CLAMP_PERMILLE(v) (((v) < 0) ? 0 : (((v) > 1000) ? 1000 : (v)))

#define CLAMP(v, lo, hi) (((v) < (lo)) ? (lo) : (((v) > (hi)) ? (hi) : (v)))

/* Clamp any throttle target into the soft travel window. */
#define CLAMP_THROTTLE_TARGET(v) CLAMP((v), THROTTLE_SOFT_MIN, THROTTLE_SOFT_MAX)


#define RAW_TPS1_TO_THROTTLE_PERCENTAGE_SLOPE           SLOPE(1000, 0, TPS1_OPERATION_UB, TPS1_OPERATION_LB)
#define RAW_TPS1_TO_THROTTLE_PERCENTAGE_INTERCEPT       INTERCEPT(1000, 0, TPS1_OPERATION_UB, TPS1_OPERATION_LB)

#define RAW_TPS2_TO_THROTTLE_PERCENTAGE_SLOPE           SLOPE(1000, 0, TPS2_OPERATION_UB, TPS2_OPERATION_LB)
#define RAW_TPS2_TO_THROTTLE_PERCENTAGE_INTERCEPT       INTERCEPT(1000, 0, TPS2_OPERATION_UB, TPS2_OPERATION_LB)

#define RAW_APPS1_TO_PEDAL_PERCENTAGE_SLOPE             SLOPE(1000, 0, APPS1_OPERATION_LB, APPS1_OPERATION_UB)
#define RAW_APPS1_TO_PEDAL_PERCENTAGE_INTERCEPT         INTERCEPT(1000, 0, APPS1_OPERATION_LB, APPS1_OPERATION_UB)

#define RAW_APPS2_TO_PEDAL_PERCENTAGE_SLOPE             SLOPE(1000, 0, APPS2_OPERATION_UB, APPS2_OPERATION_LB)
#define RAW_APPS2_TO_PEDAL_PERCENTAGE_INTERCEPT         INTERCEPT(1000, 0, APPS2_OPERATION_UB, APPS2_OPERATION_LB)

#define RAW_BS1_TO_BRAKE_PERCENTAGE_SLOPE               SLOPE(1000, 0, BSE1_OPERATION_UB, BSE1_OPERATION_LB)
#define RAW_BS1_TO_BRAKE_PERCENTAGE_INTERCEPT           INTERCEPT(1000, 0, BSE1_OPERATION_UB, BSE1_OPERATION_LB)

#define RAW_BS2_TO_BRAKE_PERCENTAGE_SLOPE               SLOPE(1000, 0, BSE2_OPERATION_UB, BSE2_OPERATION_LB)
#define RAW_BS2_TO_BRAKE_PERCENTAGE_INTERCEPT           INTERCEPT(1000, 0, BSE2_OPERATION_UB, BSE2_OPERATION_LB)

#define APPS_PEDAL_PERCENT_TO_THROTTLE_2D_PERCENTAGE_SLOPE      SLOPE(69000, 1, 1000, 0)
#define APPS_PEDAL_PERCENT_TO_THROTTLE_2D_PERCENTAGE_INTERCEPT  INTERCEPT(69000, 1, 1000, 0)

/* CAN field accessors.
 *
 * The little-endian one used to cast uint8_t* to uint16_t* and index it.  That
 * is a strict-aliasing violation (it breaks under -O2, which the Release build
 * now uses) and it would hard-fault on Cortex-M0+ for any odd offset.  Byte-wise
 * assembly is correct for every offset and compiles to the same two loads. */

#define GET_16BIT_LITTLEENDIAN_CAN_VALUE(pointer_8bit, offset) \
  ((uint16_t)((uint16_t)((pointer_8bit)[(offset)]) | \
              ((uint16_t)((pointer_8bit)[(offset) + 1]) << 8)))

#define GET_16BIT_BIGENDIAN_CAN_VALUE(pointer_8bit, offset) \
  ((uint16_t)((uint16_t)((pointer_8bit)[(offset) + 1]) | \
              ((uint16_t)((pointer_8bit)[(offset)]) << 8)))

#define GET_8BIT_CAN_VALUE(pointer_8bit, offset) ((pointer_8bit)[(offset)])




#ifdef __cplusplus
}
#endif
#endif /* __CONSTANTS_H */
