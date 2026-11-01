/**
  ******************************************************************************
  * @file    pid.h
  * @brief   Fixed-point PID controller shared by the throttle position loop and
  *          the idle rpm loop.
  *
  * No HAL dependency on purpose: this file and pid.c compile on the host so the
  * control math can be tested off-target (see tests/run.sh).
  ******************************************************************************
  *
  * UNITS
  * -----
  *  setpoint / measurement   "permille", 0..1000, matching the rest of the
  *                           firmware's 0% .. 100% convention.
  *  output                   controller units.  For the throttle loop this is
  *                           PWM counts (0..ARR+1); for the idle loop it is a
  *                           throttle position in permille.
  *  gains                    stored multiplied by 1000, as constants.h already
  *                           does.  So a stored 1900 means K = 1.9.
  *
  *      Kp  [output / permille]
  *      Ki  [output / (permille * second)]
  *      Kd  [output / (permille / millisecond)]
  *
  * Kd is per *millisecond* of rate, not per second.  Full plate travel in 50 ms
  * is 20 permille/ms, so useful Kd lands in roughly the 0.05 .. 50 decade and the
  * x1000 storage gives good resolution.  Kd per second would have forced useful
  * values down to ~0.001 and the stored integer would quantise to nothing.
  *
  * WHY THE INTEGRAL IS IN MICRO-OUTPUT UNITS
  * -----------------------------------------
  * The original loop did `integral += (dt * error * Ki) / 1000` in whole output
  * counts.  With Ki=35 and dt=1 ms that integer division truncates to zero for
  * every |error| < 29 permille, so the integral was dead across the entire
  * +/-2.9% band being tuned.  Accumulating at 1e-6 output resolution means an
  * error of 1 permille still integrates.
  ******************************************************************************
  */

#ifndef __PID_H
#define __PID_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * Two gain sets.  A spring-returned throttle body is genuinely asymmetric: the
 * return spring fights the opening direction and assists the closing one, so
 * one set of gains cannot serve both.  The "rev" set is used when error <= 0.
 *
 * Unlike the original loop, switching sets does NOT touch the integral.  The
 * integral is stored in output units, not error units, so it stays valid across
 * a gain change and the switch is bumpless.  Zeroing it on every zero crossing
 * is what prevented the loop from ever settling against the spring.
 */
typedef struct {
  int32_t kp_m, ki_m, kd_m;             /* gains x1000, error > 0  (opening) */
  int32_t kp_m_rev, ki_m_rev, kd_m_rev; /* gains x1000, error <= 0 (closing) */

  int32_t out_min, out_max;   /* output clamp, output units */
  int32_t i_max;              /* |integral| clamp, output units */
  int32_t deadband;           /* |error| <= this: hold, freeze integral */
  uint8_t d_lpf_shift;        /* derivative EMA, tau ~= dt * 2^shift. 0 = off */
} pid_cfg_t;

typedef struct {
  int32_t i_acc;    /* integral accumulator, micro-output units (1e-6 output) */
  int32_t d_filt;   /* filtered derivative, milli-output units */
  int32_t pv_prev;
  uint8_t primed;   /* pv_prev holds a real sample */
} pid_state_t;

void pid_reset(pid_state_t *s);

/**
 * One control step.
 *
 * @param ff     feedforward in output units, added before clamping.  Used for
 *               the throttle return-spring curve and the idle base opening.
 * @param dt_ms  measured elapsed time since the previous step, clamped to >= 1.
 * @return       output, clamped to [out_min, out_max].
 *
 * Derivative is taken on the measurement and negated, not on the error, so a
 * setpoint step produces no derivative kick.
 *
 * Anti-windup is conditional integration: if the step would saturate the output
 * and the integral increment pushes further into that saturation, the increment
 * is discarded.  Combined with i_max this is what stops the integral pinning
 * full PWM against a mechanical stop.
 */
int32_t pid_step(const pid_cfg_t *c, pid_state_t *s,
                 int32_t sp, int32_t pv, int32_t dt_ms, int32_t ff);

/** Seed pv_prev without producing an output. Call after a mode change so the
 *  first real step does not see a huge synthetic rate. */
void pid_prime(pid_state_t *s, int32_t pv);

#ifdef __cplusplus
}
#endif
#endif /* __PID_H */
