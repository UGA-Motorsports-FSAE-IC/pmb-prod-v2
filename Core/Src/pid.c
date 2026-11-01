/**
  ******************************************************************************
  * @file    pid.c
  * @brief   Fixed-point PID.  See pid.h for units and rationale.
  ******************************************************************************
  * Intermediate products are int64_t.  On a Cortex-M0+ a 64-bit multiply costs
  * ~50 cycles; at a 1 kHz control rate that is 0.1% of the CPU, and it removes
  * every overflow argument from the math.  Overflow reasoning in a throttle
  * controller is exactly the bug nobody finds until it bites.
  ******************************************************************************
  */

#include "pid.h"

#define I_SCALE   1000000  /* integral accumulator units per output unit */
#define M_SCALE   1000     /* milli-output units per output unit         */
#define I_PER_M   (I_SCALE / M_SCALE)

#define D_LPF_SHIFT_MAX 16

static int32_t clamp64(int64_t v, int32_t lo, int32_t hi) {
  if (v < (int64_t)lo) return lo;
  if (v > (int64_t)hi) return hi;
  return (int32_t)v;
}

void pid_reset(pid_state_t *s) {
  s->i_acc  = 0;
  s->d_filt = 0;
  s->pv_prev = 0;
  s->primed = 0;
}

void pid_prime(pid_state_t *s, int32_t pv) {
  s->pv_prev = pv;
  s->primed  = 1;
}

int32_t pid_step(const pid_cfg_t *c, pid_state_t *s,
                 int32_t sp, int32_t pv, int32_t dt_ms, int32_t ff)
{
  /* The RTOS tick is 1 ms, so two samples in the same tick read as dt = 0. */
  if (dt_ms < 1) dt_ms = 1;

  const int32_t e = sp - pv;

  /* ---- derivative, on the measurement and negated -----------------------
     Updated on every call including inside the deadband, otherwise pv_prev
     goes stale and the first step after leaving the deadband sees a huge
     synthetic rate. */
  int32_t rate_pm = 0;  /* milli-permille per millisecond */
  if (s->primed) {
    rate_pm = (int32_t)(((int64_t)(pv - s->pv_prev) * M_SCALE) / dt_ms);
  }
  s->pv_prev = pv;
  s->primed  = 1;

  const int32_t kp = (e > 0) ? c->kp_m : c->kp_m_rev;
  const int32_t ki = (e > 0) ? c->ki_m : c->ki_m_rev;
  const int32_t kd = (e > 0) ? c->kd_m : c->kd_m_rev;

  const int32_t d_target_m = (int32_t)(-((int64_t)kd * rate_pm) / M_SCALE);
  if (c->d_lpf_shift == 0) {
    s->d_filt = d_target_m;
  } else {
    uint8_t sh = (c->d_lpf_shift > D_LPF_SHIFT_MAX) ? D_LPF_SHIFT_MAX
                                                    : c->d_lpf_shift;
    s->d_filt += (d_target_m - s->d_filt) / (int32_t)(1u << sh);
  }

  /* ---- deadband ---------------------------------------------------------
     Hold on feedforward plus the learned integral, and freeze the integral.
     Without this the loop hunts across the last count or two of TPS noise,
     which on a geared DC motor is audible and wears the plate. */
  if (e >= -c->deadband && e <= c->deadband) {
    return clamp64((int64_t)ff + s->i_acc / I_SCALE, c->out_min, c->out_max);
  }

  /* ---- integral --------------------------------------------------------- */
  const int64_t i_delta = (int64_t)ki * e * dt_ms;   /* micro-output units */

  int64_t i_lim = (int64_t)c->i_max * I_SCALE;
  if (i_lim > INT32_MAX) i_lim = INT32_MAX;

  int64_t i_new = (int64_t)s->i_acc + i_delta;
  if (i_new >  i_lim) i_new =  i_lim;
  if (i_new < -i_lim) i_new = -i_lim;

  const int64_t base_m = (int64_t)ff * M_SCALE + (int64_t)kp * e + s->d_filt;
  const int64_t lo_m   = (int64_t)c->out_min * M_SCALE;
  const int64_t hi_m   = (int64_t)c->out_max * M_SCALE;

  int64_t u_m = base_m + i_new / I_PER_M;

  /* Conditional integration: discard the increment only when it is driving
     further into an already-saturated output.  This is what stops the
     integral winding up and pinning full PWM against a mechanical stop. */
  if ((u_m > hi_m && i_delta > 0) || (u_m < lo_m && i_delta < 0)) {
    i_new = s->i_acc;
    u_m   = base_m + i_new / I_PER_M;
  }
  s->i_acc = (int32_t)i_new;

  return clamp64(u_m / M_SCALE, c->out_min, c->out_max);
}
