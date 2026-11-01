/**
 * Host-side self-check for the control math.  No framework: assert() and a
 * main().  Run it with tests/run.sh.
 *
 * pid.c and constants.h have no HAL dependency specifically so this can exist.
 * Each case targets one defect that the previous firmware had, so a regression
 * fails loudly rather than quietly going back to being untunable.
 */

#include "pid.h"
#include "constants.h"

#include <assert.h>
#include <stdio.h>

static pid_cfg_t base_cfg(void)
{
  pid_cfg_t c = {0};
  c.out_min = -1000;
  c.out_max = 1000;
  c.i_max   = 1000;
  return c;
}

/* ------------------------------------------------------------------------
 * The integral dead zone.
 *
 * The old loop did `integral += (dt * e * Ki) / 1000` in whole output counts.
 * With Ki = 35 and dt = 1 that truncates to zero for every |e| < 29, so the
 * integral was dead across the entire band being tuned.
 */
static void test_no_integral_dead_zone(void)
{
  /* First, demonstrate the old bug still arithmetically exists. */
  assert((1 * 1 * PID_FORWARD_I) / 1000 == 0);   /* e = 1   -> no integration */
  assert((1 * 28 * PID_FORWARD_I) / 1000 == 0);  /* e = 28  -> no integration */
  assert((1 * 29 * PID_FORWARD_I) / 1000 == 1);  /* e = 29  -> finally moves  */

  pid_cfg_t c = base_cfg();
  c.ki_m = PID_FORWARD_I;   /* 35 */

  pid_state_t s;
  pid_reset(&s);

  /* An error of exactly 1 permille must still accumulate. */
  for (int i = 0; i < 100; i++) {
    (void)pid_step(&c, &s, 1, 0, 1, 0);
  }
  assert(s.i_acc == 100 * PID_FORWARD_I);

  /* And it must eventually reach the output, rather than truncating forever.
   * 1e6 micro-output units per output count / 35 per ms ~= 28572 ms. */
  pid_reset(&s);
  int32_t u = 0;
  for (int i = 0; i < 30000; i++) {
    u = pid_step(&c, &s, 1, 0, 1, 0);
  }
  assert(u >= 1);
}

/* ------------------------------------------------------------------------
 * Integral clamp and conditional anti-windup.  This is the jam: an unbounded
 * integral pins full PWM into a mechanical stop and the plate wedges.
 */
static void test_integral_clamp(void)
{
  pid_cfg_t c = base_cfg();
  c.ki_m  = 100000;
  c.i_max = 500;

  pid_state_t s;
  pid_reset(&s);

  for (int i = 0; i < 1000; i++) {
    int32_t u = pid_step(&c, &s, 1000, 0, 1, 0);
    assert(u <= c.out_max);
    assert(u >= c.out_min);
  }
  assert(s.i_acc == 500 * 1000000); /* clamped at i_max, not running away */
}

static void test_antiwindup_holds_integral(void)
{
  pid_cfg_t c = base_cfg();
  c.kp_m   = 1000;    /* P alone already saturates */
  c.ki_m   = 100000;
  c.out_max = 100;
  c.out_min = -100;

  pid_state_t s;
  pid_reset(&s);

  for (int i = 0; i < 100; i++) {
    int32_t u = pid_step(&c, &s, 1000, 0, 1, 0);
    assert(u == 100);
  }
  /* Output was saturated the whole time and the increment pushed further into
   * that saturation, so nothing should have accumulated. */
  assert(s.i_acc == 0);
}

/* ------------------------------------------------------------------------
 * Bumpless gain switching.  The old loop zeroed the integral on every error
 * sign change, so against a return spring it could never settle.
 */
static void test_gain_switch_is_bumpless(void)
{
  pid_cfg_t c = base_cfg();
  c.kp_m = 1000;  c.ki_m = 1000;
  c.kp_m_rev = 100; c.ki_m_rev = 0;   /* deliberately very different */

  pid_state_t s;
  pid_reset(&s);

  for (int i = 0; i < 50; i++) {
    (void)pid_step(&c, &s, 600, 500, 1, 0);   /* error +100, opening */
  }
  const int32_t i_before = s.i_acc;
  assert(i_before > 0);

  /* Cross to a negative error.  ki_m_rev is 0, so the integral must be exactly
   * preserved -- not reset. */
  (void)pid_step(&c, &s, 400, 500, 1, 0);
  assert(s.i_acc == i_before);
}

/* ------------------------------------------------------------------------
 * Derivative on measurement, not on error: a setpoint step must not kick.
 */
static void test_no_derivative_kick(void)
{
  pid_cfg_t c = base_cfg();
  c.kd_m = 1000;
  c.kd_m_rev = 1000;
  c.d_lpf_shift = 0;   /* unfiltered, so the number is exact */

  pid_state_t s;
  pid_reset(&s);
  pid_prime(&s, 500);

  /* Setpoint jumps 500 -> 600 while the plate has not moved. */
  int32_t u = pid_step(&c, &s, 600, 500, 1, 0);
  assert(u == 0);   /* old code: (e - e_prev) * Kd / dt, i.e. a 100-unit kick */

  /* Now the plate actually moves 10 permille in 1 ms, setpoint unchanged.
   * rate = 10 permille/ms, Kd = 1.0  ->  D = -10 output, opposing the motion. */
  u = pid_step(&c, &s, 600, 510, 1, 0);
  assert(u == -10);
}

/* ------------------------------------------------------------------------
 * Deadband holds on feedforward plus the learned integral, and freezes the
 * integral so it cannot drift while inside the band.
 */
static void test_deadband(void)
{
  pid_cfg_t c = base_cfg();
  c.kp_m = 1000;
  c.ki_m = 1000;
  c.deadband = 10;

  pid_state_t s;
  pid_reset(&s);

  int32_t u = pid_step(&c, &s, 505, 500, 1, 42);   /* |e| = 5 <= 10 */
  assert(u == 42);          /* feedforward only */
  assert(s.i_acc == 0);     /* frozen */

  u = pid_step(&c, &s, 520, 500, 1, 42);           /* |e| = 20 > 10 */
  assert(u != 42);
  assert(s.i_acc > 0);
}

static void test_output_clamp(void)
{
  pid_cfg_t c = base_cfg();
  c.kp_m = 100000;
  c.kp_m_rev = 100000;

  pid_state_t s;
  pid_reset(&s);

  assert(pid_step(&c, &s, 1000, 0, 1, 0) == 1000);
  pid_reset(&s);
  assert(pid_step(&c, &s, 0, 1000, 1, 0) == -1000);
}

static void test_dt_zero_is_safe(void)
{
  pid_cfg_t c = base_cfg();
  c.kd_m = 1000;

  pid_state_t s;
  pid_reset(&s);
  pid_prime(&s, 500);

  /* Two samples inside the same 1 ms tick read as dt = 0; must not divide by
   * zero and must not produce an absurd rate. */
  int32_t u = pid_step(&c, &s, 600, 501, 0, 0);
  assert(u >= c.out_min && u <= c.out_max);
}

/* ------------------------------------------------------------------------
 * Pedal map: the old QUADRATIC form returned 1008 at full pedal, i.e. it
 * commanded 0.8% past the open hard stop.
 */
static void test_throttle_map(void)
{
  assert(THROTTLE_MAP(0) == 0);
  assert(THROTTLE_MAP(1000) == 1000);

  /* Monotonic and in range across the whole pedal travel. */
  int32_t prev = -1;
  for (int32_t x = 0; x <= 1000; x++) {
    int32_t y = THROTTLE_MAP(x);
    assert(y >= 0 && y <= 1000);
    assert(y >= prev);
    prev = y;
  }

  /* Shape is preserved against the old curve to within a couple of counts. */
  assert(THROTTLE_MAP(500) >= 248 && THROTTLE_MAP(500) <= 258);

  /* The old form, for contrast. */
  assert(QUADRATIC(1000, 100, 850, 0) == 1008);
}

static void test_target_clamping(void)
{
  assert(CLAMP_THROTTLE_TARGET(-50) == THROTTLE_SOFT_MIN);
  assert(CLAMP_THROTTLE_TARGET(5000) == THROTTLE_SOFT_MAX);
  assert(CLAMP_THROTTLE_TARGET(500) == 500);

  /* Closure must not command the closed hard stop. */
  assert(THROTTLE_CLOSURE_TARGET == THROTTLE_SOFT_MIN);
  assert(THROTTLE_CLOSURE_TARGET > 0);

  assert(CLAMP_PERMILLE(-7) == 0);
  assert(CLAMP_PERMILLE(1234) == 1000);
  assert(CLAMP_PERMILLE(600) == 600);
}

/* ------------------------------------------------------------------------
 * CAN accessors: the little-endian one used to cast uint8_t* to uint16_t*,
 * which is a strict-aliasing violation and would fault on an odd offset.
 */
static void test_can_accessors(void)
{
  const uint8_t d[8] = { 0x34, 0x12, 0x78, 0x56, 0xAA, 0xBB, 0xCD, 0xAB };

  assert(GET_16BIT_LITTLEENDIAN_CAN_VALUE(d, 0) == 0x1234);
  assert(GET_16BIT_LITTLEENDIAN_CAN_VALUE(d, 2) == 0x5678);
  assert(GET_16BIT_BIGENDIAN_CAN_VALUE(d, 6) == 0xCDAB);
  assert(GET_8BIT_CAN_VALUE(d, 4) == 0xAA);

  /* Odd offset: well-defined now, a hard fault on Cortex-M0+ before. */
  assert(GET_16BIT_LITTLEENDIAN_CAN_VALUE(d, 1) == 0x7812);

  /* CUBIC was missing a closing parenthesis and would not compile if used.
   * Referencing it here keeps that from coming back. */
  assert(CUBIC(0, 1, 1, 1, 0) == 0);
}

int main(void)
{
  test_no_integral_dead_zone();
  test_integral_clamp();
  test_antiwindup_holds_integral();
  test_gain_switch_is_bumpless();
  test_no_derivative_kick();
  test_deadband();
  test_output_clamp();
  test_dt_zero_is_safe();
  test_throttle_map();
  test_target_clamping();
  test_can_accessors();

  printf("all pid/control math checks passed\n");
  return 0;
}
