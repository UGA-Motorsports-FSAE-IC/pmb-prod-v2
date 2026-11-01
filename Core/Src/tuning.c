/**
  ******************************************************************************
  * @file    tuning.c
  * @brief   See tuning.h.
  ******************************************************************************
  */

#include "tuning.h"
#include "constants.h"
#include "main.h"   /* for __DMB() via CMSIS */

/* The live tunable set.  Initialised from constants.h; the host may overwrite
 * any field over SWD in the Debug build.  Present in Release too so there is
 * exactly one definition of "the current gains" in the firmware. */
pmb_params_t pmb_params;

void pmb_params_defaults(void)
{
  pmb_params.kp_fwd        = PID_FORWARD_P;
  pmb_params.ki_fwd        = PID_FORWARD_I;
  pmb_params.kd_fwd        = PID_FORWARD_D;
  pmb_params.kp_rev        = PID_BACKWARD_P;
  pmb_params.ki_rev        = PID_BACKWARD_I;
  pmb_params.kd_rev        = PID_BACKWARD_D;
  pmb_params.kp_rev_low    = PID_LOW_BACKWARD_P;
  pmb_params.ki_rev_low    = PID_LOW_BACKWARD_I;
  pmb_params.i_max         = THROTTLE_PID_I_MAX;
  pmb_params.deadband      = THROTTLE_PID_DEADBAND;
  pmb_params.d_lpf_shift   = THROTTLE_D_LPF_SHIFT;
  pmb_params.ff_slope      = THROTTLE_FF_SLOPE;
  pmb_params.ff_offset     = THROTTLE_FF_OFFSET;
  pmb_params.stiction_pwm  = THROTTLE_STICTION_PWM;
  pmb_params.dir_hysteresis = THROTTLE_DIR_HYSTERESIS;
  pmb_params.soft_min      = THROTTLE_SOFT_MIN;
  pmb_params.soft_max      = THROTTLE_SOFT_MAX;

  pmb_params.idle_kp       = PID_RPM_P;
  pmb_params.idle_ki       = PID_RPM_I;
  pmb_params.idle_kd       = PID_RPM_D;
  pmb_params.idle_kp_save  = PID_RPM_SAVE_P;
  pmb_params.idle_i_max    = PID_RPM_I_MAX;
  pmb_params.idle_ff_hot   = IDLE_FF_HOT;
  pmb_params.idle_ff_cold  = IDLE_FF_COLD;

  pmb_params.blip_target_low  = SHIFT_BLIP_TARGET_LOW_RPM;
  pmb_params.blip_target_high = SHIFT_BLIP_TARGET_HIGH_RPM;
}

#ifdef PMB_TUNING

pmb_sample_t pmb_log[PMB_LOG_SAMPLES];

pmb_tune_t pmb_tune = {
  .magic        = PMB_TUNE_MAGIC,
  .version      = PMB_TUNE_VERSION,
  .sample_count = PMB_LOG_SAMPLES,
  .sample_size  = sizeof(pmb_sample_t),
};

void pmb_tune_init(void)
{
  pmb_tune.magic        = PMB_TUNE_MAGIC;
  pmb_tune.version      = PMB_TUNE_VERSION;
  pmb_tune.sample_count = PMB_LOG_SAMPLES;
  pmb_tune.sample_size  = sizeof(pmb_sample_t);
  pmb_tune.write_idx    = 0;
  pmb_tune.inject_key   = 0;
  pmb_tune.inject_mode  = PMB_INJECT_OFF;
  pmb_tune.inject_value = 0;
  pmb_tune.heartbeat    = 0;
  pmb_tune.pwm_ceiling  = 0;   /* host must raise this before it can drive */
}

static int16_t sat16(int32_t v)
{
  if (v >  32767) return  32767;
  if (v < -32768) return -32768;
  return (int16_t)v;
}

void pmb_log_push(uint32_t t_ms, int32_t sp, int32_t pv, int32_t pv2,
                  int32_t u, uint32_t rpm, int32_t pedal)
{
  const uint32_t i = pmb_tune.write_idx & (PMB_LOG_SAMPLES - 1u);

  pmb_log[i].t_ms  = t_ms;
  pmb_log[i].sp    = sat16(sp);
  pmb_log[i].pv    = sat16(pv);
  pmb_log[i].pv2   = sat16(pv2);
  pmb_log[i].u     = sat16(u);
  pmb_log[i].rpm   = (uint16_t)((rpm > 65535u) ? 65535u : rpm);
  pmb_log[i].pedal = sat16(pedal);

  /* Publish the index only after the payload is in memory, so a host read that
   * races the writer can never see a half-written sample. */
  __DMB();
  pmb_tune.write_idx = pmb_tune.write_idx + 1u;
}

int pmb_inject_get(uint32_t now_ms, int32_t *mode, int32_t *value,
                   int32_t *pwm_ceiling)
{
  static uint32_t last_hb = 0;
  static uint32_t last_hb_ms = 0;
  static uint8_t  hb_seen = 0;

  const uint32_t hb = pmb_tune.heartbeat;
  if (!hb_seen || hb != last_hb) {
    last_hb    = hb;
    last_hb_ms = now_ms;
    hb_seen    = 1;
  }

  if (pmb_tune.inject_key != PMB_INJECT_KEY) return 0;
  if (pmb_tune.inject_mode == PMB_INJECT_OFF) return 0;

  /* Host went away mid-session: latch injection off rather than hold whatever
   * it last commanded. */
  if ((uint32_t)(now_ms - last_hb_ms) > PMB_INJECT_HEARTBEAT_TIMEOUT_MS) {
    pmb_tune.inject_mode = PMB_INJECT_OFF;
    pmb_tune.inject_key  = 0;
    return 0;
  }

  *mode        = pmb_tune.inject_mode;
  *value       = pmb_tune.inject_value;
  *pwm_ceiling = pmb_tune.pwm_ceiling;
  return 1;
}

#endif /* PMB_TUNING */
