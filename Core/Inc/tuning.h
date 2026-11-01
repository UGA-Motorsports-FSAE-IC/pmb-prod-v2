/**
  ******************************************************************************
  * @file    tuning.h
  * @brief   Live parameter block, SWD telemetry ring, and gated setpoint
  *          injection for the Python tuning suite (tools/pmbtune.py).
  ******************************************************************************
  *
  * Two separate things live here.
  *
  * 1. `pmb_params` -- the tunable set, compiled into BOTH Debug and Release and
  *    initialised from constants.h.  Because it is a plain RAM global, the host
  *    can simply write it over SWD; no command protocol is needed to change a
  *    gain.  Costs ~110 bytes of RAM and one extra load per use, which at a
  *    1 kHz control rate is not measurable.
  *
  * 2. The telemetry ring and injection fields -- Debug only (`PMB_TUNING`).
  *    In Release they compile to nothing and the call sites become no-ops, so
  *    there is no #ifdef clutter in app_freertos.c.
  *
  * Transport is deliberately dumb: the firmware writes samples into a RAM ring
  * at the control rate and the host block-reads that ring over SWD with pyocd.
  * No framing, no UART, no protocol to get wrong, works with any ST-Link, and
  * the host sees every sample at full control-loop fidelity rather than
  * whatever a 115200 baud link can carry.
  ******************************************************************************
  */

#ifndef __TUNING_H
#define __TUNING_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* ------------------------------------------------------------------ params
 * Every field is int32_t on purpose: the Python side decodes the whole struct
 * as one array of little-endian int32, so there is no padding or layout to get
 * out of sync between the two languages.
 *
 * Units and conventions are exactly those of pid.h and the MEASURE ME block in
 * constants.h -- gains stored x1000.
 */
typedef struct {
  /* throttle position loop */
  int32_t kp_fwd, ki_fwd, kd_fwd;       /* opening */
  int32_t kp_rev, ki_rev, kd_rev;       /* closing, above THROTTLE_LOW_REGION */
  int32_t kp_rev_low, ki_rev_low;       /* closing, below it */
  int32_t i_max, deadband, d_lpf_shift;
  int32_t ff_slope, ff_offset;          /* return-spring feedforward */
  int32_t stiction_pwm, dir_hysteresis;
  int32_t soft_min, soft_max;           /* travel limits, permille */

  /* idle rpm loop */
  int32_t idle_kp, idle_ki, idle_kd, idle_kp_save, idle_i_max;
  int32_t idle_ff_hot, idle_ff_cold;

  /* downshift blip schedule */
  int32_t blip_target_low, blip_target_high;
} pmb_params_t;

extern pmb_params_t pmb_params;

/** Reload pmb_params from the compile-time constants. */
void pmb_params_defaults(void);

/* Injection modes.  Defined in both builds so the call sites read the same
 * either way; in Release pmb_inject_get() always returns 0. */
#define PMB_INJECT_OFF        0
#define PMB_INJECT_POSITION   1   /* value = target position, permille */
#define PMB_INJECT_PWM        2   /* value = raw signed PWM, open loop */

#ifdef PMB_TUNING

#define PMB_TUNE_MAGIC    0x544D4250u   /* 'PBMT' little-endian */
#define PMB_TUNE_VERSION  1u
#define PMB_LOG_SAMPLES   256u          /* must stay a power of two */
#define PMB_INJECT_KEY    0x5A33C0DEu
#define PMB_INJECT_HEARTBEAT_TIMEOUT_MS 250u

#define PMB_INJECT_OFF        0
#define PMB_INJECT_POSITION   1   /* value = target position, permille */
#define PMB_INJECT_PWM        2   /* value = raw signed PWM, open loop */

/* 16 bytes. 256 samples = 4 KB = 256 ms of history at 1 kHz. */
typedef struct {
  uint32_t t_ms;
  int16_t  sp;
  int16_t  pv;
  int16_t  pv2;
  int16_t  u;
  uint16_t rpm;
  int16_t  pedal;
} pmb_sample_t;

typedef struct {
  uint32_t magic;
  uint32_t version;
  uint32_t sample_count;
  uint32_t sample_size;
  volatile uint32_t write_idx;   /* free-running; index = write_idx % count */

  /* ---- injection.  All four conditions must hold or injection is off:
     the key matches, mode is non-zero, the host heartbeat is fresh, and the
     value survives clamping to the soft travel limits. */
  volatile uint32_t inject_key;
  volatile uint32_t heartbeat;   /* host bumps this; stale => auto-disable */
  volatile int32_t  inject_mode;
  volatile int32_t  inject_value;
  volatile int32_t  pwm_ceiling; /* hard cap on |PWM| while injecting */
} pmb_tune_t;

extern pmb_tune_t   pmb_tune;
extern pmb_sample_t pmb_log[PMB_LOG_SAMPLES];

void pmb_tune_init(void);

void pmb_log_push(uint32_t t_ms, int32_t sp, int32_t pv, int32_t pv2,
                  int32_t u, uint32_t rpm, int32_t pedal);

/**
 * @return 1 if injection is live, in which case mode, value and pwm_ceiling
 *         are filled in.
 *
 * Auto-disables (and latches off) if the host stops bumping the heartbeat, so
 * a crashed script or an unplugged probe cannot leave the throttle commanded.
 */
int pmb_inject_get(uint32_t now_ms, int32_t *mode, int32_t *value,
                   int32_t *pwm_ceiling);

#else /* !PMB_TUNING -- Release: everything above compiles out */

static inline void pmb_tune_init(void) { }

static inline void pmb_log_push(uint32_t t_ms, int32_t sp, int32_t pv,
                                int32_t pv2, int32_t u, uint32_t rpm,
                                int32_t pedal) {
  (void)t_ms; (void)sp; (void)pv; (void)pv2; (void)u; (void)rpm; (void)pedal;
}

static inline int pmb_inject_get(uint32_t now_ms, int32_t *mode, int32_t *value,
                                 int32_t *pwm_ceiling) {
  (void)now_ms; (void)mode; (void)value; (void)pwm_ceiling;
  return 0;
}

#endif /* PMB_TUNING */

#ifdef __cplusplus
}
#endif
#endif /* __TUNING_H */
