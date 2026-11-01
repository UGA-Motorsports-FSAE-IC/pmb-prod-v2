/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : FreeRTOS applicative file
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "app_freertos.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* board.h pulls in main.h (the family HAL header) plus the peripheral and pin
 * aliases.  The family-specific headers that used to be included here
 * (stm32c092xx.h, stm32c0xx_hal_*.h) are deliberately gone: nothing in this
 * file should need to know which STM32 family it is running on. */
#include "board.h"
#include "cmsis_os2.h"
#include "fdcanmessage.h"
#include "constants.h"
#include "pid.h"
#include "tuning.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

volatile uint32_t apps1;
volatile uint32_t apps2;
volatile uint32_t tps1;
volatile uint32_t tps2;
volatile uint32_t bse1;
volatile uint32_t bse2;

volatile uint32_t bspd;

volatile uint8_t shiftnumber = 1;
volatile uint8_t shiftdir = 0;
volatile uint8_t currentshiftnum = 1;
volatile uint32_t mostrecentshift;

volatile uint32_t rpm;
volatile uint32_t coolant;

volatile int actual_throttle_position;  //number from 0 to 1000 representing 0% to 100% actuation
volatile int actual_throttle_position_2nd_sensor; //same thing but calculated from other tps
volatile int target_idle_throttle_position;
volatile int target_throttle_position;

volatile int gas_pedal_position;  //number from 0 to 1000 representing 0% to 100%
volatile int gas_pedal_position_2nd_sensor; //same thing but calculated from other apps

volatile int brake_depression_percentage;
volatile int brake_depression_percentage_2nd_sensor;

volatile uint32_t apps1_updation;
volatile uint32_t apps2_updation;
volatile uint32_t tps1_updation;
volatile uint32_t tps2_updation;
volatile uint32_t bs1_updation;
volatile uint32_t bs2_updation;
volatile uint8_t sensor_data_implausibility = 0;
volatile uint8_t sensor_can_reception_implausibility = 0;
volatile uint8_t throttle_and_brakes_on_implausibility = 0;
volatile uint8_t throttle_not_at_intended_implausibility = 0;
volatile uint8_t shutdownrelay = 0;
volatile uint8_t throttlerelay = 0;
volatile uint8_t useapps = 1;

/* Downshift blip.
 *
 * This used to be a bare `shiftblipactive` flag held set by paddleshift() while
 * it slept through the blip.  If paddleshift was preempted or faulted mid-blip
 * the flag stayed set and the throttle held 80% open indefinitely.
 *
 * It is now a deadline owned by the throttle loop: paddleshift arms it, and the
 * throttle loop stops honouring it once the tick passes, whatever else happens
 * to paddleshift. */
volatile uint32_t blip_until_tick = 0;
volatile int shiftbliptarget;

/* Shutdown circuit (PA8) request flags.  See shutdown_relay_update(). */
volatile uint8_t fault_cut_active = 0;   /* implausibility wants the car dead */
volatile uint8_t shift_cut_active = 0;   /* upshift ignition cut in progress  */

/* Set when the jam guard has exhausted its unjam retries. Feeds the
 * implausibility task, which opens the throttle relay and lets the return
 * spring close the plate. */
volatile uint8_t throttle_jam_fault = 0;

/* Last applied motor command, signed, positive = opening. Telemetry only. */
volatile int32_t throttle_pwm_applied = 0;

extern osMessageQueueId_t canqueue;

static int32_t iabs32(int32_t v) { return (v < 0) ? -v : v; }

/**
 * Single computed owner for the shutdown relay.
 *
 * Previously both sensorImplausibilityMonitoring() and paddleshift() wrote PA8
 * directly.  The implausibility task re-drives that pin every 10 ms, so the
 * 200 ms ignition cut paddleshift() asked for was cancelled within 10 ms and
 * upshift cuts effectively never happened.  Both tasks now set their own
 * request flag and call this, so whoever calls last computes the same answer.
 */
static void shutdown_relay_update(void)
{
  const uint8_t cut = (fault_cut_active || shift_cut_active);
  SHUTDOWN_CIRCUIT_SET(!cut);
  shutdownrelay = cut;
}

/* USER CODE END Variables */
/* Definitions for sensorImplausibilityMonitoring */
osThreadId_t sensorImplausibilityMonitoringHandle;
const osThreadAttr_t sensorImplausibilityMonitoring_attributes = {
  .name = "sensorImplausibilityMonitoring",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for readsensors */
osThreadId_t readsensorsHandle;
const osThreadAttr_t readsensors_attributes = {
  .name = "readsensors",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for paddleshift */
osThreadId_t paddleshiftHandle;
const osThreadAttr_t paddleshift_attributes = {
  .name = "paddleshift",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 128 * 4
};
/* Definitions for throttlePositionControl */
osThreadId_t throttlePositionControlHandle;
const osThreadAttr_t throttlePositionControl_attributes = {
  .name = "throttlePositionControl",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for serialMonitoring */
osThreadId_t serialMonitoringHandle;
const osThreadAttr_t serialMonitoring_attributes = {
  .name = "serialMonitoring",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512 * 4
};
/* Definitions for rpmIdle */
osThreadId_t rpmIdleHandle;
const osThreadAttr_t rpmIdle_attributes = {
  .name = "rpmIdle",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  pmb_params_defaults();   /* load the live tunable set from constants.h */
  pmb_tune_init();         /* no-op in Release */
  LED_SET(1);
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
  /* creation of sensorImplausibilityMonitoring */
  sensorImplausibilityMonitoringHandle = osThreadNew(sensorImplausibilityMonitoring, NULL, &sensorImplausibilityMonitoring_attributes);

  /* creation of readsensors */
  readsensorsHandle = osThreadNew(readsensordata, NULL, &readsensors_attributes);

  /* creation of paddleshift */
  paddleshiftHandle = osThreadNew(paddleshift, NULL, &paddleshift_attributes);

  /* creation of throttlePositionControl */
  throttlePositionControlHandle = osThreadNew(throttlePositionControl, NULL, &throttlePositionControl_attributes);

  /* creation of serialMonitoring */
  serialMonitoringHandle = osThreadNew(serialMonitoring, NULL, &serialMonitoring_attributes);

  /* creation of rpmIdle */
  rpmIdleHandle = osThreadNew(rpmIdle, NULL, &rpmIdle_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}
/* USER CODE BEGIN Header_sensorImplausibilityMonitoring */
/**
* @brief Function implementing the sensorImplausibilityMonitoring thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_sensorImplausibilityMonitoring */
void sensorImplausibilityMonitoring(void *argument)
{
  /* USER CODE BEGIN sensorImplausibilityMonitoring */
  /* Infinite loop */


  osDelay(500);

  uint32_t potential_sensor_issue_duration = 0;
  uint32_t sensor_normal_function_duration = 0;
  uint32_t potential_throttlebody_issue_duration = 0;
  uint32_t throttlebody_normal_function_duration = 0;
  uint32_t potential_throttle_stuck_duration = 0;
  uint32_t throttlebody_not_stuck_duration = 0;

  uint32_t currenttick = osKernelGetTickCount();
  for(;;)
  {

    //the following code checks if there is any issue with the received sensor values

    uint8_t apps_issue = apps1 < APPS1_LB || apps1 > APPS1_UB || apps2 < APPS2_LB || apps2 > APPS2_UB || (abs(gas_pedal_position - gas_pedal_position_2nd_sensor) > MAX_GAS_PEDAL_SENSORS_DEVIATION);
    uint8_t tps_issue = tps1 < TPS1_LB || tps1 > TPS1_UB || tps2 < TPS2_LB || tps2 > TPS2_UB || (abs(actual_throttle_position - actual_throttle_position_2nd_sensor) > MAX_THROTTLE_SENSORS_DEVIATION);
    uint8_t bse_issue = bse1 < BS1_LB || bse1 > BS1_UB || bse2 < BS2_LB || bse2 > BS2_UB; 
    if (apps_issue || tps_issue || bse_issue) {
      sensor_normal_function_duration = 0;
      potential_sensor_issue_duration += IMPLAUSIBILITY_CHECK_INTERVAL;    
    } else {
      potential_sensor_issue_duration = 0;
      sensor_normal_function_duration += IMPLAUSIBILITY_CHECK_INTERVAL;
    }
    if (potential_sensor_issue_duration > SENSOR_IMPLAUSIBILITY_TIMEOUT) {
      sensor_data_implausibility = 1;
    }
    if (sensor_normal_function_duration > SENSOR_BACK_TO_NORMAL_TIMEOUT) {
      sensor_data_implausibility = 0;
    }

    //the following code checks if there is issues with the data retrieval of sensors over CAN

    /* Sensor ages must be measured against the real tick, not against
     * `currenttick`.  That variable is the osDelayUntil schedule anchor and is
     * advanced by a fixed IMPLAUSIBILITY_CHECK_INTERVAL every pass, so it
     * drifts behind the real clock as soon as this task overruns even once --
     * and a drifting clock makes a sensor-timeout check read young. */
    uint32_t nowtick = osKernelGetTickCount();

    sensor_can_reception_implausibility = (nowtick - apps1_updation > SENSOR_IMPLAUSIBILITY_TIMEOUT) || (nowtick - apps2_updation > SENSOR_IMPLAUSIBILITY_TIMEOUT) ||
                (nowtick - tps1_updation > SENSOR_IMPLAUSIBILITY_TIMEOUT) || (nowtick - tps2_updation > SENSOR_IMPLAUSIBILITY_TIMEOUT) ||
                (nowtick - bs1_updation > SENSOR_IMPLAUSIBILITY_TIMEOUT) || (nowtick - bs2_updation > SENSOR_IMPLAUSIBILITY_TIMEOUT);


    //the following code checks for if the throttle body flap is not actuating to the intended position
    
    if (throttle_jam_fault ||
        abs(actual_throttle_position - target_throttle_position) > MAX_THROTTLE_INTENDED_VS_ACTUAL_DEVIATION) {
      potential_throttlebody_issue_duration += IMPLAUSIBILITY_CHECK_INTERVAL;
      throttlebody_normal_function_duration = 0;
    } else {
      throttlebody_normal_function_duration += IMPLAUSIBILITY_CHECK_INTERVAL;
      potential_throttlebody_issue_duration = 0;
    }
    if (potential_throttlebody_issue_duration > THROTTLEBODY_FAILURE_TIMEOUT) {
      throttle_not_at_intended_implausibility = 1;
    }
    if (throttlebody_normal_function_duration > THROTTLEBODY_BACK_TO_NORMAL_TIMEOUT) {
      throttle_not_at_intended_implausibility = 0;
    }
    

    //the following code checks if there is hard braking and open throttle at the same time

    throttle_and_brakes_on_implausibility = brake_depression_percentage > HARD_BRAKING_PERCENTAGE_THRESHOLD;

    //the following will shut down the throttle body power if there is any issue with the sensors
    if (sensor_can_reception_implausibility || sensor_data_implausibility || throttle_not_at_intended_implausibility) {
      THROTTLE_POWER_SET(0);
      throttlerelay = 1;
    } else {
      THROTTLE_POWER_SET(1);
      throttlerelay = 0;
    }


    if (throttle_not_at_intended_implausibility) {
      potential_throttle_stuck_duration += IMPLAUSIBILITY_CHECK_INTERVAL;
      throttlebody_not_stuck_duration = 0;
    } else {
      throttlebody_not_stuck_duration += IMPLAUSIBILITY_CHECK_INTERVAL;
      potential_throttle_stuck_duration = 0;
    }
    if (potential_throttle_stuck_duration > OPEN_SHUTDOWN_CIRCUIT_TIMEOUT) {
      fault_cut_active = 1;
    }
    if (throttlebody_not_stuck_duration > CLOSE_SHUTDOWN_CIRCUIT_TIMEOUT) {
      fault_cut_active = 0;
    }

    /* Request flags only -- the pin itself is written in one place so this task
     * can no longer stomp on an in-progress upshift ignition cut. */
    shutdown_relay_update();

    currenttick += IMPLAUSIBILITY_CHECK_INTERVAL;
    osDelayUntil(currenttick);
  }
  /* USER CODE END sensorImplausibilityMonitoring */
}

/* USER CODE BEGIN Header_readsensordata */
/**
* @brief Function implementing the readsensors thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_readsensordata */
void readsensordata(void *argument)
{
  /* USER CODE BEGIN readsensors */
  /* Infinite loop */
  uint64_t currenttick = osKernelGetTickCount();
  apps1_updation = currenttick;
  apps2_updation = currenttick;
  tps1_updation = currenttick;
  tps2_updation = currenttick;
  bs1_updation = currenttick;
  bs2_updation = currenttick;

  rxmessage currentmessage;

  for(;;)
  {
    osMessageQueueGet(canqueue, &currentmessage, NULL, osWaitForever);

    currenttick = osKernelGetTickCount();

    uint8_t * candata = currentmessage.canrxdata;
    uint32_t id = currentmessage.rxheader.Identifier;

    uint8_t tps_fresh = 0;

    if (id == APPS1_CAN_ID) {
      apps1 = GET_16BIT_LITTLEENDIAN_CAN_VALUE(candata, APPS1_CAN_OFFSET);
      apps1_updation = currenttick;
    }
    if (id == APPS2_CAN_ID) {
      apps2 = GET_16BIT_LITTLEENDIAN_CAN_VALUE(candata, APPS2_CAN_OFFSET);
      apps2_updation = currenttick;      
    }
    if (id == TPS1_CAN_ID) {
      tps1 = GET_16BIT_LITTLEENDIAN_CAN_VALUE(candata, TPS1_CAN_OFFSET);
      tps1_updation = currenttick;
      tps_fresh = 1;
    }
    if (id == TPS2_CAN_ID) {
      tps2 = GET_16BIT_LITTLEENDIAN_CAN_VALUE(candata, TPS2_CAN_OFFSET);
      tps2_updation = currenttick;
    }
    if (id == BS1_CAN_ID) {
      bse1 = GET_16BIT_LITTLEENDIAN_CAN_VALUE(candata, BS1_CAN_OFFSET);
      bs1_updation = currenttick;
    }
    if (id == BS2_CAN_ID) {
      bse2 = GET_16BIT_LITTLEENDIAN_CAN_VALUE(candata, BS2_CAN_OFFSET);
      bs2_updation = currenttick;
    }
    if (id == RPM_CAN_ID) {
      rpm = GET_16BIT_BIGENDIAN_CAN_VALUE(candata, RPM_CAN_OFFSET);
    }
    if (id == SHIFT_ID) {
      shiftnumber = GET_8BIT_CAN_VALUE(candata, SHIFT_COUNT_OFFSET);
      shiftdir = GET_8BIT_CAN_VALUE(candata, SHIFT_DIR_OFFSET);
    }
    if (id == BSPD_CAN_ID) {
      bspd = GET_16BIT_LITTLEENDIAN_CAN_VALUE(candata, BSPD_CAN_OFFSET);
    }
    if (id == COOLANT_CAN_ID) {
      coolant = GET_16BIT_BIGENDIAN_CAN_VALUE(candata, COOLANT_CAN_OFFSET) / COOLANT_CAN_DIVIDE;
    }

    //Ceiling and floor checks which map apps value to operational bound if value out 
    // of operational bound but inside absolute bounds.
    if (apps1 < APPS1_OPERATION_LB && apps1 > APPS1_LB) {
      apps1 = APPS1_OPERATION_LB;
    } else if (apps1 > APPS1_OPERATION_UB && apps1 < APPS1_UB) {
      apps1 = APPS1_OPERATION_UB;
    }
    if (apps2 < APPS2_OPERATION_LB && apps2 > APPS2_LB) {
      apps2 = APPS2_OPERATION_LB;
    } else if (apps2 > APPS2_OPERATION_UB && apps2 < APPS2_UB) {
      apps2 = APPS2_OPERATION_UB;
    }

    /* All of these are clamped into 0..1000.  LINEAR truncates toward zero and
     * returns negative values just outside the calibrated span, and a negative
     * "position" has no meaning in the control law downstream. */
    actual_throttle_position = CLAMP_PERMILLE(LINEAR((int)tps1, RAW_TPS1_TO_THROTTLE_PERCENTAGE_SLOPE, RAW_TPS1_TO_THROTTLE_PERCENTAGE_INTERCEPT));
    gas_pedal_position = CLAMP_PERMILLE(LINEAR((int)apps1, RAW_APPS1_TO_PEDAL_PERCENTAGE_SLOPE, RAW_APPS1_TO_PEDAL_PERCENTAGE_INTERCEPT));

    actual_throttle_position_2nd_sensor = CLAMP_PERMILLE(LINEAR((int)tps2, RAW_TPS2_TO_THROTTLE_PERCENTAGE_SLOPE, RAW_TPS2_TO_THROTTLE_PERCENTAGE_INTERCEPT));
    gas_pedal_position_2nd_sensor = CLAMP_PERMILLE(LINEAR((int)apps2, RAW_APPS2_TO_PEDAL_PERCENTAGE_SLOPE, RAW_APPS2_TO_PEDAL_PERCENTAGE_INTERCEPT));

    brake_depression_percentage = CLAMP_PERMILLE(LINEAR((int)bse1, RAW_BS1_TO_BRAKE_PERCENTAGE_SLOPE, RAW_BS1_TO_BRAKE_PERCENTAGE_INTERCEPT));
    brake_depression_percentage_2nd_sensor = CLAMP_PERMILLE(LINEAR((int)bse2, RAW_BS2_TO_BRAKE_PERCENTAGE_SLOPE, RAW_BS2_TO_BRAKE_PERCENTAGE_INTERCEPT));

    /* Drive the control loop from sensor arrival rather than from a fixed 1 kHz
     * schedule that mostly re-processes stale values. */
    if (tps_fresh && throttlePositionControlHandle != NULL) {
      osThreadFlagsSet(throttlePositionControlHandle, THROTTLE_TPS_EVENT_FLAG);
    }
  }
  /* USER CODE END readsensors */
}

/* USER CODE BEGIN Header_paddleshift */
/**
* @brief Function implementing the paddleshift thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_paddleshift */
void paddleshift(void *argument)
{
  /* USER CODE BEGIN paddleshift */
  /* Infinite loop */

  mostrecentshift = 0;

  for(;;)
  {
    osDelay(1);

    if (currentshiftnum != shiftnumber) {
      currentshiftnum = shiftnumber;
      if ((osKernelGetTickCount() - mostrecentshift) > DISTANCE_BETWEEN_SHIFTS) {
        mostrecentshift = osKernelGetTickCount();
        LED_SET(0);

        if (shiftdir == SHIFT_DIR_DOWN) {
          /* Blip the throttle while the solenoid is driven so the engine speed
           * matches the lower gear.  Suppressed near the rev limit. */
          if (rpm < DOWNSHIFT_MAX_RPM) {
            DOWNSHIFT_SOLENOID_SET(1);
            osDelay(SHUTDOWN_CUT_DELAY);

            /* Blip size scheduled against engine speed.  Both schedule
             * endpoints default to SHIFT_BLIP_TARGET, so this is a flat 80%
             * until the endpoints are measured -- identical to the old fixed
             * value, but with somewhere to put the measurement. */
            int32_t blip = pmb_params.blip_target_low;
            if (rpm > BLIP_RPM_LOW) {
              const int32_t span = BLIP_RPM_HIGH - BLIP_RPM_LOW;
              int32_t frac = (int32_t)rpm - BLIP_RPM_LOW;
              if (frac > span) frac = span;
              blip = pmb_params.blip_target_low +
                     ((pmb_params.blip_target_high - pmb_params.blip_target_low) * frac) / span;
            }
            shiftbliptarget = CLAMP_THROTTLE_TARGET(blip);

            /* Arm the deadline rather than hold a flag.  Capped at
             * BLIP_MAX_DURATION_MS so no code path can ask for an open
             * throttle longer than that. */
            uint32_t duration = DOWNSHIFT_SOLENOID_HOLD_TIME;
            if (duration > BLIP_MAX_DURATION_MS) duration = BLIP_MAX_DURATION_MS;
            blip_until_tick = osKernelGetTickCount() + duration;

            osDelay(duration);
            blip_until_tick = 0;

            osDelay(THROTTLE_BLIP_DELAY);
            DOWNSHIFT_SOLENOID_SET(0);
          }
        } else if (shiftdir == SHIFT_DIR_UP) {
          /* Ignition cut upshift.  The cut is requested through
           * shutdown_relay_update() instead of writing PA8 here: the
           * implausibility task also drives that pin every 10 ms and used to
           * cancel this cut almost immediately. */
          UPSHIFT_SOLENOID_SET(1);
          osDelay(UPSHIFT_CUT_LEAD_TIME);

          shift_cut_active = 1;
          shutdown_relay_update();
          osDelay(UPSHIFT_CUT_DURATION);
          shift_cut_active = 0;
          shutdown_relay_update();

          osDelay(UPSHIFT_SOLENOID_RELEASE_TIME);
          UPSHIFT_SOLENOID_SET(0);
        }

        LED_SET(1);
      }
    }
  }
  /* USER CODE END paddleshift */
}

/* USER CODE BEGIN Header_throttlePositionControl */
/**
* @brief Function implementing the throttlePositionControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_throttlePositionControl */
/**
 * Decide what the plate should be doing, in priority order, and clamp the
 * answer into the soft travel window so nothing can command a hard stop.
 *
 * @param faulted  set to 1 if a safety path owns the target.
 */
static int32_t throttle_target_select(uint32_t now, uint8_t *faulted)
{
  int32_t t;
  *faulted = 0;

  if (sensor_can_reception_implausibility || sensor_data_implausibility ||
      throttle_not_at_intended_implausibility) {
    t = THROTTLE_CLOSURE_TARGET;
    *faulted = 1;
  } else if (blip_until_tick != 0 && (int32_t)(now - blip_until_tick) < 0) {
    /* Signed tick difference, so this is correct across tick wraparound and
     * expires on its own if paddleshift never clears it. */
    t = shiftbliptarget;
  } else if (throttle_and_brakes_on_implausibility && rpm > STARTER_RPM) {
    t = target_idle_throttle_position;
  } else if (gas_pedal_position < GAS_PEDAL_IDLE_THRESHOLD && rpm > STARTER_RPM) {
    t = target_idle_throttle_position;
  } else {
    t = THROTTLE_MAP(gas_pedal_position);
  }

  return CLAMP((int32_t)t, pmb_params.soft_min, pmb_params.soft_max);
}

/**
 * Motor output stage: sign to the DIR pin, magnitude to the PWM compare.
 *
 * Direction changes coast for one control period instead of flipping DIR while
 * the bridge is driving.  The control period *is* the dead-time, so this needs
 * no busy-wait and no timer.  Reversals smaller than dir_hysteresis are
 * refused outright, which is what stops TPS noise chattering the bridge.
 *
 * @return the command actually applied, signed, positive = opening.
 */
static int32_t throttle_apply(int32_t u, int32_t pwm_max)
{
  static uint8_t dir_opening = 1;
  static uint8_t dir_valid = 0;

  uint8_t want_opening = (u >= 0);
  int32_t mag = iabs32(u);

  if (mag > pwm_max) mag = pwm_max;

  if (!dir_valid) {
    THROTTLE_DIR_SET(want_opening);
    dir_opening = want_opening;
    dir_valid = 1;
  } else if (want_opening != dir_opening) {
    if (mag <= pmb_params.dir_hysteresis) {
      mag = 0;                      /* too small to be worth a reversal */
    } else {
      THROTTLE_DIR_SET(want_opening);
      dir_opening = want_opening;
      mag = 0;                      /* coast this period, drive the next one */
    }
  }

  /* Break static friction, or do not bother trying to move at all. */
  if (mag > 0 && mag < pmb_params.stiction_pwm) mag = pmb_params.stiction_pwm;
  if (mag > pwm_max) mag = pwm_max;

  THROTTLE_PWM_SET(mag);

  const int32_t applied = dir_opening ? mag : -mag;
  throttle_pwm_applied = applied;
  return applied;
}

void throttlePositionControl(void *argument)
{
  /* USER CODE BEGIN throttlePositionControl */

  osDelay(100);

  /* Full scale comes from the timer, not a literal, so a different ARR on the
   * H7 (or a lower PWM carrier for the driver) needs no change here. */
  const int32_t pwm_hw_max = THROTTLE_PWM_FULLSCALE();
  const int32_t pwm_max = (MAX_THROTTLE_MOTOR_PWM < pwm_hw_max)
                            ? MAX_THROTTLE_MOTOR_PWM : pwm_hw_max;

  pid_cfg_t cfg = {0};
  pid_state_t st;
  pid_reset(&st);
  pid_prime(&st, actual_throttle_position);

  uint32_t last_tick = osKernelGetTickCount();

  /* jam guard */
  int32_t stall_ms = 0;
  int32_t stall_ref = actual_throttle_position;
  int32_t unjam_ms = 0;
  uint8_t unjam_opening = 0;
  uint8_t unjam_tries = 0;

  uint8_t was_faulted = 0;

  for(;;)
  {
    /* Run on TPS arrival; the timeout is the watchdog that keeps the loop
     * alive (and the plate controlled) if CAN goes quiet. */
    osThreadFlagsWait(THROTTLE_TPS_EVENT_FLAG, osFlagsWaitAny,
                      THROTTLE_CONTROL_TIMEOUT_MS);

    const uint32_t now = osKernelGetTickCount();
    int32_t dt = (int32_t)(now - last_tick);
    if (dt < 1) dt = 1;
    if (dt > THROTTLE_MAX_DT_MS) dt = THROTTLE_MAX_DT_MS;
    last_tick = now;

    const int32_t actual = actual_throttle_position;

    /* Target is chosen before the output is written.  The old loop computed it
     * after osDelayUntil, so every command acted on the previous period's
     * target for no reason. */
    uint8_t faulted = 0;
    int32_t target = throttle_target_select(now, &faulted);

    /* Debug-build setpoint / open-loop injection.  Position injection is
     * clamped to the soft limits like any other target; PWM injection is
     * clamped to the host's own ceiling.  Both evaporate if the host stops
     * sending its heartbeat. */
    int32_t inj_mode = 0, inj_value = 0, inj_ceiling = 0;
    const int inject = (!faulted) &&
                       pmb_inject_get(now, &inj_mode, &inj_value, &inj_ceiling);

    if (inject && inj_mode == PMB_INJECT_POSITION) {
      target = CLAMP(inj_value, pmb_params.soft_min, pmb_params.soft_max);
    }

    target_throttle_position = target;

    if (faulted != was_faulted) {
      /* Entering or leaving a safety-owned target: drop the accumulated
       * integral rather than carry it across the discontinuity. */
      pid_reset(&st);
      pid_prime(&st, actual);
      was_faulted = faulted;
      unjam_ms = 0;
      stall_ms = 0;
    }

    int32_t applied;

    if (unjam_ms > 0) {
      /* Unjam pulse in progress: drive the opposite way at full effort to
       * break the plate out of the stop, and do not run the PID meanwhile. */
      unjam_ms -= dt;
      applied = throttle_apply(unjam_opening ? THROTTLE_UNJAM_PWM
                                            : -THROTTLE_UNJAM_PWM, pwm_max);
      if (unjam_ms <= 0) {
        unjam_ms = 0;
        pid_reset(&st);
        pid_prime(&st, actual);
        stall_ms = 0;
        stall_ref = actual;
      }
    } else if (inject && inj_mode == PMB_INJECT_PWM) {
      /* Open-loop drive for plant identification.  Capped by the host ceiling
       * as well as by pwm_max. */
      int32_t cap = inj_ceiling;
      if (cap < 0) cap = 0;
      if (cap > pwm_max) cap = pwm_max;
      applied = throttle_apply(CLAMP(inj_value, -cap, cap), cap);
      pid_reset(&st);
      pid_prime(&st, actual);
    } else {
      /* Gain schedule.  Three regions, as before: opening, closing, and
       * closing near the closed stop where the return spring does the work.
       * Switching sets no longer disturbs the integral -- the integral is in
       * output units, so it stays valid across a gain change. */
      cfg.kp_m = pmb_params.kp_fwd;
      cfg.ki_m = pmb_params.ki_fwd;
      cfg.kd_m = pmb_params.kd_fwd;
      if (actual > THROTTLE_LOW_REGION) {
        cfg.kp_m_rev = pmb_params.kp_rev;
        cfg.ki_m_rev = pmb_params.ki_rev;
      } else {
        cfg.kp_m_rev = pmb_params.kp_rev_low;
        cfg.ki_m_rev = pmb_params.ki_rev_low;
      }
      cfg.kd_m_rev   = pmb_params.kd_rev;
      cfg.out_min    = -pwm_max;
      cfg.out_max    = pwm_max;
      cfg.i_max      = pmb_params.i_max;
      cfg.deadband   = pmb_params.deadband;
      cfg.d_lpf_shift = (uint8_t)pmb_params.d_lpf_shift;

      /* Return-spring feedforward: supply the effort needed just to hold this
       * angle, so the integral only has to correct what is left over. */
      const int32_t ff = (pmb_params.ff_slope * target) / 1000
                         + pmb_params.ff_offset;

      const int32_t u = pid_step(&cfg, &st, target, actual, dt, ff);
      applied = throttle_apply(u, pwm_max);
    }

    /* --- jam / stall guard ------------------------------------------------
     * Saturated output with the plate not moving is a stalled rotor: a thermal
     * event for the motor and the driver, and the state the plate used to wedge
     * itself into. */
    if (iabs32(actual - stall_ref) >= THROTTLE_STALL_MIN_MOTION) {
      stall_ref = actual;
      stall_ms = 0;
    } else if (unjam_ms == 0 && iabs32(applied) >= pwm_max) {
      stall_ms += dt;
    } else if (unjam_ms == 0) {
      stall_ms = 0;
    }

    if (stall_ms >= THROTTLE_STALL_TIMEOUT_MS) {
      stall_ms = 0;
      if (unjam_tries < THROTTLE_UNJAM_RETRIES) {
        unjam_tries++;
        unjam_opening = (applied < 0);   /* pulse away from wherever it stuck */
        unjam_ms = THROTTLE_UNJAM_PULSE_MS;
        THROTTLE_PWM_SET(0);
      } else {
        /* Out of retries: hand it to the safety path, which opens the throttle
         * relay and lets the spring close the plate. */
        throttle_jam_fault = 1;
      }
    }

    if (iabs32(actual - target) < MAX_THROTTLE_INTENDED_VS_ACTUAL_DEVIATION) {
      unjam_tries = 0;
      throttle_jam_fault = 0;
    }

    pmb_log_push(now, target, actual, actual_throttle_position_2nd_sensor,
                 applied, rpm, gas_pedal_position);
  }
  /* USER CODE END throttlePositionControl */
}

/* USER CODE BEGIN Header_serialMonitoring */
/**
* @brief Function implementing the serialMonitoring thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_serialMonitoring */
void serialMonitoring(void *argument)
{
  /* USER CODE BEGIN serialMonitoring */
  /* Infinite loop */

  char buffer[250];

  for(;;)
  { 
    sprintf(buffer, "apps1: %lu\t| apps2: %lu\t| tps1: %lu\t| tps2: %lu\t| bs1: %lu\t| bs2: %lu\t| target: %d\t| actual: %d\t| apps1reception: %lu\t| tps1reception: %lu\t| sd: %u\t| sc: %u\t| ts: %u\t| tb: %u\t| sr: %u\t| tr: %u\t| bspd: %lu\t| rpm: %lu\t| temp: %lu\r\n", apps1, apps2, tps1, tps2, bse1, bse2, target_throttle_position, actual_throttle_position, osKernelGetTickCount() - apps1_updation, osKernelGetTickCount() - tps1_updation, sensor_data_implausibility, sensor_can_reception_implausibility, throttle_not_at_intended_implausibility, throttle_and_brakes_on_implausibility, shutdownrelay, throttlerelay, bspd, rpm, coolant);

    /* NOTE: HAL_UART_Transmit busy-polls.  At 115200 baud this line spins for
     * roughly 21 ms of every 100 ms cycle at osPriorityNormal, i.e. it burns
     * ~20% of the CPU that the control loop wants.  Moving this to DMA is
     * tracked separately -- see the plan's deferred list. */
    HAL_UART_Transmit(DEBUG_UART, (const uint8_t *)buffer, strlen(buffer), osWaitForever);

    osDelay(100);
  }
  /* USER CODE END serialMonitoring */
}

/* USER CODE BEGIN Header_rpmIdle */
/**
* @brief Function implementing the rpmIdle thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_rpmIdle */
void rpmIdle(void *argument)
{
  /* USER CODE BEGIN rpmIdle */

  /* Idle is  position = base_opening + PI(rpm error).
   *
   * The old loop was pure proportional with no base, so at target rpm it
   * commanded 0% throttle, then clamped that to the closed hard stop.  It could
   * not hold an idle by construction -- it could only react once the idle had
   * already sagged, and then close the plate again.
   *
   * The base opening (idle_ff_*) is the plate position that holds target idle
   * with zero rpm error.  It is engine-specific and MUST be measured:
   * `pmbtune.py idleff`.  While it is 0 this loop still will not hold an idle;
   * the structure is correct but uncalibrated. */

  uint32_t currenttick = osKernelGetTickCount();
  uint32_t last_tick = currenttick;

  pid_cfg_t cfg = {0};
  pid_state_t st;
  pid_reset(&st);
  pid_prime(&st, (int32_t)rpm);

  for(;;)
  {
    const uint32_t now = osKernelGetTickCount();
    int32_t dt = (int32_t)(now - last_tick);
    if (dt < 1) dt = 1;
    if (dt > 10 * PID_RPM_DT) dt = 10 * PID_RPM_DT;
    last_tick = now;

    if (gas_pedal_position > GAS_PEDAL_IDLE_THRESHOLD) {
      /* Driver has the pedal: idle is not in control, so do not let the
       * integral accumulate against an error it cannot act on. */
      pid_reset(&st);
      pid_prime(&st, (int32_t)rpm);
    }

    const uint8_t hot = (coolant > IDLE_TEMP_STEPDOWN_THRESHOLD);
    const int32_t sp  = hot ? RPM_HOT_IDLE_TARGET : RPM_COLD_IDLE_TARGET;
    const int32_t ff  = hot ? pmb_params.idle_ff_hot : pmb_params.idle_ff_cold;
    const int32_t max_open = hot ? RPM_HOT_IDLE_MAX_THROTTLE
                                 : RPM_COLD_IDLE_MAX_THROTTLE;

    /* Below RPM_TOO_LOW_THRESHOLD the engine is about to stall, so the
     * proportional gain switches to the more aggressive save value. */
    const int32_t kp = (rpm > RPM_TOO_LOW_THRESHOLD) ? pmb_params.idle_kp
                                                     : pmb_params.idle_kp_save;

    cfg.kp_m = kp;
    cfg.ki_m = pmb_params.idle_ki;
    cfg.kd_m = pmb_params.idle_kd;
    cfg.kp_m_rev = kp;
    cfg.ki_m_rev = pmb_params.idle_ki;
    cfg.kd_m_rev = pmb_params.idle_kd;
    /* Floor is the soft travel minimum, not 0: commanding 0 is what drove the
     * plate into the closed stop. */
    cfg.out_min = pmb_params.soft_min;
    cfg.out_max = CLAMP(max_open, pmb_params.soft_min, pmb_params.soft_max);
    cfg.i_max = pmb_params.idle_i_max;
    cfg.deadband = 0;
    cfg.d_lpf_shift = 0;

    target_idle_throttle_position =
      pid_step(&cfg, &st, sp, (int32_t)rpm, dt, ff);

    /* rpm arrives over CAN, so several passes here can see the same value.
     * That is harmless for the PI terms (it integrates a zero-order hold) but
     * it does make a D term meaningless -- run `pmbtune.py rate` and set
     * PID_RPM_DT to the real rpm frame interval before using idle_kd. */
    currenttick += PID_RPM_DT;
    osDelayUntil(currenttick);

  }
  /* USER CODE END rpmIdle */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

