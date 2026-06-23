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

#include "cmsis_os2.h"
#include "fdcanmessage.h"
#include "stm32c092xx.h"
#include "stm32c0xx.h"
#include "stm32c0xx_hal_fdcan.h"
#include "stm32c0xx_hal_gpio.h"
#include "stm32c0xx_hal_tim.h"
#include "stm32c0xx_hal_uart.h"
#include "constants.h"
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/_intsup.h>
#include "tim.h"
#include "usart.h"

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

volatile uint8_t shiftnumber = 1;
volatile uint8_t shiftdir = 0;
volatile uint8_t currentshiftnum = 1;
volatile uint32_t mostrecentshift;

volatile int actual_throttle_position;  //number from 0 to 1000 representing 0% to 100% actuation
volatile int actual_throttle_position_2nd_sensor; //same thing but calculated from other tps
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
volatile uint32_t implausibility_throttle_closure = 0;



volatile uint8_t useapps = 1;

volatile uint8_t shiftblipactive = 0;
volatile uint32_t shiftbliptarget;

extern osMessageQueueId_t canqueue;

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
/* Definitions for throttlePID */
osThreadId_t throttlePIDHandle;
const osThreadAttr_t throttlePID_attributes = {
  .name = "throttlePID",
  .priority = (osPriority_t) osPriorityNormal,
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

  /* creation of throttlePID */
  throttlePIDHandle = osThreadNew(throttlePID, NULL, &throttlePID_attributes);

  /* creation of throttlePositionControl */
  throttlePositionControlHandle = osThreadNew(throttlePositionControl, NULL, &throttlePositionControl_attributes);

  /* creation of serialMonitoring */
  serialMonitoringHandle = osThreadNew(serialMonitoring, NULL, &serialMonitoring_attributes);

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

  osDelay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); //led
  osDelay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); //led
  osDelay(100);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); //led  

  uint8_t sensor_data_implausibility = 0;
  uint8_t sensor_can_reception_implausibility = 0;
  uint8_t throttle_and_brakes_on_implausibility = 0;
  uint8_t throttle_not_at_intended_implausibility = 0;

  uint32_t potential_sensor_issue_duration = 0;
  uint32_t sensor_normal_function_duration = 0;

  uint32_t potential_throttlebody_issue_duration = 0;
  uint32_t throttlebody_normal_function_duration = 0;

  uint32_t potential_stuck_throttle_duration = 0;
  uint32_t no_longer_stuck_throttle_duration = 0;

  uint32_t currenttick = osKernelGetTickCount();
  for(;;)
  {

    //the following code checks if there is any issue with the received sensor values

    uint8_t apps_issue = apps1 < APPS1_LB || apps1 > APPS1_UB || apps2 < APPS2_LB || apps2 > APPS2_UB || (abs(gas_pedal_position - gas_pedal_position_2nd_sensor) < MAX_GAS_PEDAL_SENSORS_DEVIATION);
    uint8_t tps_issue = tps1 < TPS1_LB || tps1 > TPS1_UB || tps2 < TPS2_LB || tps2 > TPS2_UB || (abs(actual_throttle_position - actual_throttle_position_2nd_sensor) < MAX_THROTTLE_SENSORS_DEVIATION);
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

    sensor_can_reception_implausibility = (currenttick - apps1_updation > SENSOR_IMPLAUSIBILITY_TIMEOUT) || (currenttick - apps2_updation > SENSOR_IMPLAUSIBILITY_TIMEOUT) || 
                (currenttick - tps1_updation > SENSOR_IMPLAUSIBILITY_TIMEOUT) || (currenttick - tps2_updation > SENSOR_IMPLAUSIBILITY_TIMEOUT) || 
                (currenttick - bs1_updation > SENSOR_IMPLAUSIBILITY_TIMEOUT) || (currenttick - bs2_updation > SENSOR_IMPLAUSIBILITY_TIMEOUT);


    //the following code checks for if the throttle body flap is not actuating to the intended position
    
    if (abs(actual_throttle_position - target_throttle_position) > MAX_THROTTLE_INTENDED_VS_ACTUAL_DEVIATION) {
      potential_throttlebody_issue_duration += IMPLAUSIBILITY_CHECK_INTERVAL;
      throttlebody_normal_function_duration = 0;
    } else {
      throttlebody_normal_function_duration += IMPLAUSIBILITY_CHECK_INTERVAL;
      potential_throttlebody_issue_duration = 0;
    }
    if (potential_throttlebody_issue_duration > THROTTLEBODY_FAILURE_TIMEOUT) {
      throttle_not_at_intended_implausibility = 1;
      potential_throttlebody_issue_duration += IMPLAUSIBILITY_CHECK_INTERVAL;
      no_longer_stuck_throttle_duration = 0;
      //remove power to the throttle body
    }
    if (throttlebody_normal_function_duration > THROTTLEBODY_BACK_TO_NORMAL_TIMEOUT) {
      throttle_not_at_intended_implausibility = 0;
      no_longer_stuck_throttle_duration += IMPLAUSIBILITY_CHECK_INTERVAL;
      potential_throttlebody_issue_duration = 0;
      //give back power to the throttle body
    }

    if (potential_stuck_throttle_duration > OPEN_SHUTDOWN_CIRCUIT_TIMEOUT) {
      //open the shutdown circuit, removing power from injector spark fuel
    }
    if (no_longer_stuck_throttle_duration > CLOSE_SHUTDOWN_CIRCUIT_TIMEOUT) {
      //close the shutdown circuit, giving power back to injector spark fuel
    }

    //the following code checks if there is hard braking and open throttle at the same time

    throttle_and_brakes_on_implausibility = actual_throttle_position > OPEN_THROTTLE_PERCENTAGE_THRESHOLD && brake_depression_percentage > HARD_BRAKING_PERCENTAGE_THRESHOLD;

    //this boolean will be true if there is any reason from above to bring the throttle flap to idle position. 
    // The throttlePositionControl thread monitors this boolean and acts accordingly.
    implausibility_throttle_closure = sensor_can_reception_implausibility || sensor_data_implausibility || throttle_not_at_intended_implausibility || throttle_and_brakes_on_implausibility;

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
    uint16_t * candata_2_byte = (uint16_t *)(currentmessage.canrxdata);
    uint32_t id = currentmessage.rxheader.Identifier;
    
    if (id == APPS1_CAN_ID) {
      apps1 = candata_2_byte[APPS1_CAN_OFFSET];
      apps1_updation = currenttick;
    }
    if (id == APPS2_CAN_ID) {
      apps2 = candata_2_byte[APPS2_CAN_OFFSET];
      apps2_updation = currenttick;      
    }
    if (id == TPS1_CAN_ID) {
      tps1 = candata_2_byte[TPS1_CAN_OFFSET];
      tps1_updation = currenttick;
    }
    if (id == TPS2_CAN_ID) {
      tps2 = candata_2_byte[TPS2_CAN_OFFSET];
      tps2_updation = currenttick;
    }
    if (id == BS1_CAN_ID) {
      bse1 = candata_2_byte[BS1_CAN_OFFSET];
      bs1_updation = currenttick;
    }
    if (id == BS2_CAN_ID) {
      bse2 = candata_2_byte[BS2_CAN_OFFSET];
      bs2_updation = currenttick;
    }
    if (id == SHIFT_ID) {
      shiftnumber = candata[SHIFT_COUNT_OFFSET];
      shiftdir = candata[SHIFT_DIR_OFFSET];
    }


    //here, "percentages" are represented as a number from 0 to 1000.
    actual_throttle_position = LINEAR(tps1, RAW_TPS1_TO_THROTLE_PERCENTAGE_SLOPE, RAW_TPS1_TO_THROTTLE_PERCENTAGE_INTERCEPT);
    gas_pedal_position = LINEAR(apps1, RAW_APPS1_TO_PEDAL_PERCENTAGE_SLOPE, RAW_APPS1_TO_PEDAL_PERCENTAGE_INTERCEPT);

    actual_throttle_position_2nd_sensor = LINEAR(tps2, RAW_TPS2_TO_THROTTLE_PERCENTAGE_SLOPE, RAW_TPS2_TO_THROTTLE_PERCENTAGE_INTERCEPT);
    gas_pedal_position_2nd_sensor = LINEAR(apps2, RAW_APPS2_TO_PEDAL_PERCENTAGE_SLOPE, RAW_APPS2_TO_PEDAL_PERCENTAGE_INTERCEPT);

    brake_depression_percentage = LINEAR(bse1, RAW_BS1_TO_BRAKE_PERCENTAGE_SLOPE, RAW_BS1_TO_BRAKE_PERCENTAGE_INTERCEPT);
    brake_depression_percentage_2nd_sensor = LINEAR(bse2, RAW_BS2_TO_BRAKE_PERCENTAGE_SLOPE, RAW_BS2_TO_BRAKE_PERCENTAGE_INTERCEPT);
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
      if ((osKernelGetTickCount() - mostrecentshift) > (SHIFT_SOLENOID_HOLD_TIME + 20)) {
        mostrecentshift = osKernelGetTickCount();
        if (shiftdir == 1) {
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); //led
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_SET); //downshift relay
        } else if (shiftdir == 2) {
          //do shift cut over can????
          HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET); //led
          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET); //upshift relay
        }
        osDelay(SHIFT_SOLENOID_HOLD_TIME);

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); //led
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET); //downshift relay
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET); //upshift relay
      }
    }
  }
  /* USER CODE END paddleshift */
}

/* USER CODE BEGIN Header_throttlePID */
/**
* @brief Function implementing the throttlePID thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_throttlePID */
void throttlePID(void *argument)
{
  /* USER CODE BEGIN throttlePID */
  /* Infinite loop */

  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); //throttle relay
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); //polulu direction forward

  uint32_t currenttick = osKernelGetTickCount();

  int integral = 0;
  int previouserror = 0;

  for(;;)
  {

    int proportion = 0;
    int derivative = 0;
    int change_in_integral;

    int error = ((int)target_throttle_position - (int)actual_throttle_position);

    if (error > 0) {
      proportion = error * PID_FORWARD_P;
      change_in_integral = PID_DT * error * PID_FORWARD_I;
      derivative = (error - previouserror) * PID_FORWARD_D;
    } else {
      proportion = error * PID_BACKWARD_P;
      change_in_integral = PID_DT * error * PID_BACKWARD_I;
      derivative = (error - previouserror) * PID_BACKWARD_D;
    }

    previouserror = error;
    
    int finalresult = ((proportion + change_in_integral + derivative) / 1000) + integral;
    integral += ((change_in_integral / 1000) + integral);
    
    if (finalresult > 0) {
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_RESET); //polulu direction forward
    } else {
      HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, GPIO_PIN_SET); //polulu direction reverse
      finalresult *= -1;
    }

    if (finalresult > MAX_THROTTLE_MOTOR_PWM) {
      finalresult = MAX_THROTTLE_MOTOR_PWM;
    }

    __HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, finalresult);

    currenttick += PID_DT;
    osDelayUntil(currenttick);
  }
  /* USER CODE END throttlePID */
}

/* USER CODE BEGIN Header_throttlePositionControl */
/**
* @brief Function implementing the throttlePositionControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_throttlePositionControl */
void throttlePositionControl(void *argument)
{
  /* USER CODE BEGIN throttlePositionControl */
  /* Infinite loop */

  osDelay(100);
  uint32_t currenttick = osKernelGetTickCount();



  for(;;)
  {
    if (implausibility_throttle_closure) {
      target_throttle_position = THROTTLE_IDLE_TARGET;
    } else if (shiftblipactive) {
      target_throttle_position = shiftbliptarget;
    } else if (useapps) {
      target_throttle_position = gas_pedal_position;
    } 

    currenttick += THROTTLE_UPDATION_DELTA;
    osDelayUntil(currenttick);
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

  char buffer[200];

  for(;;)
  { 
    sprintf(buffer, "apps1: %lu\t| apps2: %lu\t| tps1: %lu\t| tps2: %lu\t| bs1: %lu\t| bs2: %lu\t| target: %d\r\n", apps1, apps2, tps1, tps2, bse1, bse2, target_throttle_position);

    HAL_UART_Transmit(&huart1, (const uint8_t *)buffer, strlen(buffer), osWaitForever);

    osDelay(100);
  }
  /* USER CODE END serialMonitoring */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

