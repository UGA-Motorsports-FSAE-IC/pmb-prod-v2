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
#include <string.h>
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

uint8_t shiftnumber = 1;
uint8_t shiftdir = 0;
uint8_t currentshiftnum = 1;
uint32_t mostrecentshift;

volatile uint32_t targetvalue = 300;


uint64_t apps1_updation;
uint64_t apps2_updation;
uint64_t tps1_updation;
uint64_t tps2_updation;
uint64_t bs1_updation;
uint64_t bs2_updation;
uint8_t sensor_implausibility = 0;
uint8_t throttle_and_brakes_on = 0;
uint8_t throttle_not_at_intended = 0;
uint8_t useapps = 1;

extern osMessageQueueId_t canqueue;

uint32_t tps_target;

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


  uint32_t potential_sensor_issue_time = 0;
  uint32_t currenttick = osKernelGetTickCount();
  for(;;)
  {

    uint8_t apps_issue = apps1 < APPS1_LB || apps1 > APPS1_UB || apps2 < APPS2_LB || apps2 > APPS2_UB || (currenttick - apps1_updation > 100) || (currenttick - apps2_updation > 100);
    uint8_t tps_issue = tps1 < TPS1_LB || tps1 > TPS1_UB || tps2 < TPS2_LB || tps2 > TPS2_UB || (currenttick - tps1_updation > 100) || (currenttick - tps2_updation > 100);
    uint8_t bse_issue = bse1 < BS1_LB || bse1 > BS1_UB || bse2 < BS2_LB || bse2 > BS2_UB || (currenttick - bs1_updation > 100) || (currenttick - bs2_updation > 100); 

    if (apps_issue || tps_issue || bse_issue) {
      potential_sensor_issue_time += 1;    
    } else {
      potential_sensor_issue_time = 0;
    }

    sensor_implausibility = (potential_sensor_issue_time > 100);

    osDelayUntil(++currenttick);

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

        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_7, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_RESET);
      }
    }
    
    osDelay(1);


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
    int error = targetvalue - (int)tps1;

    int change_in_integral;
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

    //TIM16->CCR1 = 30; //have this commented out
    

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
    if (throttle_and_brakes_on || throttle_not_at_intended) {
      targetvalue = THROTTLE_IDLE_TARGET;
    } else if (useapps) {
      targetvalue = ((APPS_TO_TPS_TARGET_SLOPE * ((int)apps1)) + APPS_TO_TPS_TARGET_INTERCEPT) / 1000;
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
    sprintf(buffer, "apps1: %lu\t| apps2: %lu\t| tps1: %lu\t| tps2: %lu\t| bs1: %lu\t| bs2: %lu\t| target: %lu\r\n", apps1, apps2, tps1, tps2, bse1, bse2, targetvalue);

    HAL_UART_Transmit(&huart1, (const uint8_t *)buffer, strlen(buffer), osWaitForever);

    osDelay(100);
  }
  /* USER CODE END serialMonitoring */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

