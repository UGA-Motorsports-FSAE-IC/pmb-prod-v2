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
#include "stm32c0xx.h"
#include "stm32c0xx_hal_fdcan.h"
#include "tuning_constants.h"
#include <stdint.h>

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

uint32_t apps1;
uint32_t apps2;
uint32_t tps1;
uint32_t tps2;
uint32_t bse1;
uint32_t bse2;

uint64_t apps1_updation;
uint64_t apps2_updation;
uint64_t tps1_updation;
uint64_t tps2_updation;
uint64_t bs1_updation;
uint64_t bs2_updation;
uint8_t sensor_implausibility = 0;
uint8_t throttle_and_brakes_on = 0;
uint8_t throttle_not_at_intended = 0;

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

  uint64_t potential_sensor_issue_time = 0;
  uint64_t currenttick = osKernelGetTickCount();

  for(;;)
  {
    osDelay(1);
    osDelay(1);

    uint8_t apps_issue = apps1 < APPS1_LB || apps1 > APPS1_UB || apps2 < APPS2_LB || apps2 > APPS2_UB;
    uint8_t tps_issue = tps1 < TPS1_LB || tps1 > TPS1_UB || tps2 < TPS2_LB || tps2 > TPS2_UB;
    uint8_t bse_issue = bse1 < BS1_LB || bse1 > BS1_UB || bse2 < BS2_LB || bse2 > BS2_UB;

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
  for(;;)
  {
    osDelay(1);
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
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END paddleshift */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

