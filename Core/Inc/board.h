/**
  ******************************************************************************
  * @file    board.h
  * @brief   Single edit point for the board/MCU mapping.
  *
  * This is the seam for the STM32H7 migration.  Application code must not
  * include family headers (stm32c0xx_*.h, stm32c092xx.h) or touch peripheral
  * registers directly: it goes through the aliases below.  Porting to the H7
  * should mean editing this file, the CubeMX .ioc, and two lines in
  * CMakePresets.json -- nothing in app_freertos.c.
  ******************************************************************************
  */

#ifndef __BOARD_H
#define __BOARD_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"   /* family HAL header + CubeMX pin macros */
#include "tim.h"
#include "usart.h"

/* ---------------------------------------------------------------- peripherals
 */
#define THROTTLE_PWM_TIM      (&htim16)
#define THROTTLE_PWM_CHANNEL  TIM_CHANNEL_1
#define DEBUG_UART            (&huart1)

/* PWM full scale read from the timer rather than hardcoded, so a different ARR
 * (or a different timer clock) on the H7 needs no code change.
 *
 * Today: TIM16 at 48 MHz, PSC = 0, ARR = 999  ->  full scale 1000, 48 kHz.
 *
 * 48 kHz is above what some brushed H-bridge drivers tolerate.  The Pololu part
 * number is still unconfirmed; if it needs a lower carrier, change PSC/ARR in
 * CubeMX (tim.c is generated -- do not hand-edit it) and this macro follows
 * automatically.  Nothing in the control loop assumes 1000. */
#define THROTTLE_PWM_FULLSCALE() \
  ((int32_t)__HAL_TIM_GET_AUTORELOAD(THROTTLE_PWM_TIM) + 1)

#define THROTTLE_PWM_SET(v) \
  __HAL_TIM_SET_COMPARE(THROTTLE_PWM_TIM, THROTTLE_PWM_CHANNEL, (uint32_t)(v))

/* ---------------------------------------------------------------------- pins
 */

/* Shift solenoids.
 *
 * WARNING: the CubeMX GPIO labels for PA6/PA7 are reversed with respect to the
 * harness.  main.h says upshift_Pin == PA6 and downshift_Pin == PA7; the car
 * says PA6 fires the DOWNSHIFT solenoid (confirmed by downshifts working with
 * the original code, which wrote PA6 for downshift).
 *
 * These two aliases are authoritative.  Do not "fix" them to match main.h --
 * fix the .ioc labels at the next CubeMX regeneration instead, then delete this
 * comment. */
#define DOWNSHIFT_SOLENOID_PORT   GPIOA
#define DOWNSHIFT_SOLENOID_PIN    GPIO_PIN_6
#define UPSHIFT_SOLENOID_PORT     GPIOA
#define UPSHIFT_SOLENOID_PIN      GPIO_PIN_7

#define DOWNSHIFT_SOLENOID_SET(on) \
  HAL_GPIO_WritePin(DOWNSHIFT_SOLENOID_PORT, DOWNSHIFT_SOLENOID_PIN, \
                    (on) ? GPIO_PIN_SET : GPIO_PIN_RESET)
#define UPSHIFT_SOLENOID_SET(on) \
  HAL_GPIO_WritePin(UPSHIFT_SOLENOID_PORT, UPSHIFT_SOLENOID_PIN, \
                    (on) ? GPIO_PIN_SET : GPIO_PIN_RESET)

/* Relays.  Labels for these are correct, so they come from main.h. */

/* Throttle body motor power. */
#define THROTTLE_POWER_SET(on) \
  HAL_GPIO_WritePin(throttlerelay_GPIO_Port, throttlerelay_Pin, \
                    (on) ? GPIO_PIN_SET : GPIO_PIN_RESET)

/* Shutdown / ignition circuit.  closed = engine may run. */
#define SHUTDOWN_CIRCUIT_SET(closed) \
  HAL_GPIO_WritePin(shutdown_GPIO_Port, shutdown_Pin, \
                    (closed) ? GPIO_PIN_SET : GPIO_PIN_RESET)

/* H-bridge direction.  opening = plate toward open. */
#define THROTTLE_DIR_SET(opening) \
  HAL_GPIO_WritePin(dir_GPIO_Port, dir_Pin, \
                    (opening) ? GPIO_PIN_RESET : GPIO_PIN_SET)

#define LED_SET(on) \
  HAL_GPIO_WritePin(led_GPIO_Port, led_Pin, (on) ? GPIO_PIN_SET : GPIO_PIN_RESET)

#ifdef __cplusplus
}
#endif
#endif /* __BOARD_H */
