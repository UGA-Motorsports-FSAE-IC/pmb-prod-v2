
//values can be changed as needed

#ifndef __TUNING_CONSTANTS_H
#define __TUNING_CONSTANTS_H

#ifdef __cplusplus
extern "C" {
#endif

#define APPS1_UB    4000
#define APPS1_LB    100
#define APPS2_UB    4000
#define APPS2_LB    100

#define TPS1_UB     4000
#define TPS1_LB     100
#define TPS2_UB     4000
#define TPS2_LB     100

#define BS1_UB      4000
#define BS1_LB      100
#define BS2_UB      4000
#define BS2_LB      100

//the following slopes and intercepts and pid values are all divided by 1000 in the code. Multiply the desired value you are inputting by 1000 and input it here.

#define APPS_1_2_SLOPE      10
#define APPS_1_2_INTERCEPT  10
#define TPS_1_2_SLOPE       10
#define TPS_1_2_INTERCEPT   10
#define BSE_1_2_SLOPE       10
#define BSE_1_2_INTERCEPT   10

#define APPS_TO_TPS_TARGET_SLOPE        10
#define APPS_TO_TPS_TARGET_INTERCEPT    10

#define PID_FORWARD_P   0
#define PID_BACKWARD_P  0
#define PID_FORWARD_I   0
#define PID_BACKWARD_I  0
#define PID_FORWARD_D   0
#define PID_BACKWARD_D  0
#define PID_DT          1

// the following are constants for can frames

#define APPS1_CAN_ID        210
#define APPS1_CAN_OFFSET    0
#define APPS2_CAN_ID        215
#define APPS2_CAN_OFFSET    3
#define TPS1_CAN_ID         215
#define TPS1_CAN_OFFSET     2
#define TPS2_CAN_ID         215
#define TPS2_CAN_OFFSET     1
#define BS1_CAN_ID          215
#define BS1_CAN_OFFSET      0
#define BS2_CAN_ID          214
#define BS2_CAN_OFFSET      3

#define RPM_CAN_ID          1000000
#define RPM_CAN_OFFSET      0

#define SHIFT_ID            172
#define SHIFT_DIR_OFFSET    0
#define SHIFT_COUNT_OFFSET  1

#define SHIFT_SOLENOID_HOLD_TIME    100

#define THROTTLE_UPDATION_DELTA     2

#define THROTTLE_IDLE_TARGET    700



#ifdef __cplusplus
}
#endif
#endif /* __TUNING_CONSTANTS_H */