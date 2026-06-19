
//values can be changed as needed

#ifndef __TUNING_CONSTANTS_H
#define __TUNING_CONSTANTS_H

#ifdef __cplusplus
extern "C" {
#endif

//the following constants determine the absolute upper and lower bounds that determine sensor failure

#define APPS1_UB    3500
#define APPS1_LB    100
#define APPS2_UB    3500
#define APPS2_LB    100

#define TPS1_UB     3800
#define TPS1_LB     100
#define TPS2_UB     3800
#define TPS2_LB     100

#define BS1_UB      3800
#define BS1_LB      100
#define BS2_UB      3800
#define BS2_LB      100

//the following constants determine the upper and lower bounds of normal measurable range

#define APPS1_OPERATION_LB  1965    //opposite slope sensor
#define APPS1_OPERATION_UB  695
#define APPS2_OPERATION_LB  2125    //positive slope sensor
#define APPS2_OPERATION_UB  3250 

#define TPS1_OPERATION_LB   700     //positive slope sensor
#define TPS1_OPERATION_UB   3615
#define TPS2_OPERATION_LB   3140    //opposite slope sensor
#define TPS2_OPERATION_UB   225

#define BSE1_OPERATION_LB   405     //front brake circuit
#define BSE1_OPERATION_UB   1000    //pressed with hand as hard as possible
#define BSE2_OPERATION_LB   405     //back brake circuit
#define BSE2_OPERATION_UB   675     //pressed with hand as hard as possible

//the following pid tuning values are all divided by 1000 in th code, multiply the desired value you are inputting by 1000 and input it here

#define PID_FORWARD_P   50
#define PID_BACKWARD_P  40
#define PID_FORWARD_I   15
#define PID_BACKWARD_I  0
#define PID_FORWARD_D   0
#define PID_BACKWARD_D  50
#define PID_DT          1 //not divided by 1000. This value is in millisecond units, and represents time between each pid updation.

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

#define RPM_CAN_ID          1000000 //?? need to get this one
#define RPM_CAN_OFFSET      0

#define SHIFT_ID            172
#define SHIFT_DIR_OFFSET    0
#define SHIFT_COUNT_OFFSET  1

//the following are timing constants

#define SHIFT_SOLENOID_HOLD_TIME    100

#define THROTTLE_UPDATION_DELTA     2

#define SENSOR_IMPLAUSIBILITY_TIMEOUT   100
#define SENSOR_BACK_TO_NORMAL_TIMEOUT   50

//the following determines throttle idle position, it is a number from 0 to 1000, representing 0% to 100% actuation

#define THROTTLE_IDLE_TARGET    10

//the following is the raw rpm value for engine idle

#define IDLE_RPM    1500

//the following determines maximum allowable current for running the throttle, it is a number from 0 to 1000 representing 0% to 100%

#define MAX_THROTTLE_MOTOR_PWM    700

//the following slopes and intercepts are all calculated with equations, they shouldnt need to be modified, just change the OPERATION_UB and OPERATION_LB values above

#define SLOPE(y1, y2, x1, x2) ((y2 - y1) * 1000 / (x2 - x1))                      //m = (y2 - y1) / (x2 - x1)
#define INTERCEPT(y1, y2, x1, x2) ((y1 * 1000) - (SLOPE(x1, x2, y1, y2) * x1))    //b = y2 - (m*x2)

#define RAW_TPS1_TO_THROTLE_PERCENTAGE_SLOPE            SLOPE(1000, 0, TPS1_OPERATION_UB, TPS1_OPERATION_LB)
#define RAW_TPS1_TO_THROTTLE_PERCENTAGE_INTERCEPT       INTERCEPT(1000, 0, TPS1_OPERATION_UB, TPS1_OPERATION_LB)

#define RAW_APPS1_TO_PEDAL_PERCENTAGE_SLOPE             SLOPE(1000, 0, APPS1_OPERATION_UB, APPS1_OPERATION_LB) 
#define RAW_APPS1_TO_PEDAL_PERCENTAGE_INTERCEPT         INTERCEPT(1000, 0, APPS1_OPERATION_UB, APPS1_OPERATION_LB) 



#ifdef __cplusplus
}
#endif
#endif /* __TUNING_CONSTANTS_H */