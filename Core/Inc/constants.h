
//values can be changed as needed

#ifndef __CONSTANTS_H
#define __CONSTANTS_H

#ifdef __cplusplus
extern "C" {
#endif

//the following constants determine the absolute upper and lower bounds that determine sensor failure

#define APPS1_UB    3900
#define APPS1_LB    100
#define APPS2_UB    3900
#define APPS2_LB    100

#define TPS1_UB     3900
#define TPS1_LB     100
#define TPS2_UB     3900
#define TPS2_LB     100

#define BS1_UB      3800
#define BS1_LB      100
#define BS2_UB      3800
#define BS2_LB      100

//the following constants determine the upper and lower bounds of normal measurable range

#define APPS1_OPERATION_LB  2000    //opposite slope sensor
#define APPS1_OPERATION_UB  320
#define APPS2_OPERATION_LB  2080    //positive slope sensor
#define APPS2_OPERATION_UB  3580

#define TPS1_OPERATION_LB   700     //positive slope sensor
#define TPS1_OPERATION_UB   3615
#define TPS2_OPERATION_LB   3140    //opposite slope sensor
#define TPS2_OPERATION_UB   225

#define BSE1_OPERATION_LB   405     //front brake circuit
#define BSE1_OPERATION_UB   1000    //pressed with hand as hard as possible, could be pressed harder
#define BSE2_OPERATION_LB   405     //back brake circuit
#define BSE2_OPERATION_UB   675     //pressed with hand as hard as possible, could be pressed harder

//the following are constants for tuning implausibility checks (dual sensor deviation, hard braking, open throttle)
// "percentage" is represented as a number from 0 to 1000, corresponding to 0% and 100%

#define MAX_GAS_PEDAL_SENSORS_DEVIATION             100
#define MAX_THROTTLE_SENSORS_DEVIATION              100

#define MAX_THROTTLE_INTENDED_VS_ACTUAL_DEVIATION   200

#define HARD_BRAKING_PERCENTAGE_THRESHOLD           160
#define OPEN_THROTTLE_PERCENTAGE_THRESHOLD          400


//the following pid tuning values are for throttle flap control, all are divided by 1000 in th code, multiply the desired value you are inputting by 1000 and input it here

#define PID_FORWARD_P   1700
#define PID_BACKWARD_P  1700
#define PID_FORWARD_I   15
#define PID_BACKWARD_I  15
#define PID_FORWARD_D   0
#define PID_BACKWARD_D  0
#define PID_DT          1  //not divided by 1000. This value is in millisecond units, and represents time between each pid updation.

// the following are constants for can frames

#define APPS1_CAN_ID        210
#define APPS1_CAN_OFFSET    0
#define APPS2_CAN_ID        215
#define APPS2_CAN_OFFSET    6
#define TPS1_CAN_ID         215
#define TPS1_CAN_OFFSET     4
#define TPS2_CAN_ID         215
#define TPS2_CAN_OFFSET     2
#define BS1_CAN_ID          215
#define BS1_CAN_OFFSET      0
#define BS2_CAN_ID          214
#define BS2_CAN_OFFSET      6

#define COOLANT_CAN_ID      1522
#define COOLANT_CAN_OFFSET  6
#define COOLANT_CAN_DIVIDE  10

#define RPM_CAN_ID          1520 
#define RPM_CAN_OFFSET      6     //since this is 2 byte number, offset 3 means offset 6 bytes.

#define BSPD_CAN_ID         214
#define BSPD_CAN_OFFSET     4

#define SHIFT_ID            172
#define SHIFT_DIR_OFFSET    0
#define SHIFT_COUNT_OFFSET  1

//the following are timing constants

#define SHIFT_SOLENOID_HOLD_TIME                100

#define THROTTLE_UPDATION_DELTA                 2

#define SENSOR_IMPLAUSIBILITY_TIMEOUT           100
#define SENSOR_BACK_TO_NORMAL_TIMEOUT           50

#define THROTTLEBODY_FAILURE_TIMEOUT            200
#define THROTTLEBODY_BACK_TO_NORMAL_TIMEOUT     200

#define OPEN_SHUTDOWN_CIRCUIT_TIMEOUT           600
#define CLOSE_SHUTDOWN_CIRCUIT_TIMEOUT          1200

#define IMPLAUSIBILITY_CHECK_INTERVAL           10

//the following determines gas pedal threshold for using idle pid

#define GAS_PEDAL_IDLE_THRESHOLD    100     //using PID idle control when gas pedal below 10%

//the following determines what is considered closed throttle

#define THROTTLE_CLOSURE_TARGET     0       //throttle closure means throttle at 0%

//the following determines how much blip the downshift should get

#define SHIFT_BLIP_TARGET       800 //80%

//the following pid tuning values are for the idle rpm contol, all are divided by 1000 in the code, multiply your wanted values by 1000 before inputting them here.

#define PID_RPM_P       500
#define PID_RPM_I       0
#define PID_RPM_D       0
#define PID_RPM_DT      20

//the following determines hot vs cold idle targets and thresholds for controlling idle

#define RPM_HOT_IDLE_MAX_THROTTLE       400     //40%
#define RPM_COLD_IDLE_MAX_THROTTLE      400     //40%
#define RPM_HOT_IDLE_TARGET             1800    //rpm
#define RPM_COLD_IDLE_TARGET            1800    //rpm
#define IDLE_TEMP_STEPDOWN_THRESHOLD    170     //170 degrees fahrenheit

//the following is the rpm that the starter motor spins at

#define STARTER_RPM     300


//the following determines maximum allowable current for running the throttle, it is a number from 0 to 1000 representing 0% to 100%

#define MAX_THROTTLE_MOTOR_PWM    1000

//the following slopes and intercepts are all calculated with equations, they shouldnt need to be modified, just change the OPERATION_UB and OPERATION_LB values above
// "percentages" are represented as a number from 0 to 1000, corresponding to 0% and 100%

#define SLOPE(y1, y2, x1, x2) ((y2 - y1) * 1000 / (x2 - x1))                      //m = (y2 - y1) / (x2 - x1)
#define INTERCEPT(y1, y2, x1, x2) ((y1 * 1000) - (SLOPE(y1, y2, x1, x2) * x1))    //b = y2 - (m*x2)

//this computes a cubic polynomial (ax^3 + bx^2 + cx + d). Only works with coefficients that are multiplied by 1000
#define CUBIC(x, a, b, c, d) (((a*x*x*x) + (b*x*x) + (c*x) + d) / 1000) 

//this computes a linear function (y = mx + b). Only works with slope and intercepts that are multiplied by 1000
#define LINEAR(x, m, b) (((m*x) + b) / 1000)


#define RAW_TPS1_TO_THROTTLE_PERCENTAGE_SLOPE           SLOPE(1000, 0, TPS1_OPERATION_UB, TPS1_OPERATION_LB)
#define RAW_TPS1_TO_THROTTLE_PERCENTAGE_INTERCEPT       INTERCEPT(1000, 0, TPS1_OPERATION_UB, TPS1_OPERATION_LB)

#define RAW_TPS2_TO_THROTTLE_PERCENTAGE_SLOPE           SLOPE(1000, 0, TPS2_OPERATION_UB, TPS2_OPERATION_LB)
#define RAW_TPS2_TO_THROTTLE_PERCENTAGE_INTERCEPT       INTERCEPT(1000, 0, TPS2_OPERATION_UB, TPS2_OPERATION_LB)

#define RAW_APPS1_TO_PEDAL_PERCENTAGE_SLOPE             SLOPE(1000, 0, APPS1_OPERATION_UB, APPS1_OPERATION_LB) 
#define RAW_APPS1_TO_PEDAL_PERCENTAGE_INTERCEPT         INTERCEPT(1000, 0, APPS1_OPERATION_UB, APPS1_OPERATION_LB) 

#define RAW_APPS2_TO_PEDAL_PERCENTAGE_SLOPE             SLOPE(1000, 0, APPS2_OPERATION_UB, APPS2_OPERATION_LB) 
#define RAW_APPS2_TO_PEDAL_PERCENTAGE_INTERCEPT         INTERCEPT(1000, 0, APPS2_OPERATION_UB, APPS2_OPERATION_LB) 

#define RAW_BS1_TO_BRAKE_PERCENTAGE_SLOPE               SLOPE(1000, 0, BSE1_OPERATION_UB, BSE1_OPERATION_LB)
#define RAW_BS1_TO_BRAKE_PERCENTAGE_INTERCEPT           INTERCEPT(1000, 0, BSE1_OPERATION_UB, BSE1_OPERATION_LB)

#define RAW_BS2_TO_BRAKE_PERCENTAGE_SLOPE               SLOPE(1000, 0, BSE2_OPERATION_UB, BSE2_OPERATION_LB)
#define RAW_BS2_TO_BRAKE_PERCENTAGE_INTERCEPT           INTERCEPT(1000, 0, BSE2_OPERATION_UB, BSE2_OPERATION_LB)

//the following defines big and little endian conversion for getting can values

#define GET_16BIT_LITTLEENDIAN_CAN_VALUE(pointer_8bit, offset) (((uint16_t *)pointer_8bit)[offset / 2])
#define GET_16BIT_BIGENDIAN_CAN_VALUE(pointer_8bit, offset) (((uint16_t)(pointer_8bit)[(offset) + 1]) | ((uint16_t)(pointer_8bit)[(offset)] << 8))
#define GET_8BIT_CAN_VALUE(pointer_8bit, offset) ((pointer_8bit)[(offset)])



#ifdef __cplusplus
}
#endif
#endif /* __CONSTANTS_H */