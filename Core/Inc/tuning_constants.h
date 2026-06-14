

#ifndef __TUNING_CONSTNATS_H
#define __TUNING_CONSTNATS_H

#ifdef __cplusplus
extern "C" {
#endif

//values can be changed as needed

#define APPS1_UB    10
#define APPS1_LB    10
#define APPS2_UB    10
#define APPS2_LB    10

#define TPS1_UB     10
#define TPS1_LB     10
#define TPS2_UB     10
#define TPS2_LB     10

#define BS1_UB      10
#define BS1_LB      10
#define BS2_UB      10
#define BS2_LB      10

//the following slopes and intercepts and pid values are all divided by 1000 in the code. Multiply the desired value you are inputting by 1000 and input it here.

#define APPS_1_2_SLOPE      10
#define APPS_1_2_INTERCEPT  10
#define TPS_1_2_SLOPE       10
#define TPS_1_2_INTERCEPT   10
#define BSE_1_2_SLOPE       10
#define BSE_1_2_INTERCEPT   10

#define APPS_TO_TPS_TARGET_SLOPE        10
#define APPS_TO_TPS_TARGET_INTERCEPT    10

#define PID_FORWARD_P   10
#define PID_BACKWARD_P  10
#define PID_FORWARD_I   10
#define PID_BACKWARD_I  10
#define PID_FORWARD_D   10
#define PID_BACKWARD_D  10





#ifdef __cplusplus
}
#endif
#endif /* __TUNING_CONSTNATS_H */