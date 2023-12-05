#ifndef EVENT_DEFINE_H
#define EVENT_DEFINE_H
#include "HStateMachine.h"
#ifdef __cplusplus
extern "C" {
#endif
#include "stm32f4xx.h"

#define HSM_RECEIVE_DATA (HSM_START)


HSM_STATE CHASSIS_Ready;
HSM_STATE CHASSIS_Correcting;
HSM_STATE CHASSIS_Running;
HSM_STATE CHASSIS_Stop;
HSM_STATE CHASSIS_Aimming;
HSM_STATE CHASSIS_Error;





#ifdef __cplusplus
}
#endif
#endif