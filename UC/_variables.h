//
// Created by tony on 23-11-4.
//

#ifndef _VARIABLES_H
#define _VARIABLES_H




#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
#include "steering_wheel_chassis.h"
#include "wtr_mavlink.h"

#define UP_RESET_EVENT 0x01
#define CHASE_BALL_EVENT 0x02
#define CATCH_BALL_EVENT 0x04
#define PUT_BALL_EVENT 0x08

extern EventGroupHandle_t UP_Control_Event_Handle;
extern xTaskHandle upControlTaskHandle;
extern xTaskHandle mavlinkTaskHandle;
extern xTaskHandle chassisTaskHandle;
extern swChassis_t mychassis;
extern mavlink_pose_t  actual_pose;
extern mavlink_speed_t mv_cmd;
extern SemaphoreHandle_t data_mutex;
extern SemaphoreHandle_t sync_mutex;
#ifdef __cplusplus
}
#endif
#endif //_VARIABLES_H
