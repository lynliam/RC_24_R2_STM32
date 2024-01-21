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


#define UP_RESET_EVENT 1
#define CHASE_BALL_EVENT 2
#define CATCH_BALL_EVENT 4
#define PUT_BALL_EVENT 8
// extern geometry_msgs__msg__Pose2D chassis_mv_cmd;
// extern geometry_msgs__msg__Pose2D chassis_actual_pose;

extern EventGroupHandle_t UP_Control_Event_Handle;

extern xTaskHandle upControlTaskHandle;
extern xTaskHandle microrosTaskHandle;
extern xTaskHandle chassisTaskHandle;
extern swChassis_t mychassis;
extern SemaphoreHandle_t data_mutex;
extern SemaphoreHandle_t sync_mutex;
#ifdef __cplusplus
}
#endif
#endif //_VARIABLES_H
