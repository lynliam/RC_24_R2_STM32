//
// Created by tony on 23-11-4.
//

#ifndef _VARIABLES_H
#define _VARIABLES_H




#include "std_msgs/msg/detail/u_int32__struct.h"
#ifdef __cplusplus
extern "C" {
#endif
#include <rclc/rclc.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/u_int32.h>
#include <std_msgs/msg/string.h>
#include <geometry_msgs/msg/pose2_d.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "event_groups.h"
#include "steering_wheel_chassis.h"


#define UP_RESET_EVENT 0x01
#define CHASE_BALL_EVENT 0x02
#define CATCH_BALL_EVENT 0x04
#define PUT_BALL_EVENT 0x08
extern geometry_msgs__msg__Pose2D chassis_mv_cmd;
extern geometry_msgs__msg__Pose2D chassis_actual_pose;
extern EventGroupHandle_t UP_Control_Event_Handle;

extern xTaskHandle upControlTaskHandle;
extern xTaskHandle mavlinkTaskHandle;
extern xTaskHandle chassisTaskHandle;
extern swChassis_t mychassis;
extern SemaphoreHandle_t data_mutex;
extern SemaphoreHandle_t sync_mutex;
#ifdef __cplusplus
}
#endif
#endif //_VARIABLES_H
