//
// Created by tony on 23-11-2.
//

#include "_micro_ros_thread.h"
#include "FreeRTOS.h"
#include "main.h"
#include "cmsis_os.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <stdbool.h>
#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>
#include "usart.h"
#include "_variables.h"
#include "ops.h"

#define M_PI		3.14159265358979323846
bool cubemx_transport_open(struct uxrCustomTransport *transport);
bool cubemx_transport_close(struct uxrCustomTransport *transport);
size_t cubemx_transport_write(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err);
size_t cubemx_transport_read(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err);
void *microros_allocate(size_t size, void *state);
void microros_deallocate(void *pointer, void *state);
void *microros_reallocate(void *pointer, size_t size, void *state);
void *microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void *state);



rcl_subscription_t chassis_mv_cmd_subscriber;
rcl_subscription_t up_control_cmd_subscriber;

rcl_publisher_t debug_publisher;

rcl_publisher_t actual_pose_publisher;
rcl_timer_t timer1;

std_msgs__msg__String debugmsg;
std_msgs__msg__UInt32 up_control_cmd_msg;


geometry_msgs__msg__Pose2D chassis_actual_pose;
geometry_msgs__msg__Pose2D _chassis_actual_pose;
geometry_msgs__msg__Pose2D _chassis_mv_cmd;
geometry_msgs__msg__Pose2D chassis_mv_cmd;
//接收回调函数

//2
void chassis_mv_cmd_subscribe_callback(const void *msgin){
    const geometry_msgs__msg__Pose2D *_chassis_mv_cmd = (const geometry_msgs__msg__Pose2D *) msgin;
    swChassis_set_targetVelocity(&mychassis, _chassis_mv_cmd->x, _chassis_mv_cmd->y, _chassis_mv_cmd->theta);
}
void up_control_cmd_subscribe_callback(const void *msgin){
    const std_msgs__msg__UInt32 *_up_control_cmd_msg = (const std_msgs__msg__UInt32 *) msgin;
    //?
    xEventGroupSetBits(UP_Control_Event_Handle,_up_control_cmd_msg->data);
}

void timer1_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    debugmsg.data.capacity = 30;
    debugmsg.data.size = 30;
    debugmsg.data.data = (char *) pvPortMalloc(20 * sizeof(char));
    //sprintf(debugmsg.data.data, "time:%dwater:%d heap:%d", xTaskGetTickCount(),uxTaskGetStackHighWaterMark(microrosTaskHandle),xPortGetFreeHeapSize());
    sprintf(debugmsg.data.data, "time:%d event %d", xTaskGetTickCount(),up_control_cmd_msg.data);
    rcl_publish(&debug_publisher, &debugmsg, NULL);
    vPortFree(debugmsg.data.data);
    taskENTER_CRITICAL();
    _chassis_actual_pose.x = OPS_Data.pos_y/1000.f;
    _chassis_actual_pose.y = -OPS_Data.pos_x/1000.f;
    _chassis_actual_pose.theta = OPS_Data.z_angle/180.f*3.1415926f;
    taskEXIT_CRITICAL();
    rcl_publish(&actual_pose_publisher, &_chassis_actual_pose, NULL);

}

void StartMicrorosTask(void *argument) {

    HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_SET);
    if (rmw_uros_set_custom_transport(
            true,
            (void *) &huart3,
            cubemx_transport_open,
            cubemx_transport_close,
            cubemx_transport_write,
            cubemx_transport_read) == RMW_RET_ERROR)
        HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_RESET);

    rcl_allocator_t freeRTOS_allocator = rcutils_get_zero_initialized_allocator();
    freeRTOS_allocator.allocate = microros_allocate;
    freeRTOS_allocator.deallocate = microros_deallocate;
    freeRTOS_allocator.reallocate = microros_reallocate;
    freeRTOS_allocator.zero_allocate = microros_zero_allocate;

    if (!rcutils_set_default_allocator(&freeRTOS_allocator))
        HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_RESET);
    // micro-ROS app
    rclc_support_t support;
    rcl_allocator_t allocator;
    rclc_executor_t executor;
    rcl_node_t node;
    allocator = rcl_get_default_allocator();
    //create support
    HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_RESET);
    if (rclc_support_init(&support, 0, NULL, &allocator)
        == RCL_RET_OK)
        HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_SET);

    // create node
    HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_RESET);
    if (rclc_node_init_default(&node, "stm32_node", "", &support)
        == RCL_RET_OK)
        HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_SET);
    std_msgs__msg__String__init(&debugmsg);
    // HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_SET);
    // // create publisher
    // 1.debug_publisher
    HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_RESET);
    if (rclc_publisher_init_default(
            &debug_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
            "debug")
        == RCL_RET_OK)
        HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_SET);

    // 3.actual_pose_publisher
    HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_RESET);
    if (rclc_publisher_init_default(
            &actual_pose_publisher,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D),
            "chassis_actual_pose")
        == RCL_RET_OK)
        HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_SET);
    //create subscriber
    // 1.chassis_mv_cmd_subscriber
    HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_RESET);
    if (rclc_subscription_init_default(
            &chassis_mv_cmd_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Pose2D),
            "chassis_mv_cmd")
        == RCL_RET_OK)
    HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_SET);
    //2.up_control_subscriber
    HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_RESET);
    if (rclc_subscription_init_default(
            &up_control_cmd_subscriber,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
            "up_cmd")
        == RCL_RET_OK)
    HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_SET);
    //create timer
    HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_RESET);
    if( rclc_timer_init_default(
            &timer1,
            &support,
            RCL_MS_TO_NS(50),
            timer1_callback)
        == RCL_RET_OK)
        HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_SET);
    //create executor
    executor = rclc_executor_get_zero_initialized_executor();
    HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_RESET);
    if (rclc_executor_init(&executor, &support.context, 3, &allocator)
        == RCL_RET_OK)
        HAL_GPIO_WritePin(LDR_GPIO_Port, LDR_Pin, GPIO_PIN_SET);
    //add subscriber and timer to executor
    rclc_executor_add_subscription(
            &executor,
            &chassis_mv_cmd_subscriber,
            &_chassis_mv_cmd,
            &chassis_mv_cmd_subscribe_callback,
            ON_NEW_DATA);
    rclc_executor_add_subscription(
            &executor,
            &up_control_cmd_subscriber,
            &up_control_cmd_msg,
            &up_control_cmd_subscribe_callback,
            ON_NEW_DATA);            
    rclc_executor_add_timer(&executor, &timer1);
    rclc_executor_spin(&executor);
    for (;;) {

    }
}
