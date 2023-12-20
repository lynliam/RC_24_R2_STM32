#include "main.h"
#include "cmsis_os.h"
#include "_chassis_thread.h"

#include "_variables.h"
#include "usart.h"
#include "tim.h"
#include "FeeTech.hpp"
#include "mapping.hpp"
#include "Unitree_user.h"
#include<math.h>
#include "ops.h"
#include "steering_wheel_chassis.h"
// const float unitree_offset =0.3;
// Mapping feetMap;
// int p;

//上层控制线程
void StartUpControlTask(void *argument) {

    // Mapping servo1Map = Mapping(75, 115);
    // Mapping servo2Map = Mapping(40, 75);
    // //Genetic_Servo genetic_feet = Genetic_Servo(1700, 4000);
    // feetMap = Mapping(266, 3000);
    // FEET_Servo feet_servo;
    // UnitreeMotor  * unitree_motor1 =Unitree_Create_Motor();
    // feet_servo.FEET_Servo_Init(huart7);
    // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    // servo1Map.set_trans_param_i2o(0, 1.57, 0, 125, 75, 125);
    // servo2Map.set_trans_param_i2o(1.57, 0, 1.57, 75, 25, 75);
    // feetMap.set_trans_param_i2o(3.14, -3.14, 0, 0, 4096, 1592);
    // while(Unitree_init(unitree_motor1,&huart6)==HAL_ERROR)
    // {
    //     vTaskDelay(100 / portTICK_RATE_MS);
    //     HAL_GPIO_TogglePin(LDG_GPIO_Port, LDG_Pin);
    // }
    // xSemaphoreTake(sync_mutex, portMAX_DELAY);
    // vTaskDelay(100 / portTICK_RATE_MS);
    // xSemaphoreGive(sync_mutex);
    // uint32_t xLastWakeTime = xTaskGetTickCount();
    // for (;;) {
    //     if (xSemaphoreTake(data_mutex, 100 / portTICK_RATE_MS) != pdTRUE)
    //         continue;

    //     __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, servo1Map.trans_i2o(msgj_in.position.data[2]));
    //     __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, servo2Map.trans_i2o(msgj_in.position.data[2]));
    //     //p =feet_servo.Servo_Read_Pos();
    //     feet_servo.Servo_Write_PosEx(feetMap.trans_i2o(msgj_in.position.data[1]), 254, 254);
    //     float tf = 0.7*sin(unitree_motor1->data.Pos / UNITREE_REDUCTION_RATE-unitree_offset);
    //     //Unitree_UART_tranANDrev(unitree_motor1, 0, 1, 0, 0, msgj_in.position.data[0]+unitree_offset, 0, 0);
    //     Unitree_UART_tranANDrev(unitree_motor1, 0, 1, tf, 0, msgj_in.position.data[0]-unitree_offset, 0.15, 0.1);
    //     pos_feedback.data.data[2] = msgj_in.position.data[2];
    //     pos_feedback.data.data[0]= unitree_motor1->data.Pos / UNITREE_REDUCTION_RATE+unitree_offset;
    //     pos_feedback.data.data[1]=(float)feet_servo.Servo_Read_Pos();
    //     xSemaphoreGive(data_mutex);
    //     vTaskDelayUntil(&xLastWakeTime, 100 / portTICK_RATE_MS);
    // }
}