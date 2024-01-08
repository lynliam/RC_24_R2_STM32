
// #include "cmsis_os.h"
// #include "_chassis_thread.h"
#include "FreeRTOS.h"
#include "task.h"
#include "_variables.h"
// #include "_variables.h"
// #include "usart.h"
#include "tim.h"
#include <stdint.h>
// #include "FeeTech.hpp"
#include "mapping.h"
#include "Unitree_user.h"
// #include<math.h>
// #include "ops.h"
// #include "steering_wheel_chassis.h"
// Mapping feetMap;
// int p;
UnitreeMotor  * unitree_motor1;

void fjaw_close()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 105);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 65);
}
void fjaw_open_b()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 55);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 115);
}
void fjaw_open_s()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 75);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 95);
}

void upjaw_open()
{
    // mapping_param_t servo1Map;
    // mapping_param_t servo2Map;
    // mapping_param_init(&servo1Map,0,1.57,0,125,75,125);
    // mapping_limit_o(&servo1Map,75,125);
    // mapping_param_init(&servo2Map,1.57,0,1.57,75,25,75);
    // mapping_limit_o(&servo2Map,50,75);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,115);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,50);
}
void upjaw_close()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1,100);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2,65);
}
//上层控制线程
void StartUpControlTask(void *argument) {
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    fjaw_close();
    

    //夹爪功能被注释，勿动
    //Mapping servo1Map = Mapping(75, 115);
    //Mapping servo2Map = Mapping(40, 75);
    // //Genetic_Servo genetic_feet = Genetic_Servo(1700, 4000);
    // feetMap = Mapping(266, 3000);
    // FEET_Servo feet_servo;
    unitree_motor1 = Unitree_Create_Motor();
    
    // feet_servo.FEET_Servo_Init(huart7);
    // feetMap.set_trans_param_i2o(3.14, -3.14, 0, 0, 4096, 1592);
    while(Unitree_init(unitree_motor1,&huart6)==HAL_ERROR)
    {
        vTaskDelay(100 / portTICK_RATE_MS);
        HAL_GPIO_TogglePin(LDR_GPIO_Port, LDR_Pin);
    }
    // xSemaphoreTake(sync_mutex, portMAX_DELAY);
    // vTaskDelay(100 / portTICK_RATE_MS);
    // xSemaphoreGive(sync_mutex);
    uint32_t xLastWakeTime = xTaskGetTickCount();
    for (;;) {
    //     if (xSemaphoreTake(data_mutex, 100 / portTICK_RATE_MS) != pdTRUE)
    //         continue;
    //     //p =feet_servo.Servo_Read_Pos();
    //     feet_servo.Servo_Write_PosEx(feetMap.trans_i2o(msgj_in.position.data[1]), 254, 254);
    //     //Unitree_UART_tranANDrev(unitree_motor1, 0, 1, 0, 0, msgj_in.position.data[0]+unitree_offset, 0, 0);
    //    Unitree_UART_tranANDrev(unitree_motor1, 0, 1, tf, 0, msgj_in.position.data[0]-unitree_offset, 0.15, 0.1);

    //     pos_feedback.data.data[2] = msgj_in.position.data[2];
    //     pos_feedback.data.data[0]= unitree_motor1->data.Pos / UNITREE_REDUCTION_RATE+unitree_offset;
    //     pos_feedback.data.data[1]=(float)feet_servo.Servo_Read_Pos();
    //     xSemaphoreGive(data_mutex);
        vTaskDelayUntil(&xLastWakeTime, 5 / portTICK_RATE_MS);
    }
}
void GetBallTask()
{
    const float unitree_offset =-1.32;
    fjaw_open_b();
    vTaskDelay(1000/portTICK_PERIOD_MS);
    fjaw_close();
    vTaskDelay(1000/portTICK_PERIOD_MS);
    float tf = 0.5*sin(unitree_motor1->data.Pos / UNITREE_REDUCTION_RATE-unitree_offset);
    Unitree_UART_tranANDrev(unitree_motor1, 0, 1, tf, 0, 0, 0, 0);

}
//抓球测试
// void get_ball_test()
// {
     // int l0,r0;
    // mapping_param_t map_chan3;
    // mapping_param_t map_chan4;
    // mapping_param_init(&map_chan3, 0, 180, 0, 125, 25, 125);
    // mapping_limit_o(&map_chan3, 55, 105);    
    // mapping_param_init(&map_chan4, 0, 180, -35, 25, 125, 25);
    // mapping_limit_o(&map_chan4, 65, 115);
    // l0 = (int)mapping_i2o(&map_chan3, 90);
    // r0 = (int)mapping_i2o(&map_chan4, 90);
    // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, (int)mapping_i2o(&map_chan3, 90));
    // __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, (int)mapping_i2o(&map_chan4, 90));

//     uint8_t stage = 0;
//     for(;;)
//     {
//         if(stage<5)
//         {
//             jaw_open_b();
//             swChassis_set_targetVelocity(&mychassis,-6,0,0);
//             stage++;
//         }
//         else {
//             swChassis_set_targetVelocity(&mychassis,0,0,0);
//             vTaskDelay(200/portTICK_RATE_MS);
//             jaw_close();
            
//         }
//         vTaskDelay(200/portTICK_RATE_MS);
//     }
// }