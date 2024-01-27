
#include "FreeRTOS.h"
#include "portmacro.h"
#include "projdefs.h"
#include "task.h"
#include "_variables.h"
#include "_up_control_thread.h"
#include "tim.h"
#include <math.h>
#include <stdint.h>
#include "feetech.h"
#include "mapping.h"
#include "Unitree_user.h"
#include "event_groups.h"

xTaskHandle getBallTaskHandle;
UnitreeMotor *unitree_motor1;
float unitree_offset;
mapping_param_t feet_map;
void GetBallTask(void *argument);
void Unitree_start(UnitreeMotor * hm);
void arm_rotate(UnitreeMotor * hm,float pos,float kp,float kw,uint16_t time);
void up_reset();
void fjaw_close()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 102);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 68);
}
void fjaw_open_b()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 55);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 115);
}
void fjaw_open_s()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 80);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_4, 90);
}

void upjaw_close()
{
    // mapping_param_t servo1Map;
    // mapping_param_t servo2Map;
    // mapping_param_init(&servo1Map,0,1.57,0,125,75,125);
    // mapping_limit_o(&servo1Map,75,125);
    // mapping_param_init(&servo2Map,1.57,0,1.57,75,25,75);
    // mapping_limit_o(&servo2Map,50,75);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 120);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 45);
}
void upjaw_seize()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 115);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 50);
}
void upjaw_open()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 100);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 65);
}
void up_reset()
{
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    arm_rotate(unitree_motor1, 0.523,  0.05,  0.03, 300);
    Servo_Write_PosEx(feetservo, mapping_i2o(&feet_map, -1.5), 254, 254);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    upjaw_open();    
    fjaw_close();
}
// 上层控制线程
void StartUpControlTask(void *argument)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    unitree_motor1 = Unitree_Create_Motor();
    Unitree_start(unitree_motor1);
    mapping_param_init(&feet_map, 3.14, -3.14, 0, 0, 4096, 1592);
    if (Servo_Init(1, huart7)!=0)
            printf("feet init fail/n");

    up_reset();
    for (;;) {
        EventBits_t bit = xEventGroupWaitBits(UP_Control_Event_Handle,CHASE_BALL_EVENT|UP_RESET_EVENT, pdTRUE, pdFALSE, portMAX_DELAY);
        if(bit==CHASE_BALL_EVENT)
            xTaskCreate(GetBallTask, "get ball", 800, NULL, 2, &getBallTaskHandle);
        if(bit&UP_RESET_EVENT)
        {
            up_reset();
            vTaskDelete(getBallTaskHandle);
            xEventGroupClearBits(UP_Control_Event_Handle,CHASE_BALL_EVENT|PUT_BALL_EVENT|CATCH_BALL_EVENT);
        }   

    }
}
void GetBallTask(void *argument)
{
    fjaw_open_b();
    arm_rotate(unitree_motor1,-2.2,0.2,0.03,1000);
    upjaw_close();
    //等待球进入
    vTaskDelay(300/portTICK_PERIOD_MS);
    xEventGroupWaitBits(UP_Control_Event_Handle,(uint32_t)CATCH_BALL_EVENT, pdFALSE, pdTRUE, portMAX_DELAY);
    fjaw_open_s();
    vTaskDelay(1000/portTICK_PERIOD_MS);
    Servo_Write_PosEx(feetservo, mapping_i2o(&feet_map, 1.2), 50, 100);
    vTaskDelay(200/portTICK_PERIOD_MS);
    Unitree_UART_tranANDrev(unitree_motor1, 0, 1, 0, 0, 0, 0, 0.2);
    vTaskDelay(1500/portTICK_PERIOD_MS);
    upjaw_open();
    fjaw_close();
    
    arm_rotate(unitree_motor1,-1.2,0.2,0.03,2000);
    Servo_Write_PosEx(feetservo, mapping_i2o(&feet_map, 1), 254, 254);
    vTaskDelay(500/portTICK_PERIOD_MS);
    upjaw_close();
    vTaskDelay(300/portTICK_PERIOD_MS);
    Servo_Write_PosEx(feetservo, mapping_i2o(&feet_map, 1.5), 100, 254);
    arm_rotate(unitree_motor1,-4.4,0.2,0.08,1000);
    Servo_Write_PosEx(feetservo, mapping_i2o(&feet_map, 0), 254, 254);
    vTaskDelay(200/portTICK_PERIOD_MS);
    arm_rotate(unitree_motor1,-4.4,0.3,0.1,100);
    //准备放球
    vTaskDelay(300/portTICK_PERIOD_MS);
    xEventGroupWaitBits(UP_Control_Event_Handle,(uint32_t)PUT_BALL_EVENT, pdFALSE, pdTRUE, portMAX_DELAY);
    upjaw_open();
    vTaskDelay(1000/portTICK_PERIOD_MS);
    Servo_Write_PosEx(feetservo, mapping_i2o(&feet_map, -1.57), 254, 254);
    arm_rotate(unitree_motor1,-0.8,0.15,0.1,800);
    Unitree_UART_tranANDrev(unitree_motor1, 0, 1, 0, 0, 0, 0, 0.1);
    xEventGroupSetBits(UP_Control_Event_Handle, UP_RESET_EVENT);
    vTaskSuspend(NULL);
    for(;;)
    {
        vTaskDelay(1000/portTICK_PERIOD_MS);
    }
}

void arm_rotate(UnitreeMotor * hm,float pos,float kp,float kw,uint16_t time)
{
for (int i = 0; i < (int)time/5; i++) {
        //Servo_Write_PosEx(feetservo, mapping_i2o(&feet_map, 0), 254, 254);
        //Servo_Write_PosEx(&feetservo[0], 2000, 254, 254);
        float tf = 0.5 * sin(unitree_motor1->data.Pos / UNITREE_REDUCTION_RATE - unitree_offset);
        Unitree_UART_tranANDrev(unitree_motor1, 0, 1, tf, 0, pos+unitree_offset, kp, kw); // 0.2,0.03
        //printf("i:%d,mark:%d,pos:%d\n,tf:%d", i, uxTaskGetStackHighWaterMark(NULL), pos,(int)(200*tf));
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }
}
void Unitree_start(UnitreeMotor * hm)
{
    //r2从车宇树一侧向内看，顺时针旋转pos值增大
    while (Unitree_init(hm, &huart6) == HAL_ERROR) {
        vTaskDelay(100 / portTICK_RATE_MS);
        HAL_GPIO_TogglePin(LDR_GPIO_Port, LDR_Pin);
    }
    Unitree_UART_tranANDrev(hm, 0, 0, 0, 0, 0, 0, 0);
    vTaskDelay(100);
    Unitree_UART_tranANDrev(hm, 0, 0, 0, 0, 0, 0, 0);
    //新版unitree代码不要除减速比，对于r2夹爪，初始位置是让夹爪在车抓球侧自然下垂到两铝管接触(与垂直方向成30度)
    unitree_offset = hm->data.Pos/UNITREE_REDUCTION_RATE+0.5236;
}
// 抓球测试
//  void get_ball_test()
//  {
//  int l0,r0;
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