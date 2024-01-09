
#include "FreeRTOS.h"
#include "task.h"
#include "_variables.h"
#include "_up_control_thread.h"
#include "tim.h"
#include <stdint.h>
#include "FeeTech.h"
#include "mapping.h"
#include "Unitree_user.h"

xTaskHandle getBallTaskHandle;
UnitreeMotor *unitree_motor1;
float unitree_offset;
void GetBallTask(void *argument);
void Unitree_start(UnitreeMotor * hm);
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
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 115);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 50);
}
void upjaw_close()
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 100);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 65);
}
// 上层控制线程
void StartUpControlTask(void *argument)
{
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
    fjaw_close();



    unitree_motor1 = Unitree_Create_Motor();
    Unitree_start(unitree_motor1);
    // xSemaphoreTake(sync_mutex, portMAX_DELAY);
    // vTaskDelay(100 / portTICK_RATE_MS);
    // xSemaphoreGive(sync_mutex);

    xTaskCreate(GetBallTask, "get ball", 512, NULL, 2, &getBallTaskHandle);
    for (;;) {
        vTaskDelay(50 / portTICK_PERIOD_MS);
    }
}
void GetBallTask(void *argument)
{
    //ServoMotor MyServo();
    
    fjaw_open_b();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    mapping_param_t feet_map;
    mapping_param_init(&feet_map, 3.14, -3.14, 0, 0, 4096, 1592);
    int res = Servo_Init(1, huart7);
    if (res!=0)
        printf("feet init fail %d/n", res);

    fjaw_close();
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    for (int i = 0; i < 20000; i++) {
        //HAL_UART_Transmit(&huart7,(uint8_t *)a,4,200);
        //Servo_Write_PosEx(feetservo, mapping_i2o(&feet_map, 0), 254, 254);
        //Servo_Write_PosEx(&feetservo[0], 2000, 254, 254);
        float tf = 0 * sin(unitree_motor1->data.Pos / UNITREE_REDUCTION_RATE - unitree_offset);
        int pos  = 1000 * (unitree_motor1->data.Pos / UNITREE_REDUCTION_RATE);
        Unitree_UART_tranANDrev(unitree_motor1, 0, 1, 0, 0, 0, 0, 0); // 4.35,0.03
        printf("i:%d,mark:%d,pos:%d\n", i, uxTaskGetStackHighWaterMark(NULL), pos);
        vTaskDelay(5 / portTICK_PERIOD_MS);
    }

    vTaskDelete(NULL);
}

void Unitree_start(UnitreeMotor * hm)
{
    while (Unitree_init(hm, &huart6) == HAL_ERROR) {
        vTaskDelay(100 / portTICK_RATE_MS);
        HAL_GPIO_TogglePin(LDR_GPIO_Port, LDR_Pin);
    }
    Unitree_UART_tranANDrev(hm, 0, 0, 0, 0, 0, 0, 0);
    vTaskDelay(100);
    Unitree_UART_tranANDrev(hm, 0, 0, 0, 0, 0, 0, 0);
    //新版unitree代码不要除减速比
    unitree_offset = hm->data.Pos/UNITREE_REDUCTION_RATE;
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