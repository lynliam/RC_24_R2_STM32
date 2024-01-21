////
//// Created by tony on 23-10-3.
////


// 底盘控制线程
#include "steering_wheel_chassis.h"
#include "cmsis_os.h"
#include "ops.h"
#include "_variables.h"
//#include "stm32f427xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "Event_Define.h"
#include "stm32f4xx_hal_uart.h"
#include "usart.h"
#include <stdint.h>
#include "HWT101CT_sdk.h"
swChassis_t mychassis;

void StartChassisTask(void *argument)
{
    HW101_Init();
    HSM_CHASSIS_Init(&mychassis, "chassis");
    HSM_CHASSIS_Run(&mychassis, HSM_CHASSIS_START, NULL);
    HAL_UART_Receive_IT(&huart8, (uint8_t *)&ch, 1);
    for (;;) {
        //HAL_UART_Receive_DMA(&huart8, data, 28);
        //swChassis_set_targetVelocity(&mychassis, chassis_mv_cmd.x, chassis_mv_cmd.y, chassis_mv_cmd.theta);
        HSM_CHASSIS_Run(&mychassis, Next_Event, NULL);
        
        vTaskDelay(5 / portTICK_RATE_MS);
    }
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    swChassis_EXTI_Callback(&mychassis, GPIO_Pin);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == OPS_UART) {
        OPS_Decode();
    }
    WIT_UART_RxCpltCallback(huart);
}
