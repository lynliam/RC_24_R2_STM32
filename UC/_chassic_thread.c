////
//// Created by tony on 23-10-3.
////


// 底盘控制线程
#include "steering_wheel_chassis.h"
#include "cmsis_os.h"
//#include "ops.h"
#include "_variables.h"
//#include "stm32f427xx.h"
#include "stm32f4xx_hal_gpio.h"
#include "Event_Define.h"
swChassis_t mychassis;

void StartChassisTask(void *argument)
{
    //swChassis_init(&mychassis);
    // swChassis_startCorrect(&mychassis);
    HSM_CHASSIS_Init(&mychassis, "chassis");
    HSM_CHASSIS_Run(&mychassis, HSM_CHASSIS_START, NULL);

    for (;;) {
        swChassis_set_targetVelocity(&mychassis, mv_cmd.vx, mv_cmd.vy, mv_cmd.vw);
        HSM_CHASSIS_Run(&mychassis, Next_Event, NULL);
        //        for (int i=1;i<5;i++)
        //        {
        //          VESC_CAN_SET_ERPM(&hVESC[i-1],1000);
        ////            CanTransmit_DJI_1234(&hcan1,10,10,0,0);
        //
        //        }
        vTaskDelay(5 / portTICK_RATE_MS);
    }
}



void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    swChassis_EXTI_Callback(&mychassis, GPIO_Pin);
}
