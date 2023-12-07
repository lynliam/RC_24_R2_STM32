#include "./bsp_GPIO_ENABLE.h"
#include "stm32f4xx_hal_gpio.h"

void MY_GPIO_Init()
{
	GPIO_InitTypeDef GPIO_LED_Init;

    __HAL_RCC_GPIOG_CLK_ENABLE();

    GPIO_LED_Init.Pin=GPIO_PIN_1;
    GPIO_LED_Init.Mode=GPIO_MODE_OUTPUT_PP;
    GPIO_LED_Init.Speed=GPIO_SPEED_FREQ_HIGH;
    GPIO_LED_Init.Pull=GPIO_NOPULL;

    HAL_GPIO_Init(GPIOG,&GPIO_LED_Init);

    GPIO_LED_Init.Pin=GPIO_PIN_2;
    HAL_GPIO_Init(GPIOG,&GPIO_LED_Init);

    GPIO_LED_Init.Pin=GPIO_PIN_3;
    HAL_GPIO_Init(GPIOG,&GPIO_LED_Init);

    GPIO_LED_Init.Pin=GPIO_PIN_4;
    HAL_GPIO_Init(GPIOG,&GPIO_LED_Init);

    GPIO_LED_Init.Pin=GPIO_PIN_5;
    HAL_GPIO_Init(GPIOG,&GPIO_LED_Init);

    GPIO_LED_Init.Pin=GPIO_PIN_6;
    HAL_GPIO_Init(GPIOG,&GPIO_LED_Init);

    GPIO_LED_Init.Pin=GPIO_PIN_7;
    HAL_GPIO_Init(GPIOG,&GPIO_LED_Init);

    GPIO_LED_Init.Pin=GPIO_PIN_8;
    HAL_GPIO_Init(GPIOG,&GPIO_LED_Init);


    //HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET);

}
