/**
  ******************************************************************************
  * @file    bsp_led.c
  * @author  JPT
  * @version V1.0
  * @date    2015-xx-xx
  * @brief   led应用函数接口
  ******************************************************************************
  */

#include "bsp_led.h"

 /**
  * @brief  初始化控制LED的IO
  * @param  无
  * @retval 无
  */
void LED_GPIO_Config(void)
{
	/*定义一个GPIO_InitTypeDef类型的结构体*/
	GPIO_InitTypeDef GPIO_InitStructure;

	/*开启LED相关的GPIO外设时钟*/
	RCC_AHB1PeriphClockCmd ( LED1_GPIO_CLK|
						   LED2_GPIO_CLK|
						   LED3_GPIO_CLK|
						   LED4_GPIO_CLK, ENABLE);

	/*选择要控制的GPIO引脚*/
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/*调用库函数，使用上面配置的GPIO_InitStructure初始化GPIO*/
	GPIO_InitStructure.GPIO_Pin = LED1_PIN;
	GPIO_Init(LED1_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LED2_PIN;
    GPIO_Init(LED2_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LED3_PIN;
    GPIO_Init(LED3_GPIO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = LED4_PIN;
    GPIO_Init(LED4_GPIO_PORT, &GPIO_InitStructure);
}
/*********************************************END OF FILE**********************/
