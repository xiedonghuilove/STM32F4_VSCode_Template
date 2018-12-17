/**
  ******************************************************************************
  * @file    main.c
  * @author  XDH
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   文件描述
  ******************************************************************************
 */
#include "stm32f4xx.h"
#include "bsp_led.h"
#include "bsp_sys.h"
#include <stdint.h>
#include "bsp_os.h"

int main(void)
{
	/* 程序来到main函数之前，启动文件：statup_stm32f429xx.s已经调用
	* SystemInit()函数把系统时钟初始化成180MHZ
	* SystemInit()在system_stm32f4xx.c中定义
	* 如果用户想修改系统时钟，可自行编写程序修改
	*/
	delay_init(180);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  //配置中断优先级分组2 2位抢占式，
	LED_GPIO_Config();
	
	while(1)
	{
		Task_Process();
	}

}
