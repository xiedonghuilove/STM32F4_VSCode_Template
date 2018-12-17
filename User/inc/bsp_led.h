#ifndef __LED_H
#define	__LED_H

#include "stm32f4xx.h"

/*
RUN_LED --> PE8
ERR_LED --> PE9
STATE1 --> PE10
STATE2 --> PE11

*/
//引脚定义
/*******************************************************/
//LED1
#define LED1_PIN                  GPIO_Pin_8
#define LED1_GPIO_PORT            GPIOE
#define LED1_GPIO_CLK             RCC_AHB1Periph_GPIOE

//LED2
#define LED2_PIN                  GPIO_Pin_9
#define LED2_GPIO_PORT            GPIOE
#define LED2_GPIO_CLK             RCC_AHB1Periph_GPIOE

//LED3
#define LED3_PIN                  GPIO_Pin_10
#define LED3_GPIO_PORT            GPIOE
#define LED3_GPIO_CLK             RCC_AHB1Periph_GPIOE

//LED4
#define LED4_PIN                  GPIO_Pin_11
#define LED4_GPIO_PORT            GPIOE
#define LED4_GPIO_CLK             RCC_AHB1Periph_GPIOE

/************************************************************/

#define LED1_Out()  PEout(8)   //LED1
#define LED2_Out()  PEout(9)   //LED2
#define LED3_Out()  PEout(10) //LED3
#define LED4_Out()  PEout(11) //LED4
/************************************************************/

void LED_GPIO_Config(void);

#endif /* __LED_H */
