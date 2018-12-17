#ifndef __BSP_USART_H
#define	__BSP_USART_H

#include "stm32f4xx.h"
#include <stdio.h>
#include "bsp_sys.h"

#define USART_REC_LEN  						100  	//定义最大接收字节数 200

//引脚定义
/*
	RS232_RX  --> PD6 --> USART2_RX
	RS232_TX  --> PD5 --> USART2_TX

	DMA1_Stream6
	DMA_CHANNEL_4

*/
/*******************************************************/
#define RS232_USART                             USART2
#define RS232_USART_CLK                         RCC_APB1Periph_USART2
#define RS232_USART_BAUDRATE                    115200  //串口波特率

//DMA
#define RS232_USART_DR_BASE               (USART2_BASE+0x04)//USART2->DR寄存器
#define SENDBUFF_SIZE                     15				//发送的数据量
#define RS232_USART_DMA_CLK               RCC_AHB1Periph_DMA1
#define RS232_USART_DMA_CHANNEL           DMA_Channel_4
#define RS232_USART_DMA_STREAM            DMA1_Stream6

#define RS232_USART_RX_GPIO_PORT                GPIOD
#define RS232_USART_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOD
#define RS232_USART_RX_PIN                      GPIO_Pin_6
#define RS232_USART_RX_AF                       GPIO_AF_USART2
#define RS232_USART_RX_SOURCE                   GPIO_PinSource6

#define RS232_USART_TX_GPIO_PORT                GPIOD
#define RS232_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOD
#define RS232_USART_TX_PIN                      GPIO_Pin_5
#define RS232_USART_TX_AF                       GPIO_AF_USART2
#define RS232_USART_TX_SOURCE                   GPIO_PinSource5

#define RS232_USART_IRQHandler                  USART2_IRQHandler
#define RS232_USART_IRQ                 		    USART2_IRQn

extern uint8_t g_ucaUSART2_TX_BUF[USART_REC_LEN];
/************************************************************/
//引脚定义
/*
	RS485_RX  --> PC11 --> USART3_RX
	RS485_TX  --> PC10 --> USART3_TX

	RS485_EN --> PA15

	DMA1_Stream3
	DMA_CHANNEL_4

*/
/*******************************************************/
#define RS485_USART                             USART3
#define RS485_USART_CLK                         RCC_APB1Periph_USART3
#define RS485_USART_BAUDRATE                    115200  //串口波特率

//DMA
#define RS485_USART_DR_BASE               (USART3_BASE+0x04)//USART2->DR寄存器
#define SENDBUFF_485_SIZE                     15				//发送的数据量
#define RS485_USART_DMA_CLK               RCC_AHB1Periph_DMA1
#define RS485_USART_DMA_CHANNEL           DMA_Channel_4
#define RS485_USART_DMA_STREAM            DMA1_Stream3

#define RS485_USART_RX_GPIO_PORT                GPIOC
#define RS485_USART_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOC
#define RS485_USART_RX_PIN                      GPIO_Pin_11
#define RS485_USART_RX_AF                       GPIO_AF_USART3
#define RS485_USART_RX_SOURCE                   GPIO_PinSource11

#define RS485_USART_TX_GPIO_PORT                GPIOC
#define RS485_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOC
#define RS485_USART_TX_PIN                      GPIO_Pin_10
#define RS485_USART_TX_AF                       GPIO_AF_USART3
#define RS485_USART_TX_SOURCE                   GPIO_PinSource10

#define RS485_USART_IRQHandler                  USART3_IRQHandler
#define RS485_USART_IRQ                 		USART3_IRQn

#define RS485_EN_GPIO_PORT                      GPIOA
#define RS485_EN_GPIO_CLK                       RCC_AHB1Periph_GPIOA
#define RS485_EN_PIN                            GPIO_Pin_15

#define RS485EN_RX()       PAout(15)=0
#define RS485EN_TX()       PAout(15)=1

extern uint8_t g_ucaUSART3_TX_BUF[USART_REC_LEN];

/************************************************************/


typedef struct
{
	uint8_t Buff[USART_REC_LEN];
	uint8_t Cnt;
}UART_Rx_TypeDef;

typedef enum
{
	HOST_FRAME_IDLE=0,//空闲状态
	HOST_FRAME_HANDER,//帧头
  	HOST_FRAME_SLAVEADDR,//丛机地址
	HOST_FRAME_FUNCODE,//读写功能码
	HOST_FRAME_COMMAND,//帧命令
  	HOST_FRAME_LENGTH,//长度
	HOST_FRAME_DATA,//帧数据
	HOST_FRAME_CRC,
	HOST_FRAME_FINISH,
}MasterUsartRxFrame_E;

typedef struct
{
	uint16_t FrameHander;//帧头
	uint8_t SlaveAddr;//丛机地址 ID
	uint8_t Funcode;//功能码
  	uint8_t Command;//命令
  	uint8_t Length;//长度
  	uint32_t Data;//数据 --> 只对写有效
	uint16_t CheckSum;//CRC校验
}MasterPacket_T;

typedef struct
{
    MasterUsartRxFrame_E FrameStatus;//帧状态
    MasterPacket_T Package;//帧包
}MasterUsartData_T;
extern MasterUsartData_T		UART2_RxData;

typedef struct
{
    uint16_t FrameHander;//帧头
    uint8_t SlaveAddr;//丛机地址 ID
    uint8_t Funcode;//功能码
    uint8_t Command;//命令
    uint8_t Length;//长度
    uint16_t* pData;//数据 --> 只对写有效
    uint16_t CheckSum;//CRC校验
}ModulePacket_T;

typedef struct
{
    MasterUsartRxFrame_E FrameStatus;//帧状态
    ModulePacket_T Package;//帧包
}ModuleUsartData_T;
extern ModuleUsartData_T tUART3_RxData;

extern uint8_t g_ucSendOverFlag;

void RS232_USART_Config(void);
void RS485_USART_Init(void);


#endif /* __USART1_H */
