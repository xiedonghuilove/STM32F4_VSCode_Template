/**
  ******************************************************************************
  * @file    bsp_RS232_usart.c
  * @author  JPT
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   重定向c库printf函数到usart端口，中断接收模式
						 DMA发送数据，
  ******************************************************************************
  */

#include "bsp_usart.h"
#include "command_parse.h"
#include "data.h"
#include "bsp_warning.h"

uint8_t g_ucaUSART2_TX_BUF[USART_REC_LEN] = {0};     //接收缓冲,最大USART_REC_LEN个字节.
uint8_t g_ucaUSART3_TX_BUF[USART_REC_LEN] = {0};     //接收缓冲,最大USART_REC_LEN个字节.

uint8_t g_ucaUsart2RxBuf[50] = {0};
uint16_t g_usUsart2RxState = 0;     //接受状态标记

uint8_t g_ucSendOverFlag = 0;

UART_Rx_TypeDef	UART2_RxTmp,tUART3_RxTmp;
MasterUsartData_T		UART2_RxData = {HOST_FRAME_IDLE};
ModuleUsartData_T		tUART3_RxData = {HOST_FRAME_IDLE};
/*
*********************************************************************************************************
*	函 数 名: RS232_USART_Config
*	功能说明: 初始化串口硬件，并对全局变量赋初值.
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void RS232_USART_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	//使能时钟
	RCC_AHB1PeriphClockCmd(RS232_USART_RX_GPIO_CLK|RS232_USART_TX_GPIO_CLK,ENABLE);
	RCC_APB1PeriphClockCmd(RS232_USART_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(RS232_USART_DMA_CLK, ENABLE);

	/* GPIO初始化 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* 配置Tx引脚为复用功能  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = RS232_USART_TX_PIN  ;
	GPIO_Init(RS232_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	/* 配置Rx引脚为复用功能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = RS232_USART_RX_PIN;
	GPIO_Init(RS232_USART_RX_GPIO_PORT, &GPIO_InitStructure);

	/* 连接串口对应引脚复用映射*/
	GPIO_PinAFConfig(RS232_USART_RX_GPIO_PORT,RS232_USART_RX_SOURCE,RS232_USART_RX_AF);
	GPIO_PinAFConfig(RS232_USART_TX_GPIO_PORT,RS232_USART_TX_SOURCE,RS232_USART_TX_AF);

	/* 配置串RS232_USART 模式 */
	USART_InitStructure.USART_BaudRate = RS232_USART_BAUDRATE;/* 波特率设置：RS232_USART_BAUDRATE */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  /* 字长(数据位+校验位)：8 */
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  /* 停止位：1个停止位 */
	USART_InitStructure.USART_Parity = USART_Parity_No;  /* 校验位选择：不使用校验 */
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  /* 硬件流控制：不使用硬件流 */
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  /* USART模式控制：同时使能接收和发送 */
	USART_Init(RS232_USART, &USART_InitStructure);   /* 完成USART初始化配置 */
	/* 使能串口 */
	USART_Cmd(RS232_USART, ENABLE);

	/* 使能串口接收中断 */
	USART_ITConfig(RS232_USART, USART_IT_RXNE, ENABLE);

	/* 嵌套向量中断控制器NVIC配置 */
	NVIC_InitStructure.NVIC_IRQChannel = RS232_USART_IRQ;  /* 配置USART为中断源 */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  /* 抢断优先级为1 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  /* 子优先级为1 */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  /* 使能中断 */
	NVIC_Init(&NVIC_InitStructure);  /* 初始化配置NVIC */

	//DMA配置
	DMA_DeInit(RS232_USART_DMA_STREAM);	/* 复位初始化DMA数据流 */
	while (DMA_GetCmdStatus(RS232_USART_DMA_STREAM) != DISABLE)  {
	}  /* 确保DMA数据流复位完成 */

	DMA_InitStructure.DMA_Channel = RS232_USART_DMA_CHANNEL;//通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = RS232_USART_DR_BASE;//DMA外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)g_ucaUSART2_TX_BUF;  //内存地址(要传输的变量的指针)
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  //方向：从内存到外设
	DMA_InitStructure.DMA_BufferSize = SENDBUFF_SIZE; //传输大小DMA_BufferSize=SENDBUFF_SIZE
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址不增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址自增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //外设数据长度:8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //存储器数据长度:8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //DMA模式：单次
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  //优先级：中
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  //禁用FIFO
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;  //存储器突发传输 16个节拍
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;  //外设突发传输 1个节拍
	DMA_Init(RS232_USART_DMA_STREAM, &DMA_InitStructure);  //配置DMA2的数据流7

	DMA_Cmd(RS232_USART_DMA_STREAM, ENABLE);  /*使能DMA*/
	while(DMA_GetCmdStatus(RS232_USART_DMA_STREAM) != ENABLE)
	{
	}/* 等待DMA数据流有效*/

}

/*
*********************************************************************************************************
*	函 数 名: RS485_USART_Init
*	功能说明: 初始化串口硬件，并对全局变量赋初值.
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void RS485_USART_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	//使能时钟
	RCC_AHB1PeriphClockCmd(RS485_USART_RX_GPIO_CLK|RS485_USART_TX_GPIO_CLK,ENABLE);
	RCC_APB1PeriphClockCmd(RS485_USART_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(RS485_USART_DMA_CLK, ENABLE);
	RCC_AHB1PeriphClockCmd(RS485_EN_GPIO_CLK,ENABLE);//485_EN引脚

	/* GPIO初始化 */
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	/* 配置485_EN引脚为推挽输出  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Pin = RS485_EN_PIN  ;
	GPIO_Init(RS485_EN_GPIO_PORT, &GPIO_InitStructure);


	/* 配置Tx引脚为复用功能  */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = RS485_USART_TX_PIN  ;
	GPIO_Init(RS485_USART_TX_GPIO_PORT, &GPIO_InitStructure);

	/* 配置Rx引脚为复用功能 */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = RS485_USART_RX_PIN;
	GPIO_Init(RS485_USART_RX_GPIO_PORT, &GPIO_InitStructure);

	/* 连接串口对应引脚复用映射*/
	GPIO_PinAFConfig(RS485_USART_RX_GPIO_PORT,RS485_USART_RX_SOURCE,RS485_USART_RX_AF);
	GPIO_PinAFConfig(RS485_USART_TX_GPIO_PORT,RS485_USART_TX_SOURCE,RS485_USART_TX_AF);

	/* 配置串RS485_USART 模式 */
	USART_InitStructure.USART_BaudRate = RS485_USART_BAUDRATE;/* 波特率设置：RS485_USART_BAUDRATE */
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;  /* 字长(数据位+校验位)：8 */
	USART_InitStructure.USART_StopBits = USART_StopBits_1;  /* 停止位：1个停止位 */
	USART_InitStructure.USART_Parity = USART_Parity_No;  /* 校验位选择：不使用校验 */
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  /* 硬件流控制：不使用硬件流 */
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  /* USART模式控制：同时使能接收和发送 */
	USART_Init(RS485_USART, &USART_InitStructure);   /* 完成USART初始化配置 */
	/* 使能串口 */
	USART_Cmd(RS485_USART, ENABLE);

	/* 使能串口接收中断 */
	USART_ITConfig(RS485_USART, USART_IT_RXNE, ENABLE);
	/* 禁止发送完成中断 */
	USART_ITConfig(RS485_USART, USART_IT_TC, DISABLE);

	/* 嵌套向量中断控制器NVIC配置 */
	NVIC_InitStructure.NVIC_IRQChannel = RS485_USART_IRQ;  /* 配置USART为中断源 */
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  /* 抢断优先级为1 */
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;  /* 子优先级为1 */
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  /* 使能中断 */
	NVIC_Init(&NVIC_InitStructure);  /* 初始化配置NVIC */

	//DMA配置
	DMA_DeInit(RS485_USART_DMA_STREAM);	/* 复位初始化DMA数据流 */
	while (DMA_GetCmdStatus(RS485_USART_DMA_STREAM) != DISABLE)  {
	}  /* 确保DMA数据流复位完成 */

	DMA_InitStructure.DMA_Channel = RS485_USART_DMA_CHANNEL;//通道选择
	DMA_InitStructure.DMA_PeripheralBaseAddr = RS485_USART_DR_BASE;//DMA外设地址
	DMA_InitStructure.DMA_Memory0BaseAddr = (u32)g_ucaUSART3_TX_BUF;  //内存地址(要传输的变量的指针)
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;  //方向：从内存到外设
	DMA_InitStructure.DMA_BufferSize = SENDBUFF_485_SIZE; //传输大小DMA_BufferSize=SENDBUFF_SIZE
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;  //外设地址不增
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;  //内存地址自增
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;  //外设数据长度:8位
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte; //存储器数据长度:8位
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;  //DMA模式：单次
	DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;  //优先级：中
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;  //禁用FIFO
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;  //存储器突发传输 16个节拍
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;  //外设突发传输 1个节拍
	DMA_Init(RS485_USART_DMA_STREAM, &DMA_InitStructure);  //配置DMA2的数据流7

	DMA_Cmd(RS485_USART_DMA_STREAM, ENABLE);  /*使能DMA*/
	while(DMA_GetCmdStatus(RS485_USART_DMA_STREAM) != ENABLE)
	{
	}/* 等待DMA数据流有效*/

	RS485EN_RX();//默认为接受模式

}

/*
*********************************************************************************************************
*	函 数 名: RS232_USART_IRQHandler
*	功能说明: 串口中断服务函数  fe ef ff 03 10 11 12 13 14 55 aa
*	形    参:  无
*	返 回 值:  无
*********************************************************************************************************
*/
void RS232_USART_IRQHandler(void)
{
	uint8_t res = 0;
	uint16_t check_sum_data = 0;
	if(USART_GetITStatus( RS232_USART, USART_IT_RXNE ) != RESET)
	{
		USART_ClearITPendingBit(RS232_USART,USART_IT_RXNE);//清标志位
		res = USART_ReceiveData( RS232_USART );//读取接收到的数据 USART1->DR 自动清除标志位;
		if(UART2_RxData.FrameStatus == HOST_FRAME_IDLE)
			UART2_RxTmp.Cnt = 0;
		UART2_RxTmp.Buff[UART2_RxTmp.Cnt++] = res;//先取值，后自加
		switch (UART2_RxData.FrameStatus)
		{
			case HOST_FRAME_IDLE:
			case HOST_FRAME_HANDER://帧头  2字节 0xfe 0xef
				if(UART2_RxTmp.Cnt == 1)//接收到的第一个字节
				{
					if(res != 0xFE) UART2_RxData.FrameStatus = HOST_FRAME_IDLE;
					else UART2_RxData.FrameStatus = HOST_FRAME_HANDER;
				}
				if(UART2_RxTmp.Cnt == 2)//接收到第二个字节
				{
					if(res != 0xEF) UART2_RxData.FrameStatus = HOST_FRAME_IDLE;
					else UART2_RxData.FrameStatus = HOST_FRAME_SLAVEADDR;
				}
				break;
			case HOST_FRAME_SLAVEADDR://丛机地址   3
                UART2_RxData.FrameStatus = HOST_FRAME_FUNCODE;//读写功能
                break;
			case HOST_FRAME_FUNCODE://读写功能    4
                UART2_RxData.FrameStatus = HOST_FRAME_COMMAND;
                break;
			case HOST_FRAME_COMMAND://帧命令    5
                UART2_RxData.FrameStatus = HOST_FRAME_LENGTH;
                break;
			case HOST_FRAME_LENGTH://数据帧长    6
                UART2_RxData.FrameStatus = HOST_FRAME_DATA;
                break;
			case HOST_FRAME_DATA://帧数据    7 8 9 10
                if(UART2_RxTmp.Cnt == 10)
                    UART2_RxData.FrameStatus = HOST_FRAME_CRC;
                break;
			case HOST_FRAME_CRC://帧校验  11 12 0x55 0xaa
                if(UART2_RxTmp.Cnt == 12)
                {
                    check_sum_data = (UART2_RxTmp.Buff[10]<<8) | UART2_RxTmp.Buff[11];
                    if(0x55aa == check_sum_data)//校验正确 解析命令 错误则不解析
                    {
                        UART2_RxData.Package.FrameHander = (UART2_RxTmp.Buff[0]<<8) | UART2_RxTmp.Buff[1];
                        UART2_RxData.Package.SlaveAddr = UART2_RxTmp.Buff[2];
                        UART2_RxData.Package.Funcode = UART2_RxTmp.Buff[3];
                        UART2_RxData.Package.Command = UART2_RxTmp.Buff[4];
                        UART2_RxData.Package.Length = UART2_RxTmp.Buff[5];
                        UART2_RxData.Package.Data = (*(uint32_t*)(UART2_RxTmp.Buff+6));//6 7 8 9
                        UART2_RxData.Package.CheckSum = (UART2_RxTmp.Buff[10]<<8) | UART2_RxTmp.Buff[11];
                        PC_CommandParse(&UART2_RxData.Package,&tMasterData);
                    }
                    UART2_RxData.FrameStatus = HOST_FRAME_IDLE;//重新开始
                }
				break;
			default:
				break;
		}
	}

}
/*
*********************************************************************************************************
*	函 数 名: RS485_USART_IRQHandler
*	功能说明: 串口中断服务函数  fe ef ff 03 10 11 12 13 14 55 aa
*	形    参:  无
*	返 回 值:  无
*********************************************************************************************************
*/
void RS485_USART_IRQHandler(void)
{
	uint8_t res = 0,i = 0;
	uint16_t check_sum_data = 0;
	if(USART_GetITStatus( RS485_USART, USART_IT_RXNE ) != RESET)
	{
		USART_ClearITPendingBit(RS485_USART,USART_IT_RXNE);//接受完成清标志位
		res = USART_ReceiveData( RS485_USART );//读取接收到的数据 USART3->DR 自动清除标志位;
		if(tUART3_RxData.FrameStatus == HOST_FRAME_IDLE)
			tUART3_RxTmp.Cnt = 0;
		tUART3_RxTmp.Buff[tUART3_RxTmp.Cnt++] = res;//先取值，后自加
		switch (tUART3_RxData.FrameStatus)
		{
			case HOST_FRAME_IDLE:
			case HOST_FRAME_HANDER://帧头  2字节 0xfe 0xef
				if(tUART3_RxTmp.Cnt == 1)//接收到的第一个字节
				{
					if(res != 0xFE) tUART3_RxData.FrameStatus = HOST_FRAME_IDLE;
					else tUART3_RxData.FrameStatus = HOST_FRAME_HANDER;
				}
				if(tUART3_RxTmp.Cnt == 2)//接收到第二个字节
				{
					if(res != 0xEF) tUART3_RxData.FrameStatus = HOST_FRAME_IDLE;
					else tUART3_RxData.FrameStatus = HOST_FRAME_SLAVEADDR;
				}
				break;
			case HOST_FRAME_SLAVEADDR://丛机地址   3
                tUART3_RxData.FrameStatus = HOST_FRAME_FUNCODE;//读写功能
                break;
			case HOST_FRAME_FUNCODE://读写功能    4
                tUART3_RxData.FrameStatus = HOST_FRAME_COMMAND;
                break;
			case HOST_FRAME_COMMAND://帧命令    5
                tUART3_RxData.FrameStatus = HOST_FRAME_LENGTH;
                break;
			case HOST_FRAME_LENGTH://数据帧长    6
                tUART3_RxData.FrameStatus = HOST_FRAME_DATA;
                break;
			case HOST_FRAME_DATA://帧数据    7 8 9 10
                // if(tUART3_RxTmp.Cnt == 10)
                if(tUART3_RxTmp.Cnt == tUART3_RxTmp.Buff[5]+6)
                    tUART3_RxData.FrameStatus = HOST_FRAME_CRC;
                break;
			case HOST_FRAME_CRC://帧校验  11 12 0x55 0xaa
                if(tUART3_RxTmp.Cnt == tUART3_RxTmp.Buff[5]+6+2)
                {
                    check_sum_data = (tUART3_RxTmp.Buff[tUART3_RxTmp.Buff[5]+6]<<8) | tUART3_RxTmp.Buff[tUART3_RxTmp.Buff[5]+6+1];
                    if(0x55aa == check_sum_data)//校验正确 解析命令 错误则不解析
                    {
                        tUART3_RxData.Package.FrameHander = (tUART3_RxTmp.Buff[0]<<8) | tUART3_RxTmp.Buff[1];
                        tUART3_RxData.Package.SlaveAddr = tUART3_RxTmp.Buff[2];
                        tUART3_RxData.Package.Funcode = tUART3_RxTmp.Buff[3];
                        tUART3_RxData.Package.Command = tUART3_RxTmp.Buff[4];
                        tUART3_RxData.Package.Length = tUART3_RxTmp.Buff[5];
                        tUART3_RxData.Package.pData = (uint16_t*)(tUART3_RxTmp.Buff+6);//6 7
                        tUART3_RxData.Package.CheckSum = (tUART3_RxTmp.Buff[tUART3_RxTmp.Buff[5]+6]<<8) | tUART3_RxTmp.Buff[tUART3_RxTmp.Buff[5]+6+1];
                        // PC_CommandParse(&tUART3_RxData.Package,&tMasterData);
                        for (i = 1;i<=MODULE_NUM;i++)
                        {
                            if(tUART3_RxData.Package.SlaveAddr == i)break;
                        }

                        //急停报警，清除低压报警
                        if(tMasterData.AlarmLowBit & ERROR_MASTER_E_STOP)
                        {
                            taModuleData[i-1].SubControlAlarm &= ~ERROR_SUB_LOW_V;
                        }
                        
                        RS485_CommandParse(&tUART3_RxData.Package,tUART3_RxData.Package.Length/2, &tUART3_RxTmp.Buff[6], &taModuleData[i-1]);
                        tMasterData.Power = taModuleData[i-1].SubControlPower;
                        //添加模块异常
                        g_ulaModuleCommunicationCnt[i-1] = 0;

                    }
                    tUART3_RxData.FrameStatus = HOST_FRAME_IDLE;//重新开始
                }
				break;
			default:
				break;
		}

	}
	if(USART_GetITStatus( RS485_USART, USART_IT_TC ) != RESET)
	{
		USART_ClearITPendingBit(RS485_USART,USART_IT_TC);//发送完成清标志位

		RS485EN_RX();//发送完成切换为接受模式
		USART_ITConfig(RS485_USART, USART_IT_RXNE, ENABLE);/* 使能串口接收中断 */
		USART_ITConfig(RS485_USART, USART_IT_TC, DISABLE);/* 禁止发送完成中断 */
	}
}


///重定向c库函数printf到串口，重定向后可使用printf函数
int fputc(int ch, FILE *f)
{
	/* 发送一个字节数据到串口 */
	USART_SendData(RS232_USART, (uint8_t) ch);

	/* 等待发送完毕 */
	while (USART_GetFlagStatus(RS232_USART, USART_FLAG_TXE) == RESET);

	return (ch);
}

///重定向c库函数scanf到串口，重写向后可使用scanf、getchar等函数
int fgetc(FILE *f)
{
		/* 等待串口输入数据 */
	while (USART_GetFlagStatus(RS232_USART, USART_FLAG_RXNE) == RESET);

	return (int)USART_ReceiveData(RS232_USART);
}


/*********************************************END OF FILE**********************/
