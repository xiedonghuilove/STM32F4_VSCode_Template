/**
  ******************************************************************************
  * @file    bsp_os.c
  * @author  XDH
  * @version V1.0
  * @date    2018-xx-xx
  * @brief   文件描述
  ******************************************************************************
 */
#include "bsp_os.h"
#include "bsp_led.h"
#include "bsp_usart.h"
#include "bsp_dma.h"




/*
	全局运行时间，单位1ms
	最长可以表示 24.85天，如果你的产品连续运行时间超过这个数，则必须考虑溢出问题
*/
__IO int32_t g_iRunTime = 0;



void osRun_10MS_Task(void)
{
    LED1_Out() = !LED1_Out();
}
/*
*********************************************************************************************************
*   函 数 名: osGetTemp_500MS_Task
*   功能说明: 500MS延时任务
*   形    参: NULL
*   返 回 值: NULL
*********************************************************************************************************
*/
void osGetTemp_500MS_Task(void)
{
	
	LED2_Out() = !LED2_Out();

    if(DMA_GetFlagStatus(RS232_USART_DMA_STREAM,DMA_FLAG_TCIF7)!=RESET)//等待DMA2_Steam7传输完成
    {
        DMA_ClearFlag(RS232_USART_DMA_STREAM,DMA_FLAG_TCIF7);//清除DMA2_Steam7传输完成标志
    }
    DMA_Enable(RS232_USART_DMA_STREAM,15);     //开始一次DMA传输！
}


//测试 RS232 DMA发送模式
void task3(void)
{
    LED3_Out() = !LED3_Out();
//    printf("Task3\r\n");
}

/*
*********************************************************************************************************
*   函 数 名: osRun_1S_Task
*   功能说明: 运行1S任务
*   形    参: NULL
*   返 回 值: NULL
*********************************************************************************************************
*/
void osRun_1S_Task(void)
{
    LED4_Out() = !LED4_Out();
}

TaskStruct_T tasks[] =
{
   {0,250,250,osRun_10MS_Task},  //10ms 用各个任务的函数名初始化
   {0,500,500,osGetTemp_500MS_Task},	//500ms
   {0,1000,1000,task3},	//50ms
   {0,2000,2000,osRun_1S_Task}	//1000ms
};

//定义任务数量
u32 task_count = sizeof(tasks) / sizeof(TaskStruct_T);

/*
*********************************************************************************************************
*	函 数 名: bsp_GetRunTime
*	功能说明: 获取CPU运行时间，单位1ms。最长可以表示 24.85天，如果你的产品连续运行时间超过这个数，则必须考虑溢出问题
*	形    参:  无
*	返 回 值: CPU运行时间，单位1ms
*********************************************************************************************************
*/
int32_t bsp_GetRunTime(void)
{
	int32_t runtime;

	DISABLE_INT();  	/* 关中断 */

	runtime = g_iRunTime;	/* 这个变量在Systick中断中被改写，因此需要关中断进行保护 */

	ENABLE_INT();  		/* 开中断 */

	return runtime;
}
/*
*********************************************************************************************************
*   函 数 名: TaskSysClk_Init
*   功能说明: 任务系统时钟初始化
*   形    参: period        计数值
*             precaler     预分频值
*   返 回 值: NULL
*********************************************************************************************************
*/
void TaskSysClk_Init(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStruct;
    NVIC_InitTypeDef NVIC_InitStructure;
    RCC_ClocksTypeDef Get_RCC_Clocks;

    RCC_APB1PeriphClockCmd(OS_TASK_CLK,ENABLE);

    TIM_TimeBaseInitStruct.TIM_Prescaler = 1000-1;//定时器的预分频系数
    TIM_TimeBaseInitStruct.TIM_Period = Get_RCC_Clocks.SYSCLK_Frequency/1000000-1;//定时器的计数值
    TIM_TimeBaseInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStruct.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(OS_TASK_TIM,&TIM_TimeBaseInitStruct);
    TIM_ITConfig(OS_TASK_TIM,TIM_IT_Update,ENABLE); //允许更新中断

    NVIC_InitStructure.NVIC_IRQChannel = OS_TASK_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;//抢占优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_Init(&NVIC_InitStructure); //中断初始化

    TIM_Cmd(OS_TASK_TIM,ENABLE);//使能定时器
 }



 //TIMER3中断  1ms一次
void OS_TASK_IRQHandler(void)
{
	u8 i = 0;
	if (RESET != TIM_GetITStatus(OS_TASK_TIM,TIM_IT_Update))//检查TIM3更新中断发生与否
	{
		TIM_ClearITPendingBit(OS_TASK_TIM,TIM_IT_Update);
			
		/* 全局运行时间每1ms增1 */
		g_iRunTime++;
		if (g_iRunTime == 0x7FFFFFFF)	/* 这个变量是 int32_t 类型，最大数为 0x7FFFFFFF */
		{
			g_iRunTime = 0;
		}
		
		//时间片轮询
		for (i=0; i < task_count; ++i) //遍历任务数组
		{
			if (tasks[i].TimerSlice)  //判断时间片是否到了
			{
				--tasks[i].TimerSlice;
				if (0 == tasks[i].TimerSlice) //时间片到了
				{
					tasks[i].isRun = 0x01;//置位  表示任务可以执行
					tasks[i].TimerSlice = tasks[i].SliceNumber; //重新加载时间片值，为下次做准备
				}
			}
		}
	}
}


void Task_Process(void)
{
     u8 i = 0;
     for (i=0; i < task_count; ++i) //遍历任务数组
     {
         if (tasks[i].isRun) //若任务可执行，则执行任务
         {
             tasks[i].TaskPointer(); // 运行任务  --> 改变指针函数
             tasks[i].isRun = 0;//将标志位清零
         }
     }
}
