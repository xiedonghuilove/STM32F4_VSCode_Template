#ifndef __BSP_OS_H__
#define __BSP_OS_H__

#include "stm32f4xx.h"
#include "stdio.h"
#include "bsp_sys.h"

//任务结构
typedef struct tagTaskStruct_T
{
  uint8_t isRun;    //表示任务是否运行
  uint16_t TimerSlice;   //分配给任务的时间片
  uint16_t SliceNumber; //时间片个数，在TimeSlice为0时，将其赋值给TimerSlice重新计数。
  void (*TaskPointer)(void);  //任务函数指针
}TaskStruct_T;

//数组大小
#define ARRAYSIZE(a) (sizeof(a) / sizeof((a)[0]))

#define OS_TASK_TIM            TIM3
#define OS_TASK_CLK            RCC_APB1Periph_TIM3
#define OS_TASK_IRQ            TIM3_IRQn
#define OS_TASK_IRQHandler     TIM3_IRQHandler

void Task_Process(void);
void TaskSysClk_Init(void);
int32_t bsp_GetRunTime(void);

#endif //__OS_H__
