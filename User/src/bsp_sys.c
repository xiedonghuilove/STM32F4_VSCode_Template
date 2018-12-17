#include "bsp_sys.h"

static uint8_t  fac_us=0;							//us延时倍乘数
//static uint16_t fac_ms=0;							//ms延时倍乘数,在os下,代表每个节拍的ms数

//初始化延迟函数
//当使用OS的时候,此函数会初始化OS的时钟节拍
//SYSTICK的时钟固定为AHB时钟的1/8
//SYSCLK:系统时钟频率
void delay_init(u8 SYSCLK)
{
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);//配置时钟源来源 HCLK  或者HCLK/8
  fac_us = SYSCLK;						//不论是否使用OS,fac_us都需要使用
//  fac_ms = (uint16_t)fac_us*1000;			//非OS下,代表每个ms需要的systick时钟数
}

//不用ucos时
//延时nus
//nus为要延时的us数.
//注意:nus的值,不要大于798915us(最大值即2^24/fac_us@fac_us=21)
void delay_us(u32 nus)
{
	u32 temp;
	SysTick->LOAD=nus*fac_us; 				//时间加载
	SysTick->VAL=0x00;        				//清空计数器
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk ; //使能Systick计数器开始倒数
	do
	{
		temp=SysTick->CTRL;//读取systick
	}while((temp&0x01)&&!(temp&(1<<16)));	//等待时间到达
	SysTick->CTRL&=~SysTick_CTRL_ENABLE_Msk; //关闭计数器
	SysTick->VAL =0X00;       				//清空计数器
}

//延时nms
//nms:0~65535
void delay_ms(u16 nms)
{
	u32 i;
	for(i=0;i<nms;i++) delay_us(1000);
}

/*
*********************************************************************************************************
*	函 数 名: SystemTaskTimer_Init
*	功能说明:  系统任务定时器初始化
*	形    参:  无
*	返 回 值: 无
*********************************************************************************************************
*/
void SystemTaskTimer_Init(uint16_t _psc,uint32_t _arr)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;

  //使能TIM5时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,ENABLE);

	TIM_TimeBaseInitStructure.TIM_Period =_arr; 								//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=_psc;          				//定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInit(TIM5,&TIM_TimeBaseInitStructure);    				//初始化TIM5

	TIM_Cmd(TIM5,ENABLE);
}

/*
*********************************************************************************************************
*	函 数 名: GetSystemTimerCnt
*	功能说明:  获取当前系统计数器值
*	形    参:  无
*	返 回 值: 计数器值
*********************************************************************************************************
*/
uint32_t GetSystemTimerCnt(void)
{
  return TIM5->CNT;
}

/*
*********************************************************************************************************
*	函 数 名: GetTimerDelay
*	功能说明:  获取当前系统计数器值
*	形    参:  无
*	返 回 值: 计数器值
*********************************************************************************************************
*/
uint32_t GetTimerDelay(uint32_t _lastTime)
{
  uint32_t time = 0;
  if(_lastTime > TIM5->CNT)
  {
    time = (0xFFFFFFFF -_lastTime + (TIM5->CNT));
  }
  else
  {
    time = (TIM5->CNT - _lastTime);
  }
  return time;
}
