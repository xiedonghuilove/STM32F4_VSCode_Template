#ifndef __BSP_DMA_H
#define __BSP_DMA_H

#include "stm32f4xx.h"

void DMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx,u32 par,u32 mar,u16 ndtr);//配置DMAx_CHx
void DMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);	//使能一次DMA传输

#endif
