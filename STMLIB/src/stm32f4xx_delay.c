#include "stm32f4xx_delay.h"

/** @Brief: TIMER 16 bits delay in us
**	@Args : TIMx and period
**	@Ret	: None
**/
void	TIM16_Delay_US(TIM_TypeDef *TIMx, uint16_t period)
{
		/** @Enable @RCC @Clock **/
		GetTIMxClockCmd(TIMx,ENABLE);
		TIMx->PSC = 49;    // STM32F411 has 16 bits TIM4 50MHz APB1 bus 50*10^6 / 50 = 1MHz => 1us
		TIMx->ARR = period - 1;   // agr 16 bits unsigned int
		TIMx->CNT = 0;
		TIMx->EGR = 1;		//update register;
		
		TIMx->SR = 0;			//clear overflow flag
		TIMx->CR1 = 1;    //Enable Timer
		while(!TIMx->SR);
		TIMx->CR1 = 0;		//Stop Timer
		GetTIMxClockCmd(TIMx,DISABLE);
}

/** @Brief: TIMER 16 bits delay in ms
**	@Args : TIMx and period
**	@Ret	: None
**/
void TIM16_Delay_MS(TIM_TypeDef *TIMx, uint16_t period)
{
		/** @Enable @RCC @Clock **/
		GetTIMxClockCmd(TIMx,ENABLE);
		TIM4->PSC = 4999;    // STM32F411 has 16 bits TIM4 50MHz APB1 bus 50*10^6 / 50 = 1MHz => 1us
		TIM4->ARR = period*20 - 1;   // agr 16 bits unsigned int
		TIM4->CNT = 0;
	
		TIM4->EGR = 1;		//update register;
		
		TIM4->SR = 0;			//clear overflow flag
		TIM4->CR1 = 1;    //Enable Timer
		while(!TIM4->SR);
		TIM4->CR1 = 0;		//Stop Timer
		GetTIMxClockCmd(TIMx,DISABLE);
}

void Core_Delay(uint32_t count)
{
		while(count--);
}
	
	
	

	
	
	
	

















