#include "stm32f411_RCCEnable.h"

/** @Brief: Enable or disable GPIOx clock (x : A,B,C,D,E,F,G,H,I,J)
**	@Args : TIMx and period
**	@Ret	: None
**/
void GetGPIOxClockCmd(GPIO_TypeDef *GPIOx, FunctionalState NewState)
{
	if(GPIOx == GPIOA)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,NewState);
	else if(GPIOx == GPIOB)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,NewState);
	else if(GPIOx == GPIOC)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,NewState);
	else if(GPIOx == GPIOD)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,NewState);
	else if(GPIOx == GPIOE)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE,NewState);
	else if(GPIOx == GPIOF)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF,NewState);
	else if(GPIOx == GPIOG)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG,NewState);
	else if(GPIOx == GPIOH)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOH,NewState);
	else if(GPIOx == GPIOI)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOI,NewState);
	else if(GPIOx == GPIOJ)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOJ,NewState);
}

/** @Brief: Enable or disable TIMx clock
**	@Args : TIMx and period
**	@Ret	: None
**/
void GetTIMxClockCmd(TIM_TypeDef *TIMx, FunctionalState NewState)
{
	if(TIMx == TIM1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1,NewState);
	else if(TIMx == TIM2)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2,NewState);
	else if(TIMx == TIM3)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,NewState);
	else if(TIMx == TIM4)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,NewState);
	else if(TIMx == TIM5)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5,NewState);
	else if(TIMx == TIM6)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6,NewState);
	else if(TIMx == TIM7)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7,NewState);
	else if(TIMx == TIM8)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8,NewState);
	else if(TIMx == TIM9)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,NewState);
	else if(TIMx == TIM10)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10,NewState);
	else if(TIMx == TIM11)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11,NewState);
	else if(TIMx == TIM12)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12,NewState);
	else if(TIMx == TIM13)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13,NewState);
}

/** @Brief: Get IRQn_Type for TIMx
**	@Args : TIMx
**	@Ret	: None
**/
IRQn_Type GetIRQHandlerFromTIMxCC(TIM_TypeDef *TIMx)
{
	if(TIMx == TIM1)
		return TIM1_CC_IRQn;
	else if(TIMx == TIM2)
		return TIM2_IRQn;
	else if(TIMx == TIM3)
		return TIM3_IRQn;
	else if(TIMx == TIM4)
		return TIM4_IRQn;
	else if(TIMx == TIM5)
		return TIM5_IRQn;
	else if(TIMx == TIM9)
		return TIM1_BRK_TIM9_IRQn;
	else if(TIMx == TIM10)
		return TIM1_UP_TIM10_IRQn;
	else
		return TIM1_TRG_COM_TIM11_IRQn;
}


/** @Brief: Enable or disable I2Cx clock
**	@Args : TIMx and period
**	@Ret	: None
**/
void GetI2CxClockCmd(I2C_TypeDef *I2Cx, FunctionalState NewState)
{
	if(I2Cx == I2C1)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1,NewState);
	else if(I2Cx == I2C2)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2,NewState);
	else if(I2Cx == I2C3)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C3,NewState);
}

/** @Brief: Enable or disable USARTx clock
**	@Args : TIMx and period
**	@Ret	: None
**/
void GetUSARTxClockCmd(USART_TypeDef *USARTx, FunctionalState NewState)
{
	if(USARTx == USART1)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,NewState);
	else if(USARTx == USART2)
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,NewState);
	else if(USARTx == USART6)
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,NewState);
}

/** @Brief: Enable or disable DMAx clock
**	@Args : Dma and state
**	@Ret	: None
**/
void GetDMAxClockCmd(DMA_TypeDef *DMAx, FunctionalState NewState)
{
	if(DMAx == DMA1)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1,NewState);
	else if(DMAx == DMA2)
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2,NewState);
}

/** @Brief: Get pin source from GPIO pin
**	@Args : GPIO_Pin
**	@Ret	: GPIO_PinSource
**/
uint8_t GetPinSourceFromGPIOPin(uint16_t GPIO_Pin)
{
	if(GPIO_Pin == 0x0001)
		return GPIO_PinSource0;
	else if(GPIO_Pin == 0x0002)
		return GPIO_PinSource1;
	else if(GPIO_Pin == 0x0004)
		return GPIO_PinSource2;
	else if(GPIO_Pin == 0x0008)
		return GPIO_PinSource3;
	else if(GPIO_Pin == 0x0010)
		return GPIO_PinSource4;
	else if(GPIO_Pin == 0x0020)
		return GPIO_PinSource5;
	else if(GPIO_Pin == 0x0040)
		return GPIO_PinSource6;
	else if(GPIO_Pin == 0x0080)
		return GPIO_PinSource7;
	else if(GPIO_Pin == 0x0100)
		return GPIO_PinSource8;
	else if(GPIO_Pin == 0x0200)
		return GPIO_PinSource9;
	else if(GPIO_Pin == 0x0400)
		return GPIO_PinSource10;
	else if(GPIO_Pin == 0x0800)
		return GPIO_PinSource11;
	else if(GPIO_Pin == 0x1000)
		return GPIO_PinSource12;
	else if(GPIO_Pin == 0x2000)
		return GPIO_PinSource13;
	else if(GPIO_Pin == 0x4000)
		return GPIO_PinSource14;
	else if(GPIO_Pin == 0x8000)
		return GPIO_PinSource15;
	else return 0;
}

/** @Brief: Get alternate funcion pin from TIM
**	@Args : TIMERs
**	@Ret	: GPIO_AF value
**/
uint8_t	GetAFFromTIM(TIM_TypeDef *TIMx)
{
	if((TIMx == TIM1) || (TIMx == TIM2))
		return 0x01;
	else if((TIMx == TIM3) || (TIMx == TIM4) || (TIMx == TIM5))
		return 0x02;
	else if((TIMx == TIM8) || (TIMx == TIM9) || (TIMx == TIM10) || (TIMx == TIM11))
		return 0x03;
	else return 0;
}


/** @Brief: Enable or disable SPI clock
**	@Args : SPI and state
**	@Ret	: None
**/
void GetSPIxPeriphClock(SPI_TypeDef *SPIx, FunctionalState NewState)
{
	if(SPIx == SPI1)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,NewState);
	}
	else if(SPIx == SPI2)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2,NewState);
	}
	else if(SPIx == SPI3)
	{
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3,NewState);
	}
	else if(SPIx == SPI4)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI4,NewState);
	}
	else if(SPIx == SPI5)
	{
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI5,NewState);
	}
}

/** @Brief: Enable or disable SPI clock
**	@Args : SPI and state
**	@Ret	: None
**/
uint8_t	GetAFFromSPI(SPI_TypeDef *pSPIx)
{
	return (uint8_t)0x05;
}




