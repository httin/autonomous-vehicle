#include "stm32f4xx.h"

/* Export functions */
/* GPIO Config section */
void 				GetGPIOxClockCmd(GPIO_TypeDef *GPIOx, FunctionalState NewState);
uint8_t 		GetPinSourceFromGPIOPin(uint16_t GPIO_Pin);
/* TIM Config section */
void 				GetTIMxClockCmd(TIM_TypeDef *TIMx, FunctionalState NewState);
uint8_t			GetAFFromTIM(TIM_TypeDef *TIMx);
IRQn_Type		GetIRQHandlerFromTIMxCC(TIM_TypeDef *TIMx);
/* DMA Config section */
void 				GetDMAxClockCmd(DMA_TypeDef *DMAx, FunctionalState NewState);
/* USART Config section */
void		  	GetUSARTxClockCmd(USART_TypeDef *USARTx, FunctionalState NewState);
/* I2C Config section */
void 				GetI2CxClockCmd(I2C_TypeDef *I2Cx, FunctionalState NewState);
/* SPI Config section */
void 				GetSPIxPeriphClock(SPI_TypeDef *SPIx, FunctionalState NewState);
uint8_t			GetAFFromSPI(SPI_TypeDef *pSPIx);




















