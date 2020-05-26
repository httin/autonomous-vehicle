#include "stm32f4xx.h"
#include "stm32f411_RCCEnable.h"

void TIM16_Delay_US(TIM_TypeDef *TIMx, uint16_t period);
void TIM16_Delay_MS(TIM_TypeDef *TIMx, uint16_t period);
void Core_Delay(uint32_t count);

