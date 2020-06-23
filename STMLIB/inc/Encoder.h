#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f4xx.h"
#include <math.h>


/* Hardware config */
/*----- Hardware Encoder config M1: PA6, PA7 ------*/
#define 				M1_TIMx								TIM3
#define					M1_GPIOx							GPIOA
#define					M1_RCC_PeriphClock					RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE)
#define					M1_RCC_AHB1Periph_GPIOx				RCC_AHB1Periph_GPIOA
#define					M1_GPIO_Pin_x1						GPIO_Pin_6	// PA6 = CHA M1
#define					M1_GPIO_Pin_x2						GPIO_Pin_7	// PA7 = CHB M1
#define					M1_GPIO_AF_TIMx						GPIO_AF_TIM3
#define					M1_GPIO_PinSourcex1					GPIO_PinSource6
#define					M1_GPIO_PinSourcex2					GPIO_PinSource7
#define					M1_TIMx_IRQn						TIM3_IRQn
/*----- Hardware Encoder config M2: PD12, PD13 ------*/
#define 				M2_TIMx								TIM4
#define					M2_GPIOx							GPIOD
#define					M2_RCC_PeriphClock					RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4,ENABLE)
#define					M2_RCC_AHB1Periph_GPIOx				RCC_AHB1Periph_GPIOD
#define					M2_GPIO_Pin_x1						GPIO_Pin_12 // PD12 = CHA M2
#define					M2_GPIO_Pin_x2						GPIO_Pin_13 // PD13 = CHB M2
#define					M2_GPIO_AF_TIMx						GPIO_AF_TIM4
#define					M2_GPIO_PinSourcex1					GPIO_PinSource12
#define					M2_GPIO_PinSourcex2					GPIO_PinSource13
#define					M2_TIMx_IRQn						TIM4_IRQn
/*----- Hardware PWM config for Motor 1 -*/
#define					PWM_TIMx							TIM9
#define					PWM_GPIOx							GPIOA
#define					PWM_GPIO_Pin_OC1					GPIO_Pin_2 	//TIM9_CH1: PA2=PWM+ M1; GND=PWM- M1
#define					PWM_GPIO_Pin_OC2					GPIO_Pin_3 	//TIM9_CH2: PA3=PWM+ M2; GND=PWM- M2
#define					PWM_GPIO_AF_TIMx					GPIO_AF_TIM9
#define					PWM_GPIO_PinSourceOC1				GPIO_PinSource2
#define					PWM_GPIO_PinSourceOC2				GPIO_PinSource3
#define					PWM_RCC_PeriphClock					RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9,ENABLE)
#define					PWM_RCC_AHB1Periph_GPIOx			RCC_AHB1Periph_GPIOA
/*----- Direction Pin  -----------------*/
#define					Dir_GPIOx							GPIOC
#define					Dir_RCC_AHB1Periph_GPIOx			RCC_AHB1Periph_GPIOC
#define					Dir_GPIO_Pin_M1						GPIO_Pin_3 //Dir pin for M1: PC3=DIR+ M1; GND=DIR- M1
#define					Dir_GPIO_Pin_M2						GPIO_Pin_4 //Dir pin for M2: PC4=DIR+ M2; GND=DIR- M2
/*----- Parameter define ----------------*/
#define         Frequency_20KHz							  5000	/* 100MHz/20KHz */
/* Data Types */
/* Export Functions */
#define M1_Forward()	GPIO_SetBits(Dir_GPIOx, Dir_GPIO_Pin_M1)
#define M1_Backward()	GPIO_ResetBits(Dir_GPIOx, Dir_GPIO_Pin_M1)
#define M2_Forward()	GPIO_SetBits(Dir_GPIOx, Dir_GPIO_Pin_M2)
#define M2_Backward()	GPIO_ResetBits(Dir_GPIOx, Dir_GPIO_Pin_M2)

#define Robot_Forward()\
	M1_Forward();\
	M2_Forward();


#define Robot_Backward()\
	M1_Backward();\
	M2_Backward();

#define Robot_Clockwise()\
	M1_Backward();\
	M2_Forward();

#define Robot_AntiClockwise()\
	M1_Forward();\
	M2_Backward();

#define Stop_Motor()\
	PWM_TIMx->CCR1   = 0;\
	PWM_TIMx->CCR2   = 0;

void 	Robot_Run(double duty_right, double duty_left);
void 	Encoder_Config(void);

#endif
