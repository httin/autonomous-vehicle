#ifndef ENCODER_H
#define ENCODER_H

#include "stm32f4xx.h"
#include "stm32f411_RCCEnable.h"
#include "functions.h"
#include <math.h>


/*----- Parameter define ----------------*/
#define         PWM_FREQUENCY       20000
#if defined(STM32F40_41xxx)
	#define     PWM_PERIOD          (168000000 / PWM_FREQUENCY)
#endif 

/*----- Hardware Encoder config M1: PA6, PA7 ------*/
#define 				M1_TIMx								TIM3
#define					M1_GPIOx							GPIOA
#define					M1_RCC_PeriphClock					RCC_APB1Periph_TIM3
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
#define					M2_RCC_PeriphClock					RCC_APB1Periph_TIM4
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
#define					PWM_RCC_PeriphClock					RCC_APB2Periph_TIM9
#define					PWM_RCC_AHB1Periph_GPIOx			RCC_AHB1Periph_GPIOA
/*----- Direction Pin  -----------------*/
#define					Dir_GPIOx							GPIOC
#define					Dir_RCC_AHB1Periph_GPIOx			RCC_AHB1Periph_GPIOC
#define					Dir_GPIO_Pin_M1						GPIO_Pin_3 //Dir pin for M1: PC3=DIR+ M1; GND=DIR- M1
#define					Dir_GPIO_Pin_M2						GPIO_Pin_4 //Dir pin for M2: PC4=DIR+ M2; GND=DIR- M2

/* Data Types */
/* Export Functions */
#ifdef REVERSE_MOTOR

#define M1_Forward()	GPIO_SetBits(Dir_GPIOx, Dir_GPIO_Pin_M1)
#define M1_Backward()	GPIO_ResetBits(Dir_GPIOx, Dir_GPIO_Pin_M1)
#define M2_Forward()	GPIO_SetBits(Dir_GPIOx, Dir_GPIO_Pin_M2)
#define M2_Backward()	GPIO_ResetBits(Dir_GPIOx, Dir_GPIO_Pin_M2)

#else

#define M1_Backward()	GPIO_SetBits(Dir_GPIOx, Dir_GPIO_Pin_M1)
#define M1_Forward()	GPIO_ResetBits(Dir_GPIOx, Dir_GPIO_Pin_M1)
#define M2_Backward()	GPIO_SetBits(Dir_GPIOx, Dir_GPIO_Pin_M2)
#define M2_Forward()	GPIO_ResetBits(Dir_GPIOx, Dir_GPIO_Pin_M2)

#endif

#define Robot_Forward()\
	M1_Forward();\
	M2_Forward();

#define Robot_Backward()\
	M1_Backward();\
	M2_Backward();

/* ROTATE RIGHT, velocity M1 is negative, velocity M2 is positive */
#define Robot_Clockwise()\
	M1_Backward();\
	M2_Forward();

/* ROTATE LEFT, velocity M1 is positive, velocity M2 is negative */
#define Robot_AntiClockwise()\
	M1_Forward();\
	M2_Backward();


#define Stop_Motor()\
	PWM_TIMx->CCR1 = 0;\
	PWM_TIMx->CCR2 = 0;

void Robot_RunVersion1(double duty_v1, double duty_v2);
void Robot_RunVersion2(double duty_v1, double duty_v2);
void Encoder_Config(void);
void EncoderProcessing(DCMotor* Motor, TIM_TypeDef *TIMx, Time* pTime);

#endif
