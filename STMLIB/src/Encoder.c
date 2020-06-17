#include "Encoder.h"

TIM_TimeBaseInitTypeDef 			En_TIM_BaseStruct;
GPIO_InitTypeDef 						En_GPIO_Struct;
NVIC_InitTypeDef						En_NVIC_Struct;

/* Functions of Encoder and PWM */

/** 
	@Brief: Configuration of Encoder for Motor 1
 	@Arg:   None 
 	@Retval: None
 **/
static void M1_Encoder_Config(void)
{
	RCC_AHB1PeriphClockCmd(M1_RCC_AHB1Periph_GPIOx, ENABLE);
	M1_RCC_PeriphClock;
	/* Config PA8 as AF pin */
	En_GPIO_Struct.GPIO_Pin = M1_GPIO_Pin_x1|M1_GPIO_Pin_x2;
	En_GPIO_Struct.GPIO_Mode = GPIO_Mode_AF;
	En_GPIO_Struct.GPIO_OType = GPIO_OType_PP;
	En_GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
	En_GPIO_Struct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(M1_GPIOx, &En_GPIO_Struct);
	GPIO_PinAFConfig(M1_GPIOx, M1_GPIO_PinSourcex1, M1_GPIO_AF_TIMx);
	GPIO_PinAFConfig(M1_GPIOx, M1_GPIO_PinSourcex2, M1_GPIO_AF_TIMx);
	/* Config Time Base */
	En_TIM_BaseStruct.TIM_Prescaler = 0;
	En_TIM_BaseStruct.TIM_Period = 0xFFFF;
	En_TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	En_TIM_BaseStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(M1_TIMx, &En_TIM_BaseStruct);
	TIM_ARRPreloadConfig(M1_TIMx,ENABLE);
	TIM_EncoderInterfaceConfig(M1_TIMx, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_TimeBaseInit(M1_TIMx, &En_TIM_BaseStruct);
	/* Config interrupt */
	En_NVIC_Struct.NVIC_IRQChannel = M1_TIMx_IRQn;
	En_NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 2;
	En_NVIC_Struct.NVIC_IRQChannelSubPriority = 0;
	En_NVIC_Struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&En_NVIC_Struct);
	TIM_ITConfig(M1_TIMx, TIM_IT_Update,ENABLE);
	TIM_Cmd(M1_TIMx, ENABLE);
}

/** @Brief: Configuration of Encoder for Motor 2
 *   @Arg:   None 
 *   @Retval: None
 **/
static void M2_Encoder_Config(void)
{
	RCC_AHB1PeriphClockCmd(M2_RCC_AHB1Periph_GPIOx,ENABLE);
	M2_RCC_PeriphClock;
	/* Config PA8 as AF pin */
	En_GPIO_Struct.GPIO_Pin = M2_GPIO_Pin_x1|M2_GPIO_Pin_x2;
	En_GPIO_Struct.GPIO_Mode = GPIO_Mode_AF;
	En_GPIO_Struct.GPIO_OType = GPIO_OType_PP;
	En_GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
	En_GPIO_Struct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(M2_GPIOx,&En_GPIO_Struct);
	GPIO_PinAFConfig(M2_GPIOx,M2_GPIO_PinSourcex1,M2_GPIO_AF_TIMx);
	GPIO_PinAFConfig(M2_GPIOx,M2_GPIO_PinSourcex2,M2_GPIO_AF_TIMx);
	/* Config Time Base */
	En_TIM_BaseStruct.TIM_Prescaler = 0;
	En_TIM_BaseStruct.TIM_Period = 0xFFFF;
	En_TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	En_TIM_BaseStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(M2_TIMx, &En_TIM_BaseStruct);
	TIM_ARRPreloadConfig(M2_TIMx, ENABLE);
	TIM_EncoderInterfaceConfig(M2_TIMx, TIM_EncoderMode_TI12,TIM_ICPolarity_Rising,TIM_ICPolarity_Rising);
	TIM_TimeBaseInit(M2_TIMx,&En_TIM_BaseStruct);
	/* Config interrupt */
	En_NVIC_Struct.NVIC_IRQChannel = M2_TIMx_IRQn;
	En_NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 2;
	En_NVIC_Struct.NVIC_IRQChannelSubPriority = 0;
	En_NVIC_Struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&En_NVIC_Struct);
	TIM_ITConfig(M2_TIMx, TIM_IT_Update, ENABLE);
	TIM_Cmd(M2_TIMx,ENABLE);
}

/** @Brief:  PWM1 Config 2 pins
**  @Arg:    None
**  @Retval: None
**/
static void PWM_Config(uint16_t freq)
{
	TIM_OCInitTypeDef 						En_TIM_OCStruct;
	/* Enable RCC clock for each Peripheral */
	PWM_RCC_PeriphClock;
	RCC_AHB1PeriphClockCmd(PWM_RCC_AHB1Periph_GPIOx, ENABLE);
	/* GPIO Config */
	En_GPIO_Struct.GPIO_Pin 			= 	PWM_GPIO_Pin_OC1|PWM_GPIO_Pin_OC2;
	En_GPIO_Struct.GPIO_Mode 			= 	GPIO_Mode_AF;
	En_GPIO_Struct.GPIO_OType 		= 	GPIO_OType_PP;
	En_GPIO_Struct.GPIO_Speed 		= 	GPIO_Speed_100MHz;
	En_GPIO_Struct.GPIO_PuPd 			= 	GPIO_PuPd_NOPULL;
	GPIO_Init(PWM_GPIOx,&En_GPIO_Struct);
	/* GPIO Config AF */
	GPIO_PinAFConfig(PWM_GPIOx, PWM_GPIO_PinSourceOC1, PWM_GPIO_AF_TIMx);
	GPIO_PinAFConfig(PWM_GPIOx, PWM_GPIO_PinSourceOC2, PWM_GPIO_AF_TIMx);
	/* Time Base Init */
	En_TIM_BaseStruct.TIM_Prescaler 					= 0;
	En_TIM_BaseStruct.TIM_Period 							= freq - 1;
	En_TIM_BaseStruct.TIM_CounterMode 				= TIM_CounterMode_Up;
	En_TIM_BaseStruct.TIM_ClockDivision 			= TIM_CKD_DIV1;
	TIM_TimeBaseInit(PWM_TIMx, &En_TIM_BaseStruct);
	/* PWM Config */
	En_TIM_OCStruct.TIM_OCMode 			= TIM_OCMode_PWM1;
	En_TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
	En_TIM_OCStruct.TIM_OCPolarity 	= TIM_OCPolarity_High;
	En_TIM_OCStruct.TIM_Pulse 			= 0;
	TIM_OC1Init(PWM_TIMx, &En_TIM_OCStruct);
	TIM_OC1PreloadConfig(PWM_TIMx, TIM_OCPreload_Enable);
	TIM_OC2Init(PWM_TIMx, &En_TIM_OCStruct);
	TIM_OC2PreloadConfig(PWM_TIMx, TIM_OCPreload_Enable);
	TIM_Cmd(PWM_TIMx,ENABLE);
}
/** @Brief:  2 pins control 2 drivers motor
**  @Arg:    None
**  @Retval: None
**/
static void DirPinConfig(void)
{
	RCC_AHB1PeriphClockCmd(Dir_RCC_AHB1Periph_GPIOx,ENABLE);
	
	En_GPIO_Struct.GPIO_Mode					= GPIO_Mode_OUT;
	En_GPIO_Struct.GPIO_Pin						= Dir_GPIO_Pin_M1|Dir_GPIO_Pin_M2;
	En_GPIO_Struct.GPIO_OType					= GPIO_OType_PP;
	En_GPIO_Struct.GPIO_PuPd					= GPIO_PuPd_NOPULL;
	En_GPIO_Struct.GPIO_Speed					= GPIO_Speed_50MHz;
	GPIO_Init(Dir_GPIOx,&En_GPIO_Struct);
}

void Robot_Run(double duty_right, double duty_left)
{
	duty_right = fabs(duty_right);
	duty_left  = fabs(duty_left);
	PWM_TIMx->CCR1 = (uint16_t)((duty_right * Frequency_20KHz) / 100);
	PWM_TIMx->CCR2 = (uint16_t)((duty_left * Frequency_20KHz) / 100);
}

void Encoder_Config(void)
{
	M1_Encoder_Config();
	M2_Encoder_Config();
	PWM_Config(Frequency_20KHz);
	DirPinConfig();
	Robot_Forward();
}
