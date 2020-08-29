#include "Encoder.h"

TIM_TimeBaseInitTypeDef 			En_TIM_BaseStruct;
GPIO_InitTypeDef 						En_GPIO_Struct;
NVIC_InitTypeDef						En_NVIC_Struct;

/**	@Brief: Configuration of Encoder for Motor 1
 	@Arg:   None 
 	@Retval: None
 **/
static void M1_Encoder_Config(void)
{
	RCC_AHB1PeriphClockCmd(M1_RCC_AHB1Periph_GPIOx, ENABLE);
	RCC_APB1PeriphClockCmd(M1_RCC_PeriphClock, ENABLE);
	/* Config PA8 as AF pin */
	En_GPIO_Struct.GPIO_Pin = M1_GPIO_Pin_x1 | M1_GPIO_Pin_x2;
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
	TIM_ARRPreloadConfig(M1_TIMx, ENABLE);
	TIM_EncoderInterfaceConfig(M1_TIMx, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_TimeBaseInit(M1_TIMx, &En_TIM_BaseStruct);
#ifdef ENCODER_IT
	/* Config interrupt */
	En_NVIC_Struct.NVIC_IRQChannel = M1_TIMx_IRQn;
	En_NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 2;
	En_NVIC_Struct.NVIC_IRQChannelSubPriority = 0;
	En_NVIC_Struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&En_NVIC_Struct);
	TIM_ITConfig(M1_TIMx, TIM_IT_Update, ENABLE);
#endif
	TIM_Cmd(M1_TIMx, ENABLE);
	TIM_SetCounter(M1_TIMx, 0);
}

/** @Brief: Configuration of Encoder for Motor 2
    @Arg:   None 
    @Retval: None
 **/
static void M2_Encoder_Config(void)
{
	RCC_AHB1PeriphClockCmd(M2_RCC_AHB1Periph_GPIOx, ENABLE);
	RCC_APB1PeriphClockCmd(M2_RCC_PeriphClock, ENABLE);
	/* Config PA8 as AF pin */
	En_GPIO_Struct.GPIO_Pin = M2_GPIO_Pin_x1|M2_GPIO_Pin_x2;
	En_GPIO_Struct.GPIO_Mode = GPIO_Mode_AF;
	En_GPIO_Struct.GPIO_OType = GPIO_OType_PP;
	En_GPIO_Struct.GPIO_Speed = GPIO_Speed_50MHz;
	En_GPIO_Struct.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(M2_GPIOx,&En_GPIO_Struct);
	GPIO_PinAFConfig(M2_GPIOx, M2_GPIO_PinSourcex1, M2_GPIO_AF_TIMx);
	GPIO_PinAFConfig(M2_GPIOx, M2_GPIO_PinSourcex2, M2_GPIO_AF_TIMx);
	/* Config Time Base */
	En_TIM_BaseStruct.TIM_Prescaler = 0;
	En_TIM_BaseStruct.TIM_Period = 0xFFFF;
	En_TIM_BaseStruct.TIM_CounterMode = TIM_CounterMode_Up;
	En_TIM_BaseStruct.TIM_ClockDivision = 0;
	TIM_TimeBaseInit(M2_TIMx, &En_TIM_BaseStruct); 
	TIM_ARRPreloadConfig(M2_TIMx, ENABLE); 
	TIM_EncoderInterfaceConfig(M2_TIMx, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
	TIM_TimeBaseInit(M2_TIMx, &En_TIM_BaseStruct);
#ifdef ENCODER_IT
	/* Config interrupt */
	En_NVIC_Struct.NVIC_IRQChannel = M2_TIMx_IRQn;
	En_NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 2;
	En_NVIC_Struct.NVIC_IRQChannelSubPriority = 0;
	En_NVIC_Struct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&En_NVIC_Struct);
	TIM_ITConfig(M2_TIMx, TIM_IT_Update, ENABLE);
#endif
	TIM_Cmd(M2_TIMx, ENABLE);
	TIM_SetCounter(M2_TIMx, 0);
}

/** @Brief:  PWM1 Config 2 pins
**  @Arg:    None
**  @Retval: None
**/
static void PWM_Config()
{
	TIM_OCInitTypeDef En_TIM_OCStruct;
	/* Enable RCC clock for each Peripheral */
	RCC_APB2PeriphClockCmd(PWM_RCC_PeriphClock, ENABLE);
	RCC_AHB1PeriphClockCmd(PWM_RCC_AHB1Periph_GPIOx, ENABLE);
	/* GPIO Config */
	En_GPIO_Struct.GPIO_Pin 		= 	PWM_GPIO_Pin_OC1|PWM_GPIO_Pin_OC2;
	En_GPIO_Struct.GPIO_Mode 		= 	GPIO_Mode_AF;
	En_GPIO_Struct.GPIO_OType 		= 	GPIO_OType_PP;
	En_GPIO_Struct.GPIO_Speed 		= 	GPIO_Speed_100MHz;
	En_GPIO_Struct.GPIO_PuPd 		= 	GPIO_PuPd_NOPULL;
	GPIO_Init(PWM_GPIOx,&En_GPIO_Struct);
	/* GPIO Config AF */
	GPIO_PinAFConfig(PWM_GPIOx, PWM_GPIO_PinSourceOC1, PWM_GPIO_AF_TIMx);
	GPIO_PinAFConfig(PWM_GPIOx, PWM_GPIO_PinSourceOC2, PWM_GPIO_AF_TIMx);
	/* Time Base Init */
	En_TIM_BaseStruct.TIM_Prescaler     = 0;
	En_TIM_BaseStruct.TIM_Period        = (GetTIMxFrequency(PWM_TIMx) / PWM_FREQUENCY) - 1;
	En_TIM_BaseStruct.TIM_CounterMode   = TIM_CounterMode_Up;
	En_TIM_BaseStruct.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseInit(PWM_TIMx, &En_TIM_BaseStruct);
	/* PWM Config */
	En_TIM_OCStruct.TIM_OCMode      = TIM_OCMode_PWM1;
	En_TIM_OCStruct.TIM_OutputState = TIM_OutputState_Enable;
	En_TIM_OCStruct.TIM_OCPolarity 	= TIM_OCPolarity_High;
	En_TIM_OCStruct.TIM_Pulse       = 0;
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
	
	En_GPIO_Struct.GPIO_Mode	= GPIO_Mode_OUT;
	En_GPIO_Struct.GPIO_Pin		= Dir_GPIO_Pin_M1|Dir_GPIO_Pin_M2;
	En_GPIO_Struct.GPIO_OType	= GPIO_OType_PP;
	En_GPIO_Struct.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	En_GPIO_Struct.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init(Dir_GPIOx, &En_GPIO_Struct);
}

void Robot_RunVersion1(double duty_v1, double duty_v2)
{
	if (duty_v1 >= 0 && duty_v2 >= 0) // v1 > 0 && v2 > 0
	{
		Robot_Forward();
		PWM_TIMx->CCR1 = (uint16_t)((duty_v1 * PWM_PERIOD) / 100);
		if (duty_v2 < 15)
			duty_v2 -= 3;
		PWM_TIMx->CCR2 = (uint16_t)((duty_v2 * PWM_PERIOD) / 100);
	} else if (duty_v1 >= 0 && duty_v2 <= 0) // v1 > 0 && v2 < 0
	{
		Robot_AntiClockwise();
		PWM_TIMx->CCR1 = (uint16_t)((duty_v1 * PWM_PERIOD) / 100);
		PWM_TIMx->CCR2 = (uint16_t)((-duty_v2 * PWM_PERIOD) / 100);
	} else if (duty_v1 <= 0 && duty_v2 >= 0) // v1 < 0 && v2 > 0
	{
		Robot_Clockwise();
		if (duty_v2 < 15)
			duty_v2 -= 3;
		PWM_TIMx->CCR1 = (uint16_t)((-duty_v1 * PWM_PERIOD) / 100);
		PWM_TIMx->CCR2 = (uint16_t)((duty_v2 * PWM_PERIOD) / 100);
	} else if (duty_v1 <= 0 && duty_v2 <= 0) // v1 < 0 && v2 < 0
	{
		Robot_Backward();
		PWM_TIMx->CCR1 = (uint16_t)((-duty_v1 * PWM_PERIOD) / 100);
		PWM_TIMx->CCR2 = (uint16_t)((-duty_v2 * PWM_PERIOD) / 100);
	} else {
		Stop_Motor();
	}
}

void Robot_RunVersion2(double duty_v1, double duty_v2)
{
	if(duty_v1 >= 0)
	{
		M1_Forward();
	}
	else 
	{
		duty_v1 = -duty_v1;
		M1_Backward();
	}

	if(duty_v2 >= 0)
	{
		M2_Forward();
	}
	else 
	{
		duty_v2 = -duty_v2;
		M2_Backward();
	}
	
	PWM_TIMx->CCR1 = (uint16_t)((duty_v1 * PWM_PERIOD) / 100);
	PWM_TIMx->CCR2 = (uint16_t)((duty_v2 * PWM_PERIOD) / 100);
}

void Encoder_Config(void)
{
	M1_Encoder_Config();
	M2_Encoder_Config();
	PWM_Config();
	DirPinConfig();
	Robot_Forward();
}

/*
 * 1 resolution = 39400 pulses encoder
 * 10ms velocity sampling -> 1 pulse/10ms ~ 100 pulses/s ~  0.9137 degree/s
 *                        or 1 pulse/10ms ~ 6000 pulses/m ~ 0.1523 rpm
 * Max Differential Encoder = 950 pulses (per 10 ms) ~ Max Velocity = 140 rpm, at PWM 92%
 */
void EncoderProcessing(DCMotor* Motor, TIM_TypeDef *TIMx, Time* pTime)
{
	Motor->pre_v = Motor->current_v;
	Motor->PreEnc = Motor->Enc;
	Motor->Enc = (TIMx == TIM4) ? (-TIMx->CNT) : TIMx->CNT;
	Motor->Diff_Encoder = Motor->Enc - Motor->PreEnc;

	if (Motor->Diff_Encoder > 30000)
		Motor->Diff_Encoder = Motor->Enc - Motor->PreEnc - 0xFFFF;
	else if (Motor->Diff_Encoder < -30000)
		Motor->Diff_Encoder = Motor->Enc - Motor->PreEnc + 0xFFFF;

	Motor->Total_Encoder += Motor->Diff_Encoder;
	Motor->current_v = (((double)Motor->Diff_Encoder / 39400) * 60) / pTime->velocity_T; // rpm

	/* set velocity handling */
	// Motor->current_set_v += Motor->delta_v;
	// if( ((Motor->delta_v > 0) && (Motor->current_set_v > Motor->target_v)) ||
	// 	((Motor->delta_v < 0) && (Motor->current_set_v < Motor->target_v))
	// 	)
	// 	Motor->current_set_v = Motor->target_v;
}

