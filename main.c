#include "main.h"

/** @Control @Characters **/
/** Motor Control
 ** @A  : 
 ** @W  : 
 ** @S  : 
 ** @D  : 
 ** @C  : Config Peripheral.
 ** Format: C , v , angle , M1kp , M1kd , M1ki , M2kp , M2kd , M2ki
 ** Send PC format: VelM1,VelM2,angle,Y/N,xcor,ycor
 **/

/******************************************************************
 ************************ USED PERIPHERALS ************************
 * TIM3: Encoder Motor1, PA6(TIM3_CH1) & PA7(TIM3_CH2)
 * TIM4: Encoder Motor2, PD12(TIM4_CH1) & PD13(TIM4_CH2)
 * TIM9: PWM, PA2(TIM9_CH1) & PA3(TIM9_CH2)
 ******************************************************************
 ******************************************************************/

/* Global Struct Variables */
TIM_TimeBaseInitTypeDef			Main_TIM_Struct;

/** @brief  : TIM5 interrupt count config
**  @agr    : void
**  @retval : void
**/
static void TimerInterruptConfig(TIM_TypeDef *TIMx)   // ms
{
	GetTIMxClockCmd(TIMx, ENABLE);
	NVIC_InitTypeDef Main_NVIC_Struct;
	
	Main_TIM_Struct.TIM_Prescaler  			=  50000 - 1; //0.5ms
	Main_TIM_Struct.TIM_Period     			=  0;
	Main_TIM_Struct.TIM_ClockDivision 		=  TIM_CKD_DIV1;
	Main_TIM_Struct.TIM_CounterMode  		=  TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIMx,&Main_TIM_Struct);
	
	Main_NVIC_Struct.NVIC_IRQChannel  		=  GetIRQHandlerFromTIMxCC(TIMx);
	Main_NVIC_Struct.NVIC_IRQChannelCmd 	=  ENABLE;
	Main_NVIC_Struct.NVIC_IRQChannelPreemptionPriority = 3;
	Main_NVIC_Struct.NVIC_IRQChannelSubPriority = 2;
	NVIC_Init(&Main_NVIC_Struct); 
}

void StartTimer(TIM_TypeDef *TIMx, uint32_t DelayTime)
{
	Main_TIM_Struct.TIM_Period = DelayTime * 2 - 1;
	TIM_TimeBaseInit(TIMx, &Main_TIM_Struct);

	VehStt.Veh_Timer_Finish = Check_NOK;
	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIMx, ENABLE);
}

void StopTimer(TIM_TypeDef *TIMx)
{
	TIM_Cmd(TIMx, DISABLE);
	TIM_ITConfig(TIMx, TIM_IT_Update, DISABLE);
}

void GPS_ReadParametersFromFlash(FlashMemory *pflash, GPS *pgps)
{
	ReadFromFlash(pflash, FLASH_GPSPara_BaseAddr);
	GetMessageInfo((char*)pflash->ReadOutBuffer, Flash.Message, ',');
	pgps->NbOfWayPoints = GetValueFromString(&Flash.Message[0][0]);
	for(int i = 0; i < pgps->NbOfWayPoints; i++)
	{
		pgps->Latitude 	= GetValueFromString(&Flash.Message[i * 2 + 1][0]);
		pgps->Longitude	= GetValueFromString(&Flash.Message[i * 2 + 2][0]);
		GPS_LatLonToUTM(pgps);
		pgps->Path_X[i]	= pgps->CorX;
		pgps->Path_Y[i]	= pgps->CorY;
	}
	GPS_UpdatePathYaw(pgps);
}

void GPS_SaveManual(void)
{
	Flash.Length = 0;
	Flash.Length += ToChar(2,&Flash.WriteInBuffer[Flash.Length],2);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length += ToChar(10.2565,&Flash.WriteInBuffer[Flash.Length],4);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length += ToChar(106.3256,&Flash.WriteInBuffer[Flash.Length],4);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length += ToChar(10.3536,&Flash.WriteInBuffer[Flash.Length],4);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length += ToChar(106.1156,&Flash.WriteInBuffer[Flash.Length],4);
	EraseMemory(FLASH_Sector_6);
	WriteToFlash(&Flash,FLASH_Sector_6,FLASH_GPSPara_BaseAddr);
}
/** @brief  : Read PID parameters from internal flash memory
**  @agr    : void
**  @retval : void
**/
void PID_ReadParametersFromFlash(void)
{
	ReadFromFlash(&Flash, FLASH_PIDPara_BaseAddr);
	GetMessageInfo((char*)Flash.ReadOutBuffer, Flash.Message,',');
	PID_ParametersUpdate(&M1, GetValueFromString(&Flash.Message[0][0]),
						GetValueFromString(&Flash.Message[1][0]),
						GetValueFromString(&Flash.Message[2][0]));
	PID_ParametersUpdate(&M2, GetValueFromString(&Flash.Message[3][0]),
						GetValueFromString(&Flash.Message[4][0]),
						GetValueFromString(&Flash.Message[5][0]));
}

void PID_SaveManual(void)
{
	Flash.Length	= 0;
	Flash.Length    += ToChar(0.060000,&Flash.WriteInBuffer[Flash.Length],6);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length    += ToChar(0.080000,&Flash.WriteInBuffer[Flash.Length],6);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length    += ToChar(0.001000,&Flash.WriteInBuffer[Flash.Length],6);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length    += ToChar(0.050000,&Flash.WriteInBuffer[Flash.Length],6);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length    += ToChar(0.100000,&Flash.WriteInBuffer[Flash.Length],6);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length    += ToChar(0.001000,&Flash.WriteInBuffer[Flash.Length],6);
	EraseMemory(FLASH_Sector_7);
	WriteToFlash(&Flash,FLASH_Sector_7,FLASH_PIDPara_BaseAddr);
}
void SendStatusData(void)
{
	Veh.SendData_Ind = 0;
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'$';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'V';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'E';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'H';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'S';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'T';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(Veh.Mode,&U6_TxBuffer[Veh.SendData_Ind],1); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(GPS_NEO.NbOfWayPoints,&U6_TxBuffer[Veh.SendData_Ind],1); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(GPS_NEO.K,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(Veh.Max_Velocity,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(M1.Kp,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(M1.Ki,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(M1.Kd,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(M2.Kp,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(M2.Ki,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(M2.Kd,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(Mag.Ke,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(Mag.Kedot,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(Mag.Ku,&U6_TxBuffer[Veh.SendData_Ind],3); 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	uint8_t checksum = LRCCalculate(&U6_TxBuffer[1],Veh.SendData_Ind - 1);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex((checksum & 0xF0) >> 4);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex(checksum & 0x0F);
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0D;
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0A;
	U6_SendData(Veh.SendData_Ind);
}

void SendData_0(void)
{
	Veh.SendData_Ind = 0;
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'$';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'V';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'I';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'N';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'F';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'O';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'0';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind     	+= ToChar(M1.Set_Vel,&U6_TxBuffer[Veh.SendData_Ind],3); // M1 SetVelocity
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind			+= ToChar(M2.Set_Vel,&U6_TxBuffer[Veh.SendData_Ind],3); // M2 SetVelocity
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind			+= ToChar(M1.Current_Vel,&U6_TxBuffer[Veh.SendData_Ind],3); // M1 Velocity
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind  		+= ToChar(M2.Current_Vel,&U6_TxBuffer[Veh.SendData_Ind],3); // M2 velocity
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind			+= ToChar(Mag.Set_Angle,&U6_TxBuffer[Veh.SendData_Ind],3); // Set angle
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind   		+= ToChar(Mag.Angle,&U6_TxBuffer[Veh.SendData_Ind],3); // Current angle
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind   		+= ToChar(Veh.Sensor_Angle,&U6_TxBuffer[Veh.SendData_Ind],3); // Current angle
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	uint8_t checksum = LRCCalculate(&U6_TxBuffer[1],Veh.SendData_Ind - 1);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex((checksum & 0xF0) >> 4);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex(checksum & 0x0F);
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0D;
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0A;
	U6_SendData(Veh.SendData_Ind);
}

void SendData_1(void)
{
	Veh.SendData_Ind = 0;
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'$';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'V';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'I';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'N';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'F';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'O';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'1';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	if(VehStt.GPS_ValidGPS)
	{
		U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'Y';
		U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	}
	else
	{
		U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'N';
		U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	}
	Veh.SendData_Ind   			+= ToChar(GPS_NEO.GPS_Quality,&U6_TxBuffer[Veh.SendData_Ind],1);
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind   			+= ToChar(GPS_NEO.Latitude,&U6_TxBuffer[Veh.SendData_Ind],13);
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind   			+= ToChar(GPS_NEO.Longitude,&U6_TxBuffer[Veh.SendData_Ind],13);
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	uint8_t checksum = LRCCalculate(&U6_TxBuffer[1],Veh.SendData_Ind - 1);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex((checksum & 0xF0) >> 4);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex(checksum & 0x0F);
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0D;
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0A;
	U6_SendData(Veh.SendData_Ind);
}

void SendData_2(void)
{
	Veh.SendData_Ind = 0;
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'$';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'V';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'I';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'N';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'F';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'O';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)'2';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind				+= ToChar(GPS_NEO.Thetae * (180/pi),&U6_TxBuffer[Veh.SendData_Ind],3);
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind				+= ToChar(GPS_NEO.Thetad * (180/pi),&U6_TxBuffer[Veh.SendData_Ind],3);
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind				+= ToChar(GPS_NEO.Delta_Angle,&U6_TxBuffer[Veh.SendData_Ind],3);
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind				+= ToChar(GPS_NEO.dmin,&U6_TxBuffer[Veh.SendData_Ind],3);
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind				+= ToChar(GPS_NEO.efa,&U6_TxBuffer[Veh.SendData_Ind],3);
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	uint8_t checksum = LRCCalculate(&U6_TxBuffer[1],Veh.SendData_Ind - 1);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex((checksum & 0xF0) >> 4);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex(checksum & 0x0F);
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0D;
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0A;
	U6_SendData(Veh.SendData_Ind);
}

void GPS_StanleyCompute()
{
	if(Veh.Controller == Stanley_Controller)
	{
		GPS_StanleyControl(&GPS_NEO, Timer.T, M1.Current_Vel, M2.Current_Vel);
	}
	else if(Veh.Controller == Pursuit_Controller)
	{
		GPS_PursuitControl(&GPS_NEO, Timer.T, M1.Current_Vel, M2.Current_Vel);
	}
	IMU_UpdateSetAngle(&Mag,GPS_NEO.Delta_Angle);
	IMU_UpdateFuzzyInput(&Mag,&Timer.T);
	Defuzzification_Max_Min(&Mag);
	if(Mag.Fuzzy_Out >= 0)
	{
		PID_UpdateSetVel(&M1, (1 - fabs(Mag.Fuzzy_Out)) * Veh.Max_Velocity);
		PID_UpdateSetVel(&M2, (1 + fabs(Mag.Fuzzy_Out)) * Veh.Max_Velocity);
	}
	else
	{
		PID_UpdateSetVel(&M1, (1 + fabs(Mag.Fuzzy_Out)) * Veh.Max_Velocity);
		PID_UpdateSetVel(&M2, (1 - fabs(Mag.Fuzzy_Out)) * Veh.Max_Velocity);
	}
	if(GPS_NEO.Goal_Flag)
	{
		PID_UpdateSetVel(&M1, 0);
		PID_UpdateSetVel(&M2, 0);
	}
}

static void GetVehicleVelocity(void)
{
	M1.Current_Vel = (GPIO_ReadOutputDataBit(Dir_GPIOx, Dir_GPIO_Pin_M1) == 0) ? 
		(((double)((M1.Enc + (M1.OverFlow - 1) * 65535) - M1.PreEnc)/ 39400) * 60) / Timer.T : // Backward up counting
		(((double)(((65535 - M1.Enc) + (M1.OverFlow - 1) * 65535) - (65535 - M1.PreEnc))/ 39400) * 60) / Timer.T; // Front down

	M1.Current_Vel = (M1.Current_Vel < 0) ? 0 : M1.Current_Vel;
			
	M2.Current_Vel = (GPIO_ReadOutputDataBit(Dir_GPIOx,Dir_GPIO_Pin_M2) == 0) ? 
		(((double)((M2.Enc + (M2.OverFlow - 1) * 65535) - M2.PreEnc)/ 39400) * 60) / Timer.T :
		(((double)(((65535 - M2.Enc) + (M2.OverFlow - 1) * 65535) - (65535 - M2.PreEnc))/ 39400) * 60) / Timer.T;

	M2.Current_Vel = (M2.Current_Vel < 0) ? 0 : M2.Current_Vel;
}
/************************************************************************
************************ Initialization Functions ***********************
*************************************************************************/
static void Led_Init(void)
{
	GPIO_InitTypeDef GPIO_Struct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Struct.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_Struct.GPIO_Pin	= GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
	GPIO_Struct.GPIO_OType	= GPIO_OType_PP;
	GPIO_Struct.GPIO_PuPd	= GPIO_PuPd_NOPULL;
	GPIO_Struct.GPIO_Speed	= GPIO_Speed_50MHz;
	GPIO_Init(GPIOD, &GPIO_Struct);
}

static void Parameters_Init(void)
{
	/*-----------------Error Init ---------------*/
	Veh_Error.Error_Index = 0;
	/*-----------------Timer Init ---------------*/
	Time_ParametersInit(&Timer, SAMPLE_TIME_50MS, SEND_TIME_1S);
	Status_ParametersInit(&VehStt);
	/*------------PID Parameter Init-------------*/
	PID_ReadParametersFromFlash();
	PID_ParametersInitial(&M1);
	PID_ParametersInitial(&M2);
	/*------------Fuzzy parametes Init ----------*/
	Fuzzy_ParametersInit();
	/*------------------AngleControl-------------*/
	IMU_ParametesInit(&Mag);
	IMU_UpdateFuzzyCoefficients(&Mag, (double)1/180, (double)1/30, (double)1);
	/*------------------StanleyParameter---------*/
	//GPS_ReadParametersFromFlash(&Flash,&GPS_NEO);
	//EraseMemory(FLASH_Sector_6);
	GPS_ParametersInit(&GPS_NEO);
	/*------------------Vehicle Init--------------*/
	Veh_ParametersInit(&Veh);
	/*----------------- Robot two wheel Init-----*/
	Self_ParametersInit(&selfPosition);
}

static void Peripheral_Config(void)
{
	Led_Init();
	TimerInterruptConfig(TIM5);
	USART1_Config(U1_Baudrate);	//IMU vs VXL
	USART2_Config(U2_Baudrate); 	//GPS vs VXL
	USART6_Config(U6_Baudrate); 	//Lora-PC vs Lora-VXL
	Encoder_Config();				
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

int main(void)
{
	Parameters_Init();
	Peripheral_Config();
	//PID_SaveManual();
	//GPS_SaveManual();

	SysTick_Config(SystemCoreClock / SYSTICK_INTERRUPT_1MS); // SysTick_Handler every 1ms
	while(1)
	{
		/*-----------------------------------------------------------------*/
		/*------------ Algorithm section ----------------------------------*/
		if(VehStt.Veh_Sample_Time)
		{
			M1.Enc = TIM_GetCounter(TIM3);
			M2.Enc = TIM_GetCounter(TIM4);
			GetVehicleVelocity();
			switch((int)Veh.Mode)
			{
				/*-------------- Auto mode section ------------------*/
				/*---------------------------------------------------*/
				case Auto_Mode:
					if(VehStt.Veh_Auto_Flag)
					{
						if(VehStt.GPS_Coordinate_Received)
						{
							VehStt.GPS_Coordinate_Received = Check_NOK;
							if((GPS_NEO.GPS_Quality == Fixed_RTK) || (GPS_NEO.GPS_Quality == Float_RTK))
							{
								OverWritePosition(&selfPosition, GPS_NEO.CorX, GPS_NEO.CorY);
								GPS_StanleyCompute();
							}
							else
							{
								if((VehStt.GPS_SelfUpdatePosition_Flag) && (VehStt.GPS_FirstGetPosition))
								{
									SelfPositionUpdateParams(&selfPosition, M2.Current_Vel, M1.Current_Vel, Mag.Angle, Timer.T);
									GPS_UpdateCoordinateXY(&GPS_NEO, selfPosition.x, selfPosition.y);
									GPS_StanleyCompute();
								}
								else
								{
									PID_UpdateSetVel(&M1,0);
									PID_UpdateSetVel(&M2,0);
								}
							}
						}
						else if((VehStt.GPS_FirstGetPosition) && (VehStt.GPS_SelfUpdatePosition_Flag))
						{
							SelfPositionUpdateParams(&selfPosition, M2.Current_Vel, M1.Current_Vel, Mag.Angle, Timer.T);
							GPS_UpdateCoordinateXY(&GPS_NEO, selfPosition.x, selfPosition.y);
							GPS_StanleyCompute();
						}
						else if((GPS_NEO.GPS_Quality == Fixed_RTK) || (GPS_NEO.GPS_Quality == Float_RTK))
						{
							GPS_NEO.NewDataAvailable = 0;
							GPS_StanleyCompute();
						}
						else
						{
							PID_UpdateSetVel(&M1, 0);
							PID_UpdateSetVel(&M2, 0);
						}
					}
					else /* stop motor */
					{
						PID_UpdateSetVel(&M1, 0);
						PID_UpdateSetVel(&M2, 0);
					}
					PID_Compute(&M1);
					PID_Compute(&M2);
					Robot_Run(M1.PID_Out, M2.PID_Out);   //Forward down counting Set bit
					IMU_UpdatePreAngle(&Mag);
					GPS_NEO.Pre_CorX = GPS_NEO.CorX;
					GPS_NEO.Pre_CorY = GPS_NEO.CorY;
					break;
				
				/*--------------- Manual mode section --------------- -*/
				/*----------------------------------------------------*/
				/* Notes: This mode is uses for angle control test purposes */
				case Manual_Mode: 
					Veh_UpdateVehicleFromKey(&Veh);
					IMU_UpdateFuzzyInput(&Mag,&Timer.T);
					Defuzzification_Max_Min(&Mag);
					if(Mag.Fuzzy_Out >= 0)
					{
						PID_UpdateSetVel(&M1,(1 - fabs(Mag.Fuzzy_Out)) * Veh.Manual_Velocity);
						PID_UpdateSetVel(&M2,(1 + fabs(Mag.Fuzzy_Out)) * Veh.Manual_Velocity);
					}
					else
					{
						PID_UpdateSetVel(&M1,(1 + fabs(Mag.Fuzzy_Out)) * Veh.Manual_Velocity);
						PID_UpdateSetVel(&M2,(1 - fabs(Mag.Fuzzy_Out)) * Veh.Manual_Velocity);
					}
					PID_Compute(&M1);
					PID_Compute(&M2);
					Robot_Run(M1.PID_Out, M2.PID_Out);   //Forward down counting Set bit
					IMU_UpdatePreAngle(&Mag);
					Veh.ManualCtrlKey = 0;
					break;
				
				/*--------------- Calibration section ------------------*/
				/*------------------------------------------------------*/
				case Calib_Mode:
					if(VehStt.Veh_Timer_Finish)
					{
						if(VehStt.Veh_Calib_Flag)
						{
							if(Veh.Distance < 39400)
							{
								Veh.Distance += (M2.Enc + (M2.OverFlow - 1) * 65535) - M2.PreEnc;
							}
							else
							{
								Veh.Distance = 0;
								if(Veh.TotalDistance < 20)
								{
									Veh.TotalDistance++;
								}
								else
								{
									Veh.TotalDistance = 0;
									PID_UpdateSetVel(&M1,0);
									PID_UpdateSetVel(&M2,0);
									VehStt.Veh_Calib_Flag = Check_NOK;
									VehStt.IMU_Calib_Finish = Check_OK;
								}
							}
						}
						if(VehStt.IMU_Calib_Finish)
						{
							if((M1.Current_Vel == 0) && (M2.Current_Vel == 0))
							{
								U1_SendData(FeedBack(U1_TxBuffer,"$MAGST"));
								U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1"));
								VehStt.IMU_Calib_Finish = Check_NOK;
							}
						}
						PID_Compute(&M1);
						PID_Compute(&M2);
						Robot_Run(M1.PID_Out,M2.PID_Out);
					}
					break;
				
				/*-------------- Test section --------------------------*/
				/*------------------------------------------------------*/
				case KeyBoard_Mode:
					//SelfPositionUpdateParams(&selfPosition,M2.Current_Vel/60,M1.Current_Vel/60,Timer.T);
					PID_Compute(&M1);
					PID_Compute(&M2);
					Robot_Run(M1.PID_Out,M2.PID_Out);   //Forward down counting Set bit
					break;
				
				case Soft_Reset_Mode:
					if(VehStt.Veh_Timer_Finish)
					{
						NVIC_SystemReset();
					}
					break;
			}
			PID_ResetEncoder(&M1);
			PID_ResetEncoder(&M2);
			VehStt.Veh_Sample_Time = Check_NOK;
		}
		
		/*-----------------------------------------------------------------*/
		/*------------------------ Send Data section ----------------------*/
		/*-----------------------------------------------------------------*/
		if(VehStt.Veh_SendData_Flag)
		{
			if(VehStt.Veh_Send_Data)
			{
				SendData_0();
				while(!DMA_GetFlagStatus(DMA2_Stream6, DMA_FLAG_TCIF6)); // transfer complete
				SendData_1();
				while(!DMA_GetFlagStatus(DMA2_Stream6, DMA_FLAG_TCIF6)); // transfer complete
				SendData_2();
			}
			VehStt.Veh_Send_Data = Check_NOK;
		}
	}
}

