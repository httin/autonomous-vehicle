#include "main.h"

/******************************************************************
 ************************ USED PERIPHERALS ************************
 * TIM3: Encoder Motor1, PA6(TIM3_CH1) & PA7(TIM3_CH2)
 * TIM4: Encoder Motor2, PD12(TIM4_CH1) & PD13(TIM4_CH2)
 * TIM9: PWM, PA2(TIM9_CH1) & PA3(TIM9_CH2)
 * GPIO: Direction, PC3(Pin_Out), PC4(Pin_Out)
 * USART6: PC6(TX), PC7(RX)
 * USART2: PD5(TX), PD6(RX)
 * USART1: PB6(TX), PB7(RX)
 ******************************************************************
 ******************************************************************/

void GPS_ReadParametersFromFlash(FlashMemory *pflash, GPS *pgps)
{
	ReadFromFlash(pflash, FLASH_GPSPara_BaseAddr);
	GetMessageInfo((char*)pflash->ReadOutBuffer, pflash->Message, ',');
	pgps->NbOfWayPoints = GetValueFromString( (char*)pflash->Message);
	for(int i = 0; i < pgps->NbOfWayPoints; i++)
	{
		pgps->Latitude 	= GetValueFromString( (char*)&pflash->Message[i * 2 + 1][0] );
		pgps->Longitude	= GetValueFromString( (char*)&pflash->Message[i * 2 + 2][0] );
		GPS_LatLonToUTM(pgps);
		pgps->P_X[i] = pgps->CorX;
		pgps->P_Y[i] = pgps->CorY;
	}
	GPS_UpdatePathYaw(pgps);
}

void GPS_SaveManual(void)
{
	Flash.Length = 0;
	Flash.Length += ToChar(2,&Flash.WriteInBuffer[Flash.Length],2);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length += ToChar(10.2565,&Flash.WriteInBuffer[Flash.Length], 4);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length += ToChar(106.3256,&Flash.WriteInBuffer[Flash.Length], 4);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length += ToChar(10.3536,&Flash.WriteInBuffer[Flash.Length], 4);
	Flash.WriteInBuffer[Flash.Length++] = (uint8_t)',';
	Flash.Length += ToChar(106.1156,&Flash.WriteInBuffer[Flash.Length], 4);
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

int append_string_to_buffer(uint8_t *buf, const char *str)
{
    int i = 0;
    while (str[i] != '\0')
    {
        buf[i] = (uint8_t)str[i];
        ++i;
    }
    return i;
}

static void send_VDATA(void){
	int index = 0;

	index += append_string_to_buffer(&U1_TxBuffer[index], "$VDATA,");
	index += ToChar(GPS_NEO.wheelPosX, &U1_TxBuffer[index], 13);
	U1_TxBuffer[index++] = (uint8_t)',';
	index += ToChar(GPS_NEO.wheelPosY, &U1_TxBuffer[index], 13);
	U1_TxBuffer[index++] = (uint8_t)',';
	index += ToChar(GPS_NEO.heading_angle, &U1_TxBuffer[index], 5);
	U1_TxBuffer[index++] = (uint8_t)',';
	index += ToChar(GPS_NEO.refPointIndex, &U1_TxBuffer[index], 1);
	U1_TxBuffer[index++] = (uint8_t)',';

	// uint8_t checksum;
	// checksum = LRCCalculate(&U1_TxBuffer[1], index - 1);
	// U1_TxBuffer[index++] = ToHex((checksum & 0xF0) >> 4);
	// U1_TxBuffer[index++] = ToHex(checksum & 0x0F);
	index += append_string_to_buffer(&U1_TxBuffer[index], "\r\n");	

	U1_SendData(index);
}

static void send_information(void){
	int temp_index;
	uint8_t checksum;
	Veh.SendData_Ind = 0; // reset index usart 6 tx buffer

	/* -------------------------------- vehicle information -------------------------------- */
	Veh.SendData_Ind += append_string_to_buffer(&U6_TxBuffer[Veh.SendData_Ind], "$VINFO,0,");
	Veh.SendData_Ind += ToChar(M1.current_set_v, &U6_TxBuffer[Veh.SendData_Ind], 3); // 2. M1 SetVelocity
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind += ToChar(M2.current_set_v, &U6_TxBuffer[Veh.SendData_Ind], 3); // 3. M2 SetVelocity
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind += ToChar(M1.current_v, &U6_TxBuffer[Veh.SendData_Ind], 3); // 4. M1 velocity
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind += ToChar(M2.current_v, &U6_TxBuffer[Veh.SendData_Ind], 3); // 5. M2 velocity
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind += ToChar(Mag.Set_Angle, &U6_TxBuffer[Veh.SendData_Ind], 3); // 6. Reference angle
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind += ToChar(Mag.Angle, &U6_TxBuffer[Veh.SendData_Ind], 3); // 7. Current angle
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';

	checksum = LRCCalculate(&U6_TxBuffer[1], Veh.SendData_Ind - 1);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex((checksum & 0xF0) >> 4);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex(checksum & 0x0F);
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0D;
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0A;
	/* -------------------------------- gps information -------------------------------- */
	temp_index = Veh.SendData_Ind;

	Veh.SendData_Ind += append_string_to_buffer(&U6_TxBuffer[Veh.SendData_Ind], "$VINFO,1,");
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)(GPS_NEO.GPS_Quality + '0');
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)(VehStt.Veh_Avoid_Flag + '0');
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';	
	Veh.SendData_Ind += ToChar(GPS_NEO.Latitude, &U6_TxBuffer[Veh.SendData_Ind], 11); // 2.
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind += ToChar(GPS_NEO.Longitude, &U6_TxBuffer[Veh.SendData_Ind], 11); // 3.
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';

	checksum = LRCCalculate(&U6_TxBuffer[temp_index + 1], Veh.SendData_Ind - (temp_index + 1));
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex((checksum & 0xF0) >> 4);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex(checksum & 0x0F);
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0D;
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0A;
	/* -------------------------------- stanley information --------------------------------*/
	temp_index = Veh.SendData_Ind;

	Veh.SendData_Ind += append_string_to_buffer(&U6_TxBuffer[Veh.SendData_Ind], "$VINFO,2,");
	Veh.SendData_Ind += ToChar(GPS_NEO.Thetae, &U6_TxBuffer[Veh.SendData_Ind],3); // 2.
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind += ToChar(GPS_NEO.Thetad, &U6_TxBuffer[Veh.SendData_Ind],3); // 3. 
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind += ToChar(GPS_NEO.efa, &U6_TxBuffer[Veh.SendData_Ind],3); // 4.
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind += ToChar(GPS_NEO.goal_radius, &U6_TxBuffer[Veh.SendData_Ind], 3); // 5. goal radius
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	Veh.SendData_Ind += ToChar(GPS_NEO.refPointIndex, &U6_TxBuffer[Veh.SendData_Ind], 1); // 6. point's index
	U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
	
	checksum = LRCCalculate(&U6_TxBuffer[temp_index + 1], Veh.SendData_Ind - (temp_index + 1));
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex((checksum & 0xF0) >> 4);
	U6_TxBuffer[Veh.SendData_Ind++] = ToHex(checksum & 0x0F);
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0D;
	U6_TxBuffer[Veh.SendData_Ind++] = 0x0A;	
	/* -------------------------------- manual information --------------------------------*/
	if (Veh.Mode == Manual_Mode) 
	{
		temp_index = Veh.SendData_Ind;

		Veh.SendData_Ind += append_string_to_buffer(&U6_TxBuffer[Veh.SendData_Ind], "$VINFO,3,");
		Veh.SendData_Ind += ToChar(Veh.Max_Velocity, &U6_TxBuffer[Veh.SendData_Ind], 3); 
		U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
		Veh.SendData_Ind += ToChar(Veh.Manual_Velocity, &U6_TxBuffer[Veh.SendData_Ind], 3); 
		U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
		Veh.SendData_Ind += ToChar(Veh.Manual_Angle, &U6_TxBuffer[Veh.SendData_Ind], 1); // integer
		U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';
		Veh.SendData_Ind += ToChar(Mag.Fuzzy_Out, &U6_TxBuffer[Veh.SendData_Ind], 3); 
		U6_TxBuffer[Veh.SendData_Ind++] = (uint8_t)',';

		checksum = LRCCalculate(&U6_TxBuffer[temp_index + 1], Veh.SendData_Ind - (temp_index + 1));
		U6_TxBuffer[Veh.SendData_Ind++] = ToHex((checksum & 0xF0) >> 4);
		U6_TxBuffer[Veh.SendData_Ind++] = ToHex(checksum & 0x0F);
		U6_TxBuffer[Veh.SendData_Ind++] = 0x0D;
		U6_TxBuffer[Veh.SendData_Ind++] = 0x0A;
	}

	U6_SendData(Veh.SendData_Ind);
}

void GPS_StanleyCompute()
{
	if(GPS_NEO.Goal_Flag)
		return;

	if(Veh.Controller == Stanley_Controller)
	{
		GPS_StanleyControl(&GPS_NEO, M1.current_v, M2.current_v);
	}
	else if(Veh.Controller == Pursuit_Controller)
	{
		GPS_PursuitControl(&GPS_NEO, M1.current_v, M2.current_v);
	}

	if( !VehStt.Veh_Avoid_Flag )
	{
		Veh.Auto_Velocity = Veh.Max_Velocity;
	}
	
	Mag.Set_Angle = Degree_To_Degree(Mag.Angle + GPS_NEO.Delta_Angle); // [-180, 180]
	IMU_UpdateFuzzyInput(&Mag);
	Mag.Fuzzy_Out = Defuzzification_Max_Min(Mag.Fuzzy_Error, Mag.Fuzzy_Error_dot);
	// Mag.Fuzzy_Out = Mag.Pre_Fuzzy_Out + Mag.Fuzzy_Out * Timer.T;

	if(Mag.Fuzzy_Out >= 0)
	{
		PID_UpdateSetVel(&M1, (1 - fabs(Mag.Fuzzy_Out)) * Veh.Auto_Velocity);
		PID_UpdateSetVel(&M2, (1 + fabs(Mag.Fuzzy_Out)) * Veh.Auto_Velocity);
	}
	else
	{
		PID_UpdateSetVel(&M1, (1 + fabs(Mag.Fuzzy_Out)) * Veh.Auto_Velocity);
		PID_UpdateSetVel(&M2, (1 - fabs(Mag.Fuzzy_Out)) * Veh.Auto_Velocity);
	}

	if(GPS_NEO.goal_radius <= 0.5)
	{
		GPS_NEO.Goal_Flag = Check_OK;
		PID_UpdateSetVel(&M1, 0);
		PID_UpdateSetVel(&M2, 0);
	}
}


/************************************************************************
************************ Initialization Functions ***********************
*************************************************************************/
static void TimerInterruptConfig(TIM_TypeDef *TIMx, uint16_t pres, uint32_t period)
{
	TIM_TimeBaseInitTypeDef	TIM_InitStruct;
	NVIC_InitTypeDef NVIC_InitStruct;
	GetTIMxClockCmd(TIMx, ENABLE);
	
	TIM_InitStruct.TIM_Prescaler         =  pres - 1; // 0.5ms
	TIM_InitStruct.TIM_Period            =  period - 1;
	TIM_InitStruct.TIM_ClockDivision     =  TIM_CKD_DIV1;
	TIM_InitStruct.TIM_CounterMode       =  TIM_CounterMode_Up;
    TIM_InitStruct.TIM_RepetitionCounter = 0;
	TIM_TimeBaseInit(TIMx, &TIM_InitStruct);
	
	NVIC_InitStruct.NVIC_IRQChannel     = GetIRQHandlerFromTIMxCC(TIMx);
	NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStruct.NVIC_IRQChannelCmd  = ENABLE;
	NVIC_Init(&NVIC_InitStruct); 
}

static void Led_Init(uint32_t gpio_pin)
{
	GPIO_InitTypeDef GPIO_Struct;
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
	GPIO_Struct.GPIO_Mode 	= GPIO_Mode_OUT;
	GPIO_Struct.GPIO_Pin	= gpio_pin;
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
	TimerInterruptConfig(TIM2, 1, 4200000); // period = 4.2MHz -> Interrupt Frequency = 20Hz
	Timer.T = sampleTimeCalc(GetTIMxFrequency(TIM2), 1, 4200000);
	StartTimer(TIM2);
	SetSysTick(100);
	Timer.velocity_T = (double)1/100;
	/* ------------------ Status ------------------ */

	/*------------PID Parameter Init-------------*/
	PID_ReadParametersFromFlash();
	/*------------Fuzzy parametes Init ----------*/
	Fuzzy_ParametersInit();
	/*------------------AngleControl-------------*/
	IMU_UpdateFuzzyCoefficients(&Mag, (double)1/180, (double)1/30, (double)1);
	/*------------------StanleyParameter---------*/
	//GPS_ReadParametersFromFlash(&Flash,&GPS_NEO);
	//EraseMemory(FLASH_Sector_6);
	GPS_ParametersInit(&GPS_NEO);
	/* ------------------ Vehicle ------------------ */
	Veh = (Vehicle) {
		.Max_Velocity = MPS2RPM(1),
		.Mode = None_Mode,
		.Veh_Error = Veh_NoneError,
		.Controller = Stanley_Controller
	};
	/* ---------------- SelfPosition -------------- */
	selfPosition = (SelfPosition) {
    	.R = Wheel_Radius
	};
}

static void Peripheral_Config(void)
{
	Led_Init(LED_RED_PIN | LED_BLUE_PIN);
	USART1_Config(U1_Baudrate);	//IMU vs VXL
	USART2_Config(U2_Baudrate); //GPS vs VXL
	USART6_Config(U6_Baudrate); //Lora-PC vs Lora-VXL
	Encoder_Config();
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
}

int main(void)
{
	Parameters_Init();
	Peripheral_Config();

	//PID_SaveManual();
	//GPS_SaveManual();
	while(1)
	{
		if(VehStt.Veh_SampleState)
		{
			switch((int)Veh.Mode)
			{
				/*-------------- Auto mode section ------------------*/
				/*---------------------------------------------------*/
				case Auto_Mode:
					if(VehStt.Veh_Auto_Flag)
					{
						if(VehStt.GPS_DataValid)
						{
							VehStt.GPS_DataValid = Check_NOK;

							if((GPS_NEO.GPS_Quality == Fixed_RTK) || (GPS_NEO.GPS_Quality == Float_RTK)
								|| VehStt.GPS_SelfUpdatePosition_Flag)
							{
								GPS_StanleyCompute();
							}
							else
							{
								PID_UpdateSetVel(&M1, 0);
								PID_UpdateSetVel(&M2, 0);
							}

							GPS_NEO.NewDataAvailable = 0;
						}
						else if((GPS_NEO.GPS_Quality == Fixed_RTK) || (GPS_NEO.GPS_Quality == Float_RTK) // gps(k-1)
								|| VehStt.GPS_SelfUpdatePosition_Flag) 
						{
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

					if(VehStt.Veh_MapAvailable && VehStt.Veh_AvoidEnable)
						send_VDATA();
					break;
				
				/*--------------- Manual mode section ----------------*/
				/*----------------------------------------------------*/
				case Manual_Mode: 
					IMU_UpdateFuzzyInput(&Mag);
					Mag.Fuzzy_Out = Defuzzification_Max_Min(Mag.Fuzzy_Error, Mag.Fuzzy_Error_dot);
					if(Mag.Fuzzy_Out >= 0) // turn right
					{
						PID_UpdateSetVel(&M1, (- fabs(Mag.Fuzzy_Out)) * Veh.Manual_Velocity);
						PID_UpdateSetVel(&M2, (+ fabs(Mag.Fuzzy_Out)) * Veh.Manual_Velocity);
					}
					else // turn left
					{
						PID_UpdateSetVel(&M1, (+ fabs(Mag.Fuzzy_Out)) * Veh.Manual_Velocity);
						PID_UpdateSetVel(&M2, (- fabs(Mag.Fuzzy_Out)) * Veh.Manual_Velocity);
					}
					break;
				
				/*--------------- Calibration section ------------------*/
				/*------------------------------------------------------*/
				case Calib_Mode:
					if(VehStt.Veh_Calib_Flag)
					{
						if(Veh.Distance < 39400 * 30)
						{
							Veh.Distance = (M2.Total_Encoder > 0) ? M2.Total_Encoder : -M2.Total_Encoder;
						}
						else // calibration is finished after rotating more than 10 revolutions
						{
							Veh.Distance = 0;
							PID_UpdateSetVel(&M1, 0);
							PID_UpdateSetVel(&M2, 0);
							VehStt.Veh_Calib_Flag = Check_NOK;
						}
					}
					else if ((M1.current_v == 0) && (M2.current_v == 0))
					{
						U1_SendData(FeedBack(U1_TxBuffer, "$MAGST")); // send stop calib command to IMU
						U6_SendData(FeedBack(U6_TxBuffer, "$CALIB,1\r\n"));	
						Veh.Mode = None_Mode;			
					}
					break;

				case KeyBoard_Mode:
					/* TODO */
					break;
				
				case Soft_Reset_Mode:
					Veh.Mode = None_Mode;
					NVIC_SystemReset();
					break;

				case None_Mode:
					break;

				default:
					break;
			}
			
			VehStt.Veh_SampleState = Check_NOK;
		}

		/*-----------------------------------------------------------------*/
		/*--------------------- Velocity Control section ------------------*/
		/*-----------------------------------------------------------------*/
		if(VehStt.Veh_SampleVelo)
		{
			EncoderProcessing(&M1, TIM3, &Timer);
			EncoderProcessing(&M2, TIM4, &Timer);

			PID_Compute(&M1, &Timer);
			PID_Compute(&M2, &Timer);
			Robot_RunVersion2(M1.PID_Out, M2.PID_Out);
			
			VehStt.Veh_SampleVelo = Check_NOK;
		}

		/*-----------------------------------------------------------------*/
		/*------------------------ Send Data section ----------------------*/
		/*-----------------------------------------------------------------*/
		if(VehStt.Veh_Enable_SendData)
		{
			if(VehStt.Veh_Send_Data)
			{
				LED_TOGGLE(LED_BLUE_PIN);
				while(!DMA_GetFlagStatus(DMA2_Stream6, DMA_FLAG_TCIF6));
				send_information();
				VehStt.Veh_Send_Data = Check_NOK;
			}
		}
	}
}

