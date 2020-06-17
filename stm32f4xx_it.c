/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include <math.h>
#include "stm32f4xx_it.h"
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  *         - Access of data outside valid memory
  *         - Invalid instructions
  *         - Division by zero
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
	
}
/** Declare Variable **/
/*C,0.3,0,0.8,0.5,0.001,0.8,0.3,0.001*/
uint8_t temp;
struct queue u6_queue;
uint8_t u6_message[100]; // a message ends up with "\r\n"
uint8_t get_message_flag; // set when receive "\r\n" indicating we get a full message to handle

/**
  * @brief  This function Save data to internal flash memory
  * @param  None
  * @retval None
  */
void SaveDataToInternalFlash(int key)
{
	Flash.Length = 0;
	switch(key)
	{
		/* ------ PID parameters ------- */
		case 1:
			PID_SavePIDParaToFlash(&Flash,&M1,&M2);
			break;
		/* ------- GPS parameter -------*/
		case 2:
			GPS_SavePathCoordinateToFlash(&GPS_NEO,&Flash);
			break;
	}
}
/**
  * @brief  This function reset motor
  * @param  None
  * @retval None
  */
void Reset_Motor()
{
	PID_UpdateSetVel(&M1, 0);
	PID_UpdateSetVel(&M2, 0);
	PID_ResetPID(&M1);
	PID_ResetPID(&M2);
	Stop_Motor();
	Veh.Manual_Velocity = 0;
	Veh.Manual_Angle = 0;
	VehStt.Veh_Auto_Flag = Check_NOK;
}


/**
  * @brief  This function handles SysTick Handler every 1ms.
  * @param  None
  * @retval None
  */
/* M1 - Right motor, M2 - Left motor */
void SysTick_Handler(void)
{
	if(!VehStt.Veh_Sample_Time)
	{
		if(Timer.sample_count < Timer.sample_time)
		{
			++Timer.sample_count;
		}
		else
		{
			Timer.sample_count = 0;
			VehStt.Veh_Sample_Time = Check_OK; // Time to get sample - every 50 ms
		}
	}

	if(!VehStt.Veh_Send_Data)
	{
		if(Timer.send_count < Timer.send_time)
		{
			++Timer.send_count;
		}
		else
		{
			Timer.send_count = 0;
			VehStt.Veh_Send_Data = Check_OK; // Time to send data - every 1s
		}
	}
}

/** brief : USART1 interrupt Rx handler **/
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
void USART1_IRQHandler(void)
{
	if(USART_GetITStatus(USART1, USART_IT_IDLE))
	{
		USART_ClearFlag(USART1, USART_FLAG_IDLE);
		temp = USART_ReceiveData(USART1);
		DMA_Cmd(DMA2_Stream5, DISABLE);
	}
}

void DMA2_Stream5_IRQHandler(void)
{
	DMA_ClearITPendingBit(DMA2_Stream5, DMA_IT_TCIF5);
	Veh.Veh_Error = IMU_GetValueFromMessage(&Mag, U1_RxBuffer);
	if(Veh.Veh_Error != Veh_NoneError) 
		Error_AppendError(&Veh_Error, Veh.Veh_Error);

	if((!VehStt.IMU_FirstSetAngle) && (Mag.Angle != 0))
	{
		IMU_UpdateSetAngle(&Mag, 0);
		VehStt.IMU_FirstSetAngle = Check_OK; // update vehicle status
	}
	DMA_Cmd(DMA2_Stream5, ENABLE);
}

/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
/** @brief : USART2 interrupt Rx Handler **/
void USART2_IRQHandler(void)
{
	if(USART_GetITStatus(USART2, USART_IT_IDLE))
	{
		USART_ClearFlag(USART2, USART_FLAG_IDLE);
		temp = USART_ReceiveData(USART2);
		DMA_Cmd(DMA1_Stream5, DISABLE);
	}
}

void DMA1_Stream5_IRQHandler(void)
{
	DMA_ClearITPendingBit(DMA1_Stream5, DMA_IT_TCIF5);
	GPS_NEO.GPS_Error = GPS_GetLLQMessage(&GPS_NEO, U2_RxBuffer, U2.Message);
	if(GPS_NEO.GPS_Error == Veh_NoneError)
	{
		VehStt.GPS_Coordinate_Received = Check_OK;
		VehStt.GPS_ValidGPS = Check_OK;

		if((GPS_NEO.GPS_Quality == Fixed_RTK) || (GPS_NEO.GPS_Quality == Float_RTK))
		{
			/* first time get the Fix Mode */
			if(!VehStt.GPS_FirstGetPosition)
			{
				VehStt.GPS_FirstGetPosition = Check_OK;
				OverWritePosition(&selfPosition, GPS_NEO.CorX, GPS_NEO.CorY);
				GPS_NEO.Pre_CorX = GPS_NEO.CorX;
				GPS_NEO.Pre_CorY = GPS_NEO.CorY;
			}
		}
	}
	else
	{
		VehStt.GPS_Coordinate_Received = Check_NOK;
		VehStt.GPS_ValidGPS = Check_NOK;
		Error_AppendError(&Veh_Error, Veh.Veh_Error);
	}
	DMA_Cmd(DMA1_Stream5, ENABLE);
}

/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
/** brief : USART6 interrupt Rx Handler **/
void USART6_IRQHandler(void)
{
	if(USART_GetITStatus(USART6, USART_IT_IDLE))
	{
		USART_ClearFlag(USART6, USART_FLAG_IDLE);
		temp = USART_ReceiveData(USART6);
		DMA_Cmd(DMA2_Stream2, DISABLE);
	}
}

/*-----------------------------------------------------*/
/*------------------ COMMAND FROM PC ------------------*/
/*-----------------------------------------------------*/
/*
**** 1. $VEHCF,Max_Velocity,M1.Kp,M1.Ki,M1.Kd,M2.Kp,M2.Ki,M2.Kd,CC
**** 2. $TSAMP,Sample_Time,CC     (-- in ms --)
**** 3. $TSEND,SendData_Time,CC       (-- in ms --)
**** 4. $IMUCF,IMU commands sequences,CC   (-- Command sequences in documents --)
**** 5. $SFRST,CC  
**** 6. $MACON,Manual commands sequences   (-- 1: Manual mode enable, 0: Stop motor
                                                  and disable manual mode
                                                  W/S/A/D: Vehicle control Key --)
**** 7. $AUCON,Auto commands sequences     (-- 1: Auto mode enable, 0: Disable mode --)
**** 8. $VPLAN,NBOfWP,lat,lon,....			(-- Numbers of lat lon --)
**** 9. $FSAVE,"PID"/"GPS"
****10. $KCTRL,"START"/"STOP"/'W,!,!,!'...
*/
void DMA2_Stream2_IRQHandler(void)
{
	DMA_ClearITPendingBit(DMA2_Stream2, DMA_IT_TCIF2);
	int index = 0, i = 0;
	while (U6_RxBuffer[index] != 0) 
	{
		u6_queue.buffer[u6_queue.front++] = U6_RxBuffer[index];
		if (U6_RxBuffer[index] == '\r' && U6_RxBuffer[index + 1] == '\n')
		{
			++index;
			u6_queue.buffer[u6_queue.front++] = U6_RxBuffer[index];
			for (; i < u6_queue.front; ++i)
			{
				u6_message[i] = u6_queue.buffer[i];
			}
			u6_queue.front = 0;
			get_message_flag = 1;
		}
		++index;
	}
	/* clear rx buffer after getting all data */
	while(--index >= 0)
		U6_RxBuffer[index] = 0;

	if(get_message_flag)
	{
		get_message_flag = 0;
		if(Veh_SplitMsg(u6_message, U6.Message) == Veh_NoneError)
		{
			switch(Veh_MsgToCmd(&U6.Message[0][0]))
			{
				/*----------------- Vehicle Config -------------------*/
				case None:
					U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,0"));
					break;
				
				case Vehicle_Config:
					if(StringHeaderCompare(&U6.Message[1][0],"FUZZY"))
					{
						IMU_UpdateFuzzyCoefficients(&Mag,
							GetValueFromString(&U6.Message[2][0]),
							GetValueFromString(&U6.Message[3][0]),
							GetValueFromString(&U6.Message[4][0]));
					}
					else if(StringHeaderCompare(&U6.Message[1][0],"DATA")) 
					{
						if(U6.Message[2][0] == '1')
							VehStt.Veh_SendData_Flag = Check_OK; //F7, vehicle sending data
						else 
							VehStt.Veh_SendData_Flag = Check_NOK; //F8, vehicle stop sending data
					}
					else
					{
						Veh_UpdateMaxVelocity(&Veh, MPS2RPM(GetValueFromString(&U6.Message[1][0]))); // 0.5 m/s
						PID_ParametersUpdate(&M1, GetValueFromString(&U6.Message[2][0]),
							GetValueFromString(&U6.Message[3][0]), GetValueFromString(&U6.Message[4][0]));
						PID_ParametersUpdate(&M2, GetValueFromString(&U6.Message[5][0]),
							GetValueFromString(&U6.Message[6][0]), GetValueFromString(&U6.Message[7][0]));
					}
					U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1")); //ACK to PC
					break;
				
				/*---------------- Sample time config (us)------------------*/	
				case Sample_Time: 
					Time_SampleTimeUpdate(&Timer, (uint32_t)GetValueFromString(&U6.Message[1][0]));
					U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1"));
					break;
				
				/*---------------- Send data time config (ms)---------------*/
				case SendData_Time: 
					Time_SendTimeUpdate(&Timer, (uint32_t)GetValueFromString(&U6.Message[1][0]));
					U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1"));
					break;
				
				/*---------------- IMU cmd and read data config --------*/
				case IMU_Config: 
					if(StringHeaderCompare(&U6.Message[1][0],"MAG2D"))
					{
						U1_SendData(FeedBack(U1_TxBuffer,"$MAG2D"));
						Reset_Motor();
						Robot_AntiClockwise();
						Veh.Mode = Calib_Mode;
						VehStt.Veh_Calib_Flag = Check_OK; // Update Calibration IMU status
						PID_UpdateSetVel(&M1,50);
						PID_UpdateSetVel(&M2,50);
						StartTimer(TIM5, 3000);
					}
					U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1"));
					break;
				
				/*---------------- Software reset --------------------------*/
				case Soft_Reset: 
					Veh.Mode = Soft_Reset_Mode;
					StartTimer(TIM5, 2000);
					U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1"));
					break;
				
				/*---------------- Manual mode config ----------------------*/
				case Manual_Config: 
					if(StringHeaderCompare(&U6.Message[1][0],"START"))
					{
						Robot_Forward();
						Reset_Motor();
						Veh.Mode = Manual_Mode;
						if(VehStt.IMU_FirstSetAngle)
						{
							IMU_UpdateSetAngle(&Mag,0);
						}
					}
					else if(StringHeaderCompare(&U6.Message[1][0],"STOP"))
					{
						Reset_Motor();
					}
					else
						Veh.ManualCtrlKey = U6.Message[1][0];
					U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1"));
					break;
				
				/*---------------- Auto mode config ------------------------*/
				case Auto_Config: 
					if(StringHeaderCompare(&U6.Message[1][0],"START"))
					{
						Reset_Motor();
						Robot_Forward();
						Veh.Mode = Auto_Mode;
						GPS_NEO.Goal_Flag = Check_NOK;
						U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1"));
					}
					else if(StringHeaderCompare(&U6.Message[1][0],"STOP"))
					{
						Reset_Motor();
						U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1"));
					}
					else if(StringHeaderCompare(&U6.Message[1][0],"RUN"))
					{
						if(Veh.Mode == Auto_Mode)
						{
							if(GPS_NEO.NbOfWayPoints != 0)
							{
								VehStt.Veh_Auto_Flag = Check_OK;
								U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1"));
							}
							else
							{
								VehStt.Veh_Auto_Flag = Check_NOK;
								U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,0"));
							}
						}
					}
					else if(StringHeaderCompare(&U6.Message[1][0],"PAUSE"))
					{
						VehStt.Veh_Auto_Flag = Check_NOK;
						U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1"));
					}
/*
					else if(StringHeaderCompare(&U6.Message[1][0],"BACK"))
					{
						if((GPS_NEO.NbOfWayPoints != 0) && (GPS_NEO.Goal_Flag))
						{
							GPS_NEO.Goal_Flag = Check_NOK;
							VehStt.Veh_Auto_Flag = Check_NOK;
							Convert_Double_Array(GPS_NEO.Path_X, GPS_NEO.NbOfWayPoints);
							Convert_Double_Array(GPS_NEO.Path_Y, GPS_NEO.NbOfWayPoints);
							GPS_NEO.NbOfP = 0;
							GPS_ClearPathBuffer(&GPS_NEO);
							GPS_PathPlanning(&GPS_NEO, GPS_NEO.Step);
							GPS_UpdatePathYaw(&GPS_NEO);
							U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1"));
						}
					}
*/
					else if(StringHeaderCompare(&U6.Message[1][0],"DATA"))
					{
						if(Veh.Mode == Auto_Mode)
						{
							Veh_UpdateMaxVelocity(&Veh,MPS2RPM(GetValueFromString(&U6.Message[2][0])));
							GPS_NEO.K = GetValueFromString(&U6.Message[3][0]);
							GPS_NEO.Step = GetValueFromString(&U6.Message[4][0]);
							U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1"));
						}
						else
						{
							U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,0"));
						}
					}
					else if(StringHeaderCompare(&U6.Message[1][0],"SUPDT"))
					{
						if(U6.Message[2][0] == '1')
						{
							VehStt.GPS_SelfUpdatePosition_Flag = Check_OK;
							U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1"));
						}
						else if(U6.Message[2][0] == '0')
						{
							VehStt.GPS_SelfUpdatePosition_Flag = Check_NOK;
							U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1"));
						}
					}
/*
					else if(StringHeaderCompare(&U6.Message[1][0],"SCTRL")) // $AUCON,SCTRL,STLEY/PSUIT,CC<CR><LF>
					{
						if(StringHeaderCompare(&U6.Message[2][0],"STLEY"))
						{
							Veh.Controller = Stanley_Controller;
						}
						else if(StringHeaderCompare(&U6.Message[2][0],"PSUIT"))
						{
							Veh.Controller = Pursuit_Controller;
						}
						U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1"));
					}
*/
					else
						U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,0"));
					break;
				
				/*---------------- Receive path coordinate -----------------*/
				case Path_Plan: 
					if (StringHeaderCompare(&U6.Message[1][0], "STOP"))
					{
						VehStt.GPS_Start_Receive_PathCor = Check_NOK;
						LED_OFF(LED_BLUE_PIN);  // Ket thuc nhan toa do, led off
						GPS_UpdatePathYaw(&GPS_NEO);
						U6_SendData(FeedBack(U6_TxBuffer, "$SINFO,VPLAN,0"));
					}
					else if (StringHeaderCompare(&U6.Message[1][0],"SPLINE"))
					{
						GPS_NEO.NbOfWayPoints = (int)GetValueFromString(&U6.Message[2][0]);
						GPS_NEO.Cor_Index = 0;
						GPS_NEO.NbOfP = 0; /* set current index of array map */
						GPS_NEO.Goal_Flag = Check_NOK;
						VehStt.Veh_Auto_Flag = Check_NOK;
						VehStt.GPS_Start_Receive_PathCor = Check_OK; 
						GPS_ClearPathBuffer(&GPS_NEO); 
						GPS_ClearPathCorBuffer(&GPS_NEO);
						LED_ON(LED_BLUE_PIN); // Bat dau nhan toa do, led on
						U6_SendData(FeedBack(U6_TxBuffer, "$SINFO,VPLAN,1"));
					}
					else if(VehStt.GPS_Start_Receive_PathCor)
					{
						GPS_NEO.NbOfP = (int)GetValueFromString(&U6.Message[1][0]);	// Index
						GPS_NEO.P_X[GPS_NEO.NbOfP] = GetValueFromString(&U6.Message[2][0]); // x
						GPS_NEO.P_Y[GPS_NEO.NbOfP] = GetValueFromString(&U6.Message[3][0]); // y
						U6_SendData(FeedBack(U6_TxBuffer, "$SINFO,1"));
					}
					else
					{
						U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,VPLAN,?"));
					}
					break;
					
				/*--------------- Save data in internal flash memory ---------------*/
				case Flash_Save:
					if(StringHeaderCompare(&U6.Message[1][0],"PID"))
					{
						SaveDataToInternalFlash(1);
					}
					else if(StringHeaderCompare(&U6.Message[1][0],"GPS"))
					{
						SaveDataToInternalFlash(2);
					}
					U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1"));
					break;
				
				/*--------------- Save data in internal flash memory --------*/
				/* Format: $KCTRL,1,max velocity 
							$KCTRL,0
							$KCTRL,W,S,A,D,level
				*/
				case KeyBoard_Control:
					if(StringHeaderCompare(&U6.Message[1][0],"START"))
					{
						if(Veh.Mode != KeyBoard_Mode)
						{
							Reset_Motor();
							Robot_Forward();
							Veh.Mode = KeyBoard_Mode;
							Veh_UpdateMaxVelocity(&Veh,MPS2RPM(GetValueFromString(&U6.Message[2][0])));
						}
						U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1"));
					}
					else if(StringHeaderCompare(&U6.Message[1][0],"STOP"))
					{
						Veh.Mode = None_Mode;
						Reset_Motor();
						U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1"));
					}
					else
					{
						if ((U6.Message[1][0] == 'W') && /* W - move forward */
							(U6.Message[2][0] == '!') && 
							(U6.Message[3][0] == '!') && 
							(U6.Message[4][0] == '!'))
						{
							Robot_Forward();
							PID_UpdateSetVel(&M1,((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity);
							PID_UpdateSetVel(&M2,((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity);
							U6_SendData(FeedBack(U6_TxBuffer,"$KCTRL,F"));
						}
						else if((U6.Message[1][0] == '!') && /* S - move backward */
								(U6.Message[2][0] == 'S') && 
								(U6.Message[3][0] == '!') && 
								(U6.Message[4][0] == '!'))
						{
							Robot_Backward();
							PID_UpdateSetVel(&M1,((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity);
							PID_UpdateSetVel(&M2,((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity);
							U6_SendData(FeedBack(U6_TxBuffer,"$KCTRL,B"));
						}
						else if((U6.Message[1][0] == '!') && /* A - move left */
								(U6.Message[2][0] == '!') && 
								(U6.Message[3][0] == 'A') && 
								(U6.Message[4][0] == '!'))
						{
							Robot_AntiClockwise();
							PID_UpdateSetVel(&M1,((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity);
							PID_UpdateSetVel(&M2,((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity);
							U6_SendData(FeedBack(U6_TxBuffer,"$KCTRL,L"));
						}
						else if((U6.Message[1][0] == '!') && /* D - move right */
								(U6.Message[2][0] == '!') && 
								(U6.Message[3][0] == '!') && 
								(U6.Message[4][0] == 'D'))
						{
							Robot_Clockwise();
							PID_UpdateSetVel(&M1,((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity);
							PID_UpdateSetVel(&M2,((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity);
							U6_SendData(FeedBack(U6_TxBuffer,"$KCTRL,R"));
						}
						else if((U6.Message[1][0] == 'W') && /* WA - move left+forward */
								(U6.Message[2][0] == '!') && 
								(U6.Message[3][0] == 'A') && 
								(U6.Message[4][0] == '!'))
						{
							Robot_Forward();
							PID_UpdateSetVel(&M1,((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity);
							PID_UpdateSetVel(&M2,(((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity) * 0.5);
							U6_SendData(FeedBack(U6_TxBuffer,"$KCTRL,LF"));
						}
						else if((U6.Message[1][0] == 'W') && /* WD - move right+forward */
								(U6.Message[2][0] == '!') && 
								(U6.Message[3][0] == '!') && 
								(U6.Message[4][0] == 'D'))
						{
							Robot_Forward();
							PID_UpdateSetVel(&M1,(((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity) * 0.5);
							PID_UpdateSetVel(&M2,((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity);
							U6_SendData(FeedBack(U6_TxBuffer,"$KCTRL,RF"));
						}
						else if((U6.Message[1][0] == '!') && /* SA - move left+backward */
								(U6.Message[2][0] == 'S') && 
								(U6.Message[3][0] == 'A') && 
								(U6.Message[4][0] == '!'))
						{
							Robot_Backward();
							PID_UpdateSetVel(&M1,((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity);
							PID_UpdateSetVel(&M2,(((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity) * 0.5);
							U6_SendData(FeedBack(U6_TxBuffer,"$KCTRL,LB"));
						}
						else if((U6.Message[1][0] == '!') && /* SD - move right+backward */
								(U6.Message[2][0] == 'S') && 
								(U6.Message[3][0] == '!') && 
								(U6.Message[4][0] == 'D'))
						{
							Robot_Backward();
							PID_UpdateSetVel(&M1,(((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity) * 0.5);
							PID_UpdateSetVel(&M2,(((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity));
							U6_SendData(FeedBack(U6_TxBuffer,"$KCTRL,RB"));
						}
						else
						{
							PID_UpdateSetVel(&M1,0);
							PID_UpdateSetVel(&M2,0);
							U6_SendData(FeedBack(U6_TxBuffer, (char *)u6_message));
						}
					}
					Veh_CheckStateChange(&M1, GPIO_ReadOutputDataBit(Dir_GPIOx, Dir_GPIO_Pin_M1));
					Veh_CheckStateChange(&M2, GPIO_ReadOutputDataBit(Dir_GPIOx, Dir_GPIO_Pin_M2));
					break;
			} // end switch command
		} // end Veh_SplitMsg
		else 
		{
			U6_SendData(FeedBack(U6_TxBuffer,"$WRONG_CKSUM")); 
		}
	} // end get_message_flag
	DMA_Cmd(DMA2_Stream2, ENABLE);
}
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
void TIM3_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	M1.OverFlow++;
}

void TIM4_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	M2.OverFlow++;
}

void TIM5_IRQHandler(void)
{
	TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	if(!VehStt.Veh_Timer_Start)
	{
		VehStt.Veh_Timer_Start = Check_OK;
	}
	else
	{
		StopTimer(TIM5);
		VehStt.Veh_Timer_Start = Check_NOK;
		VehStt.Veh_Timer_Finish = Check_OK;
	}
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
