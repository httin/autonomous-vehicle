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
static void SaveDataToInternalFlash(int key)
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
	Veh.Manual_Velocity = 0;
	Veh.Manual_Angle = 0;
	VehStt.Veh_Auto_Flag = Check_NOK;
	VehStt.Veh_Avoid_Flag = Check_NOK;
}

/**
  * @brief  This function handles SysTick Handler every 1/f (s).
  * @param  None
  * @retval None
  */
void SetSysTick(uint32_t f) {
	if (SysTick_Config(SystemCoreClock / f)) {
		// Capture error
		while (1){};
	}
}

void SysTick_Handler(void)
{
	/* TODO: Systick handler interrupt */
	VehStt.Veh_SampleVelo = Check_OK;
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
	if(U1_RxBuffer[0] == 0x0A)
	{
		if( (Veh.Veh_Error = IMU_GetValueFromMessage(&Mag, U1_RxBuffer)) == Veh_NoneError )
		{
			if((!VehStt.IMU_FirstGetAngle))
			{
				Mag.Set_Angle = Mag.Angle;
				VehStt.IMU_FirstGetAngle = Check_OK; // from now on, angle read from IMU won't impact Set Angle
			}
		}
	}
	else if (U1_RxBuffer[0] == '$') 
	{
		if(Veh_SplitMsg(U1_RxBuffer, U1.Message) == Veh_NoneError)      
		{
			if(StringHeaderCompare(U1.Message[0], "$PCDAT")) 
			{
				if(U1.Message[1][0] == '0')
				{
					VehStt.Veh_Avoid_Flag = Check_NOK;
				}
				else if(U1.Message[1][0] == '1')
				{
					VehStt.Veh_Avoid_Flag = Check_OK;
					Veh.Auto_Velocity = MPS2RPM(GetValueFromString(&U1.Message[2][0]));
					GPS_NEO.Delta_Angle = GetValueFromString(&U1.Message[3][0])*180/pi;
				}
			}
		}
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
	GPS_NEO.GPS_Error = GPS_NMEA_Message(&GPS_NEO, U2_RxBuffer, U2.Message);

	if(GPS_NEO.GPS_Error == Veh_NoneError)
	{
		VehStt.GPS_DataValid = Check_OK;
	}
	else
	{
		VehStt.GPS_DataValid = Check_NOK;
		Error_AppendError(&Veh_Error, GPS_NEO.GPS_Error);
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
					break;
				
				case Vehicle_Config:
					if(StringHeaderCompare(&U6.Message[1][0], "FUZZY"))
					{
						IMU_UpdateFuzzyCoefficients(&Mag,
							GetValueFromString(&U6.Message[2][0]),
							GetValueFromString(&U6.Message[3][0]),
							GetValueFromString(&U6.Message[4][0]));
					}
					else if(StringHeaderCompare(&U6.Message[1][0], "DATA")) 
					{
						VehStt.Veh_Enable_SendData = (enum_Status) (U6.Message[2][0] == '1');
					}
					else
					{
						Veh_UpdateMaxVelocity(&Veh, MPS2RPM(GetValueFromString(&U6.Message[1][0]))); // 0.5 m/s
						PID_ParametersUpdate(&M1, GetValueFromString(&U6.Message[2][0]),
							GetValueFromString(&U6.Message[3][0]), GetValueFromString(&U6.Message[4][0]));
						PID_ParametersUpdate(&M2, GetValueFromString(&U6.Message[5][0]),
							GetValueFromString(&U6.Message[6][0]), GetValueFromString(&U6.Message[7][0]));
					}
					U6_SendData(FeedBack(U6_TxBuffer, "$SINFO,1\r\n")); //ACK to PC
					break;
				
				/*---------------- Sample time config ------------------*/	
				case Sample_Time: 
					U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1\r\n"));
					break;
				
				/*---------------- Send data time config (ms)---------------*/
				case SendData_Time: 
					U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1\r\n"));
					break;
				
				/*---------------- IMU cmd and read data config --------*/
				case IMU_Config: 
					if(StringHeaderCompare(&U6.Message[1][0],"MAG2D"))
					{
						U1_SendData(FeedBack(U1_TxBuffer,"$MAG2D\r\n"));
						Reset_Motor();
						Veh.Mode = Calib_Mode;
						VehStt.Veh_Calib_Flag = Check_OK;
						PID_UpdateSetVel(&M1, -30);
						PID_UpdateSetVel(&M2, 30);
					}
					else if(StringHeaderCompare(&U6.Message[1][0],"START"))
					{
						U1_SendData(FeedBack(U1_TxBuffer,"$START\r\n"));
					}

					U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1\r\n"));
					break;
				
				/*------------------------ Software reset --------------------------*/
				case Soft_Reset: 
					Veh.Mode = Soft_Reset_Mode;
					U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1\r\n"));
					break;
				
				/*------------------------ Manual mode ----------------------*/
				case Manual_Config: 
					if(StringHeaderCompare(&U6.Message[1][0], "START"))
					{
						Reset_Motor();
						Veh.Mode = Manual_Mode;
						U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1\r\n"));
					}
					else if(StringHeaderCompare(&U6.Message[1][0], "STOP"))
					{
						Reset_Motor();
						Veh.Mode = None_Mode;
						U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1\r\n"));
					}
					else if(StringHeaderCompare(&U6.Message[1][0], "CMD")) 
					{
						Veh.ManualCtrlKey = U6.Message[2][0];
						if(Veh.ManualCtrlKey == 'W')
						{
							Veh.Manual_Velocity += 0.1 * Veh.Max_Velocity;
							if (Veh.Manual_Velocity > Veh.Max_Velocity)
								Veh.Manual_Velocity = Veh.Max_Velocity;
						}
						else if(Veh.ManualCtrlKey == 'S')
						{
							Veh.Manual_Velocity -= 0.1 * Veh.Max_Velocity;
							if (Veh.Manual_Velocity < 0)
								Veh.Manual_Velocity = 0;
						}
						else if(Veh.ManualCtrlKey == 'D')
						{
							Veh.Manual_Angle += 30;
							if(Veh.Manual_Angle > 180)
								Veh.Manual_Angle -= 360;
							Mag.Set_Angle = Degree_To_Degree(Mag.Set_Angle + 30);
						}
						else if(Veh.ManualCtrlKey == 'A')
						{
							Veh.Manual_Angle -= 30;
							if(Veh.Manual_Angle < -180) 
								Veh.Manual_Angle += 360;
							Mag.Set_Angle = Degree_To_Degree(Mag.Set_Angle - 30);
						}
						Veh.ManualCtrlKey = 0;
						U6_SendData(FeedBack(U6_TxBuffer,"$MACON,1\r\n"));
					}
					break;
				
				/*------------------------ Auto mode ------------------------*/
				case Auto_Config: 
					if(StringHeaderCompare(&U6.Message[1][0],"START"))
					{
						Reset_Motor();
						Veh.Mode = Auto_Mode;
						U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1\r\n"));
					}
					else if(StringHeaderCompare(&U6.Message[1][0],"STOP"))
					{
						Reset_Motor();
						Veh.Mode = None_Mode;
						U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1\r\n"));
					}
					else if(StringHeaderCompare(&U6.Message[1][0],"RUN"))
					{
						if(Veh.Mode == Auto_Mode)
						{
							if(GPS_NEO.NbOfWayPoints != 0)
							{
								VehStt.Veh_Auto_Flag = Check_OK;
								U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1\r\n"));
							}
							else
							{
								VehStt.Veh_Auto_Flag = Check_NOK;
								U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,0\r\n"));
							}
						}
					}
					else if(StringHeaderCompare(&U6.Message[1][0],"PAUSE"))
					{
						VehStt.Veh_Auto_Flag = Check_NOK;
						U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1\r\n"));
					}
					else if(StringHeaderCompare(&U6.Message[1][0],"BACK"))
					{
						if(GPS_NEO.Goal_Flag)
						{
							GPS_NEO.refPointIndex = -1;
							GPS_NEO.Goal_Flag = Check_NOK;
							Convert_Double_Array(GPS_NEO.P_X, GPS_NEO.NbOfWayPoints);
							Convert_Double_Array(GPS_NEO.P_Y, GPS_NEO.NbOfWayPoints);
							GPS_UpdatePathYaw(&GPS_NEO);
							VehStt.Veh_Auto_Flag = Check_NOK;
							U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1\r\n"));
						}
					}
					else if(StringHeaderCompare(&U6.Message[1][0],"DATA"))
					{
						Veh_UpdateMaxVelocity(&Veh, MPS2RPM(GetValueFromString(&U6.Message[2][0])));
						GPS_NEO.K = GetValueFromString(&U6.Message[3][0]);
						GPS_NEO.Ksoft = GetValueFromString(&U6.Message[4][0]);
						VehStt.Veh_AvoidEnable = (enum_Status)U6.Message[5][0];
						U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1\r\n"));
					}
					else if(StringHeaderCompare(&U6.Message[1][0],"SUPDT"))
					{
						if(U6.Message[2][0] == '1')
						{
							VehStt.GPS_SelfUpdatePosition_Flag = Check_OK;
							U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1\r\n"));
						}
						else if(U6.Message[2][0] == '0')
						{
							VehStt.GPS_SelfUpdatePosition_Flag = Check_NOK;
							U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1\r\n"));
						}
					}

					//else if(StringHeaderCompare(&U6.Message[1][0],"SCTRL")) // $AUCON,SCTRL,STLEY/PSUIT,CC<CR><LF>
					//{
					//	if(StringHeaderCompare(&U6.Message[2][0],"STLEY"))
					//	{
					//		Veh.Controller = Stanley_Controller;
					//	}
					//	else if(StringHeaderCompare(&U6.Message[2][0],"PSUIT"))
					//	{
					//		Veh.Controller = Pursuit_Controller;
					//	}
					//	U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1\r\n"));
					//}

					else
						U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,0\r\n"));
					break;
				
				/*---------------- Receive path coordinate -----------------*/
				case Path_Plan: 
					if (StringHeaderCompare(&U6.Message[1][0], "STOP"))
					{
						U1_SendData(FeedBack(U1_TxBuffer, (char *)u6_message));

						VehStt.GPS_Start_Receive_PathCor = Check_NOK;
						LED_OFF(LED_RED_PIN);  // Ket thuc nhan toa do, led off
						GPS_UpdatePathYaw(&GPS_NEO);
						U6_SendData(FeedBack(U6_TxBuffer, "$SINFO,VPLAN,0\r\n"));
						VehStt.Veh_MapAvailable = Check_OK;
					}
					else if (StringHeaderCompare(&U6.Message[1][0],"SPLINE"))
					{
						U1_SendData(FeedBack(U1_TxBuffer, (char *)u6_message));

						GPS_NEO.NbOfWayPoints = (int)GetValueFromString(&U6.Message[2][0]);
						GPS_NEO.NbOfP = 0; /* set current index of array map */
						GPS_NEO.Goal_Flag = Check_NOK;
						GPS_NEO.refPointIndex = -1;
						VehStt.Veh_MapAvailable = Check_NOK;
						VehStt.GPS_Start_Receive_PathCor = Check_OK; 
						GPS_ClearPathBuffer(&GPS_NEO); 
						LED_ON(LED_RED_PIN); // Bat dau nhan toa do, led on
						U6_SendData(FeedBack(U6_TxBuffer, "$SINFO,VPLAN,1\r\n"));
					}
					else if(VehStt.GPS_Start_Receive_PathCor)
					{
						U1_SendData(FeedBack(U1_TxBuffer, (char *)u6_message));

						GPS_NEO.NbOfP = (int)GetValueFromString(&U6.Message[1][0]);	// Index
						GPS_NEO.P_X[GPS_NEO.NbOfP] = GetValueFromString(&U6.Message[2][0]); // x
						GPS_NEO.P_Y[GPS_NEO.NbOfP] = GetValueFromString(&U6.Message[3][0]); // y
						U6_SendData(FeedBack(U6_TxBuffer, "$SINFO,1\r\n"));
					}
					else
					{
						U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,VPLAN,2\r\n"));
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
					U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1\r\n"));
					break;
				
				/*--------------- Control vehicle velocity by Keyboard ---------------*/
				case KeyBoard_Control:
					if(StringHeaderCompare(&U6.Message[1][0],"START"))
					{
						Reset_Motor();
						Veh.Mode = KeyBoard_Mode;
						Veh_UpdateMaxVelocity(&Veh, MPS2RPM(GetValueFromString(&U6.Message[2][0])));
						U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1\r\n"));
					}
					else if(StringHeaderCompare(&U6.Message[1][0],"STOP"))
					{
						Veh.Mode = None_Mode;
						Reset_Motor();
						U6_SendData(FeedBack(U6_TxBuffer,"$SINFO,1\r\n"));
					}
					else
					{
						double velo_linear;
						velo_linear = ((double)(U6.Message[5][0] - 48)/10) * Veh.Max_Velocity;
						if ((U6.Message[1][0] == 'W') && /* W - move forward */
							(U6.Message[2][0] == '!') && 
							(U6.Message[3][0] == '!') && 
							(U6.Message[4][0] == '!'))
						{
							PID_UpdateSetVel(&M1, velo_linear);
							PID_UpdateSetVel(&M2, velo_linear);
						}
						else if((U6.Message[1][0] == '!') && /* S - move backward */
								(U6.Message[2][0] == 'S') && 
								(U6.Message[3][0] == '!') && 
								(U6.Message[4][0] == '!'))
						{
							PID_UpdateSetVel(&M1, -velo_linear);
							PID_UpdateSetVel(&M2, -velo_linear);
						}
						else if((U6.Message[1][0] == '!') && /* A - move left */
								(U6.Message[2][0] == '!') && 
								(U6.Message[3][0] == 'A') && 
								(U6.Message[4][0] == '!'))
						{
							PID_UpdateSetVel(&M1, velo_linear);
							PID_UpdateSetVel(&M2, 0);
						}
						else if((U6.Message[1][0] == '!') && /* D - move right */
								(U6.Message[2][0] == '!') && 
								(U6.Message[3][0] == '!') && 
								(U6.Message[4][0] == 'D'))
						{
							PID_UpdateSetVel(&M1, 0);
							PID_UpdateSetVel(&M2, velo_linear);
						}
						else if((U6.Message[1][0] == 'W') && /* WA - move left+forward */
								(U6.Message[2][0] == '!') && 
								(U6.Message[3][0] == 'A') && 
								(U6.Message[4][0] == '!'))
						{
							PID_UpdateSetVel(&M1, velo_linear);
							PID_UpdateSetVel(&M2, velo_linear * 0.3);
						}
						else if((U6.Message[1][0] == 'W') && /* WD - move right+forward */
								(U6.Message[2][0] == '!') && 
								(U6.Message[3][0] == '!') && 
								(U6.Message[4][0] == 'D'))
						{
							PID_UpdateSetVel(&M1, velo_linear * 0.3);
							PID_UpdateSetVel(&M2, velo_linear);
						}
						else if((U6.Message[1][0] == '!') && /* SA - move left+backward */
								(U6.Message[2][0] == 'S') && 
								(U6.Message[3][0] == 'A') && 
								(U6.Message[4][0] == '!'))
						{
							PID_UpdateSetVel(&M1, -velo_linear);
							PID_UpdateSetVel(&M2, -velo_linear * 0.3);
						}
						else if((U6.Message[1][0] == '!') && /* SD - move right+backward */
								(U6.Message[2][0] == 'S') && 
								(U6.Message[3][0] == '!') && 
								(U6.Message[4][0] == 'D'))
						{
							PID_UpdateSetVel(&M1, -velo_linear * 0.3);
							PID_UpdateSetVel(&M2, -velo_linear);
						}
					}
					break;
			} // end switch command
		} // end Veh_SplitMsg
		else 
		{
			U6_SendData(FeedBack(U6_TxBuffer,"$WRONG_CKSUM\r\n")); 
		}
	} // end get_message_flag
	DMA_Cmd(DMA2_Stream2, ENABLE);
}
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/
/** ----------------------------------------------------------- **/

#ifdef ENCODER_IT
void TIM3_IRQHandler(void)
{
	M1.OverFlow++;
	TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
}

void TIM4_IRQHandler(void)
{
	M2.OverFlow++;
	TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
}
#endif

void StartTimer(TIM_TypeDef *TIMx)
{
	TIM_SetCounter(TIMx, 0);
	TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);
	TIM_Cmd(TIMx, ENABLE); // start counting
}

void StopTimer(TIM_TypeDef *TIMx)
{
	TIM_ITConfig(TIMx, TIM_IT_Update, DISABLE);
	TIM_Cmd(TIMx, DISABLE);
}

void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update))
	{
		++Timer.times_sample;
		++Timer.times_send;
		if(Timer.times_send == 20) {
			VehStt.Veh_Send_Data = Check_OK;
			Timer.times_send = 0;
		}

		VehStt.Veh_SampleState = Check_OK;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
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
