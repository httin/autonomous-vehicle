/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.h 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   This file contains the headers of the interrupt handlers.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_IT_H
#define __STM32F4xx_IT_H

//#ifdef __cplusplus
// extern "C" {
//#endif  

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/

void NMI_Handler(void);
void HardFault_Handler(void);
void MemManage_Handler(void);
void BusFault_Handler(void);
void UsageFault_Handler(void);
void SVC_Handler(void);
void DebugMon_Handler(void);
void PendSV_Handler(void);
void SysTick_Handler(void); // System Timer Interrupt

/***** Queue for RX buffer *****/
#define RX_QUEUE_SIZE 200

struct queue 
{
  uint8_t buffer[RX_QUEUE_SIZE];
  int front;
};

/***** Alternate Function Interrupt *****/
void USART1_IRQHandler(void);
void DMA2_Stream5_IRQHandler(void); // USART1 RX - IMU vs VXL

void USART2_IRQHandler(void);
void DMA1_Stream5_IRQHandler(void); // USART2 RX - Rover GPS vs VXL

void USART6_IRQHandler(void); 
void DMA2_Stream2_IRQHandler(void); // USART6 RX - Lora PC vs Lora VXL

#ifdef ENCODER_IT
void TIM3_IRQHandler(void); // Overflow encoder interrupt
void TIM4_IRQHandler(void); // Overflow encoder interrupt
#endif
void TIM2_IRQHandler(void); 
/****************************************/

extern void SetSysTick(uint32_t f);
extern void StartTimer(TIM_TypeDef *TIMx);
extern void StopTimer(TIM_TypeDef *TIMx);
static void SaveDataToInternalFlash(int key);
//#ifdef __cplusplus
//}
//#endif

#endif /* __STM32F4xx_IT_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
