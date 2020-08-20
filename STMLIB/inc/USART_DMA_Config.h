#ifndef USART_DMA_CONFIG_H
#define USART_DMA_CONFIG_H

#include "stm32f4xx.h"

/*-------- Hardware config USART1 (can change) ---------
 * @brief:	Baudrate 115200
 *			Used to communicate between MCU & IMU
 * @pinout: PB6: TX -> RX IMU
 *			PB7: RX -> TX IMU
 */
#define		U1_Baudrate						115200
#define 	U1_GPIOx						GPIOB
#define 	U1_GPIO_Pin_Tx        			GPIO_Pin_6
#define 	U1_GPIO_Pin_Rx					GPIO_Pin_7
#define		U1_GPIO_PinSourceTx				GPIO_PinSource6
#define		U1_GPIO_PinSourceRx				GPIO_PinSource7
#define 	U1_RCC_AHB1Periph_GPIOx			RCC_AHB1Periph_GPIOB
/*-------- Hardware config USART2 (can change) ---------
 * @brief:	Baudrate 115200
 *			Used to communicate between LORA-GPS & LORA-BASE
 * @pinout:	PD5: TX -> RX ROVER-GPS 
 *			PD6: RX -> TX ROVER-GPS
 */
#define		U2_Baudrate						115200
#define 	U2_GPIOx						GPIOD
#define 	U2_GPIO_Pin_Tx        			GPIO_Pin_5
#define 	U2_GPIO_Pin_Rx					GPIO_Pin_6
#define		U2_GPIO_PinSourceTx				GPIO_PinSource5
#define		U2_GPIO_PinSourceRx				GPIO_PinSource6
#define 	U2_RCC_AHB1Periph_GPIOx			RCC_AHB1Periph_GPIOD
/*-------- Hardware config USART6 (can change) ---------
 * @brief:	Baudrate 115200
 *			Used to communicate between LORA-MCU & LORA-PC (C# interface)
 * @pinout: PC6: TX -> RX Lora-MCU
 *			PC7: RX -> TX Lora-MCU 	
 */
#define		U6_Baudrate						115200
#define 	U6_GPIOx						GPIOC
#define 	U6_GPIO_Pin_Tx        			GPIO_Pin_6
#define 	U6_GPIO_Pin_Rx					GPIO_Pin_7
#define		U6_GPIO_PinSourceTx				GPIO_PinSource6
#define		U6_GPIO_PinSourceRx				GPIO_PinSource7
#define 	U6_RCC_AHB1Periph_GPIOx			RCC_AHB1Periph_GPIOC
/* Types */

/* Export variables */
#define IMU_TX_BUFFERSIZE 60
#define IMU_RX_BUFFERSIZE 100
extern 		uint8_t 	U1_TxBuffer[IMU_TX_BUFFERSIZE], U1_RxBuffer[IMU_RX_BUFFERSIZE];
#define ROVER_TX_BUFFERSIZE 0
#define ROVER_RX_BUFFERSIZE 100
extern 		uint8_t		U2_RxBuffer[ROVER_RX_BUFFERSIZE];
#define LORA_RX_BUFFERSIZE	58
#define LORA_TX_BUFFERSIZE 200
extern 		uint8_t 	U6_TxBuffer[LORA_TX_BUFFERSIZE], U6_RxBuffer[LORA_RX_BUFFERSIZE + 1]; // +1 for NULL-terminated
/* Export Function */
void 	USART1_Config(uint32_t  BaudRate);
void 	USART2_Config(uint32_t  BaudRate);
void 	USART6_Config(uint32_t  BaudRate);

void 	U1_SendData(uint16_t NbOfByte);
void 	U6_SendData(uint16_t NbOfByte);

#endif
