#include "stm32f4xx.h"

/*-------- Hardware config USART1 (can change) ---------
 * @brief:	Baudrate 115200
 *			Used to communicate between MCU & IMU
 * @pinout: PA9: TX -> RX IMU
 *			PA10: RX -> TX IMU
 */
#define 	U1_GPIOx						GPIOA
#define 	U1_GPIO_Pin_Tx        			GPIO_Pin_9
#define 	U1_GPIO_Pin_Rx					GPIO_Pin_10
#define		U1_GPIO_PinSourceTx				GPIO_PinSource9
#define		U1_GPIO_PinSourceRx				GPIO_PinSource10
#define 	U1_RCC_AHB1Periph_GPIOx			RCC_AHB1Periph_GPIOA
/*-------- Hardware config USART2 (can change) ---------
 * @brief:	Baudrate 9600
 *			Used to communicate between LORA-GPS & LORA-BASE
 * @pinout:	PD5: TX -> RX Lora-ROVER 
 *			PD6: RX -> TX Lora-ROVER
 */
#define 	U2_GPIOx						GPIOD
#define 	U2_GPIO_Pin_Tx        			GPIO_Pin_5
#define 	U2_GPIO_Pin_Rx					GPIO_Pin_6
#define		U2_GPIO_PinSourceTx				GPIO_PinSource5
#define		U2_GPIO_PinSourceRx				GPIO_PinSource6
#define 	U2_RCC_AHB1Periph_GPIOx			RCC_AHB1Periph_GPIOD
/*-------- Hardware config USART6 (can change) ---------
 * @brief:	Baudrate 19200
 *			Used to communicate between LORA-MCU & LORA-PC (C# interface)
 * @pinout: PC6: TX -> RX Lora-MCU
 *			PC7: RX -> TX Lora-MCU 	
 */
#define 	U6_GPIOx						GPIOC
#define 	U6_GPIO_Pin_Tx        			GPIO_Pin_6
#define 	U6_GPIO_Pin_Rx					GPIO_Pin_7
#define		U6_GPIO_PinSourceTx				GPIO_PinSource6
#define		U6_GPIO_PinSourceRx				GPIO_PinSource7
#define 	U6_RCC_AHB1Periph_GPIOx			RCC_AHB1Periph_GPIOC
/* Types */

/* Export variables */
#define MAX_LORA_BUFFERSIZE	58

#define USART1_BUFFERSIZE_RX 20
#define USART1_BUFFERSIZE_TX 100
extern 		uint8_t 	U1_TxBuffer[20], U1_RxBuffer[100];
#define USART2_BUFFERSIZE_RX 200
#define USART2_BUFFERSIZE_TX 5
extern 		uint8_t		U2_TxBuffer[USART2_BUFFERSIZE_TX], U2_RxBuffer[USART2_BUFFERSIZE_RX];
#define USART6_BUFFERSIZE_RX 200
#define USART6_BUFFERSIZE_TX 200
extern 		uint8_t 	U6_TxBuffer[USART6_BUFFERSIZE_TX], U6_RxBuffer[USART6_BUFFERSIZE_RX];
/* Export Function */
void 	USART1_Config(uint32_t  BaudRate);
void 	USART2_Config(uint32_t  BaudRate);
void 	USART6_Config(uint32_t  BaudRate);

void 	U1_SendData(uint16_t NbOfByte);
void 	U2_SendData(uint16_t NbOfByte);	// U2 send nothing, doesn't use
void 	U6_SendData(uint16_t NbOfByte);
