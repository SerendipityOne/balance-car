#ifndef __SERIAL_DMA_H
#define __SERIAL_DMA_H

#include "main.h"

void Serial_DMA_Init(void);
void Serial_DMA_SendByte(uint8_t Byte);
void Serial_DMA_SendArray(uint8_t* Array, uint16_t Length);
void Serial_DMA_SendString(char* String);
void Serial_DMA_SendNumber(uint32_t Number, uint8_t Length);
void Serial_DMA_Printf(char* format, ...);
uint16_t Serial_DMA_GetRxLength(void);
void HAL_UART_RxIdleCallback(UART_HandleTypeDef* huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size);

#endif
