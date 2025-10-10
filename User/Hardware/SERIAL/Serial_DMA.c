#include "Serial_DMA.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// 定义接收缓冲区大小
#define SERIAL_DMA_RX_BUFFER_SIZE 256

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

// DMA串口接收相关变量
uint8_t dma_rx_buffer[SERIAL_DMA_RX_BUFFER_SIZE];
uint8_t dma_tx_buffer[SERIAL_DMA_RX_BUFFER_SIZE];
uint16_t dma_rx_head = 0;
uint16_t dma_rx_tail = 0;
uint8_t dma_uart_tx_busy = 0;

/**
  * @brief  DMA串口初始化
  * @retval 无
  */
void Serial_DMA_Init(void) {
  dma_uart_tx_busy = 0;
  dma_rx_head = 0;
  dma_rx_tail = 0;

  // 启动空闲中断和DMA接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart3, dma_rx_buffer, SERIAL_DMA_RX_BUFFER_SIZE);
}

/**
  * @brief  DMA串口发送一个字节
  * @param  Byte 要发送的一个字节
  * @retval 无
  */
void Serial_DMA_SendByte(uint8_t Byte) {
  HAL_UART_Transmit(&huart3, &Byte, 1, 1000);
}

/**
  * @brief  DMA串口发送一个数组
  * @param  Array 要发送数组的首地址
  * @param  Length 要发送数组的长度
  * @retval 无
  */
void Serial_DMA_SendArray(uint8_t* Array, uint16_t Length) {
  HAL_UART_Transmit(&huart3, Array, Length, 1000);
}

/**
  * @brief  DMA串口发送一个字符串
  * @param  String 要发送字符串的首地址
  * @retval 无
  */
void Serial_DMA_SendString(char* String) {
  HAL_UART_Transmit(&huart3, (uint8_t*)String, strlen(String), 1000);
}

/**
  * @brief  DMA串口发送数字
  * @param  Number 要发送的数字，范围：0~4294967295
  * @param  Length 要发送数字的长度，范围：0~10
  * @retval 无
  */
void Serial_DMA_SendNumber(uint32_t Number, uint8_t Length) {
  uint8_t i;
  // 优化：避免重复计算幂，使用累除法
  uint32_t divisor = 1;
  for (i = 0; i < Length - 1; i++) {
    divisor *= 10;
  }

  for (i = 0; i < Length; i++) {
    Serial_DMA_SendByte(Number / divisor % 10 + '0');
    divisor /= 10;
  }
}

/**
  * @brief  DMA格式化打印函数
  * @param  format 格式化字符串
  * @param  ... 可变参数
  * @retval 无
  */
void Serial_DMA_Printf(char* format, ...) {
  static char str[256];  // 使用静态变量避免栈溢出
  va_list ap;
  va_start(ap, format);
  vsnprintf(str, sizeof(str), format, ap);
  va_end(ap);

  // 等待上一次发送完成
  while (dma_uart_tx_busy) {
    // 空等待
  }

  dma_uart_tx_busy = 1;
  memcpy(dma_tx_buffer, str, strlen(str));
  if (HAL_UART_Transmit_DMA(&huart3, dma_tx_buffer, strlen(str)) != HAL_OK) {
    dma_uart_tx_busy = 0;
  }
}

/**
  * @brief  DMA串口获取接收数据长度
  * @retval 接收数据长度
  */
uint16_t Serial_DMA_GetRxLength(void) {
  return (dma_rx_head >= dma_rx_tail) ? (dma_rx_head - dma_rx_tail) : (SERIAL_DMA_RX_BUFFER_SIZE - dma_rx_tail + dma_rx_head);
}

/**
  * @brief  UART空闲中断回调函数
  * @param  huart UART句柄指针
  * @retval 无
  */
void HAL_UART_RxIdleCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance == USART3) {
    // 计算接收到的数据长度
    uint16_t rx_data_len = SERIAL_DMA_RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(&hdma_usart3_rx);

    // 更新接收数据头指针
    dma_rx_head = rx_data_len;

    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, dma_rx_buffer, SERIAL_DMA_RX_BUFFER_SIZE);
  }
}

/**
  * @brief  UART发送完成回调函数
  * @param  huart UART句柄指针
  * @retval 无
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef* huart) {
  if (huart->Instance == USART3) {
    dma_uart_tx_busy = 0;
  }
}

/**
  * @brief  UART接收事件回调函数
  * @param  huart UART句柄指针
  * @param  Size  接收到的数据大小
  * @retval 无
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size) {
  if (huart == &huart3) {
    // 更新接收缓冲区的头指针位置
    dma_rx_head = (dma_rx_head + Size) % SERIAL_DMA_RX_BUFFER_SIZE;
    // 重新启动DMA接收
    HAL_UARTEx_ReceiveToIdle_DMA(&huart3, dma_rx_buffer, SERIAL_DMA_RX_BUFFER_SIZE);
  }
}
