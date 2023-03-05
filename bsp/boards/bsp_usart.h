#ifndef BSP_USART_H
#define BSP_USART_H
#include "struct_typedef.h"
#include "stm32f4xx_hal.h"
#define Vision_DataLength 100
#define Buffer_Len 100


extern uint8_t Vision_Data[Vision_DataLength];
extern void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

extern void usart1_tx_dma_init(void);
extern void usart1_tx_dma_enable(uint8_t *data, uint16_t len);

extern void MY_UART_DMA_Init(void);
extern void MY_UART_callback(UART_HandleTypeDef *huart);

#endif
