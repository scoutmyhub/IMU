#include "bsp_usart.h"
#include "main.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;
uint8_t Vision_Data[Vision_DataLength];



void usart1_tx_dma_init(void)
{

    //enable the DMA transfer for the receiver and tramsmit request
    //ʹ��DMA���ڽ��պͷ���
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
    hdma_usart1_tx.Instance->M0AR = (uint32_t)(NULL);
    hdma_usart1_tx.Instance->NDTR = 0;


}
void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}



void usart6_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{

    //enable the DMA transfer for the receiver and tramsmit request
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAR);
    SET_BIT(huart6.Instance->CR3, USART_CR3_DMAT);

    //enalbe idle interrupt
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);



    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart6_rx);
    
    while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_rx, DMA_LISR_TCIF1);

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART6->DR);
    //memory buffer 1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, dma_buf_num);

    //enable double memory buffer
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    __HAL_DMA_ENABLE(&hdma_usart6_rx);


    //disable DMA
    //ʧЧDMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    hdma_usart6_tx.Instance->PAR = (uint32_t) & (USART6->DR);

}



void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    __HAL_DMA_DISABLE(&hdma_usart6_tx);

    while(hdma_usart6_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx, DMA_HISR_TCIF6);

    hdma_usart6_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart6_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart6_tx);
}





static int MY_UART_Receive_DMA(UART_HandleTypeDef *huart, uint8_t *pData, uint32_t Size)
{
	uint32_t tmp1 = 0;

	tmp1 = huart->RxState;
	if (tmp1 == HAL_UART_STATE_READY)
	{
		if ((pData == NULL) || (Size == 0))
		{
			return HAL_ERROR;
		}

		/* Process Locked */
		__HAL_LOCK(huart);

		huart->pRxBuffPtr = pData;
		huart->RxXferSize = Size;
		huart->ErrorCode = HAL_UART_ERROR_NONE;

		/* Enable the DMA Stream */
		HAL_DMA_Start(huart->hdmarx, (uint32_t)&huart->Instance->DR, (uint32_t)pData, Size);

		/* Enable the DMA transfer for the receiver request by setting the DMAR bit
		in the UART CR3 register */
		huart->Instance->CR3 |= USART_CR3_DMAR;

		/* Process Unlocked */
		__HAL_UNLOCK(huart);

		return HAL_OK;
	}
	else
	{
		return HAL_BUSY;
	}
}

void MY_UART_callback(UART_HandleTypeDef *huart)
{
	if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE))
	{
		__HAL_UART_CLEAR_IDLEFLAG(huart);

		uint32_t DMA_FLAGS = __HAL_DMA_GET_TC_FLAG_INDEX(huart->hdmarx);

		__HAL_DMA_DISABLE(huart->hdmarx);
		__HAL_DMA_CLEAR_FLAG(huart->hdmarx, DMA_FLAGS);
		__HAL_DMA_SET_COUNTER(huart->hdmarx, Buffer_Len);
		__HAL_DMA_ENABLE(huart->hdmarx);
	}
}

void MY_UART_DMA_Init(void)
{
	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

	MY_UART_Receive_DMA(&huart1, Vision_Data, Vision_DataLength);
}





