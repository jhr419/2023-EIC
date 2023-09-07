/**
 * encoding:UTF-8
 * @file bsp_usart.c
 * @author Brandon
 * @brief  
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     Dec-26-2018     Brandon         1. done usart6
 * 
 */

#include "main.h"
#include "bsp_usart.h"

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

void usart6_tx_dma_init(void)
{
		//使能DMA串口接受和发送
		SET_BIT(huart6.Instance->CR3 , USART_CR3_DMAR);
		SET_BIT(huart6.Instance->CR3 , USART_CR3_DMAR);
	
	  __HAL_DMA_DISABLE(&hdma_usart6_tx);
	
	  while(hdma_usart6_tx.Instance ->CR &DMA_SxCR_EN)
		{
				__HAL_DMA_DISABLE(&hdma_usart6_tx);
		}
		
		hdma_usart6_tx.Instance->PAR  = (uint32_t) & (USART6->DR);
		hdma_usart6_tx.Instance->M1AR = (uint32_t)(NULL);
		hdma_usart6_tx.Instance->NDTR = 0;
		
		
}
void usart6_tx_dma_enable(uint8_t *data, uint16_t len)
{
	//disable DMA
	//失效DMA
	//为什么每次都要失效DMA
	__HAL_DMA_DISABLE(&hdma_usart6_tx);
	
	
	while(hdma_usart6_tx.Instance ->CR & DMA_SxCR_EN)
	{
		__HAL_DMA_DISABLE(&hdma_usart6_tx);
	}
	
	__HAL_DMA_CLEAR_FLAG(&hdma_usart6_tx,DMA_HISR_TCIF6);
	
	hdma_usart6_tx.Instance->M1AR = (uint32_t)(data);
	__HAL_DMA_SET_COUNTER(&hdma_usart6_tx , len);
	
	__HAL_DMA_ENABLE(&hdma_usart6_tx);
}