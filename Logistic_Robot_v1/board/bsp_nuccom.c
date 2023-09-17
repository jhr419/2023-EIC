/**
 * encoding:UTF-8
 * @file bsp_nuccom.c
 * @author Brandon
 * @brief  使能双dma
 * @history
 *  Version    Date            Author          Modification
 *  V1.0.0     2023-9-8     Brandon         1. done usart6 rx
 * 
 */
#include "main.h"
#include "bsp_nuccom.h"

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
void NUC_com_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
	//使能DMA串口接收
	SET_BIT(huart6.Instance->CR3 , USART_CR3_DMAR);
	//使能空闲中断
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
	
	__HAL_DMA_DISABLE(&hdma_usart6_rx);
	while(hdma_usart6_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart6_rx);
    }

    hdma_usart6_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    //memory buffer 1
    //内存缓冲区1
    hdma_usart6_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //memory buffer 2
    //内存缓冲区2
    hdma_usart6_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //data length
    //数据长度
    hdma_usart6_rx.Instance->NDTR = dma_buf_num;
    //enable double memory buffer
    //使能双缓冲区
    SET_BIT(hdma_usart6_rx.Instance->CR, DMA_SxCR_DBM);

    //enable DMA
    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart6_rx);
}