#ifndef __UART_H
#define __UART_H


#include "stm32f4xx.h"
#include <rtdevice.h>
#include <rthw.h>
#include "serial.h"


struct uart_cfg
{
//    rt_uint8_t tx_int_en;
    rt_uint8_t rx_int_en;
    rt_uint8_t tx_dma_en;
    rt_uint8_t rx_dma_en;
};


#define UART_CLK                RCC_APB2Periph_USART1
#define UART_GPIO_CLK           RCC_AHB1Periph_GPIOA
#define UART_DMA_TX_STREAM      DMA2_Stream7
#define UART_DMA_RX_STREAM      DMA2_Stream5
#define UART_PORT               GPIOA
#define UART_INSTANCE           USART1
#define UART_IRQ                USART1_IRQn
#define UART_TX_PIN             GPIO_Pin_9
#define UART_RX_PIN             GPIO_Pin_10
#define UART_TX_PIN_AF          GPIO_PinSource9
#define UART_RX_PIN_AF          GPIO_PinSource10
#define UART_AF                 GPIO_AF_USART1

#define UART_NAME               "uart1"















#endif /* __UART_H */
