/*
 * sys_usart_3.h
 *
 *  Created on: Jan 5, 2014
 *      Author: abatross
 */

#ifndef SYS_usart_3_H_
#define SYS_usart_3_H_


#include "definitions_global.h"

#include "IO_DEFINITIONS_BOARD_BASE.h"

#include "LED_SYS_PUBLIC.H"

#include "sys_usart_3_definitions_public.h"

unsigned char	u8_sys_usart_3_MODE=0;

u8	u8_sys_usart_3_PACKAGE_CAPTURE_INTERVAL_in_ms=10;

u16 u16_array_sys_usart_3__test_l5[5];

//u8 u8_array_sys_usart_3_tx[const_u8_usart_3_tx_buffer_length];
//u8 u8_array_sys_usart_3_rx[const_u8_usart_3_rx_buffer_length];
//u8 u8_array_sys_usart_3_rx_PUBLIC[const_u8_usart_3_rx_buffer_length];

u16 u16_array_sys_usart_3_tx[const_u16_usart_3_tx_buffer_length];
u16 u16_array_sys_usart_3_rx[const_u16_usart_3_rx_buffer_length];
u16 u16_array_sys_usart_3_rx_PUBLIC[const_u16_usart_3_rx_buffer_length];

unsigned short u16_sys_usart_3_number_of_rx_bytes_PUBLIC=0;

unsigned char	u8_sys_usart_3_State_Main=0;
#define def_u8_sys_usart_3_State_Main_IDLE		0
#define def_u8_sys_usart_3_State_Main_STARTUP	1
#define def_u8_sys_usart_3_State_Main_READY		2
#define def_u8_sys_usart_3_State_Main_ERROR		3

unsigned char	u8_sys_usart_3_State_Sub_TX=0;
#define def_u8_sys_usart_3_State_Sub_TX_IDLE	0
#define def_u8_sys_usart_3_State_Sub_TX_BUSY	1
#define def_u8_sys_usart_3_State_Sub_TX_READY	2

unsigned char	u8_sys_usart_3_State_Sub_RX=0;
#define def_u8_sys_usart_3_State_Sub_RX_IDLE			0
#define def_u8_sys_usart_3_State_Sub_RX_BUSY			1
#define def_u8_sys_usart_3_State_Sub_RX_READY			2
#define def_u8_sys_usart_3_State_Sub_RX_DATA_READY		3

unsigned short u16_sys_usart_3_time_counter_0=0;
unsigned short u16_sys_usart_3_time_counter_TX=0;
unsigned short u16_sys_usart_3_time_counter_RX=0;

USART_InitTypeDef 	USART_InitStructure_sys_usart_3;
GPIO_InitTypeDef 	GPIO_InitStructure_sys_usart_3;
NVIC_InitTypeDef 	NVIC_InitStructure_sys_usart_3;
DMA_InitTypeDef  	DMA_InitStructure_sys_usart_3_TX;
DMA_InitTypeDef  	DMA_InitStructure_sys_usart_3_RX;

//DMA USAGE
//Table 20. DMA1 request mapping
//ve
//Table 21. DMA2 request mapping
//de ki yerleşime bağımlı setup

//Bu durumda channel ın pek anlamı yok, aynı anda kullanılabilirlik olarak.
//yani DMA1&2 deki toplam 16 streamın 8 i 4 usart a gidicek full dublex desteği için!!!


//DMA CHANNEL VE STREAM PAYLAŞIMI:

//DMA 1, CHANNEL 4 ÜZERİNDE, USART3RX=>STREAM 1
//DMA 1, CHANNEL 4 ÜZERİNDE, USART3TX=>STREAM 3

//DMA 1, CHANNEL 4 ÜZERİNDE, USART2RX=>STREAM 5
//DMA 1, CHANNEL 4 ÜZERİNDE, USART2TX=>STREAM 6


//DMA 2, CHANNEL 4 ÜZERİNDE, USART1RX=>STREAM 2
//DMA 2, CHANNEL 4 ÜZERİNDE, USART1TX=>STREAM 7

//DMA 2, CHANNEL 5 ÜZERİNDE, USART6RX=>STREAM 2
//DMA 2, CHANNEL 5 ÜZERİNDE, USART6TX=>STREAM 7


//usart1 tx tabloda dma2 stream 7 channel 4 de!!! görünüyor.
//#define USART2_DR_ADDRESS                ((uint32_t)USART2 + 0x04)
#define USART3_DR_ADDRESS                ((u32)USART3 + 0x04)
#define USART3_TX_DMA_CHANNEL            DMA_Channel_4
#define USART3_TX_DMA_STREAM             DMA1_Stream3
#define USART3_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF3
#define USART3_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF3
#define USART3_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF3
#define USART3_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF3
#define USART3_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF3
#define USART3_TX_DMA_IRQn               DMA1_Stream3_IRQn
#define USART3_TX_DMA_IRQHandler         DMA1_Stream3_IRQHandler,

//usart1 rx tabloda dma2 stream 5 channel 4 de!!! görünüyor.
#define USART3_RX_DMA_CHANNEL            DMA_Channel_4
#define USART3_RX_DMA_STREAM             DMA1_Stream1
#define USART3_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF1
#define USART3_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF1
#define USART3_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF1
#define USART3_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF1
#define USART3_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF1
#define USART3_RX_DMA_IRQn               DMA1_Stream1_IRQn
#define USART3_RX_DMA_IRQHandler         DMA1_Stream1_IRQHandler


unsigned short u16_sys_usart_3_number_of_rx_bytes=0;
unsigned short u16_sys_usart_3_number_of_rx_bytes_previous=0;


//======================================================================



//below is OLD!!!










////bool
////      bool_usart1_byte_received=FALSE,
////      bool_usart1_package_received=FALSE,
////      bool_usart1_protect_receive_buffer=FALSE,
////      bool_usart1_tx_busy=FALSE;
////volatile u8
//u8
//      u8_usart1_rx_timeout_down_counter=0,
//      u8_usart1_rx_buffer_w_index=0,
//      u8_array_usart1_rx_buffer[const_u8_usart1_rx_buffer_length],
//      u8_usart1_tx_buffer_r_index_counter=0,
//      u8_usart1_tx_buffer_r_index_limit=0,
//      u8_array_usart1_tx_buffer[const_u8_usart1_tx_buffer_length];
//
//
//u8 u8_sys_uart_1_state;
//u16 u16_sys_uart_1_wait_down_counter=1000;
//
//USART_InitTypeDef USART_InitStructure_1;
//  GPIO_InitTypeDef GPIO_InitStructure_1;
//
//  extern u8 usbhidtxbuf[];
//  extern u8 usbhidrxbuf[];
//
//
//  volatile u8_byte u8_sys_uart_1_status_flags;
//#define  u1_sys_uart_1_status_flag_uart_package_received			u8_sys_uart_1_status_flags.bits.b7
//#define  u1_sys_uart_1_status_flag_uart_byte_received				u8_sys_uart_1_status_flags.bits.b6
//#define  u1_sys_uart_1_status_flag_uart_protect_receive_buffer		u8_sys_uart_1_status_flags.bits.b5
//#define  u1_sys_uart_1_status_flag_uart_tx_busy						u8_sys_uart_1_status_flags.bits.b4



#endif /* SYS_USART_1_H_ */
