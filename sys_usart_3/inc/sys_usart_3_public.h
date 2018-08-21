/*
 * sys_usart_3_public.h
 *
 *  Created on: Jan 7, 2014
 *      Author: abatross
 */

#ifndef SYS_usart_3_PUBLIC_H_
#define SYS_usart_3_PUBLIC_H_

#include "sys_usart_3_definitions_public.h"

extern void f_sys_usart_3_STARTUP(
		u8	uart_mode,
		uint32_t USART_BaudRate,
		uint16_t USART_WordLength,
		uint16_t USART_StopBits,
		uint16_t USART_Parity,
		uint16_t USART_HardwareFlowControl
		);

extern void f_tasking_1_ms_base_sys_usart_3(void);

extern void f_sys_usart_3_TX_request(u16 tx[], u8 length);

extern unsigned char f_sys_usart_3_is_RX_data_ready();

extern u16 u16_array_sys_usart_3_rx_PUBLIC[];
extern unsigned short u16_sys_usart_3_number_of_rx_bytes_PUBLIC;

extern void f_sys_usart_3_RELEASE_RX_package();

extern 	u8	u8_sys_usart_3_PACKAGE_CAPTURE_INTERVAL_in_ms;

//extern void f_tasking_1_ms_base_sys_uart_1(void);
//extern   u8 f_uart_1_command_from_usb(void);
//
//#define const_u8_usart1_tx_buffer_length   64
//#define const_u8_usart1_rx_buffer_length   64
//
//extern void usart1_setup(u32 baud,u8 length,u8 stop,u8 parity);
//
//extern void task_usart1(void);
//
//extern void usart1_transmit(u8 kelime[],u16 length);
//
//extern void f_uart_1_rs_485_data_enable(void);
//extern void f_uart_1_rs_485_read_enable(void);
//
//
////extern
////bool
////      bool_usart1_byte_received,
////      bool_usart1_package_received,
////      bool_usart1_protect_receive_buffer,
////      bool_usart1_tx_busy;
//extern    volatile u8_byte u8_sys_uart_1_status_flags;
//#define  u1_sys_uart_1_status_flag_uart_package_received			u8_sys_uart_1_status_flags.bits.b7
//#define  u1_sys_uart_1_status_flag_uart_byte_received				u8_sys_uart_1_status_flags.bits.b6
//#define  u1_sys_uart_1_status_flag_uart_protect_receive_buffer		u8_sys_uart_1_status_flags.bits.b5
//#define  u1_sys_uart_1_status_flag_uart_tx_busy						u8_sys_uart_1_status_flags.bits.b4
//
//
//extern
//u8
//      u8_usart1_rx_timeout_down_counter,
//      u8_usart1_rx_buffer_w_index,
//      u8_array_usart1_rx_buffer[],
//      u8_usart1_tx_buffer_r_index_counter,
//      u8_usart1_tx_buffer_r_index_limit,
//      u8_array_usart1_tx_buffer[];
//
//extern USART_InitTypeDef USART_InitStructure_1;
//
//void f_uart_1_start(void);
//void f_uart_1_kill(void);

#endif /* UART_SYS_1_PUBLIC_H_ */
