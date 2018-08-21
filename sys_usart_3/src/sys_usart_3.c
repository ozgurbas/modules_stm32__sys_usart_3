/*
 * sys_usart_3.c
 *
 *  Created on: Jan 5, 2014
 *      Author: abatross
 */

#include 	"sys_usart_3.h"



#ifdef def_enable_sys_usart_3

void f_sys_usart_3_RELEASE_RX_package()
{
	u8_sys_usart_3_State_Sub_RX=def_u8_sys_usart_3_State_Sub_RX_READY;
}

unsigned char f_sys_usart_3_is_RX_data_ready()
{
	if(u8_sys_usart_3_State_Sub_RX==def_u8_sys_usart_3_State_Sub_RX_DATA_READY)
	{
		return READY;
	}
	else
	{
		return WAIT;
	}
}

void f_sys_usart_3_RX_copy_to_PUBLIC_array()
{
	u16 i;
	u16_sys_usart_3_number_of_rx_bytes_PUBLIC=u16_sys_usart_3_number_of_rx_bytes;

	for(i=0;i<const_u16_usart_3_rx_buffer_length;i++) //silsin diye full length basılıyor!!!
	{
		u16_array_sys_usart_3_rx_PUBLIC[i]=u16_array_sys_usart_3_rx[i];
	}
}

void f_sys_usart_3_RX_flush()
{
	u16 i;
	for(i=0;i<const_u16_usart_3_rx_buffer_length;i++)
	{
		u16_array_sys_usart_3_rx[i]=0;
	}
}

void f_sys_usart_3_RX_request()
{
	if( 	(u8_sys_usart_3_State_Sub_RX==def_u8_sys_usart_3_State_Sub_RX_READY)
			)
	{

		USART_ClearFlag(USART3,USART_FLAG_RXNE);

		//ref : ___STM32F4xx_StdPeriph_Examples-USART-USART_TwoBoards-DataExchangeDMA
		//===========================
		/* Configure DMA controller to manage USART TX and RX DMA request ----------*/
		DMA_InitStructure_sys_usart_3_RX.DMA_PeripheralBaseAddr = USART3_DR_ADDRESS;
		DMA_InitStructure_sys_usart_3_RX.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
		DMA_InitStructure_sys_usart_3_RX.DMA_MemoryInc = DMA_MemoryInc_Enable;

		DMA_InitStructure_sys_usart_3_RX.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
		DMA_InitStructure_sys_usart_3_RX.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;

		DMA_InitStructure_sys_usart_3_RX.DMA_Mode = DMA_Mode_Normal;
		DMA_InitStructure_sys_usart_3_RX.DMA_Priority = DMA_Priority_VeryHigh;
		//DMA_InitStructure_sys_usart_3_RX.DMA_FIFOMode = DMA_FIFOMode_Enable;//SIÇ!!! DATAYI FİFODA BEKLETİR!!!
		DMA_InitStructure_sys_usart_3_RX.DMA_FIFOMode = DMA_FIFOMode_Disable;
		//DMA_InitStructure_sys_usart_3_RX.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
		DMA_InitStructure_sys_usart_3_RX.DMA_MemoryBurst = DMA_MemoryBurst_Single;
		DMA_InitStructure_sys_usart_3_RX.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
		/* Here only the unchanged parameters of the DMA initialization structure are
		configured. During the program operation, the DMA will be configured with
		different parameters according to the operation phase */

		//=============================

	    DMA_DeInit(USART3_RX_DMA_STREAM);
	    DMA_InitStructure_sys_usart_3_RX.DMA_Channel = USART3_RX_DMA_CHANNEL;
	    DMA_InitStructure_sys_usart_3_RX.DMA_DIR = DMA_DIR_PeripheralToMemory;
	    /****************** USART will Receive Specific Command *******************/
	    /* Configure the DMA to receive 2 bytes (transaction command), in case of USART receiver */
	    DMA_InitStructure_sys_usart_3_RX.DMA_Memory0BaseAddr = (uint32_t)u16_array_sys_usart_3_rx;
	    DMA_InitStructure_sys_usart_3_RX.DMA_BufferSize = (uint16_t)(const_u16_usart_3_rx_buffer_length); 	//max
	    DMA_Init(USART3_RX_DMA_STREAM, &DMA_InitStructure_sys_usart_3_RX);

	    /* Enable the USART Rx DMA request */
	    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	    /* Enable the DMA RX Stream */
	    DMA_Cmd(USART3_RX_DMA_STREAM, ENABLE);

	    //receive started, package capture will be processed in state machine!!!

	    u8_sys_usart_3_State_Sub_RX=def_u8_sys_usart_3_State_Sub_RX_BUSY;

	    u16_sys_usart_3_number_of_rx_bytes=0;
	    u16_sys_usart_3_number_of_rx_bytes_previous=0;

	}
}

void f_sys_usart_3_TX_request(u16 tx[], u16 length)
{
	u16 i;
	if( 	(length<(const_u16_usart_3_tx_buffer_length+1)) 	&&
			(u8_sys_usart_3_State_Sub_TX==def_u8_sys_usart_3_State_Sub_TX_READY) &&
			(length>0)
			)
	{
//			USART_ClearFlag(USART3,USART_FLAG_TC);
//			USART_ClearFlag(USART3,USART_FLAG_TXE);

			for(i=0;i<length;i++)
			{
				u16_array_sys_usart_3_tx[i]=tx[i];
			}

			//ref : ___STM32F4xx_StdPeriph_Examples-USART-USART_TwoBoards-DataExchangeDMA
			//===========================
			/* Configure DMA controller to manage USART TX and RX DMA request ----------*/
			DMA_InitStructure_sys_usart_3_TX.DMA_PeripheralBaseAddr = USART3_DR_ADDRESS;
			DMA_InitStructure_sys_usart_3_TX.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
			DMA_InitStructure_sys_usart_3_TX.DMA_MemoryInc = DMA_MemoryInc_Enable;

			DMA_InitStructure_sys_usart_3_TX.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
			DMA_InitStructure_sys_usart_3_TX.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;

			DMA_InitStructure_sys_usart_3_TX.DMA_Mode = DMA_Mode_Normal;
			DMA_InitStructure_sys_usart_3_TX.DMA_Priority = DMA_Priority_Low;
			//DMA_InitStructure_sys_usart_3_TX.DMA_FIFOMode = DMA_FIFOMode_Enable;//SIÇ!!! DATAYI FİFODA BEKLETİR!!!
			DMA_InitStructure_sys_usart_3_TX.DMA_FIFOMode = DMA_FIFOMode_Disable;
			//DMA_InitStructure_sys_usart_3_TX.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
			DMA_InitStructure_sys_usart_3_TX.DMA_MemoryBurst = DMA_MemoryBurst_Single;
			DMA_InitStructure_sys_usart_3_TX.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;


			/* Here only the unchanged parameters of the DMA initialization structure are
			configured. During the program operation, the DMA will be configured with
			different parameters according to the operation phase */

			//=============================

			DMA_DeInit(USART3_TX_DMA_STREAM);
			DMA_InitStructure_sys_usart_3_TX.DMA_Channel = USART3_TX_DMA_CHANNEL;
			DMA_InitStructure_sys_usart_3_TX.DMA_DIR = DMA_DIR_MemoryToPeripheral;


			// Prepare the DMA to transfer the x bytes from the memory to the USART
			DMA_InitStructure_sys_usart_3_TX.DMA_Memory0BaseAddr = (uint32_t)u16_array_sys_usart_3_tx;
			DMA_InitStructure_sys_usart_3_TX.DMA_BufferSize = (uint16_t)length;
			DMA_Init(USART3_TX_DMA_STREAM, &DMA_InitStructure_sys_usart_3_TX);

			// Enable the USART DMA requests
			USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);

			// Clear the TC bit in the SR register by writing 0 to it
			USART_ClearFlag(USART3, USART_FLAG_TC);

			GPIO_WriteBit(def_GPIO_of_usart_3_FLW,def_Pin_of_usart_3_FLW,Bit_SET); //write enabled!!!

			// Enable the DMA TX Stream, USART will start sending the x bytes...
			DMA_Cmd(USART3_TX_DMA_STREAM, ENABLE);


			u8_sys_usart_3_State_Sub_TX=def_u8_sys_usart_3_State_Sub_TX_BUSY;
			u16_sys_usart_3_time_counter_TX=3000; //min 9600 >>> 255 byte ~ 2.5 sec
	}

}

void f_sys_usart_3_STARTUP(
		u8	uart_mode,
		uint32_t USART_BaudRate, //min 9600!!!
		uint16_t USART_WordLength,
		uint16_t USART_StopBits,
		uint16_t USART_Parity,
		uint16_t USART_HardwareFlowControl
		)
{



	  /* USARTx configured as follow:
	        - BaudRate = 5250000 baud
			   - Maximum BaudRate that can be achieved when using the Oversampling by 8
			     is: (USART APB Clock / 8)
				 Example:
				    - (USART3 APB1 Clock / 8) = (42 MHz / 8) = 5250000 baud
				    - (USART3 APB2 Clock / 8) = (84 MHz / 8) = 10500000 baud
			   - Maximum BaudRate that can be achieved when using the Oversampling by 16
			     is: (USART APB Clock / 16)
				 Example: (USART3 APB1 Clock / 16) = (42 MHz / 16) = 2625000 baud
				 Example: (USART3 APB2 Clock / 16) = (84 MHz / 16) = 5250000 baud
	        - Word Length = 8 Bits
	        - one Stop Bit
	        - No parity
	        - Hardware flow control disabled (RTS and CTS signals)
	        - Receive and transmit enabled
	  */
	  USART_InitStructure_sys_usart_3.USART_BaudRate = USART_BaudRate;
	  USART_InitStructure_sys_usart_3.USART_WordLength = USART_WordLength;
	  USART_InitStructure_sys_usart_3.USART_StopBits = USART_StopBits;
	  /* When using Parity the word length must be configured to 9 bits */
	  USART_InitStructure_sys_usart_3.USART_Parity = USART_Parity;
	  USART_InitStructure_sys_usart_3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	  USART_InitStructure_sys_usart_3.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;


	  NVIC_InitStructure_sys_usart_3.NVIC_IRQChannel = USART3_IRQn;
	  NVIC_InitStructure_sys_usart_3.NVIC_IRQChannelPreemptionPriority = def_u8_sys_usart_3_IRQChannelPreemptionPriority;
	  NVIC_InitStructure_sys_usart_3.NVIC_IRQChannelSubPriority = 0;
	  NVIC_InitStructure_sys_usart_3.NVIC_IRQChannelCmd = ENABLE;

	  u8_sys_usart_3_State_Main=def_u8_sys_usart_3_State_Main_STARTUP;
	  u16_sys_usart_3_time_counter_0=200;
}


void f_tasking_1_ms_base_sys_usart_3(void)
{

	if(u16_sys_usart_3_time_counter_0!=0)
	{
		u16_sys_usart_3_time_counter_0--;
	}


	if(u8_sys_usart_3_State_Main==def_u8_sys_usart_3_State_Main_READY)
	{
		//RX machine
		//[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[
		if(u8_sys_usart_3_State_Sub_RX==def_u8_sys_usart_3_State_Sub_RX_READY)
		{
			//listen always mode
			f_sys_usart_3_RX_flush();
			f_sys_usart_3_RX_request();
			u8_sys_usart_3_State_Sub_RX=def_u8_sys_usart_3_State_Sub_RX_BUSY;

//			//{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{{
//			  //hw loop test: OK!!!
//			  u16_array_sys_usart_3__test_l5[0]=0xff10;
//			  u16_array_sys_usart_3__test_l5[1]=0xf011;
//			  u16_array_sys_usart_3__test_l5[2]=0xf112;
//			  u16_array_sys_usart_3__test_l5[3]=0xf213;
//			  u16_array_sys_usart_3__test_l5[4]=0xf314;
//
//			  f_sys_usart_3_TX_request(u16_array_sys_usart_3__test_l5, 5);
//			//}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}}
		}
		else if(u8_sys_usart_3_State_Sub_RX==def_u8_sys_usart_3_State_Sub_RX_BUSY)
		{
			//CHECK DMA COUNTER, WHILE NOT ZERO, CHECK FOR STABLE NUMBER OF BYTES!!!

			//uint16_t DMA_GetCurrDataCounter(DMA_Stream_TypeDef* DMAy_Streamx);



			u16_sys_usart_3_number_of_rx_bytes=const_u16_usart_3_rx_buffer_length-DMA_GetCurrDataCounter(USART3_RX_DMA_STREAM);

			if(u16_sys_usart_3_number_of_rx_bytes!=0)
			{
				if(u16_sys_usart_3_number_of_rx_bytes!=u16_sys_usart_3_number_of_rx_bytes_previous)
				{
					u16_sys_usart_3_number_of_rx_bytes_previous=u16_sys_usart_3_number_of_rx_bytes;

					//u16_sys_usart_3_time_counter_RX=30; //2 char time for min 9600 bps
					u16_sys_usart_3_time_counter_RX=u8_sys_usart_3_PACKAGE_CAPTURE_INTERVAL_in_ms; //2 char time for min 9600 bps
				}
				else 	//equalty decrements timeout counter
				{
					u16_sys_usart_3_time_counter_RX--;
					if(u16_sys_usart_3_time_counter_RX==0)
					{
					    /* Clear all DMA Streams flags */
					    DMA_ClearFlag(USART3_RX_DMA_STREAM, USART3_RX_DMA_FLAG_HTIF | USART3_RX_DMA_FLAG_TCIF);

					    /* Disable the DMA Rx Stream */
					    DMA_Cmd(USART3_RX_DMA_STREAM, DISABLE);

					    /* Disable the USART Rx DMA requests */
					    USART_DMACmd(USART3, USART_DMAReq_Rx, DISABLE);

						u8_sys_usart_3_State_Sub_RX=def_u8_sys_usart_3_State_Sub_RX_DATA_READY;

						f_sys_usart_3_RX_copy_to_PUBLIC_array();

					}
				}
			}

		}
//		else if(u8_sys_usart_3_State_Sub_RX==def_u8_sys_usart_3_State_Sub_RX_DATA_READY)
//		{
//			//test only
//			u8_sys_usart_3_State_Sub_RX=def_u8_sys_usart_3_State_Sub_RX_READY;
//		}
		//]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]


		//TX machine
		//[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[[
		if(u8_sys_usart_3_State_Sub_TX==def_u8_sys_usart_3_State_Sub_TX_READY)
		{
//			//DİRECT SEND TO TEST dma collision
//			if( (tx_test<10)&& (USART_GetFlagStatus(USART3, USART_FLAG_TC) == SET) )
//			{
//				USART_SendData(USART3, (uint8_t) tx_test);
//				tx_test++;
//			}
		}
		else if(u8_sys_usart_3_State_Sub_TX==def_u8_sys_usart_3_State_Sub_TX_BUSY)
		{
			if(u16_sys_usart_3_time_counter_TX==0) //FAILED TO TRANSMIT PACKAGE!!!
			{
			      /* Clear DMA Streams flags */
			      DMA_ClearFlag(USART3_TX_DMA_STREAM, USART3_TX_DMA_FLAG_HTIF | USART3_TX_DMA_FLAG_TCIF);

			      /* Disable the DMA Streams */
			      DMA_Cmd(USART3_TX_DMA_STREAM, DISABLE);

			      /* Disable the USART Tx DMA request */
			      USART_DMACmd(USART3, USART_DMAReq_Tx, DISABLE);


			      u8_sys_usart_3_State_Sub_TX=def_u8_sys_usart_3_State_Sub_TX_READY;

			      GPIO_WriteBit(def_GPIO_of_usart_3_FLW,def_Pin_of_usart_3_FLW,Bit_RESET); //read enabled!!!

			}
			//todo usart tx flag bakmayı mükemmel lestirmek icin dma ve dma int in acicagi usart tc interrupt a donustur.
//			else if(	(DMA_GetFlagStatus(USART3_TX_DMA_STREAM, USART3_TX_DMA_FLAG_TCIF) == RESET)&&
//						(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)
//						)  BUUUUUUUUUUUUUUUUUUUUUUG!!! COPY PASTE BUG I!!!

			else if(	(DMA_GetFlagStatus(USART3_TX_DMA_STREAM, USART3_TX_DMA_FLAG_TCIF) == SET)&&
						(USART_GetFlagStatus(USART3, USART_FLAG_TC) == SET)
						)
			{
			      /* Clear DMA Streams flags */
			      DMA_ClearFlag(USART3_TX_DMA_STREAM, USART3_TX_DMA_FLAG_HTIF | USART3_TX_DMA_FLAG_TCIF);

			      /* Disable the DMA Streams */
			      DMA_Cmd(USART3_TX_DMA_STREAM, DISABLE);

			      /* Disable the USART Tx DMA request */
			      USART_DMACmd(USART3, USART_DMAReq_Tx, DISABLE);


			      u8_sys_usart_3_State_Sub_TX=def_u8_sys_usart_3_State_Sub_TX_READY;


			      GPIO_WriteBit(def_GPIO_of_usart_3_FLW,def_Pin_of_usart_3_FLW,Bit_RESET); //read enabled!!!

			}

		}
		//]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]]



	}
	else if(u8_sys_usart_3_State_Main==def_u8_sys_usart_3_State_Main_STARTUP)
	{
		if(u16_sys_usart_3_time_counter_0==0) //initial timeout.
		{

			RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);



			//==============================
			/* USARTx GPIO configuration -----------------------------------------------*/
			/* Connect USART pins to AF7 */
			GPIO_PinAFConfig(def_GPIO_of_usart_3_TX, def_PinSource_of_usart_3_TX, GPIO_AF_USART3);
			GPIO_PinAFConfig(def_GPIO_of_usart_3_RX, def_PinSource_of_usart_3_RX, GPIO_AF_USART3);

			/* Configure USART Tx and Rx as alternate function push-pull */
			GPIO_InitStructure_sys_usart_3.GPIO_Mode = GPIO_Mode_AF;
			GPIO_InitStructure_sys_usart_3.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_InitStructure_sys_usart_3.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure_sys_usart_3.GPIO_PuPd = GPIO_PuPd_UP;

			GPIO_InitStructure_sys_usart_3.GPIO_Pin = def_Pin_of_usart_3_TX;
			GPIO_Init(def_GPIO_of_usart_3_TX, &GPIO_InitStructure_sys_usart_3);

			GPIO_InitStructure_sys_usart_3.GPIO_Pin = def_Pin_of_usart_3_RX;
			GPIO_Init(def_GPIO_of_usart_3_RX, &GPIO_InitStructure_sys_usart_3);
			//==================================

			/* Enable the USART OverSampling by 8 */
			USART_OverSampling8Cmd(USART3, ENABLE);

			USART_Init(USART3, &USART_InitStructure_sys_usart_3);


			//todo USART mode a göre init olacak
			//========================
			GPIO_InitStructure_sys_usart_3.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_InitStructure_sys_usart_3.GPIO_Speed = GPIO_Speed_100MHz;
			GPIO_InitStructure_sys_usart_3.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure_sys_usart_3.GPIO_PuPd = GPIO_PuPd_NOPULL;

			GPIO_InitStructure_sys_usart_3.GPIO_Pin = def_Pin_of_usart_3_FLW;
			GPIO_Init(def_GPIO_of_usart_3_FLW, &GPIO_InitStructure_sys_usart_3);

			GPIO_WriteBit(def_GPIO_of_usart_3_FLW,def_Pin_of_usart_3_FLW,Bit_RESET); //read enabled!!!

			//GPIO_PinLockConfig(def_GPIO_of_USART3_FLW,def_Pin_of_USART3_FLW);
			//========================

			NVIC_Init(&NVIC_InitStructure_sys_usart_3);

			USART_Cmd(USART3, ENABLE);
			//USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
			//USART_SetReceiverTimeOut(USART3,1000); //STM32F3 supported!!!
			//todo usart receive will be watched from rx dma byte counter with not zero and steady condition!!!

			u8_sys_usart_3_State_Main=def_u8_sys_usart_3_State_Main_READY;

			u8_sys_usart_3_State_Sub_TX=def_u8_sys_usart_3_State_Sub_TX_READY;
			u8_sys_usart_3_State_Sub_RX=def_u8_sys_usart_3_State_Sub_RX_READY;


//			//direct send receive test
//			USART_SendData(USART3, 'A');
//			while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)  {}
//			while(USART_GetFlagStatus(USART3,USART_FLAG_RXNE) == RESET) {}
//			u8_array_sys_usart_3_rx[0]=(u8)(USART_ReceiveData(USART3));
//
//			USART_SendData(USART3, 'b');
//			while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)  {}
//			while(USART_GetFlagStatus(USART3,USART_FLAG_RXNE) == RESET) {}
//			u8_array_sys_usart_3_rx[1]=(u8)(USART_ReceiveData(USART3));
//
//			USART_SendData(USART3, 'c');
//			while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)  {}
//			while(USART_GetFlagStatus(USART3,USART_FLAG_RXNE) == RESET) {}
//			u8_array_sys_usart_3_rx[2]=(u8)(USART_ReceiveData(USART3));
//
//			USART_SendData(USART3, 'd');
//			while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)  {}
//			while(USART_GetFlagStatus(USART3,USART_FLAG_RXNE) == RESET) {}
//			u8_array_sys_usart_3_rx[3]=(u8)(USART_ReceiveData(USART3));
//
//			USART_SendData(USART3, 'e');
//			while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET)  {}
//			while(USART_GetFlagStatus(USART3,USART_FLAG_RXNE) == RESET) {}
//			u8_array_sys_usart_3_rx[4]=(u8)(USART_ReceiveData(USART3));

		}

	}
	else if(u8_sys_usart_3_State_Main==def_u8_sys_usart_3_State_Main_IDLE)
	{
		#ifdef	def_enable_sys_usart_3_startup_on_reset
				f_sys_usart_3_STARTUP(
						def_u8_sys_usart_3_MODE_Full_dublex,
						9600,
						USART_WordLength_9b,
						USART_StopBits_1,
						USART_Parity_No,
						USART_HardwareFlowControl_None
						);

//				//SCANTECH BARCODE READER SETUP
//				f_sys_usart_3_STARTUP(
//						def_u8_sys_usart_3_MODE_Full_dublex,
//						9600,
//						USART_WordLength_8b,//USART_WordLength_8b,
//						USART_StopBits_1,
//						USART_Parity_Even, //USART_Parity_No,
//						USART_HardwareFlowControl_None
//						);
		#endif

	}
	else if(u8_sys_usart_3_State_Main==def_u8_sys_usart_3_State_Main_ERROR)
	{
		u8_sys_usart_3_State_Main=def_u8_sys_usart_3_State_Main_STARTUP;
	}
	else //state corruption!!!
	{
		u8_sys_usart_3_State_Main=def_u8_sys_usart_3_State_Main_STARTUP;
	}
}




//
//  void f_uart_1_rs_485_data_enable(void)
//  {
//	  GPIO_SetBits(def_GPIO_of_USART3_FLW, def_Pin_of_USART3_FLW);
//  }
//
//  void f_uart_1_rs_485_read_enable(void)
//  {
//	  GPIO_ResetBits(def_GPIO_of_USART3_FLW, def_Pin_of_USART3_FLW);
//  }
//
//
//  void usart1_transmit(u8 kelime[],u16 length)
//  {
//    u16 i;
//
//    if(u8_sys_uart_1_state==1) //pakckage sniff state
//    	{
//
//				if(length<(const_u8_usart1_rx_buffer_length+1))
//				{
//
//							  //if(bool_usart1_tx_busy==FALSE)
//							  if(u1_sys_uart_1_status_flag_uart_tx_busy==0)
//							  {
//								//flwcont1on(); //!!!!!!!!!!!!!!!!!!!!!!!!!!
//								  f_uart_1_rs_485_data_enable();
//								  //u1_flag_sys_led_blink_uart_tx_transection=1;
//								  u1_flag_sys_led_blink_rf_tx_transection=1;
//
//
//																for(i=0;i<length;i++)
//																{
//																  u8_array_usart1_tx_buffer[i]=kelime[i];
//																}
//
//																	//_______________
//																	u8_usart1_tx_buffer_r_index_counter =0;
//																	u8_usart1_tx_buffer_r_index_limit =length;
//																	//bool_usart1_tx_busy = TRUE;
//																	u1_sys_uart_1_status_flag_uart_tx_busy=1;
//																	USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
//																	//_______________
//							  }
//
//				}
//    	}
//
//  }
//
//
//  u8 f_uart_1_command_from_usb(void)
//  {
//  	u8 i,j;
//  	//usbhidrxbuf[0] was 'U'
//  	//usbhidrxbuf[1] was '3'
//  	if(usbhidrxbuf[2]=='T')//direct uart transmit
//  	{
//  		usart1_transmit(&usbhidrxbuf[4],usbhidrxbuf[3]);
//  		return 0;
//  	}
//
//  	return 0;
//  }
//
//
////  usbhidrxbuf[0]=='u') // uart package transmit
////  usbhidrxbuf[1]=='0') // uart channel
////  usbhidrxbuf[2]==x) // #of bytes
//  void f_uart_1_transmit_command_from_usb(void)
//  {
//	  if(usbhidrxbuf[2]<61)
//	  {
//		  usart1_transmit(&usbhidrxbuf[3],usbhidrxbuf[2]);
//	  }
//  }
//
//
//  void f_uart_1_start(void)
//  {
//	  u8_sys_uart_1_state=0;
//  }
//
//  void f_uart_1_kill(void)
//  {
//	  u8_sys_uart_1_state=254; //undefined state
//	  USART_DeInit(USART3);
//	  f_uart_1_rs_485_read_enable();
//  }
//
//void f_tasking_1_ms_base_sys_uart_1(void)
//{
//	if(u8_sys_uart_1_state==0) //reset state
//	{
//		u8_sys_uart_1_status_flags.byte=0;
//
//
//	      USART_DeInit(USART3);
//	      RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART3, ENABLE);
//
//	      //GPIO_PinRemapConfig(GPIO_FullRemap_USART3,ENABLE); ???
//	      //GPIO_PinRemapConfig(GPIO_PartialRemap_USART3,ENABLE);
//#ifdef	def_code_sw_ENABLE_USART3_GPIO_REMAP
//	      GPIO_PinRemapConfig(GPIO_Remap_USART3,ENABLE);
//#endif
//
//	      GPIO_InitStructure_1.GPIO_Speed = GPIO_Speed_50MHz;
//	      GPIO_InitStructure_1.GPIO_Mode = GPIO_Mode_AF_PP;
//	      GPIO_InitStructure_1.GPIO_Pin = def_Pin_of_USART3_TX;
//	      GPIO_Init(def_GPIO_of_USART3_TX, &GPIO_InitStructure_1);
//
//
//	      GPIO_InitStructure_1.GPIO_Mode = GPIO_Mode_IPU;
//	      GPIO_InitStructure_1.GPIO_Pin = def_Pin_of_USART3_RX;
//	      GPIO_Init(def_GPIO_of_USART3_RX, &GPIO_InitStructure_1);
//
//#ifdef def_enable_sys_uart_1_as_rs485
//	      //USART3_FLW  	pA11
//	      GPIO_InitStructure_1.GPIO_Speed = GPIO_Speed_50MHz;
//	      GPIO_InitStructure_1.GPIO_Mode = GPIO_Mode_Out_PP;
//	      GPIO_InitStructure_1.GPIO_Pin = def_Pin_of_USART3_FLW;
//	      GPIO_Init(def_GPIO_of_USART3_FLW, &GPIO_InitStructure_1);
//
//	      f_uart_1_rs_485_read_enable();
//#endif
//
//
//
//		//uart setting
//		//default 115200 one stop none parity
//
//
//	      /* USART3 configuration ------------------------------------------------------*/
//	    /* USART3 configured as follow:
//	          - BaudRate = 9600 baud
//	          - Word Length = 8 Bits
//	          - Two Stop Bit
//	          - Odd parity
//	          - Hardware flow control disabled (RTS and CTS signals)
//	          - Receive and transmit enabled
//	    */
//	    //USART_InitStructure_1.USART_BaudRate = 115200;
//	    //USART_InitStructure_1.USART_BaudRate = 9600; //PRELOADED!!!
//	    USART_InitStructure_1.USART_WordLength = USART_WordLength_8b;
//	    USART_InitStructure_1.USART_StopBits = USART_StopBits_1;
//	    //USART_InitStructure_1.USART_Parity = USART_Parity_No;//PRELOADED!!!
//	    USART_InitStructure_1.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	    USART_InitStructure_1.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//
//	    /* Configure the USART3 */
//	    USART_Init(USART3, &USART_InitStructure_1);
//
//	    /* Enable the USART Transmoit interrupt: this interrupt is generated when the
//	       USART3 transmit data register is empty */
//	   // USART_ITConfig(USART3, USART_IT_TXE, ENABLE);
//
//	    /* Enable the USART Receive interrupt: this interrupt is generated when the
//	       USART3 receive data register is not empty */
//	    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
//
//	    /* Enable USART3 */
//	    USART_Cmd(USART3, ENABLE);
//
//
//
//	      u8_sys_uart_1_state++;
//	}
//	else if(u8_sys_uart_1_state==1) //pakckage sniff state
//	{
//
//        if(u8_usart1_rx_timeout_down_counter>0)
//        {
//                u8_usart1_rx_timeout_down_counter--;
//
//                if(u8_usart1_rx_timeout_down_counter==0)
//                {
//                        //bool_usart1_package_received=TRUE;
//                        //bool_usart1_protect_receive_buffer=TRUE;
//
//                        u1_sys_uart_1_status_flag_uart_package_received=1;
//                        u1_sys_uart_1_status_flag_uart_protect_receive_buffer=1;
//
//
//                        //u1_flag_sys_led_blink_uart_rx_transection=1;
//                        u1_flag_sys_led_blink_rf_rx_transection=1;
//
//
//                }
//        }
//
//	}
//
//	else if(u8_sys_uart_1_state==255) //wait state
//	{
//		if(u16_sys_uart_1_wait_down_counter!=0)
//		{
//			u16_sys_uart_1_wait_down_counter--;
//		}
//		else
//		{
//
//			u8_sys_uart_1_state=0; //after initial wait reset uart system.
//		}
//
//	}
//
//
//}
//
//void USART3_IRQHandler(void)
//{
//	  u8 dummy=0,dummy2=0;
//
//	  if( (USART_GetITStatus(USART3,  USART_IT_RXNE) != RESET)&(USART_GetFlagStatus(USART3, USART_FLAG_RXNE)== SET) )
//	  {
//
//	      //GPIO_WriteBit(led1, (BitAction)(1 - GPIO_ReadOutputDataBit(led1)));
//	      //   GPIO_SetBits(led1);
//
//
//
//		  u1_sys_uart_1_status_flag_uart_byte_received=1;
//
//	      if(u1_sys_uart_1_status_flag_uart_protect_receive_buffer==0)
//	      {
//	    	  //old!!!
//	    	  //xxx bug: 2012 11 19 uart rx int fw stuck here!!! pc sends broadcast rs485 and fw locks!!
////	              if(u8_usart3_rx_buffer_w_index < const_u8_usart3_rx_buffer_length)
////	              {
////
////	                      u8_array_usart3_rx_buffer[u8_usart3_rx_buffer_w_index]= (u8)( USART_ReceiveData(USART3) );
////	                      u8_usart3_rx_buffer_w_index++;
////	              }
////	              u8_usart3_rx_timeout_down_counter=10; //refresh down counter to receive next byte!!!
//
//
//	    	  	  //new!!!
//	    	  	  //xxx bug: 2012 11 19 uart rx int fw stuck here!!! pc sends broadcast rs485 and fw locks!! SOLVED!!!
//	              if(u8_usart1_rx_buffer_w_index < const_u8_usart1_rx_buffer_length)
//	              {
//
//	                      u8_array_usart1_rx_buffer[u8_usart1_rx_buffer_w_index]= (u8)( USART_ReceiveData(USART3) );
//	                      u8_usart1_rx_buffer_w_index++;
//
//	                      u8_usart1_rx_timeout_down_counter=10; //refresh down counter to receive next byte!!!
//	              }
//	              else
//	              {
//		              dummy=(u8)( USART_ReceiveData(USART3) );	//to trash
//		              dummy2=dummy;
//	              }
//
//
//
//
//	      }
//	      else
//	      {
//	              dummy=(u8)( USART_ReceiveData(USART3) );	//to trash
//	              dummy2=dummy;
//	      }
//
//
//
//	  }
//
//	  else if( (USART_GetITStatus(USART3,  USART_IT_TXE) != RESET)&(USART_GetFlagStatus(USART3, USART_FLAG_TXE)== SET) )
//	  {
//	    //GPIO_WriteBit(led2, (BitAction)(1 - GPIO_ReadOutputDataBit(led2)));
//	    //   GPIO_SetBits(led2);
//
//	    USART_SendData(USART3, u8_array_usart1_tx_buffer[u8_usart1_tx_buffer_r_index_counter]);
//	    u8_usart1_tx_buffer_r_index_counter++;
//
//	    if(u8_usart1_tx_buffer_r_index_counter == u8_usart1_tx_buffer_r_index_limit)
//	    {
//	      /* Disable the USART3 Transmit interrupt */
//	      USART_ITConfig(USART3, USART_IT_TXE, DISABLE);
//	      u1_sys_uart_1_status_flag_uart_tx_busy = 0;
//	      USART_ITConfig(USART3, USART_IT_TC, ENABLE);
//
//	      //flwcont1off(); //!!!!!!!!!!!!!!!!!!!!!!!!!!
//	    }
//	  }
//
//
//
//	  else if( (USART_GetITStatus(USART3, USART_IT_TC) != RESET)&(USART_GetFlagStatus(USART3, USART_FLAG_TC)== SET) )
//	      {
//	        //flwcont1off(); //!!!!!!!!!!!!!!!!!!!!!!!!!!
//		  f_uart_1_rs_485_read_enable();
//	        USART_ITConfig(USART3, USART_IT_TC, DISABLE);
//	      }
//}

#endif
