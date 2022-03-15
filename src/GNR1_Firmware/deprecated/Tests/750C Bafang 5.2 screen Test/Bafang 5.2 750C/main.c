/**
 * Copyright (c) 2014 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 * @defgroup uart_example_main main.c
 * @{
 * @ingroup uart_example
 * @brief UART Example Application main file.
 *
 * This file contains the source code for a sample application using UART.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include "app_uart.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf.h"
#include "bsp.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif


//#define ENABLE_LOOPBACK_TEST  /**< if defined, then this example will be a loopback test, which means that TX should be connected to RX to get data loopback. */

#define MAX_TEST_DATA_BYTES     (15U)                /**< max number of test bytes to be used for tx and rx. */
#define UART_TX_BUF_SIZE 256                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 256                         /**< UART RX buffer size. */

//Display Read Cmd
#define SKIP          0x00
#define R_VERSION     0x90
#define R_STATUS      0x08
#define R_WORKSTATUS  0x31 
#define R_CURRENT     0x0A
#define R_BATCAP      0x11
#define R_RSPEED      0x20
#define R_LIGHTSTATUS 0x1B
#define R_PHOTTHRESH  0x1C




void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}


#ifdef ENABLE_LOOPBACK_TEST
/* Use flow control in loopback test. */
#define UART_HWFC APP_UART_FLOW_CONTROL_ENABLED

/** @brief Function for setting the @ref ERROR_PIN high, and then enter an infinite loop.
 */
static void show_error(void)
{

    bsp_board_leds_on();
    while (true)
    {
        // Do nothing.
    }
}


/** @brief Function for testing UART loop back.
 *  @details Transmitts one character at a time to check if the data received from the loopback is same as the transmitted data.
 *  @note  @ref TX_PIN_NUMBER must be connected to @ref RX_PIN_NUMBER)
 */
static void uart_loopback_test()
{
    uint8_t * tx_data = (uint8_t *)("\r\nLOOPBACK_TEST\r\n");
    uint8_t   rx_data;

    // Start sending one byte and see if you get the same
    for (uint32_t i = 0; i < MAX_TEST_DATA_BYTES; i++)
    {
        uint32_t err_code;
        while (app_uart_put(tx_data[i]) != NRF_SUCCESS);

        nrf_delay_ms(10);
        err_code = app_uart_get(&rx_data);

        if ((rx_data != tx_data[i]) || (err_code != NRF_SUCCESS))
        {
            show_error();
        }
    }
    return;
}
#else
/* When UART is used for communication with the host do not use flow control.*/
#define UART_HWFC APP_UART_FLOW_CONTROL_DISABLED
#endif



/**
 * @brief Function for main application entry.
 */
int main(void)
{
	  bool TrashSkipped = false;
	  uint8_t CmdByteNb = 0;   //Tells how many bytes of this cmd have been received so far
	  uint8_t CmdByteExp;      //Tells how many bytes we expect to receive for this cmd
	  uint8_t CmdBuffer[5];
	  uint8_t SendBuffer[3];
	  uint8_t NbByte2Send = 0;
	  uint8_t StartByte;
	  uint8_t CmdByte;
	  uint8_t State       = 0;
    uint32_t err_code;
    uint16_t Speed   = 175;
    uint8_t  Current = 60;
	  bool Up = true;
	
    bsp_board_init(BSP_INIT_LEDS);

    const app_uart_comm_params_t comm_params =
      {
          RX_PIN_NUMBER,
          TX_PIN_NUMBER,
          RTS_PIN_NUMBER,
          CTS_PIN_NUMBER,
          UART_HWFC,
          false,
#if defined (UART_PRESENT)
					NRF_UART_BAUDRATE_1200
         // NRF_UART_BAUDRATE_115200
					
#else
          NRF_UARTE_BAUDRATE_115200
#endif
      };

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);

#ifndef ENABLE_LOOPBACK_TEST
   // printf("\r\nUART example started.\r\n");

    while (true)
    {
        uint8_t cr;

			    while (app_uart_get(&cr) != NRF_SUCCESS);
			     
			    
			    CmdByteNb ++; 
         
			
			 switch(State)
			 {	
         case 0: //Wait for next cmd 				 
	        NbByte2Send = 0;
				  StartByte = cr;
				 
				  if(StartByte == 0x11 && CmdByteNb == 1) //Read cmd
					 {
					   State = 1;
					 }
					 else if (StartByte == 0x16 && CmdByteNb == 1) //Write cmd		
					 {
					   State = 2;
					 }
					 else
					 {
					   CmdByteNb = 0;
					 }
					break;
				 case 1://Read cmd
					 
				  if(CmdByteNb == 2)
				  {
						 CmdByte = cr;
					   				 
				    if(TrashSkipped == false && CmdByte != R_RSPEED)
				    {
				      CmdByte = SKIP;
				    }
			    
    					switch(CmdByte)
							 {
								 case R_VERSION:
									 //answer with 0x90 0x40 0x0D
								   NbByte2Send = 3;
								   SendBuffer[0] = 0x90;
								   SendBuffer[1] = 0x40;
								   SendBuffer[2] = 0x0D;
							    break;
								 case R_STATUS:
									 //answer with 0x01 twice
								    NbByte2Send = 2;
								    SendBuffer[0] = 0x01;
								    SendBuffer[1] = 0x01;							   
									 break;							 
								 case R_WORKSTATUS:
									 //answer with 0x31 0x31
								   NbByte2Send = 2;
								   SendBuffer[0] = 0x31;
								   SendBuffer[1] = 0x31;
									break;
								 case R_CURRENT:
									 //answer with 0-250 value twice
								   NbByte2Send = 2;
								 	 SendBuffer[0] = Current; 
								   SendBuffer[1] = Current;
							    break;
								 case R_BATCAP:
									 //answer with 0-99 value twice
								   NbByte2Send = 2;
								 	 SendBuffer[0] = 0x4B; //75%
								   SendBuffer[1] = 0x4B;
							    break;
								 case R_RSPEED:
									 //answer with rpm + checksum (speed + 0x20)
								   TrashSkipped = true;
								   NbByte2Send = 4;
								   SendBuffer[0] = (Speed & 0xFF00) >> 8;
								   SendBuffer[1] = (Speed & 0x00FF);
								 	 SendBuffer[2] = (uint8_t)((Speed + 0x20) & 0x00FF);
								 
								 
								   //Testing speed variation
								   if(Speed == 270 && Up)
									 {
									   Up = false;
									 }
									 else if (Speed == 0 && Up == false)
									 {
									   Up = true;
									 }
								   else if(Speed <= 265 && Up)
									 {
										 Speed += 5;
									 }
									 else if( Speed >= 5 && Up == false)
									 {
									   Speed -= 5;
									 }
									 else
                   {
									   Speed = 0;
										 Up = true;											 
									 }
							    break;
								 case R_LIGHTSTATUS:
									 //answer with 0x00 0x00 (light off)
                   NbByte2Send = 2;
								 	 SendBuffer[0] = 0x00;
								   SendBuffer[1] = 0x00;
								 
							    break;
								 case R_PHOTTHRESH:
									 //answer with threshhold 0xFF 0xFF
								   NbByte2Send = 2;
								 	 SendBuffer[0] = 0xFF;
								   SendBuffer[1] = 0xFF;
							    break;
								 case SKIP:
								 default:
									 NbByte2Send = 0;
								   CmdByteNb   = 0;
								   State       = 0;
							    break;								 
							 }								 
				 
					}
          break;//End read cmd
				 case 2://Write cmd
					
				  if(CmdByteNb == 2)
				   {
						  CmdByte = cr;
					   
				 			 
				     if(TrashSkipped == false)
				     {
				       CmdByte = SKIP;
				     }
			    
    					switch(CmdByte)
							 {
								 //For now dont do anything with info received 
								 case SKIP:
								 default:
									 NbByte2Send = 0;
								   CmdByteNb   = 0;
								   State       = 0;
							    break;								 
							 }
					 } 
          break;					 
			 }
			 

			 if(NbByte2Send > 0)
			 {
				 app_uart_flush();
			   for(int i = 0; i < NbByte2Send; i++)
				 {
          while (app_uart_put(SendBuffer[i]) != NRF_SUCCESS);
				 }						 	
				    NbByte2Send = 0;
						CmdByteNb   = 0;
						State       = 0;				 
			 }
    }
#else

    // This part of the example is just for testing the loopback .
    while (true)
    {
        uart_loopback_test();
    }
#endif
}


/** @} */
