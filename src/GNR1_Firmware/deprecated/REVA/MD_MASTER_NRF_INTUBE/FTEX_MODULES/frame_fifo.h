/**
  ******************************************************************************
  * @file    frame_fifo.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module implements a FIFO (First-In-First-Out) to handle data of type FCP_Frame_t.
  *
	******************************************************************************
	*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FRAME_FIFO_H
#define __FRAME_FIFO_H

#include <stdint.h>
#include "frame_communication_protocol.h"


#define FIFO_SUCCESS								0
#define FIFO_ERROR_NULL							1
#define FIFO_ERROR_NO_MEM						2	
#define FIFO_ERROR_NOT_FOUND				3
#define FIFO_ERROR_INVALID_LENGTH 	4

#define IS_POWER_OF_TWO(A) ( ((A) != 0) && ((((A) - 1) & (A)) == 0) )


typedef struct
{
    FCP_Frame_t *      p_buf;       		/**< Pointer to FIFO buffer memory.                      */
    uint16_t           buf_size_mask;   /**< Read/write index mask. Also used for size checking. */
    volatile uint32_t  read_pos;        /**< Next read position in the FIFO buffer.              */
    volatile uint32_t  write_pos;       /**< Next write position in the FIFO buffer.             */
} frame_fifo_t;


uint32_t frame_fifo_init(frame_fifo_t * p_fifo, FCP_Frame_t * p_buf, uint16_t buf_size);

uint32_t frame_fifo_put(frame_fifo_t * p_fifo, FCP_Frame_t frame);

uint32_t frame_fifo_get(frame_fifo_t * p_fifo, FCP_Frame_t * p_frame);

uint32_t frame_fifo_peek(frame_fifo_t * p_fifo, uint16_t index, FCP_Frame_t * p_frame);

uint32_t frame_fifo_flush(frame_fifo_t * p_fifo);

#endif /*__FRAME_FIFO_H*/
