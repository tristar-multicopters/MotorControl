/**
  ******************************************************************************
  * @file    frame_fifo.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module implements a FIFO (First-In-First-Out) to handle data of type FCP_Frame_t.
  *
	******************************************************************************
	*/


#include "frame_fifo.h"

static __inline uint32_t fifo_length(frame_fifo_t * p_fifo)
{
    uint32_t tmp = p_fifo->read_pos;
    return p_fifo->write_pos - tmp;
}

#define FIFO_LENGTH() fifo_length(p_fifo)  /**< Macro for calculating the FIFO length. */


/**@brief Put one byte to the FIFO. */
static __inline void fifo_put(frame_fifo_t * p_fifo, FCP_Frame_t frame)
{
    p_fifo->p_buf[p_fifo->write_pos & p_fifo->buf_size_mask] = frame;
    p_fifo->write_pos++;
}


/**@brief Look at one byte in the FIFO. */
static __inline void fifo_peek(frame_fifo_t * p_fifo, uint16_t index, FCP_Frame_t * p_frame)
{
    *p_frame = p_fifo->p_buf[(p_fifo->read_pos + index) & p_fifo->buf_size_mask];
}


/**@brief Get one byte from the FIFO. */
static __inline void fifo_get(frame_fifo_t * p_fifo, FCP_Frame_t * p_frame)
{
    fifo_peek(p_fifo, 0, p_frame);
    p_fifo->read_pos++;
}


uint32_t frame_fifo_init(frame_fifo_t * p_fifo, FCP_Frame_t * p_buf, uint16_t buf_size)
{
    // Check buffer for null pointer.
    if (p_buf == NULL)
    {
        return FIFO_ERROR_NULL;
    }

    // Check that the buffer size is a power of two.
    if (!IS_POWER_OF_TWO(buf_size))
    {
        return FIFO_ERROR_INVALID_LENGTH;
    }

    p_fifo->p_buf         = p_buf;
    p_fifo->buf_size_mask = buf_size - 1;
    p_fifo->read_pos      = 0;
    p_fifo->write_pos     = 0;

    return FIFO_SUCCESS;
}


uint32_t frame_fifo_put(frame_fifo_t * p_fifo, FCP_Frame_t frame)
{
    if (FIFO_LENGTH() <= p_fifo->buf_size_mask)
    {
        fifo_put(p_fifo, frame);
        return FIFO_SUCCESS;
    }

    return FIFO_ERROR_NO_MEM;
}


uint32_t frame_fifo_get(frame_fifo_t * p_fifo, FCP_Frame_t * p_frame)
{
    if (FIFO_LENGTH() != 0)
    {
        fifo_get(p_fifo, p_frame);
        return FIFO_SUCCESS;
    }

    return FIFO_ERROR_NOT_FOUND;

}


uint32_t frame_fifo_peek(frame_fifo_t * p_fifo, uint16_t index, FCP_Frame_t * p_frame)
{
    if (FIFO_LENGTH() > index)
    {
        fifo_peek(p_fifo, index, p_frame);
        return FIFO_SUCCESS;
    }

    return FIFO_ERROR_NOT_FOUND;
}


uint32_t frame_fifo_flush(frame_fifo_t * p_fifo)
{
    p_fifo->read_pos = p_fifo->write_pos;
    return FIFO_SUCCESS;
}
