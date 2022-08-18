/**
  ******************************************************************************
  * @file    board_hardware.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module defines the hardware (pins, peripherals, ...) used by the board. 
  *
	******************************************************************************
	*/

#include "board_hardware.h"


/* 
	 Function used to get the chip ID
	 ID_half decides which half of the ID is return
	 0 for lower half and 1 for upper half
*/
uint32_t GetChipID(uint8_t ID_Half)
{
	// TODO: Implement renesas specific
	return 0;

}



