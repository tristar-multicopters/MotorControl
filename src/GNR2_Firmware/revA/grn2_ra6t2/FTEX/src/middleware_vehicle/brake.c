/**
  * @file    brake.c
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles brake sensor management
  *
*/

#include "brake.h"

/* Functions ---------------------------------------------------- */

/**
 * @brief Initializes brake sensor module
 */
void BRK_Init(BRK_Handle_t * pHandle)
{	

}

bool BRK_IsPressed( BRK_Handle_t * pHandle )
{
	bool bAux = false; //TODO: check gpio input, for now just mock
	pHandle->bIsPressed = bAux ^ pHandle-> bIsInvertedLogic;
	
	return pHandle->bIsPressed;
}

