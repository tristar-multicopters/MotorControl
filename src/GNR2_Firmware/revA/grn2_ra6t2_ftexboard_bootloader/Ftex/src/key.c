/**
  ******************************************************************************
  * @file    key.c
  * @brief   This file present the Encryption the Key File
  ******************************************************************************
*/
 
// =============================== Includes ================================ // 
#include "hal_data.h"

// ==================== Public function prototypes ======================== //

/**
  * @brief  Defualt Encryption Key Call function
  * @param  size, dest encryption key vriables
  * @retval None 
*/
int default_CSPRNG(uint8_t *dest, unsigned int size)
{
    /* we are not using encryption keys in this version*/
	FSP_PARAMETER_NOT_USED(dest);
	FSP_PARAMETER_NOT_USED(size);
	return	0;
}