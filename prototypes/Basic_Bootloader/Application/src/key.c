/**
  * @file    hal_data.c
  * @brief   This file is the key file 
	*/
    
#include "hal_data.h"


/* we are not using encryption keys in this version*/

int default_CSPRNG(uint8_t *dest, unsigned int size)
{
	FSP_PARAMETER_NOT_USED(dest);
	FSP_PARAMETER_NOT_USED(size);
	return	0;
}