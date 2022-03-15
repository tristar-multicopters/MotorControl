/**
  ******************************************************************************
  * @file    flashconfig.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles storage to flash of device configuration
  *
	******************************************************************************
	*/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASHCONFIG_H
#define __FLASHCONFIG_H

/* Includes ------------------------------------------------------------------*/

#include "fds.h"
#include "throttle.h"
#include <string.h>

#define CONFIG_FILE     			0x8010
#define CONFIG_THROTTLE_KEY   0x7010

/* A dummy structure to save in flash. */
typedef struct
{
    uint32_t boot_count;
    char     device_name[16];
    bool     config1_on;
    bool     config2_on;
} configuration_t;

void Flash_Init(void);

void Flash_ReadConfig_Throttle(Throttle_Param_t * p_Throttle_Param);
void Flash_WriteConfig_Throttle(Throttle_Param_t * p_Throttle_Param);


#endif /* __FLASHCONFIG_H */
