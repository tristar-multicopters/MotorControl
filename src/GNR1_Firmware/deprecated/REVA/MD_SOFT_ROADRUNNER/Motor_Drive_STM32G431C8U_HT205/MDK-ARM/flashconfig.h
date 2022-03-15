/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __FLASHCONFIG_H
#define __FLASHCONFIG_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"

#define FLASH_PAGE 63
#define BASE_ADDR 0x0801F800

#define CONFIG_REG_NUM 14

typedef enum
{
  CONFIG_REG_TORQUE_KP,
	CONFIG_REG_TORQUE_KI,
	CONFIG_REG_FLUX_KP,
	CONFIG_REG_FLUX_KI,
	CONFIG_REG_TEMPERATURE_MAX, /* Td =  */
	CONFIG_REG_DCBUS_MAX,
	CONFIG_REG_DCBUS_MIN,
	CONFIG_REG_IPH_MAX,
	CONFIG_REG_HALL_PHASESHIFT,
	CONFIG_REG_POLEPAIRS,
	CONFIG_REG_KFF,
} Config_Reg_Name_t;

typedef union
{
	uint64_t whole;
	uint32_t half[2];
	uint16_t quarter[4];
} Config_Reg_Value_t;

typedef struct 
{
	Config_Reg_Value_t value[CONFIG_REG_NUM]; 
	bool received[CONFIG_REG_NUM];
} Config_LocalRegList_t;


void Flash_WriteConfig(Config_Reg_Value_t value, Config_Reg_Name_t reg);

Config_Reg_Value_t Flash_ReadConfig(Config_Reg_Name_t reg);

void Flash_EraseAllConfig(void);


#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif /* __FLASHCONFIG_H */
