/* Includes ------------------------------------------------------------------*/
#include "flashconfig.h"


/* Functions ---------------------------------------------------- */

void Flash_WriteConfig(Config_Reg_Value_t value, Config_Reg_Name_t reg)
{
	HAL_FLASH_Unlock();
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, BASE_ADDR+reg*sizeof(Config_Reg_Value_t), value.whole);
	HAL_FLASH_Lock();
}

Config_Reg_Value_t Flash_ReadConfig(Config_Reg_Name_t reg)
{
	const volatile Config_Reg_Value_t *p_config = (const volatile Config_Reg_Value_t *)BASE_ADDR;
	
	return p_config[reg];
}

void Flash_EraseAllConfig(void)
{
	FLASH_EraseInitTypeDef erase_config = 
	{
		.TypeErase = FLASH_TYPEERASE_PAGES,
		.Banks = FLASH_BANK_1,
		.Page = FLASH_PAGE,
		.NbPages = 1,
	};
	uint32_t pageError;
	
	HAL_FLASH_Unlock();
	HAL_FLASHEx_Erase(&erase_config, &pageError);
	HAL_FLASH_Lock();
}

