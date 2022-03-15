/**
  ******************************************************************************
  * @file    storage_management.c
	* @author  Sami Bouzid, FTEX
	* @author  Jorge Andres Polo, FTEX
  * @brief   This module manages data storage in flash or external EEPROM, and software updates.
  *
	******************************************************************************
*/

#include "storage_management.h"

/*Global variable for save all vehicle parameters*/
VParams_for_STRG_t STRG_VParams = {0};
osThreadId_t TSK_STRG_handle;
static bool m_isOverwritting = false;
static int32_t m_motor_params [MD_PARAMS_SIZE] = {0};
/***********************************************/

/* Fstorage instance definition */
NRF_FSTORAGE_DEF(nrf_fstorage_t m_fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = START_ADDR,
    .end_addr   = LAST_ADDR,
};

static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        return;
    }
//    switch (p_evt->id)
//    {
//        case NRF_FSTORAGE_EVT_WRITE_RESULT:
//        {
//        } break;

//        case NRF_FSTORAGE_EVT_ERASE_RESULT:
//        {
//        } break;

//        default:
//            break;
//    }
}

/**@brief   Sleep until an event is received. */
static void power_manage(void)
{
	__WFE();
}

/**@brief While fstorage is busy, sleep and wait for an event. **/
static void wait_for_flash_ready(nrf_fstorage_t const * p_fstorage)
{
    /* While fstorage is busy, sleep and wait for an event. */
    while (nrf_fstorage_is_busy(p_fstorage))
    {
        power_manage();
    }
}

/**@brief Function for init the storage manager module
*/
void STRG_Init( VC_Handle_t * pHandle )
{
	ret_code_t code;
	nrf_fstorage_api_t * pFS_api;
	
	/* Initialize an fstorage instance using the nrf_fstorage_nvmc backend.
   * nrf_fstorage_nvmc uses the NVMC peripheral.*/
	pFS_api = &nrf_fstorage_nvmc;
	
	code = nrf_fstorage_init(&m_fstorage, pFS_api, NULL);
	APP_ERROR_CHECK(code);
	
	/*Load vehicle parameters*/
	STRG_UpdateVehicleFromMemory(pHandle);
}

/**@brief Function for save all vehicle parameters in the memory flash
	 @p vParams: vehicle params to save.
*/

void STRG_SaveAllVehicleParams( VParams_for_STRG_t vParams )
{
		ret_code_t ret_code;
		
		// Write parameter in the flash memory
		ret_code = nrf_fstorage_write(&m_fstorage, 
																	START_ADDR,
																	&vParams,
																	sizeof(vParams),
																	NULL);
		APP_ERROR_CHECK(ret_code);
		wait_for_flash_ready(&m_fstorage);
																	
	  if(ret_code == NRF_SUCCESS)
			m_isOverwritting = false;
		// TO DO: Rise an error in case of we can't write in the flash
		
		ret_code = nrf_fstorage_write(&m_fstorage, 
																	MOTOR_PAR_ADDR,
																	&m_motor_params,
																	sizeof(m_motor_params),
																	NULL);
		APP_ERROR_CHECK(ret_code);
		wait_for_flash_ready(&m_fstorage);
																	
	  if(ret_code == NRF_SUCCESS)
			m_isOverwritting = false;
}

/**@brief Function for read all vehicle parameters in the memory flash
*/

void STRG_ReadAllVehicleParams( VParams_for_STRG_t * vParams )
{
	ret_code_t ret_code;
	// Read parameters
	ret_code = nrf_fstorage_read(&m_fstorage, 
																START_ADDR,
																vParams,
																sizeof(VParams_for_STRG_t));
	APP_ERROR_CHECK(ret_code);
	
	ret_code = nrf_fstorage_read(&m_fstorage, 
																MOTOR_PAR_ADDR,
																m_motor_params,
																sizeof(m_motor_params));
	APP_ERROR_CHECK(ret_code);
}

/**@brief Function for update vehicle parameters from memory flash
	 @p pHandle: Handle of vehicle
*/
void STRG_UpdateVehicleFromMemory(VC_Handle_t * pHandle )
{
		STRG_ReadAllVehicleParams(&STRG_VParams);
		
		/* Load throttle parameters */
		memcpy(pHandle->pThrottle, &STRG_VParams.th_params, sizeof(Throttle_Handle_t));
		/* Load other vehicle parameters */
		memcpy(pHandle->pVehiculeParam, &STRG_VParams.vParams, sizeof(VehiculeParameters_t));
		/* Load motor parameters */
		memcpy(pHandle->pMotor_params, &m_motor_params, sizeof(m_motor_params));
//		pHandle->pThrottle->hParam.F 				 = STRG_VParams.th_params.F;
//		pHandle->pThrottle->hParam.m 				 = STRG_VParams.th_params.m;
//		pHandle->pThrottle->hParam.hOffset   = STRG_VParams.th_params.hOffset;
//		pHandle->pThrottle->hParam.resp_type = STRG_VParams.th_params.resp_type;

//	VC_set_wheelDiameter(pHandle, STRG_ReadVehicleParam(VC_PARAM_ID_WHEELD));
//	
//	VC_set_maxSpeed(pHandle, STRG_ReadVehicleParam(VC_PARAM_ID_MAXSPEED));
//	
//	VC_set_gearRatio(pHandle, STRG_ReadVehicleParam(VC_PARAM_ID_GEARRATIO));
//	
//	VC_set_throttle_type(pHandle, STRG_ReadVehicleParam(VC_PARAM_ID_THROTTLE_TYPE));
//	
//	VC_set_throttle_m(pHandle, STRG_ReadVehicleParam(VC_PARAM_ID_THROTTLE_M));
//	
//	VC_set_throttle_f(pHandle, STRG_ReadVehicleParam(VC_PARAM_ID_THROTTLE_F));
//	
//	VC_set_throttle_offset(pHandle, STRG_ReadVehicleParam(VC_PARAM_ID_THROTTLE_OFFSET));
//	
//	VC_set_main_motor(pHandle, STRG_ReadVehicleParam(VC_PARAM_ID_MAINMOTOR));
//	
//	VC_set_dual_motor(pHandle, STRG_ReadVehicleParam(VC_PARAM_ID_DUALMOTOR));
}

/**@brief Function for save all vehicle parameters in memory flash
	 @p pHandle: Handle of vehicle
*/
void STRG_UpdateMemoryFromVehicle( VParams_for_STRG_t vParams )
{
	/*Erase page for memory flash. Necessary for writting in memory */
	STRG_ResetAllVehicleParams();
	
	STRG_SaveAllVehicleParams(vParams);
	
//	STRG_SaveVehicleParam(VC_PARAM_ID_WHEELD, VC_get_wheelDiameter(pHandle));
//	
//	STRG_SaveVehicleParam(VC_PARAM_ID_MAXSPEED, VC_get_maxSpeed(pHandle));
//	
//	STRG_SaveVehicleParam(VC_PARAM_ID_GEARRATIO, VC_get_gearRatio(pHandle));
//	
//	STRG_SaveVehicleParam(VC_PARAM_ID_THROTTLE_TYPE, VC_get_throttle_type(pHandle));
//	
//	STRG_SaveVehicleParam(VC_PARAM_ID_THROTTLE_M, VC_get_throttle_m(pHandle));
//	
//	STRG_SaveVehicleParam(VC_PARAM_ID_THROTTLE_F, VC_get_throttle_f(pHandle));
//	
//	STRG_SaveVehicleParam(VC_PARAM_ID_THROTTLE_OFFSET, VC_get_throttle_offset(pHandle));
//	
//	STRG_SaveVehicleParam(VC_PARAM_ID_MAINMOTOR, VC_get_main_motor(pHandle));
//	
//	STRG_SaveVehicleParam(VC_PARAM_ID_DUALMOTOR, VC_get_dual_motor(pHandle));
}

/**@brief Erase all the records on memory flash
*/
void STRG_ResetAllVehicleParams(void)
{
	ret_code_t code;
	// Erase page starting at CONFIG_PARAMS_ADDR address
	code = nrf_fstorage_erase(&m_fstorage, START_ADDR, 1, NULL);
	APP_ERROR_CHECK(code);
}

/**@brief Function for save a vehicle parameter in the memory flash
   @p param : table index to get address from m_VC_addr_tab table
	 @p value : Data to save in the flash memory
*/
void STRG_SaveVehicleParam(VC_Params_ID_t param, int32_t value)
{
		ret_code_t ret_code;
		
		// Write parameter in the flash memory
		ret_code = nrf_fstorage_write(&m_fstorage, 
																	param,
																	&value,
																	sizeof(value),
																	NULL);
		APP_ERROR_CHECK(ret_code);
		wait_for_flash_ready(&m_fstorage);
}

/**@brief Function for read a vehicle parameter in the memory flash
   @p param  : Memory address of the parameter 
	 @out data : Data to recover from the flash
*/
int32_t STRG_ReadVehicleParam(VC_Params_ID_t param)
{
	ret_code_t ret_code;
	int32_t data;
	// Read parameter
	ret_code = nrf_fstorage_read(&m_fstorage, 
																param,
																&data,
																sizeof(data));
	APP_ERROR_CHECK(ret_code);
	return data;
}

/**@brief Function for save a motor parameter in the memory flash
	 @p motorSelection: motor ID (M1 or M2)
   @p param : Memory address of the parameter 
	 @p value : Data to save in the flash memory
*/
void STRG_SaveMotorParam(uint8_t motorSelection, MD_Params_ID_t param, int32_t value)
{
	ret_code_t ret_code;
	if(motorSelection == M1)
	{
		ret_code = nrf_fstorage_write(&m_fstorage,
																	 param,
																	 &value,
																	 sizeof(value),
																	 NULL);
		APP_ERROR_CHECK(ret_code);
		wait_for_flash_ready(&m_fstorage);															 
	}
	else
	{
		ret_code = nrf_fstorage_write(&m_fstorage,
																	 param + ADDR_ESPACE,
																	 &value,
																	 sizeof(value),
																	 NULL);
		APP_ERROR_CHECK(ret_code);
		wait_for_flash_ready(&m_fstorage);	
	}
}

/**@brief Function for read a motor parameter in the memory flash
	 @p motorSelection: motor ID (M1 or M2)
   @p param  : Memory address of the parameter 
	 @out data : Data to recover from tlash
*/
int32_t STRG_ReadMotorParam(uint8_t motorSelection, MD_Params_ID_t param)
{
	ret_code_t ret_code;
	int32_t data;
	if(motorSelection == M1)
	{
		ret_code = nrf_fstorage_read(&m_fstorage,
																	param,
																	&data,
																	sizeof(data));
		APP_ERROR_CHECK(ret_code);														 
	}
	else
	{
		ret_code = nrf_fstorage_read(&m_fstorage,
																	param + ADDR_ESPACE,
																	&data,
																	sizeof(data));
		APP_ERROR_CHECK(ret_code);
	}
	
	return data;
}

/**@brief Function for managing writes in the flash memory
	 @p pvParameter: Unused parameter
*/
__NO_RETURN void TSK_StorageManagement(void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	
	while(true)
	{
		osThreadFlagsWait(STRG_FLAG, osFlagsWaitAny, osWaitForever);
		STRG_UpdateMemoryFromVehicle(STRG_VParams);
	}
}

/**@brief Function for getting motor config parameters coming for host
	 @p param: motor ID parameter
	 @p value: value to add to the motor parameter table
*/
void STRG_getMotorParam(MD_Params_ID_t param, int32_t value)
{
	m_motor_params[param] = value;
}

bool STRG_isOverWritting( void )
{
	return m_isOverwritting;
}

void STRG_setOverWrite( )
{
	m_isOverwritting = true;
}
