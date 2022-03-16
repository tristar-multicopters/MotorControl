/**
  ******************************************************************************
  * @file    storage_management.c
	* @author  Jorge Andres Polo, FTEX
  * @brief   This module manages data storage in flash or external EEPROM, and software updates.
  *
	******************************************************************************
*/

#include "storage_management.h"

/*Global variable for save all vehicle parameters*/
static STRG_handle_t STRG_handle_m = {0}; // Storage instance containing the table parameters and error code
static ret_code_t m_strg_ret_code = 0;    // ret code for fstorage functions
osThreadId_t TSK_STRG_handle;
/************************************************/

/* Fstorage instance definition */
NRF_FSTORAGE_DEF(nrf_fstorage_t m_fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = START_PAGE_ADDR,
    .end_addr   = LAST_PAGE_ADDR,
};

/**@brief Function for init the storage manager module
	 @p p_evt: fstorage event
*/
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

/**@brief Function for save all vehicle parameters in the memory flash
	 @p strg_handler: vehicle params to save.
*/
static void STRG_SaveAllVehicleParams( void )
{														
		m_strg_ret_code = nrf_fstorage_write(&m_fstorage, 
																					START_PAGE_ADDR,
																				 &STRG_handle_m.params_table,
																					PAGE_SIZE*2, // Write 2 pages from START_PAGE_ADDR
																					NULL);
		wait_for_flash_ready(&m_fstorage);
	
	  if(m_strg_ret_code != NRF_SUCCESS)
			STRG_handle_m.error_code = STRG_WRITE_ERROR;
}

/**@brief Function for read all vehicle parameters in the memory flash
*/

static void STRG_ReadAllVehicleParams( void )
{
	// Read parameters from flash
	m_strg_ret_code = nrf_fstorage_read(&m_fstorage, 
																			START_PAGE_ADDR,
																			STRG_handle_m.params_table,
																			PAGE_SIZE*NB_OF_PAGES); // Read 2 pages from START_PAGE_ADDR
	
	if(m_strg_ret_code != NRF_SUCCESS)
		STRG_handle_m.error_code = STRG_READ_ERROR;
}

/**@brief Erase all the records on memory flash
*/
static void STRG_ResetAllVehicleParams( void )
{
	// Erase 2 pages starting from FIRST_PAGE_ADDR
	m_strg_ret_code = nrf_fstorage_erase(&m_fstorage, START_PAGE_ADDR, 2, NULL);
	if(m_strg_ret_code != NRF_SUCCESS)
		STRG_handle_m.error_code = STRG_ERASE_ERROR;
}

/**@brief Function for saving vehicle parameters in memory flash
*/
static void STRG_UpdateMemoryPage( void )
{
	/*Erase page for memory flash. Necessary for writting in memory */
	STRG_ResetAllVehicleParams();
	STRG_SaveAllVehicleParams();
}

/****************************************************************************/
/**************************** PUBLIC FUNCTIONS ******************************/

/**@brief Function for init the storage manager module
*/
void STRG_Init( void )
{
	nrf_fstorage_api_t * pFS_api;
	STRG_handle_m.error_code = STRG_NO_ERROR;
	
	/* Initialize an fstorage instance using the nrf_fstorage_nvmc backend.
   * nrf_fstorage_nvmc uses the NVMC peripheral.*/
	pFS_api = &nrf_fstorage_nvmc;
	
	m_strg_ret_code = nrf_fstorage_init(&m_fstorage, pFS_api, NULL);
	
	/*Load all vehicle parameters*/
	STRG_ReadAllVehicleParams();
}

/**@brief Function for managing writes in the flash memory
	 @p pvParameter: Unused parameter
*/
__NO_RETURN void TSK_StorageManagement(void * pvParameter)
{
	UNUSED_PARAMETER(pvParameter);
	
	while(true)
	{/*Wait for a write to flash request*/
		osThreadFlagsWait(STRG_FLAG, osFlagsWaitAny, osWaitForever);
			STRG_UpdateMemoryPage();
	}
}

/**@brief Function for getting a vehicle parameters from the memory flash after having read it
	 @p id: Storage parameter ID
	 @out: value of parameter requested parameter
*/
int32_t STRG_getParam(uint16_t id)
{
	return STRG_handle_m.params_table[id];
}

/**@brief Function for getting a vehicle parameters from the memory flash table
	 @p id: Storage parameter ID
	 @p value: value to store in the local register table
*/

void STRG_setParam(uint16_t id, int32_t value)
{
	STRG_handle_m.params_table[id] = value;
}
