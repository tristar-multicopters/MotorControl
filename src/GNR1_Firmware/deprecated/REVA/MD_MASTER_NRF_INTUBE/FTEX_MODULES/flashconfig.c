/**
  ******************************************************************************
  * @file    flashconfig.h
  * @author  Sami Bouzid, FTEX
  * @brief   This module handles storage to flash of device configuration
  *
	******************************************************************************
	*/

/* Includes ------------------------------------------------------------------*/
#include "flashconfig.h"

/* Flag to check fds initialization. */
static bool volatile m_fds_initialized;

ret_code_t rc1 = 2;

/* Dummy configuration data. */
configuration_t m_dummy_cfg =
{
    .config1_on  = false,
    .config2_on  = true,
    .boot_count  = 0x0,
    .device_name = "dummy",
};

/* A record containing dummy configuration data. */
fds_record_t const m_dummy_record =
{
    .file_id           = CONFIG_FILE,
    .key               = CONFIG_THROTTLE_KEY,
    .data.p_data       = &m_dummy_cfg,
    /* The length of a record is always expressed in 4-byte units (words). */
    .data.length_words = (sizeof(m_dummy_cfg) + 3) / sizeof(uint32_t),
};

/* Functions ---------------------------------------------------- */

static void fds_evt_handler(fds_evt_t const * p_evt)
{
    if (p_evt->result == NRF_SUCCESS)
    {
    }

    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == NRF_SUCCESS)
            {
                m_fds_initialized = true;
            }
            break;

        case FDS_EVT_WRITE:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
            }
        } break;

        case FDS_EVT_DEL_RECORD:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
            }
        } break;

        default:
            break;
    }
}

/**@brief   Wait for fds to initialize. */
static void wait_for_fds_ready(void)
{
    while (!m_fds_initialized)
    {
    }
}

void Flash_Init(void)
{
	ret_code_t rc;
	
	/* Register first to receive an event when initialization is complete. */
	(void) fds_register(fds_evt_handler);
	
	rc = fds_init();
	rc1 = rc;
	
	/* Wait for fds to initialize. */
  wait_for_fds_ready();

	fds_stat_t stat = {0};
	rc = fds_stat(&stat);	
	rc1 = rc;
}

void Flash_ReadConfig_Throttle(Throttle_Param_t * p_Throttle_Param)
{	
	ret_code_t rc;
	fds_record_desc_t desc = {0};
	fds_find_token_t  tok  = {0};
	fds_flash_record_t record = {0};

	rc = fds_record_find(CONFIG_FILE, CONFIG_THROTTLE_KEY, &desc, &tok);

	if (rc == NRF_SUCCESS)
	{
		/* Open the record and read its contents. */
		rc = fds_record_open(&desc, &record);
	
		/* Copy the configuration from flash into dynamic memory. */
		memcpy(p_Throttle_Param, record.p_data, sizeof(Throttle_Param_t));
	
		/* Close the record when done reading. */
		rc = fds_record_close(&desc);
	}
}

void Flash_WriteConfig_Throttle(Throttle_Param_t * p_Throttle_Param)
{	
	ret_code_t rc;
	fds_record_desc_t desc = {0};
	fds_find_token_t  tok  = {0};
	
	Throttle_Param_t param = *p_Throttle_Param;
	fds_record_t throttle_record =
	{
			.file_id           = CONFIG_FILE,
			.key               = CONFIG_THROTTLE_KEY,
			.data.p_data       = &param,
			/* The length of a record is always expressed in 4-byte units (words). */
			.data.length_words = (sizeof(param) + 3) / sizeof(uint32_t),
	};

	rc = fds_record_find(CONFIG_FILE, CONFIG_THROTTLE_KEY, &desc, &tok);
	rc1 = rc;
	
	if (rc == NRF_SUCCESS)
	{
		rc = fds_record_update(&desc, &m_dummy_record);
	}
	else if (rc == FDS_ERR_NOT_FOUND)
	{
		rc = fds_record_write(&desc, &m_dummy_record);
	}
}

