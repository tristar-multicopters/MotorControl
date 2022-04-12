#include "hal_data.h"
#include"application_main.h"
#include "pwm_curr_fdbk.h"


/***********************************************************************************************************************
* Function Name : g_poe_overcurrent
* Description   : POEG3 Interrupt callback function
* Arguments     : p_args - Callback argument
* Return Value  : None
***********************************************************************************************************************/
void g_poe_overcurrent(poeg_callback_args_t * p_args)
	{	
			poeg_status_t status;
		
			if (NULL != p_args)
				{
						R_BSP_SoftwareDelay (10, BSP_DELAY_UNITS_SECONDS);
						(void) R_POEG_Reset(&g_poeg0_ctrl);
						do
							{
								(void) R_POEG_StatusGet(&g_poeg0_ctrl, &status);
							}	while (POEG_STATE_NO_DISABLE_REQUEST != status.state);
				}	
	}	
/* End of function g_poe_overcurrent */
	
	