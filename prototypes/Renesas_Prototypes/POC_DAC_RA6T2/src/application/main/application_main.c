#include <stdint.h>
#include "application_main.h"
#include "hal_data.h"

/***********************************************************************************************************************
* Global variables
***********************************************************************************************************************/
int16_t g_hELAngle;
qd_t g_volt_q_d;
uint16_t g_dac[2];
alphabeta_t g_volt_alpha_beta;

/***********************************************************************************************************************
* Private functions
***********************************************************************************************************************/
static void PWMC_park_clarke(qd_t Vqd, int16_t Angle, alphabeta_t * p_handle);





void application_init( void )
{
			// Initialize variables
			g_volt_q_d.q = 0;
			g_volt_q_d.d = 0;
			g_hELAngle = 0;
			g_dac[0] = 0;
			g_dac[1] = 0;
	
			// DAC Setting
			R_DAC_Open(g_dac0.p_ctrl,g_dac0.p_cfg);
			R_DAC_Open(g_dac1.p_ctrl,g_dac1.p_cfg);
			R_DAC_Start(g_dac0.p_ctrl);
			R_DAC_Start(g_dac1.p_ctrl);

			R_DAC_Write(g_dac0.p_ctrl, (uint16_t) g_dac[0]);
			R_DAC_Write(g_dac1.p_ctrl, (uint16_t) g_dac[1]);
	
}


void application_main(void)
{
		g_hELAngle += 10;
		PWMC_park_clarke(g_volt_q_d, g_hELAngle, &g_volt_alpha_beta);
	
		R_DAC_Write(g_dac0.p_ctrl, (uint16_t) ((g_volt_alpha_beta.alpha + 32768)/16));						// Output at PA06
		R_DAC_Write(g_dac1.p_ctrl, (uint16_t) ((g_volt_alpha_beta.beta + 32768)/16));						// Output at PA07
	
	
	
	R_BSP_SoftwareDelay (50,BSP_DELAY_UNITS_MICROSECONDS);
}

static void PWMC_park_clarke(qd_t Vqd, int16_t Angle, alphabeta_t * p_handle)
	{
		float fELAngle = 0;
		float sin,cos;
	
		fELAngle = (float) Angle/10430;
		sincosf(fELAngle, &sin, &cos);														// TMU is a part of BSP
		p_handle->alpha = (int16_t) (Vqd.d*sin + Vqd.q*cos);
		p_handle->beta = (int16_t) (Vqd.q*sin - Vqd.d*cos) ;

	}

