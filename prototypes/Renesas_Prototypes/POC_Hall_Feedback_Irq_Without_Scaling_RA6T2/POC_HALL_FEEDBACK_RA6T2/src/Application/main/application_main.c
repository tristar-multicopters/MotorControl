#include <stdint.h>
#include "application_main.h"
#include "hal_data.h"
#include "hall_speed_pos_fdbk.h"
// Custom file for LL operations --------------- #include "../../src/bsp/mcu/ra6t2/bsp_register_access.h"



/***********************************************************************************************************************
* Global variables
***********************************************************************************************************************/
int16_t g_hELAngle,g_hELAngle_HALL, g_hELSpeedDpp;
qd_t g_volt_q_d;
uint16_t g_dac[2];
alphabeta_t g_volt_alpha_beta;
uint16_t hT_Sqrt3 = (PWM_PERIOD_CYCLES*SQRT3FACTOR)/16384u;
uint16_t g_adc[2];
bool enable_PWM, clear_HALL;
uint32_t capture_period;
bsp_io_level_t  Pin_U,Pin_V,Pin_W;
uint8_t Prescaler, delay_count;


// Initialize Hall Handle
HALL_Handle_t HALL_M1;


/***********************************************************************************************************************
* Private functions
***********************************************************************************************************************/
static void PWMC_park_clarke(qd_t Vqd, int16_t Angle, alphabeta_t * p_handle);
static void PWMC_SetPhaseVoltage(alphabeta_t Valfa_beta );



/***********************************************************************************************************************
* Operations
***********************************************************************************************************************/

void application_init( void )
{
			// Initialize variables
			g_volt_q_d.q = 0;
			g_volt_q_d.d = 0;
			g_hELAngle = 0;
			g_dac[0] = 0;
			g_dac[1] = 0;
			enable_PWM = false;
			capture_period = 0U;
			Pin_U = false;
			Pin_V = false;
			Pin_W = false;
			Prescaler = 255;
			clear_HALL = false;
			delay_count = 0;
			g_hELSpeedDpp = 10;
			
			// Connect electrical link
			R_ELC_Open(g_elc.p_ctrl, g_elc.p_cfg);
			//R_ELC_LinkSet(g_elc.p_ctrl, ELC_PERIPHERAL_GPT_A, ELC_EVENT_OPS_UVW_EDGE);
			R_ELC_Enable(g_elc.p_ctrl);
	
			// PWM overcurrent protection
			R_POEG_Open(g_poeg0.p_ctrl, g_poeg0.p_cfg);
		
			// DAC Setting
			R_DAC_Open(g_dac0.p_ctrl,g_dac0.p_cfg);
			R_DAC_Open(g_dac1.p_ctrl,g_dac1.p_cfg);
			R_DAC_Start(g_dac0.p_ctrl);
			R_DAC_Start(g_dac1.p_ctrl);

			R_DAC_Write(g_dac0.p_ctrl, (uint16_t) g_dac[0]);
			R_DAC_Write(g_dac1.p_ctrl, (uint16_t) g_dac[1]);
	
		  // ADC Settings
			R_ADC_B_Open(g_adc0.p_ctrl, g_adc0.p_cfg);
			R_ADC_B_ScanCfg(g_adc0.p_ctrl, g_adc0.p_channel_cfg);
			R_ADC_B_CallbackSet(g_adc0.p_ctrl,adc0_eoc0_isr,NULL,NULL);
			R_ADC_B_Calibrate(g_adc0.p_ctrl,NULL);
			R_BSP_SoftwareDelay (50,BSP_DELAY_UNITS_MICROSECONDS);
			R_ADC_B_ScanGroupStart(g_adc0.p_ctrl, ADC_GROUP_MASK_0);
			
			// Timer Settings for three-phase PWM
			R_GPT_THREE_PHASE_Open(g_three_phase0.p_ctrl,g_three_phase0.p_cfg);
			if(enable_PWM == true)
			{
			R_GPT_THREE_PHASE_Start (g_three_phase0.p_ctrl);
			}
			
// 			Capture Timer settings for HALL
			
// 			Commented as timer is initialized in HALL_Init function			
//			R_GPT_Open(g_timer3.p_ctrl, g_timer3.p_cfg);
//			R_GPT_CallbackSet(g_timer3.p_ctrl,hall_callback,NULL,NULL);
//			R_GPT_Enable(g_timer3.p_ctrl);

//			R_GPT_Start(g_timer3.p_ctrl);
//			
			
			HALL_Init(&HALL_M1);
						
			// Capture Timer settings for HALL
			
			R_ICU_ExternalIrqOpen(g_external_irq0.p_ctrl,g_external_irq0.p_cfg);
			R_ICU_ExternalIrqOpen(g_external_irq1.p_ctrl,g_external_irq1.p_cfg);
			R_ICU_ExternalIrqOpen(g_external_irq2.p_ctrl,g_external_irq2.p_cfg);
			
			R_ICU_ExternalIrqEnable(g_external_irq0.p_ctrl);
			R_ICU_ExternalIrqEnable(g_external_irq1.p_ctrl);
			R_ICU_ExternalIrqEnable(g_external_irq2.p_ctrl);
			
			/* Initializes port to access pins. */
			R_IOPORT_Open(g_ioport.p_ctrl,g_ioport.p_cfg);			
			
}


void application_main(void)
{
		
		if(UPDATE_LOCATION == 0)
		{
			g_hELAngle += 10;
			PWMC_park_clarke(g_volt_q_d, g_hELAngle, &g_volt_alpha_beta);
			
			PWMC_SetPhaseVoltage(g_volt_alpha_beta);
			
			R_ADC_B_Read(g_adc0.p_ctrl,ADC_CHANNEL_0,&g_adc[0]);
			R_ADC_B_Read(g_adc0.p_ctrl,ADC_CHANNEL_1,&g_adc[1]);
			
			//R_DAC_Write(g_dac0.p_ctrl, (uint16_t) ((g_volt_alpha_beta.alpha + 32768)/16));						// Output at PA06
			//R_DAC_Write(g_dac1.p_ctrl, (uint16_t) ((g_volt_alpha_beta.beta + 32768)/16));						// Output at PA07			
	
			R_BSP_SoftwareDelay (50,BSP_DELAY_UNITS_MICROSECONDS);
		}
		
		if(enable_PWM == true)
				{
					R_GPT_THREE_PHASE_Start (g_three_phase0.p_ctrl);
					
				}
		else
				{
					R_GPT_THREE_PHASE_Stop(g_three_phase0.p_ctrl);
				}
		
		if(clear_HALL == true)
				{
					HALL_Clear(&HALL_M1);
					clear_HALL = false;
				}
				
}


	/***********************************************************************************************************************
* Function Name : PWMC_park_clarke
* Description   : Converts variables from qd to alpha_beta
* Arguments     : Vqd, Angle, p_handle
* Return Value  : None
***********************************************************************************************************************/
static void PWMC_park_clarke(qd_t Vqd, int16_t Angle, alphabeta_t * p_handle)
	{
		float fELAngle = 0;
		float sin,cos;
	
		fELAngle = (float) Angle/10430;
		sincosf(fELAngle, &sin, &cos);														// TMU is a part of BSP
		p_handle->alpha = (int16_t) (Vqd.d*sin + Vqd.q*cos);
		p_handle->beta = (int16_t) (Vqd.q*sin - Vqd.d*cos) ;

	}
	
/* End of function PWMC_park_clarke */

	/***********************************************************************************************************************
* Function Name : PWMC_SetPhaseVoltage
* Description   :  Sets duty for PWM timers
* Arguments     : Valfa_beta
* Return Value  : None
***********************************************************************************************************************/
static void PWMC_SetPhaseVoltage(alphabeta_t Valfa_beta )
	{
	int32_t wX, wY, wZ, wUAlpha, wUBeta, wTimePhA, wTimePhB, wTimePhC;
	three_phase_duty_cycle_t duty_abc;

  wUAlpha = Valfa_beta.alpha * ( int32_t ) hT_Sqrt3;
  wUBeta = -( Valfa_beta.beta * ( int32_t )( PWM_PERIOD_CYCLES )) * 2;

  wX = wUBeta;
  wY = ( wUBeta + wUAlpha ) / 2;
  wZ = ( wUBeta - wUAlpha ) / 2;

  /* Sector calculation from wX, wY, wZ */
  if ( wY < 0 )
  {
    if ( wZ < 0 )
    {
			
      wTimePhA = ( int32_t )( PWM_PERIOD_CYCLES ) / 4 + ( ( wY - wZ ) / ( int32_t )262144 );
      wTimePhB = wTimePhA + wZ / 131072;
      wTimePhC = wTimePhA - wY / 131072;

    }
    else /* wZ >= 0 */
      if ( wX <= 0 )
      {
				
        wTimePhA = ( int32_t )( PWM_PERIOD_CYCLES ) / 4 + ( ( wX - wZ ) / ( int32_t )262144 );
        wTimePhB = wTimePhA + wZ / 131072;
        wTimePhC = wTimePhB - wX / 131072;
 
      }
      else /* wX > 0 */
      {
        
        wTimePhA = ( int32_t )( PWM_PERIOD_CYCLES ) / 4 + ( ( wY - wX ) / ( int32_t )262144 );
        wTimePhC = wTimePhA - wY / 131072;
        wTimePhB = wTimePhC + wX / 131072;

      }
  }
  else /* wY > 0 */
  {
    if ( wZ >= 0 )
    {
      
      wTimePhA = ( int32_t )( PWM_PERIOD_CYCLES ) / 4 + ( ( wY - wZ ) / ( int32_t )262144 );
      wTimePhB = wTimePhA + wZ / 131072;
      wTimePhC = wTimePhA - wY / 131072;
			
    }
    else /* wZ < 0 */
      if ( wX <= 0 )
      {
        
        wTimePhA = ( int32_t )( PWM_PERIOD_CYCLES ) / 4 + ( ( wY - wX ) / ( int32_t )262144 );
        wTimePhC = wTimePhA - wY / 131072;
        wTimePhB = wTimePhC + wX / 131072;
				
      }
      else /* wX > 0 */
      {
        
        wTimePhA = ( int32_t )( PWM_PERIOD_CYCLES ) / 4 + ( ( wX - wZ ) / ( int32_t )262144 );
        wTimePhB = wTimePhA + wZ / 131072;
        wTimePhC = wTimePhB - wX / 131072;

      }
  }
	
	duty_abc.duty[0] = (uint32_t)(MAX(wTimePhA,0));
	duty_abc.duty[1] = (uint32_t)(MAX(wTimePhB,0));
	duty_abc.duty[2] = (uint32_t)(MAX(wTimePhC,0));
	
	//R_DAC_Write(g_dac0.p_ctrl, (uint16_t) ((duty_abc.duty[0] /16)));
	//R_DAC_Write(g_dac1.p_ctrl, (uint16_t) ((duty_abc.duty[1] /16)));
	
	R_GPT_THREE_PHASE_DutyCycleSet(g_three_phase0.p_ctrl,&duty_abc);

	}
	
/* End of function PWMC_SetPhaseVoltage */	
	
	
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
						R_BSP_SoftwareDelay (15, BSP_DELAY_UNITS_SECONDS);
						(void) R_POEG_Reset(&g_poeg0_ctrl);
						do
							{
								(void) R_POEG_StatusGet(&g_poeg0_ctrl, &status);
							}	while (POEG_STATE_NO_DISABLE_REQUEST != status.state);
				}	
	}	
/* End of function g_poe_overcurrent */
	
/***********************************************************************************************************************
* Function Name : adc_eoc0_isr
* Description   : POEG3 Interrupt callback function
* Arguments     : p_args - Callback argument
* Return Value  : None
***********************************************************************************************************************/
void 	adc0_eoc0_isr(adc_callback_args_t * p_args)
	{
		if (NULL != p_args)
			{
				if(UPDATE_LOCATION == 1)
					{
						int16_t mech_speed;
						g_hELAngle += g_hELSpeedDpp;
						g_hELAngle_HALL = HALL_CalcElAngle(&HALL_M1);
						
						PWMC_park_clarke(g_volt_q_d, g_hELAngle, &g_volt_alpha_beta);
						
						R_ADC_B_Read(g_adc0.p_ctrl,ADC_CHANNEL_0,&g_adc[0]);
						R_ADC_B_Read(g_adc0.p_ctrl,ADC_CHANNEL_1,&g_adc[1]);
	
						R_DAC_Write(g_dac0.p_ctrl, (uint16_t) ((g_hELAngle_HALL + 32768)/16));						// Output at PA06
						//R_DAC_Write(g_dac1.p_ctrl, (uint16_t) ((g_volt_alpha_beta.beta + 32768)/16));						// Output at PA07
		
						PWMC_SetPhaseVoltage(g_volt_alpha_beta);
						
						
						// Calculate mechanical speed at medium frequency
						delay_count ++;
						if(delay_count>=20)
						{
						
							(void) HALL_CalcAvrgMecSpeedUnit( &HALL_M1, &mech_speed );
							delay_count = 0;
												
						}

					}
			}
	}

/* End of function adc_eoc0_isr */

/***********************************************************************************************************************
* Function Name : hall_callback
* Description   : hall Interrupt callback function
* Arguments     : p_args - Callback argument
* Return Value  : None
***********************************************************************************************************************/
void 	hall_callback(timer_callback_args_t * p_args)
	{
		if (NULL != p_args)
			{
				switch (p_args->event)
				{
				  case TIMER_EVENT_CYCLE_END:
						// Update timer prescaler here		//
						// Prescaler = R_BSP_GetTimerPrescaler(g_timer3.p_ctrl);
						 HALL_TIMx_UP_IRQHandler(&HALL_M1);
					
					//	capture_period++;
							break;
					case TIMER_EVENT_CAPTURE_A:
						// Speed calculation to be done here //
						capture_period = p_args->capture;
//						Pin_U = (bool) R_BSP_PinRead(BSP_IO_PORT_11_PIN_15);
//						Pin_V = (bool) R_BSP_PinRead(BSP_IO_PORT_11_PIN_00);
//						Pin_W = (bool) R_BSP_PinRead(BSP_IO_PORT_11_PIN_14);
						
						HALL_TIMx_CC_IRQHandler(&HALL_M1,&capture_period);
					
						//capture_period++;	
							break;
					case TIMER_EVENT_CAPTURE_B:
							break;
					case TIMER_EVENT_TROUGH:
							break;
				}
			}
	}

/* End of function adc_eoc0_isr */
	