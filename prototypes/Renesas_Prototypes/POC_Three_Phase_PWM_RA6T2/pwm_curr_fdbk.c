#include "pwm_curr_fdbk.h"


uint16_t hT_Sqrt3 = (PWM_PERIOD_CYCLES*SQRT3FACTOR)/16384u;





uint16_t PWMC_SetPhaseVoltage( alphabeta_t Valfa_beta )
{
  int32_t wX, wY, wZ, wUAlpha, wUBeta, wTimePhA, wTimePhB, wTimePhC;
	three_phase_duty_cycle_t duty_abc;

  wUAlpha = Valfa_beta.alpha * ( int32_t ) hT_Sqrt3;
  wUBeta = -( Valfa_beta.beta * ( int32_t )( PWM_PERIOD_CYCLES )) * 4;

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
	
	R_GPT_THREE_PHASE_DutyCycleSet(g_three_phase0.p_ctrl,&duty_abc);

  return ( NULL );
}

															 
															 
															 
															 
															 
															 