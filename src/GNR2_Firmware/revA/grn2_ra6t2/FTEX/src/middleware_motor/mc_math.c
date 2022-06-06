/**
  * @file    mc_math.c
  * @brief   This file provides mathematics functions useful for and specific to
  *          Motor Control application
  *
	*/
/* Includes ------------------------------------------------------------------*/
#include "mc_math.h"
#include "mc_type.h"
#include "bsp_api.h"

/* Private macro -------------------------------------------------------------*/

#define divSQRT_3 (int32_t)0x49E6    /* 1/sqrt(3) in q1.15 format=0.5773315*/

const float fPi = (float)3.1416; /* pi */


AlphaBeta_t MCMath_Clarke( ab_t Input  )
{
  AlphaBeta_t Output;

  int32_t a_divSQRT3_tmp, b_divSQRT3_tmp ;
  int32_t wbeta_tmp;
  int16_t hbeta_tmp;

  /* qIalpha = qIas*/
  Output.alpha = Input.a;

  a_divSQRT3_tmp = divSQRT_3 * ( int32_t )Input.a;

  b_divSQRT3_tmp = divSQRT_3 * ( int32_t )Input.b;

  /*qIbeta = -(2*qIbs+qIas)/sqrt(3)*/
#ifdef FULL_MISRA_C_COMPLIANCY
  wbeta_tmp = ( -( a_divSQRT3_tmp ) - ( b_divSQRT3_tmp ) -
                 ( b_divSQRT3_tmp ) ) / 32768;
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */

  wbeta_tmp = ( -( a_divSQRT3_tmp ) - ( b_divSQRT3_tmp ) -
                 ( b_divSQRT3_tmp ) ) >> 15;
#endif

  /* Check saturation of Ibeta */
  if ( wbeta_tmp > INT16_MAX )
  {
    hbeta_tmp = INT16_MAX;
  }
  else if ( wbeta_tmp < ( -32768 ) )
  {
    hbeta_tmp = ( -32768 );
  }
  else
  {
    hbeta_tmp = ( int16_t )( wbeta_tmp );
  }

  Output.beta = hbeta_tmp;

  if ( Output.beta == ( int16_t )( -32768 ) )
  {
    Output.beta = -32767;
  }

  return ( Output );
}


qd_t MCMath_Park( AlphaBeta_t Input, int16_t Theta )
{
  qd_t Output;
  int32_t d_tmp_1, d_tmp_2, q_tmp_1, q_tmp_2;
  TrigComponents_t Local_Vector_Components;
  int32_t wqd_tmp;
  int16_t hqd_tmp;

  Local_Vector_Components = MCMath_TrigFunctions( Theta );

  /*No overflow guaranteed*/
  q_tmp_1 = Input.alpha * ( int32_t )Local_Vector_Components.hCos;

  /*No overflow guaranteed*/
  q_tmp_2 = Input.beta * ( int32_t )Local_Vector_Components.hSin;

  /*Iq component in Q1.15 Format */
#ifdef FULL_MISRA_C_COMPLIANCY
  wqd_tmp = ( q_tmp_1 - q_tmp_2 ) / 32768;
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */
  wqd_tmp = ( q_tmp_1 - q_tmp_2 ) >> 15;
#endif

  /* Check saturation of Iq */
  if ( wqd_tmp > INT16_MAX )
  {
    hqd_tmp = INT16_MAX;
  }
  else if ( wqd_tmp < ( -32768 ) )
  {
    hqd_tmp = ( -32768 );
  }
  else
  {
    hqd_tmp = ( int16_t )( wqd_tmp );
  }

  Output.q = hqd_tmp;

  if ( Output.q == ( int16_t )( -32768 ) )
  {
    Output.q = -32767;
  }

  /*No overflow guaranteed*/
  d_tmp_1 = Input.alpha * ( int32_t )Local_Vector_Components.hSin;

  /*No overflow guaranteed*/
  d_tmp_2 = Input.beta * ( int32_t )Local_Vector_Components.hCos;

  /*Id component in Q1.15 Format */
#ifdef FULL_MISRA_C_COMPLIANCY
  wqd_tmp = ( d_tmp_1 + d_tmp_2 ) / 32768;
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */
  wqd_tmp = ( d_tmp_1 + d_tmp_2 ) >> 15;
#endif

  /* Check saturation of Id */
  if ( wqd_tmp > INT16_MAX )
  {
    hqd_tmp = INT16_MAX;
  }
  else if ( wqd_tmp < ( -32768 ) )
  {
    hqd_tmp = ( -32768 );
  }
  else
  {
    hqd_tmp = ( int16_t )( wqd_tmp );
  }

  Output.d = hqd_tmp;

  if ( Output.d == ( int16_t )( -32768 ) )
  {
    Output.d = -32767;
  }

  return ( Output );
}


AlphaBeta_t MCMath_RevPark( qd_t Input, int16_t Theta )
{
  int32_t alpha_tmp1, alpha_tmp2, beta_tmp1, beta_tmp2;
  TrigComponents_t Local_Vector_Components;
  AlphaBeta_t Output;

  Local_Vector_Components = MCMath_TrigFunctions( Theta );

  /*No overflow guaranteed*/
  alpha_tmp1 = Input.q * ( int32_t )Local_Vector_Components.hCos;
  alpha_tmp2 = Input.d * ( int32_t )Local_Vector_Components.hSin;

#ifdef FULL_MISRA_C_COMPLIANCY
  Output.alpha = ( int16_t )( ( ( alpha_tmp1 ) + ( alpha_tmp2 ) ) / 32768 );
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
    that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
    the compiler to perform the shift (instead of LSR logical shift right) */
  Output.alpha = ( int16_t )( ( ( alpha_tmp1 ) + ( alpha_tmp2 ) ) >> 15 );
#endif

  beta_tmp1 = Input.q * ( int32_t )Local_Vector_Components.hSin;
  beta_tmp2 = Input.d * ( int32_t )Local_Vector_Components.hCos;

#ifdef FULL_MISRA_C_COMPLIANCY
  Output.beta = ( int16_t )( ( beta_tmp2 - beta_tmp1 ) / 32768 );
#else
  /* WARNING: the below instruction is not MISRA compliant, user should verify
  that Cortex-M3 assembly instruction ASR (arithmetic shift right) is used by
  the compiler to perform the shift (instead of LSR logical shift right) */
  Output.beta = ( int16_t )( ( beta_tmp2 - beta_tmp1 ) >> 15 );
#endif

  return ( Output );
}


TrigComponents_t MCMath_TrigFunctions( int16_t hAngle )
{
	float fAngle = 0;
	float fSin, fCos;
	TrigComponents_t sCosSin;

	fAngle = (float)hAngle/(INT16_MAX+1) * fPi;
	sincosf(fAngle, &fSin, &fCos);
	
	sCosSin.hSin = (int16_t)(fSin * INT16_MAX);
	sCosSin.hCos = (int16_t)(fCos * INT16_MAX);
	
  return sCosSin;

}


int32_t MCMath_Sqrt( int32_t wInput )
{
  int32_t wtemprootnew;

  if ( wInput > 0 )
  {
  uint8_t biter = 0u;
  int32_t wtemproot;

    if ( wInput <= ( int32_t )2097152 )
    {
      wtemproot = ( int32_t )128;
    }
    else
    {
      wtemproot = ( int32_t )8192;
    }

    do
    {
      wtemprootnew = ( wtemproot + wInput / wtemproot ) / ( int32_t )2;
      if ( wtemprootnew == wtemproot )
      {
        biter = 6u;
      }
      else
      {
        biter ++;
        wtemproot = wtemprootnew;
      }
    }
    while ( biter < 6u );

  }
  else
  {
    wtemprootnew = ( int32_t )0;
  }

  return ( wtemprootnew );
}


inline int16_t MCMath_PhaseComputation( int32_t wBemfalfaEst, int32_t wBemfbetaEst )
{
	float fAngle = 0;
	
	fAngle = atan2f(wBemfbetaEst>>8, wBemfalfaEst>>8); /* Reduce amplitude of BEMF to fit into float type*/

  return (int16_t)(fAngle/fPi*INT16_MAX);

}


uint32_t MCMath_FloatToIntBit( float x )
{
  uint32_t * pInt;
  pInt = ( uint32_t * )( &x );
  return *pInt;
}

