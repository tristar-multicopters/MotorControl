/**
  * @file    circle_limitation.c
  * @brief   This file provides the functions that implement the circle
  *          limitation feature of the STM32 Motor Control application.
  *
	*/

/* Includes ------------------------------------------------------------------*/
#include "circle_limitation.h"
#include "mc_math.h"
#include "mc_type.h"


#if defined (CIRCLE_LIMITATION_VD)

qd_t CircleLimitation(CircleLimitationHandle_t * pHandle, qd_t Vqd)
{
  int32_t hMaxModule;
  int32_t square_q;
  int32_t square_temp;
  int32_t square_d;
  int32_t square_sum;
  int32_t square_limit;
  int32_t vd_square_limit;
  int32_t new_q;
  int32_t new_d;
  qd_t Local_Vqd=Vqd;

  hMaxModule = pHandle->hMaxModule;

  square_q = (int32_t)(Vqd.q) * Vqd.q;
  square_d = (int32_t)(Vqd.d) * Vqd.d;
  square_limit = hMaxModule * hMaxModule;
  vd_square_limit = pHandle->hMaxVd * pHandle->hMaxVd;
  square_sum = square_q + square_d;

  if (square_sum > square_limit)
  {
    if(square_d <= vd_square_limit)
    {
      square_temp = square_limit - square_d;
      new_q = MCMath_Sqrt(square_temp);
      if(Vqd.q < 0)
      {
        new_q = -new_q;
      }
      new_d = Vqd.d;
    }
    else
    {
      new_d = pHandle->hMaxVd;
      if(Vqd.d < 0)
      {
        new_d = -new_d;
      }

      square_temp = square_limit - vd_square_limit;
      new_q = MCMath_Sqrt(square_temp);
      if(Vqd.q < 0)
      {
        new_q = - new_q;
      }
    }
    Local_Vqd.q = new_q;
    Local_Vqd.d = new_d;
  }
  return(Local_Vqd);
}
#else

/**
  * @brief Check whether Vqd.q^2 + Vqd.d^2 <= 32767^2
  *        and if not it applies a limitation keeping constant ratio
  *        Vqd.q / Vqd.d
  * @param  pHandle pointer on the related component instance
  * @param  Vqd Voltage in qd reference frame
  * @retval qd_t Limited Vqd vector
  */
qd_t CircleLimitation( CircleLimitationHandle_t * pHandle, qd_t Vqd )
{
  uint16_t table_element;
  uint32_t uw_temp;
  int32_t  sw_temp;
  qd_t local_vqd = Vqd;

  sw_temp = ( int32_t )( Vqd.q ) * Vqd.q +
            ( int32_t )( Vqd.d ) * Vqd.d;

  uw_temp = ( uint32_t ) sw_temp;

  /* uw_temp min value 0, max value 32767*32767 */
  if ( uw_temp > ( uint32_t )( pHandle->hMaxModule ) * pHandle->hMaxModule )
  {

    uw_temp /= ( uint32_t )( 16777216 );

    /* wtemp min value pHandle->bStartIndex, max value 127 */
    uw_temp -= pHandle->bStartIndex;

    /* uw_temp min value 0, max value 127 - pHandle->bStartIndex */
    table_element = pHandle->CircleLimitTable[( uint8_t )uw_temp];

    sw_temp = Vqd.q * ( int32_t )table_element;
    local_vqd.q = ( int16_t )( sw_temp / 32768 );

    sw_temp = Vqd.d * ( int32_t )( table_element );
    local_vqd.d = ( int16_t )( sw_temp / 32768 );
  }

  return ( local_vqd );
}
#endif


