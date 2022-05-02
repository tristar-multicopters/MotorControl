/**
  ******************************************************************************
  * @file    hall_speed_pos_fdbk.c
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file provides firmware functions that implement the features of
  *          the Hall Speed & Position Feedback component of the Motor Control SDK.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "hall_speed_pos_fdbk.h"


/** @addtogroup MCSDK
  * @{
  */

/** @addtogroup SpeednPosFdbk
  * @{
  */

/**
 * @defgroup hall_speed_pos_fdbk Hall Speed & Position Feedback
 *
 * @brief Hall Sensor based Speed & Position Feedback implementation
 *
  * This component is used in applications controlling a motor equipped with Hall effect sensors.
  *
  * This component uses the output of two Hall effects sensors to provide a measure of the speed
  * and the position of the rotor of the motor.
  *
 * @todo Document the Hall Speed & Position Feedback "module".
 *
 * @{
 */
 
//int16_t stateBuffer[200] = {0};
//uint8_t iter = 0;

/* Private defines -----------------------------------------------------------*/

/* Lower threshold to reques a decrease of clock prescaler */
#define LOW_RES_THRESHOLD   ((uint32_t)0x5500u)

#define HALL_COUNTER_RESET  ((uint16_t) 0u)

#define S16_120_PHASE_SHIFT (int16_t)(65536/3)
#define S16_60_PHASE_SHIFT  (int16_t)(65536/6)
#define S16_30_PHASE_SHIFT  (int16_t)(65536/12)

#define STATE_0 (uint8_t)0
#define STATE_1 (uint8_t)1
#define STATE_2 (uint8_t)2
#define STATE_3 (uint8_t)3
#define STATE_4 (uint8_t)4
#define STATE_5 (uint8_t)5
#define STATE_6 (uint8_t)6
#define STATE_7 (uint8_t)7

#define NEGATIVE          (int8_t)-1
#define POSITIVE          (int8_t)1

/* With digit-per-PWM unit (here 2*PI rad = 0xFFFF): */
#define HALL_MAX_PSEUDO_SPEED        ((int16_t)0x7FFF)

#define CCER_CC1E_Set               ((uint16_t)0x0001)
#define CCER_CC1E_Reset             ((uint16_t)0xFFFE)

// Local user Defines 


#define SPEED_UNIT										10
#define HALL_PHASE_SHIFT 		60 
#define POLE_PAIR_NUM 23
#define MAX_APPLICATION_SPEED_UNIT 250
#define MIN_APPLICATION_SPEED_UNIT 0
#define MEAS_ERRORS_BEFORE_FAULTS 6
#define TF_REGULATION_RATE_SCALED 20000
#define DPP_CONV_FACTOR 65536
#define HALL_SENSORS_PLACEMENT 0u
#define MEDIUM_FREQUENCY_TASK_RATE 10000
#define HALL_AVERAGING_FIFO_DEPTH 8
#define HALL_TIM_CLK 120000000uL
#define PWM_FREQ_SCALING 1
#define HALL_MTPA true


static void HALL_Init_Electrical_Angle( HALL_Handle_t * pHandle );

/**
  * @brief  It initializes the hardware peripherals (TIMx, GPIO and NVIC)
            required for the speed position sensor management using HALL
            sensors.
  * @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
  * @retval none
  */
	
void HALL_Init( HALL_Handle_t * pHandle )
{
	
	// Initialize the parameters here
	pHandle->_Super.bElToMecRatio = POLE_PAIR_NUM;						// Pole pair numbers 
	pHandle->_Super.hMaxReliableMecSpeedUnit = (uint16_t)(1.15*MAX_APPLICATION_SPEED_UNIT);								// Max application speed unit
	pHandle->_Super.hMinReliableMecSpeedUnit = (uint16_t)(MIN_APPLICATION_SPEED_UNIT);										// Minimum application speed unit
	pHandle->_Super.bMaximumSpeedErrorsNumber = MEAS_ERRORS_BEFORE_FAULTS;							// Maximum number of speed errors
	pHandle->_Super.hMaxReliableMecAccelUnitP = 65535;				// Maximum value of acceleration
	pHandle->_Super.hMeasurementFrequency = TF_REGULATION_RATE_SCALED;							// This is value at which current is sampled
	pHandle->_Super.DPPConvFactor = DPP_CONV_FACTOR;														// This is DPP per PWM frequency
	pHandle->SensorPlacement = HALL_SENSORS_PLACEMENT;																					// Value used to perform calculation, sensors can be 120 degrees = 0u or 60 degrees = 1u 
	pHandle->PhaseShift = (int16_t)(HALL_PHASE_SHIFT * 65536/360);
	pHandle->SpeedSamplingFreqHz = (uint16_t) MEDIUM_FREQUENCY_TASK_RATE;
	pHandle->SpeedBufferSize     = HALL_AVERAGING_FIFO_DEPTH;
	pHandle->TIMClockFreq       = HALL_TIM_CLK;
	pHandle->PWMFreqScaling      = PWM_FREQ_SCALING;
	pHandle->HallMtpa  = HALL_MTPA;
	pHandle->H1PortPin = BSP_IO_PORT_12_PIN_10;
	pHandle->H2PortPin = BSP_IO_PORT_12_PIN_11;
	pHandle->H3PortPin = BSP_IO_PORT_10_PIN_12;
	
	// Hall code same as STM
	
  uint16_t hMinReliableElSpeedUnit = pHandle->_Super.hMinReliableMecSpeedUnit *
                                     pHandle->_Super.bElToMecRatio;
  uint16_t hMaxReliableElSpeedUnit = pHandle->_Super.hMaxReliableMecSpeedUnit *
                                     pHandle->_Super.bElToMecRatio;
  uint8_t bSpeedBufferSize;
  uint8_t bIndex;

  /* Adjustment factor: minimum measurable speed is x time less than the minimum
  reliable speed */
  hMinReliableElSpeedUnit /= 4u;

  /* Adjustment factor: maximum measurable speed is x time greater than the
  maximum reliable speed */
  hMaxReliableElSpeedUnit *= 2u;

  pHandle->OvfFreq = ( uint16_t )( pHandle->TIMClockFreq / 65536u );

  /* SW Init */
  if ( hMinReliableElSpeedUnit == 0u )
  {
    /* Set fixed to 150 ms */
    pHandle->HallTimeout = 150u;
  }
  else
  {
    /* Set accordingly the min reliable speed */
    /* 1000 comes from mS 
    * 6 comes from the fact that sensors are toggling each 60 deg = 360/6 deg */
    pHandle->HallTimeout = 1000*SPEED_UNIT / ( 6u * hMinReliableElSpeedUnit );
  }

  /* Compute the prescaler to the closet value of the TimeOut (in mS )*/
  pHandle->HALLMaxRatio = ( pHandle->HallTimeout * pHandle->OvfFreq ) / 1000 ;					// prescaler so that counter counts 65536 in desired timeout value

  /* Align MaxPeriod to a multiple of Overflow.*/
  pHandle->MaxPeriod = ( pHandle->HALLMaxRatio ) * 65536uL;

  pHandle->SatSpeed = hMaxReliableElSpeedUnit;

  pHandle->PseudoFreqConv = ( ( pHandle->TIMClockFreq / 6u )
                              / ( pHandle->_Super.hMeasurementFrequency ) ) * ( pHandle->_Super.DPPConvFactor);

  pHandle->MinPeriod = ( ( SPEED_UNIT * ( pHandle->TIMClockFreq /6uL) ) 
                       / hMaxReliableElSpeedUnit);

  pHandle->PWMNbrPSamplingFreq = ( (pHandle->_Super.hMeasurementFrequency * pHandle->PWMFreqScaling) /
                                   pHandle->SpeedSamplingFreqHz ) - 1u;

  /* Reset speed reliability */
  pHandle->SensorIsReliable = true;

  /* Force the TIMx prescaler with immediate access (gen update event)
  */

	
	//  2 Open Timer  --------------------
	
	R_GPT_Open(g_timer3.p_ctrl, g_timer3.p_cfg);
	R_GPT_CallbackSet(g_timer3.p_ctrl,hall_callback,NULL,NULL);
	R_GPT_Enable(g_timer3.p_ctrl);
	
	  //  1 Initial prescaler setting ------------------------
	pHandle->Period_PostMultiplier = pHandle->HALLMaxRatio;	
	pHandle->Period = pHandle->Period_PostMultiplier * 65536uL;
	R_GPT_PeriodSet(g_timer3.p_ctrl, pHandle->Period);
	
	R_GPT_Start(g_timer3.p_ctrl);
	
  /* Erase speed buffer */
  bSpeedBufferSize = pHandle->SpeedBufferSize;

  for ( bIndex = 0u; bIndex < bSpeedBufferSize; bIndex++ )
  {
    pHandle->SensorPeriod[bIndex]  = pHandle->MaxPeriod; //(int32_t) pHandle->MaxPeriod;
  }
	
	pHandle->Sector_Start_Angle[0] = 0;  
	pHandle->Sector_Start_Angle[1] = ( int16_t ) (pHandle->PhaseShift + S16_60_PHASE_SHIFT);
	pHandle->Sector_Start_Angle[2] = ( int16_t ) (pHandle->PhaseShift + S16_60_PHASE_SHIFT + S16_120_PHASE_SHIFT);
	pHandle->Sector_Start_Angle[3] = ( int16_t ) ( pHandle->PhaseShift + S16_120_PHASE_SHIFT);
	pHandle->Sector_Start_Angle[4] = ( int16_t ) (pHandle->PhaseShift - S16_60_PHASE_SHIFT);
	pHandle->Sector_Start_Angle[5] = ( int16_t ) (pHandle->PhaseShift);
	pHandle->Sector_Start_Angle[6] = ( int16_t ) (pHandle->PhaseShift - S16_120_PHASE_SHIFT);

	pHandle->Sector_Destination_Angle[0] = 0;  
	pHandle->Sector_Destination_Angle[1] = ( int16_t ) (pHandle->PhaseShift  + S16_120_PHASE_SHIFT) ;
  pHandle->Sector_Destination_Angle[2] = ( int16_t ) (pHandle->PhaseShift - S16_120_PHASE_SHIFT );
	pHandle->Sector_Destination_Angle[3] = ( int16_t ) (pHandle->PhaseShift + S16_60_PHASE_SHIFT + S16_120_PHASE_SHIFT) ;
  pHandle->Sector_Destination_Angle[4] =  (int16_t ) (pHandle->PhaseShift );
	pHandle->Sector_Destination_Angle[5] = ( int16_t ) (pHandle->PhaseShift  + S16_60_PHASE_SHIFT);
  pHandle->Sector_Destination_Angle[6] = ( int16_t ) (pHandle->PhaseShift - S16_60_PHASE_SHIFT);
	
	pHandle->Sector_Middle_Angle[0] = 0;  
	pHandle->Sector_Middle_Angle[1] = ( int16_t ) ( pHandle->Sector_Start_Angle[1]  +  S16_30_PHASE_SHIFT );
  pHandle->Sector_Middle_Angle[2] = ( int16_t ) ( pHandle->Sector_Start_Angle[2]  +  S16_30_PHASE_SHIFT);
	pHandle->Sector_Middle_Angle[3] = ( int16_t )  ( pHandle->Sector_Start_Angle[3]  +  S16_30_PHASE_SHIFT);
  pHandle->Sector_Middle_Angle[4] = ( int16_t ) ( pHandle->Sector_Start_Angle[4]  +  S16_30_PHASE_SHIFT);
	pHandle->Sector_Middle_Angle[5] =( int16_t ) ( pHandle->Sector_Start_Angle[5]  +  S16_30_PHASE_SHIFT);
  pHandle->Sector_Middle_Angle[6] = ( int16_t ) ( pHandle->Sector_Start_Angle[6]  +  S16_30_PHASE_SHIFT);
}

/**
* @brief  Clear software FIFO where are "pushed" latest speed information
*         This function must be called before starting the motor to initialize
*         the speed measurement process.
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component*
* @retval none
*/
void HALL_Clear( HALL_Handle_t * pHandle )
{

  /* Mask interrupts to insure a clean intialization */
	// 3 Disable capture compare interrupt ----------------------------------
	
  pHandle->RatioDec = false;
  pHandle->RatioInc = false;

  /* Reset speed reliability */
  pHandle->SensorIsReliable = true;

  /* Acceleration measurement not implemented.*/
  pHandle->_Super.hMecAccelUnitP = 0;

  pHandle->FirstCapt = 0u;
  pHandle->BufferFilled = 0u;
  pHandle->OVFCounter = 0u;

  pHandle->CompSpeed = 0;

  pHandle->Direction = POSITIVE;

  /* Initialize speed buffer index */
  pHandle->SpeedFIFOIdx = 0u;


  /* Clear speed error counter */
  pHandle->_Super.bSpeedErrorNumber = 0;
  
	// Reset the prescaler variable -------------------------------------------
	pHandle->Period_PostMultiplier = pHandle->HALLMaxRatio;

	// Set counter to zero -------------------------------------------------------------
	
	R_GPT_Stop(g_timer3.p_ctrl);
	R_GPT_CounterSet(g_timer3.p_ctrl,HALL_COUNTER_RESET);
		
	// Enable counter --------------------------------------------------------------------------
	R_GPT_Start(g_timer3.p_ctrl);
  //LL_TIM_EnableCounter ( TIMx );
	
	// Enable capture compare interrupt ------------------------------------------------

  //LL_TIM_EnableIT_CC1 ( TIMx );

  HALL_Init_Electrical_Angle( pHandle );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
* @brief  Update the rotor electrical angle integrating the last measured
*         instantaneous electrical speed express in dpp.
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval int16_t Measured electrical angle in s16degree format.
*/
int16_t HALL_CalcElAngle( HALL_Handle_t * pHandle )
{

	if ( pHandle->BufferFilled < pHandle->SpeedBufferSize )
	{
	}
	else
	{
		if ( pHandle->_Super.hElSpeedDpp != HALL_MAX_PSEUDO_SPEED )
		{
			pHandle->MeasuredElAngle += pHandle->_Super.hElSpeedDpp;
			pHandle->_Super.hElAngle += pHandle->_Super.hElSpeedDpp + pHandle->CompSpeed;
			pHandle->PrevRotorFreq = pHandle->_Super.hElSpeedDpp;
		}
		else
		{
			pHandle->_Super.hElAngle += pHandle->PrevRotorFreq;
		}
		
		if ( abs(pHandle->_Super.hElAngle - pHandle->Sector_Middle_Angle[pHandle->HallState]) > S16_30_PHASE_SHIFT)
		{
			if(pHandle->Direction == POSITIVE)
					{pHandle->_Super.hElAngle = pHandle->Sector_Destination_Angle[pHandle->HallState];}
			else
					{pHandle->_Super.hElAngle = pHandle->Sector_Start_Angle[pHandle->HallState];}
		}
	}

  return pHandle->_Super.hElAngle;
}


/**
  * @brief  This method must be called - at least - with the same periodicity
  *         on which speed control is executed.
  *         This method compute and store rotor istantaneous el speed (express
  *         in dpp considering the measurement frequency) in order to provide it
  *         to HALL_CalcElAngle function and SPD_GetElAngle.
  *         Then compute rotor average el speed (express in dpp considering the
  *         measurement frequency) based on the buffer filled by IRQ, then - as
  *         a consequence - compute, store and return - through parameter
  *         hMecSpeedUnit - the rotor average mech speed, expressed in Unit.
  *         Then check, store and return the reliability state of
  *         the sensor; in this function the reliability is measured with
  *         reference to specific parameters of the derived
  *         sensor (HALL) through internal variables managed by IRQ.
  * @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
  * @param  hMecSpeedUnit pointer to int16_t, used to return the rotor average
  *         mechanical speed (expressed in the unit defined by #SPEED_UNIT)
  * @retval true = sensor information is reliable
  *         false = sensor information is not reliable
  */
bool HALL_CalcAvrgMecSpeedUnit( HALL_Handle_t * pHandle, int16_t * hMecSpeedUnit )
{
  bool bReliability;

  if ( pHandle->SensorIsReliable )
  {
    /* No errors have been detected during rotor speed information
    extrapolation */
		
		// Compare prescaler variable with ----------------------------------------------------------------------------
		
    if ( pHandle->Period_PostMultiplier >= pHandle->HALLMaxRatio )
    {
      /* At start-up or very low freq */
      /* Based on current prescaler value only */
      pHandle->_Super.hElSpeedDpp = 0;
      *hMecSpeedUnit = 0;
    }
    else
    {
      pHandle->_Super.hElSpeedDpp =  pHandle->AvrElSpeedDpp;
      if (  pHandle->AvrElSpeedDpp == 0 )
      {
        /* Speed is too low */
        *hMecSpeedUnit = 0;
      }
      else
      {
        /* Check if speed is not to fast */
        if (  pHandle->AvrElSpeedDpp != HALL_MAX_PSEUDO_SPEED )
        {
          if (pHandle->HallMtpa == true)
          {
            pHandle->CompSpeed = 0;
          }
          else  
          {
            pHandle->DeltaAngle = pHandle->MeasuredElAngle - pHandle->_Super.hElAngle;
            pHandle->CompSpeed = ( int16_t )
            ( ( int32_t )( pHandle->DeltaAngle ) /
              ( int32_t )( pHandle->PWMNbrPSamplingFreq ) );
          }
          /* Convert el_dpp to MecUnit */
          *hMecSpeedUnit = ( int16_t )( (  pHandle->AvrElSpeedDpp * 
                                        ( int32_t )pHandle->_Super.hMeasurementFrequency * (int32_t) SPEED_UNIT ) /
                                        (( int32_t ) pHandle->_Super.DPPConvFactor * ( int32_t )pHandle->_Super.bElToMecRatio ) );
        }
        else
        {
          *hMecSpeedUnit = ( int16_t )pHandle->SatSpeed;
        }
      }
    }
		bReliability = true;
    //bReliability = SPD_IsMecSpeedReliable( &pHandle->_Super, hMecSpeedUnit );
  }
  else
  {
    bReliability = false;
    pHandle->_Super.bSpeedErrorNumber = pHandle->_Super.bMaximumSpeedErrorsNumber;
    /* If speed is not reliable the El and Mec speed is set to 0 */
    pHandle->_Super.hElSpeedDpp = 0;
    *hMecSpeedUnit = 0;
  }

  pHandle->_Super.hAvrMecSpeedUnit = *hMecSpeedUnit;

  return ( bReliability );
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
* @brief  Example of private method of the class HALL to implement an MC IRQ function
*         to be called when TIMx capture event occurs
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval none
*/
void * HALL_TIMx_CC_IRQHandler( void * pHandleVoid , uint32_t * pCapture )
{
  HALL_Handle_t * pHandle = ( HALL_Handle_t * ) pHandleVoid;
  uint8_t bPrevHallState;
  int8_t PrevDirection;
  uint32_t wCaptBuf;
  uint32_t wHighSpeedCapture;
	bool bUnexpectedBehavior = false;
	bool bReliableDirectionChange = false;

  if ( pHandle->SensorIsReliable )
  {
    /* A capture event generated this interrupt */
    bPrevHallState = pHandle->HallState;
    PrevDirection = pHandle->Direction;

    if ( pHandle->SensorPlacement == DEGREES_120 )
    {
      pHandle->HallState  = (uint8_t) (R_BSP_PinRead(pHandle->H3PortPin) << 2 | R_BSP_PinRead(pHandle->H2PortPin) << 1 | R_BSP_PinRead(pHandle->H1PortPin)) ;
    }
    else
    {
			pHandle->HallState  = (uint8_t) ( (R_BSP_PinRead(pHandle->H2PortPin) ^ 1 ) << 2  |  R_BSP_PinRead(pHandle->H3PortPin) << 1  |  R_BSP_PinRead(pHandle->H1PortPin) );
    }

    switch ( pHandle->HallState )
    {
      case STATE_5:
        if ( bPrevHallState == STATE_4 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = pHandle->PhaseShift;
        }
        else if ( bPrevHallState == STATE_1 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_60_PHASE_SHIFT );
        }
        else
        {
					bUnexpectedBehavior = true;
					HALL_Init_Electrical_Angle( pHandle );
        }
        break;

      case STATE_1:
        if ( bPrevHallState == STATE_5 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = pHandle->PhaseShift + S16_60_PHASE_SHIFT;
        }
        else if ( bPrevHallState == STATE_3 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT );
        }
        else
        {
					bUnexpectedBehavior = true;
					HALL_Init_Electrical_Angle( pHandle );
        }
        break;

      case STATE_3:
        if ( bPrevHallState == STATE_1 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT );
        }
        else if ( bPrevHallState == STATE_2 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT +
                                                  S16_60_PHASE_SHIFT );
        }
        else
        {
					bUnexpectedBehavior = true;
					HALL_Init_Electrical_Angle( pHandle );
        }

        break;

      case STATE_2:
        if ( bPrevHallState == STATE_3 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT
                                                  + S16_60_PHASE_SHIFT );
        }
        else if ( bPrevHallState == STATE_6 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift - S16_120_PHASE_SHIFT );
        }
        else
        {
					bUnexpectedBehavior = true;
					HALL_Init_Electrical_Angle( pHandle );
        }
        break;

      case STATE_6:
        if ( bPrevHallState == STATE_2 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift - S16_120_PHASE_SHIFT );
        }
        else if ( bPrevHallState == STATE_4 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift - S16_60_PHASE_SHIFT );
        }
        else
        {
					bUnexpectedBehavior = true;
					HALL_Init_Electrical_Angle( pHandle );
        }
        break;

      case STATE_4:
        if ( bPrevHallState == STATE_6 )
        {
          pHandle->Direction = POSITIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift - S16_60_PHASE_SHIFT );
        }
        else if ( bPrevHallState == STATE_5 )
        {
          pHandle->Direction = NEGATIVE;
          pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift );
        }
        else
        {
					bUnexpectedBehavior = true;
					HALL_Init_Electrical_Angle( pHandle );
        }
        break;

      default:
				pHandle->HallState = bPrevHallState;
        /* Bad hall sensor configutarion so update the speed reliability */
        pHandle->SensorIsReliable = false;
				bUnexpectedBehavior = true;

        break;
    }
		
		
		if (pHandle->Direction != PrevDirection)
		{
			pHandle->DirectionChangeCounter++;
		}
		else
		{
			if (pHandle->DirectionChangeCounter != 0)
			{
				bReliableDirectionChange = true;
			}
			pHandle->DirectionChangeCounter = 0;
		}
		
		/* We need to check that the direction has not changed.
			 If it is the case, the sign of the current speed can be the opposite of the
			 average speed, and the average time can be close to 0 which lead to a 
			 computed speed close to the infinite, and bring instability. */
		if (bReliableDirectionChange)
		{
			/* Setting BufferFilled to 0 will prevent to compute the average speed based
			 on the SpeedPeriod buffer values */
			pHandle->BufferFilled = 0 ;
			pHandle->SpeedFIFOIdx = 0;
		}
		
		if ( !bUnexpectedBehavior && (pHandle->DirectionChangeCounter == 0 || pHandle->DirectionChangeCounter == 2) )
		{
			/* We need to check that the direction has not changed.
				 If it is the case, the sign of the current speed can be the opposite of the
				 average speed, and the average time can be close to 0 which lead to a 
				 computed speed close to the infinite, and bring instability. */
			if (pHandle->Direction != PrevDirection)
			{
				/* Setting BufferFilled to 0 will prevent to compute the average speed based
				 on the SpeedPeriod buffer values */
				pHandle->BufferFilled = 0 ;
				pHandle->SpeedFIFOIdx = 0;
			}

			if (pHandle->HallMtpa == true)
			{
				pHandle->_Super.hElAngle = pHandle->MeasuredElAngle;
			}
			else
			{
				/* Nothing to do */
			}

			/* Discard first capture */
			if ( pHandle->FirstCapt == 0u )
			{
				pHandle->FirstCapt++;
				// Capture the value of capture compare register  ----------------------------------------------------
				// No need to update here
			}
			else
			{
				/* used to validate the average speed measurement */
				if ( pHandle->BufferFilled < pHandle->SpeedBufferSize )
				{
					pHandle->BufferFilled++;
				}

				/* Store the latest speed acquisition */
				// Capture the value of capture compare register  -------------------------------------------------------------------------
				wHighSpeedCapture = *pCapture;
				wCaptBuf = ( uint32_t ) wHighSpeedCapture;
				// hPrscBuf =  LL_TIM_GetPrescaler ( TIMx ); prescaler variable

				/* Add the numbers of overflow to the counter */
				//wCaptBuf += ( uint32_t ) pHandle->OVFCounter * (pHandle->Period_PostMultiplier* 0x10000uL) ;

				if ( pHandle->OVFCounter != 0u )
				{
//					/* Adjust the capture using prescaler */
//					uint16_t hAux;
//					hAux = hPrscBuf + 1u;
//					wCaptBuf *= hAux;
						wCaptBuf += ( uint32_t ) pHandle->OVFCounter * (pHandle->Period_PostMultiplier* 0x10000uL) ;
					
					if ( pHandle->RatioInc )
					{
						pHandle->RatioInc = false;  /* Previous capture caused overflow */
						/* Don't change prescaler (delay due to preload/update mechanism) */
					}
					else
					{
						if ( pHandle->Period_PostMultiplier < pHandle->HALLMaxRatio ) /* Avoid OVF w/ very low freq */
						{
							//	pHandle->Period_PostMultiplier ++ ; /* To avoid OVF during speed decrease */
								R_GPT_PeriodSet(g_timer3.p_ctrl, (pHandle->Period_PostMultiplier*65536uL) );
							//  pHandle->RatioInc = true;   /* new prsc value updated at next capture only */
						}
					}
				}
				else
				{
					/* If prsc preload reduced in last capture, store current register + 1 */
					if ( pHandle->RatioDec ) /* and don't decrease it again */
					{
//						/* Adjust the capture using prescaler */
//						uint16_t hAux;
//						hAux = hPrscBuf + 2u;
//						wCaptBuf *= hAux;
							wCaptBuf += ( uint32_t ) pHandle->OVFCounter * (pHandle->Period_PostMultiplier * 0x10000uL) ;

						pHandle->RatioDec = false;
					}
					else  /* If prescaler was not modified on previous capture */
					{
						/* Adjust the capture using prescaler */
//						uint16_t hAux = hPrscBuf + 1u;
//						wCaptBuf *= hAux;
							wCaptBuf += ( uint32_t ) pHandle->OVFCounter * (pHandle->Period_PostMultiplier* 0x10000uL) ;

//						if ( wHighSpeedCapture < (pHandle->Period_PostMultiplier*LOW_RES_THRESHOLD) ) /* If capture range correct */
//						{
							if ( pHandle->Period_PostMultiplier > 0u ) /* or prescaler cannot be further reduced */
							{
									//	pHandle->Period_PostMultiplier -- ; /* Increase accuracy by decreasing prsc */
										R_GPT_PeriodSet(g_timer3.p_ctrl, (pHandle->Period_PostMultiplier*65536uL) );
								/* Avoid decrementing again in next capt.(register preload delay) */
								pHandle->RatioDec = true;
							}
						//}
					}
				}

				/* Filtering to fast speed... could be a glitch  ? */
				/* the HALL_MAX_PSEUDO_SPEED is temporary in the buffer, and never included in average computation*/
					if ( wCaptBuf < pHandle->MinPeriod )
					{
						 /* pHandle->AvrElSpeedDpp = HALL_MAX_PSEUDO_SPEED; */
					}
					else
					{
						pHandle->ElPeriodSum -= pHandle->SensorPeriod[pHandle->SpeedFIFOIdx]; /* value we gonna removed from the accumulator */
						if ( wCaptBuf >= pHandle->MaxPeriod )
						{
							pHandle->SensorPeriod[pHandle->SpeedFIFOIdx] =  pHandle->MaxPeriod;//*pHandle->Direction; 
						}
						else
						{
							pHandle->SensorPeriod[pHandle->SpeedFIFOIdx] =  wCaptBuf ;
							// pHandle->SensorPeriod[pHandle->SpeedFIFOIdx] *= pHandle->Direction;
							pHandle->ElPeriodSum += pHandle->SensorPeriod[pHandle->SpeedFIFOIdx];
						}
						/* Update pointers to speed buffer */
						pHandle->SpeedFIFOIdx++;
						if ( pHandle->SpeedFIFOIdx == pHandle->SpeedBufferSize )
						{
							pHandle->SpeedFIFOIdx = 0u;
						}
						if ( pHandle->SensorIsReliable) 
						{
//							if ( pHandle->BufferFilled < pHandle->SpeedBufferSize )
//							{
//								pHandle->AvrElSpeedDpp = ( int16_t ) ( pHandle->PseudoFreqConv / wCaptBuf )*pHandle->Direction;
//							}
//							else 
						//	{ /* Average speed allow to smooth the mechanical sensors misalignement */
								pHandle->AvrElSpeedDpp =  ( int16_t ) (pHandle->PseudoFreqConv / (pHandle->ElPeriodSum / pHandle->SpeedBufferSize )) ; /* Average value */
								pHandle->AvrElSpeedDpp =  pHandle->AvrElSpeedDpp * pHandle->Direction;
						//	}
						}
						else /* Sensor is not reliable */
						{
							pHandle->AvrElSpeedDpp = 0;
						}
					}
				/* Reset the number of overflow occurred */
				pHandle->OVFCounter = 0u;
			}
		}
  }
	
	if ( pHandle->BufferFilled < pHandle->SpeedBufferSize )
	{
		HALL_Init_Electrical_Angle(pHandle);
	}	
	
  return (void *)(0x0);
}

#if defined (CCMRAM)
#if defined (__ICCARM__)
#pragma location = ".ccmram"
#elif defined (__CC_ARM) || defined(__GNUC__)
__attribute__( ( section ( ".ccmram" ) ) )
#endif
#endif
/**
* @brief  Example of private method of the class HALL to implement an MC IRQ function
*         to be called when TIMx update event occurs
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval none
*/
void * HALL_TIMx_UP_IRQHandler( void * pHandleVoid )
{
  HALL_Handle_t * pHandle = ( HALL_Handle_t * ) pHandleVoid;
  
	
  if ( pHandle->SensorIsReliable )
  {
    uint16_t hMaxTimerOverflow;
    /* an update event occured for this interrupt request generation */
    pHandle->OVFCounter++;
    hMaxTimerOverflow =  pHandle->HALLMaxRatio / (uint16_t) pHandle->Period_PostMultiplier;
    if ( pHandle->OVFCounter >= hMaxTimerOverflow )
    {
      /* Set rotor speed to zero */
      pHandle->_Super.hElSpeedDpp = 0;

      /* Reset the electrical angle according the hall sensor configuration */
      HALL_Init_Electrical_Angle( pHandle );

      /* Reset the overflow counter */
      pHandle->OVFCounter = 0u;

      /* Reset first capture flag */
      pHandle->FirstCapt = 0u;

      /* Reset the SensorSpeed buffer*/
      uint8_t bIndex;
      for ( bIndex = 0u; bIndex < pHandle->SpeedBufferSize; bIndex++ )
      {
        pHandle->SensorPeriod[bIndex]  = pHandle->MaxPeriod;//(int32_t) pHandle->MaxPeriod;
      }
      pHandle->BufferFilled = 0 ;
      pHandle->AvrElSpeedDpp = 0;
      pHandle->SpeedFIFOIdx = 0;
      pHandle->ElPeriodSum = pHandle->MaxPeriod * pHandle->SpeedBufferSize; //(int32_t) (pHandle->MaxPeriod * pHandle->SpeedBufferSize);
    }
  }
  return (void *)(0x0);				// MC_NULL
}

/**
* @brief  Read the logic level of the three Hall sensor and individuates in this
*         way the position of the rotor (+/- 30???). Electrical angle is then
*         initialized.
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval none
*/
static void HALL_Init_Electrical_Angle( HALL_Handle_t * pHandle )
{

  if ( pHandle->SensorPlacement == DEGREES_120 )
  {
		pHandle->HallState  = (uint8_t) ( R_BSP_PinRead(pHandle->H3PortPin) << 2 | R_BSP_PinRead(pHandle->H2PortPin) << 1 | R_BSP_PinRead(pHandle->H1PortPin) );
  }
  else
  {
		pHandle->HallState  = (uint8_t) ( (R_BSP_PinRead(pHandle->H2PortPin) ^ 1 ) << 2  |  R_BSP_PinRead(pHandle->H3PortPin) << 1  |  R_BSP_PinRead(pHandle->H1PortPin) );
  }

  switch ( pHandle->HallState )
  {
    case STATE_5:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift + S16_60_PHASE_SHIFT / 2 );
      break;
    case STATE_1:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift + S16_60_PHASE_SHIFT +
                                              S16_60_PHASE_SHIFT / 2 );
      break;
    case STATE_3:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT +
                                              S16_60_PHASE_SHIFT / 2 );
      break;
    case STATE_2:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift - S16_120_PHASE_SHIFT -
                                              S16_60_PHASE_SHIFT / 2 );
      break;
    case STATE_6:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift - S16_60_PHASE_SHIFT -
                                              S16_60_PHASE_SHIFT / 2 );
      break;
    case STATE_4:
      pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift - S16_60_PHASE_SHIFT / 2 );
      break;
    default:
      /* Bad hall sensor configutarion so update the speed reliability */
      pHandle->SensorIsReliable = false;
      break;
  }

  /* Initialize the measured angle */
  pHandle->MeasuredElAngle = pHandle->_Super.hElAngle;
	
	pHandle->DirectionChangeCounter = 0;
}

/**
  * @brief  It could be used to set istantaneous information on rotor mechanical
  *         angle.
  *         Note: Mechanical angle management is not implemented in this
  *         version of Hall sensor class.
  * @param  pHandle pointer on related component instance
  * @param  hMecAngle istantaneous measure of rotor mechanical angle
  * @retval none
  */
void HALL_SetMecAngle( HALL_Handle_t * pHandle, int16_t hMecAngle )
{
	if(pHandle!= NULL)
	{
		hMecAngle = 0;
	}
	
	
}


