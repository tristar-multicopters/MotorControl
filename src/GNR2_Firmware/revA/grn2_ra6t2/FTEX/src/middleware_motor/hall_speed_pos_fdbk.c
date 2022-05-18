 /**
  ******************************************************************************
  * @file    hall_speed_pos_fdbk.c
  * @author  FTEX inc
  * @brief   This file provides firmware functions that implement the features of
  *          the Hall Speed & Position Feedback component of the Motor Control SDK.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "speed_pos_fdbk.h"
#include "hall_speed_pos_fdbk.h"
#include "mc_type.h"
#include "stdlib.h"

/* Private defines -----------------------------------------------------------*/

/* Lower threshold to reques a decrease of clock prescaler */
#define LOW_RES_THRESHOLD   ((uint32_t)0x5500u)

#define HALL_COUNTER_RESET  ((uint16_t) 0u)

#define S16_120_PHASE_SHIFT (int16_t)(65536/3)
#define S16_60_PHASE_SHIFT  (int16_t)(65536/6)
#define S16_30_PHASE_SHIFT  (int16_t)(65536/12)
#define S16_40_PHASE_SHIFT  (int16_t)((40*65536)/360)

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

static void HALL_Init_Electrical_Angle( HALL_Handle_t * pHandle );

/**
  * @brief  It initializes the hardware peripherals (Timer, GPIO and NVIC)
            required for the speed position sensor management using HALL
            sensors.
  * @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
  * @retval none
  */
	
void HALL_Init( HALL_Handle_t * pHandle )
{
    // Calculate loacl variables and HALL handle variables
    uint16_t hMinReliableElSpeedUnit = pHandle->_Super.hMinReliableMecSpeedUnit * pHandle->_Super.bElToMecRatio;
    uint16_t hMaxReliableElSpeedUnit = pHandle->_Super.hMaxReliableMecSpeedUnit * pHandle->_Super.bElToMecRatio;
    uint8_t bSpeedBufferSize;
    uint8_t bIndex;
    /* Adjustment factor: minimum measurable speed is x time less than the minimum reliable speed */
    hMinReliableElSpeedUnit /= 4u;
    /* Adjustment factor: maximum measurable speed is x time greater than the maximum reliable speed */
    hMaxReliableElSpeedUnit *= 2u;
    pHandle->OvfFreq = ( uint16_t )( pHandle->TIMClockFreq / 524287u );
    /* Initialize hall-timeout so that it can be used for further calculations */
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
    pHandle->MaxPeriod = ( pHandle->HALLMaxRatio ) * 524287uL;
    pHandle->MaxElSum = pHandle->MaxPeriod * pHandle->SpeedBufferSize;
    pHandle->SatSpeed = hMaxReliableElSpeedUnit;
    pHandle->PseudoFreqConv = ( ( pHandle->TIMClockFreq/6u)/( pHandle->_Super.hMeasurementFrequency ) ) * ( pHandle->_Super.DPPConvFactor);
    pHandle->MinPeriod = ( ( SPEED_UNIT * ( pHandle->TIMClockFreq /6uL) ) / hMaxReliableElSpeedUnit);
    pHandle->PWMNbrPSamplingFreq = ( (pHandle->_Super.hMeasurementFrequency * pHandle->PWMFreqScaling) / pHandle->SpeedSamplingFreqHz ) - 1u;
    /* Reset speed reliability */
    pHandle->SensorIsReliable = true;
    /* Erase speed buffer */
    bSpeedBufferSize = pHandle->SpeedBufferSize;
    for ( bIndex = 0u; bIndex < bSpeedBufferSize; bIndex++ )
    {
        pHandle->SensorPeriod[bIndex]  = pHandle->MaxPeriod;
    }
    /* Initialize Array used for angle confinement */
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

void HALL_Clear( HALL_Handle_t * pHandle )
{
    /* Disable timer0 interrupts and counter */
    R_GPT_Disable(pHandle->TIMx->p_ctrl);
    R_GPT_Stop(pHandle->TIMx->p_ctrl);
    /*   Clear the timer for reinitialization  */
    R_GPT_CounterSet(pHandle->TIMx->p_ctrl,HALL_COUNTER_RESET);
    /* Reinitialize variables in HALL handle */
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
    HALL_Init_Electrical_Angle( pHandle );
    /*  Enable timer0 interrupts and counter */
    R_GPT_Start(pHandle->TIMx->p_ctrl);
    R_GPT_Enable(pHandle->TIMx->p_ctrl);
}

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
        /*      Calculate angle by adding electrical speed in Dpp      */
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
        /*      Confine angle calculation to 60-degrees      */
		int32_t hAngle_Diff = abs(pHandle->_Super.hElAngle - pHandle->Sector_Middle_Angle[pHandle->HallState] ); 
		if((hAngle_Diff>S16_40_PHASE_SHIFT))
		{
            uint16_t hAngle_Diffu = ((uint16_t) pHandle->_Super.hElAngle) - ((uint16_t) pHandle->Sector_Middle_Angle[pHandle->HallState]);			
			if(abs((int16_t)hAngle_Diffu)>S16_40_PHASE_SHIFT)
            {
                if(pHandle->Direction == POSITIVE)
                {
                    pHandle->_Super.hElAngle = pHandle->Sector_Destination_Angle[pHandle->HallState];
                }
                else
                {
                    pHandle->_Super.hElAngle = pHandle->Sector_Start_Angle[pHandle->HallState];
                }
            }
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
    if (pHandle->SensorIsReliable)
    {
        // Compare sum of the period with maximum permissible period
        if(pHandle->ElPeriodSum >= pHandle->MaxElSum)
        {
            /* At start-up or very low freq */
            /* Based on current prescaler value only */
            pHandle->_Super.hElSpeedDpp = 0;
            *hMecSpeedUnit = 0;
        }
        else
        {
            pHandle->_Super.hElSpeedDpp =  pHandle->AvrElSpeedDpp;
            if ( pHandle->AvrElSpeedDpp == 0)
            {
                /* Speed is too low */
                *hMecSpeedUnit = 0;
            }
            else
            {
                /* Check if speed is not to fast */
                if (pHandle->AvrElSpeedDpp != HALL_MAX_PSEUDO_SPEED)
                {
                    if (pHandle->HallMtpa == true)
                    {
                        pHandle->CompSpeed = 0;
                    }
                    else  
                    {
                        pHandle->DeltaAngle = pHandle->MeasuredElAngle - pHandle->_Super.hElAngle;
                        pHandle->CompSpeed = (int16_t)((int32_t)(pHandle->DeltaAngle)/( int32_t )( pHandle->PWMNbrPSamplingFreq ) );
                    }
                    /* Convert el_dpp to MecUnit */
                    *hMecSpeedUnit = ( int16_t )( (  pHandle->AvrElSpeedDpp * ( int32_t )pHandle->_Super.hMeasurementFrequency * (int32_t) SPEED_UNIT ) /
                                                (( int32_t ) pHandle->_Super.DPPConvFactor * ( int32_t )pHandle->_Super.bElToMecRatio ) );
                }
                else
                {
                    *hMecSpeedUnit = ( int16_t )pHandle->SatSpeed;
                }
            }
        }
        bReliability = true;
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
    return (bReliability);
}

void * HALL_TIMx_CC_IRQHandler( void * pHandleVoid , uint32_t * pCapture )
{
    HALL_Handle_t * pHandle = ( HALL_Handle_t * ) pHandleVoid;
    uint8_t bPrevHallState;
    int8_t PrevDirection;
    uint32_t wCaptBuf;
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
                    pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT );
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
                    pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT );//
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
                    pHandle->MeasuredElAngle = ( int16_t )( pHandle->PhaseShift - S16_60_PHASE_SHIFT );//
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
                wCaptBuf = *pCapture;
                /* Add the numbers of overflow to the counter */
                if ( pHandle->OVFCounter != 0u )
                {
                    wCaptBuf += (uint32_t) pHandle->OVFCounter*0x80000uL;
                }
                else
                {
                    // Do nothing here
                }
                /* the HALL_MAX_PSEUDO_SPEED is temporary in the buffer, and never included in average computation*/
                if ( wCaptBuf < pHandle->MinPeriod )
                {
                    /* Commented next line to prevent false drequency injection when moving from stop to moving condition */
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
                        pHandle->SensorPeriod[pHandle->SpeedFIFOIdx] =  wCaptBuf;
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
                        //	if ( pHandle->BufferFilled < pHandle->SpeedBufferSize )
                        //	{
                        //	    pHandle->AvrElSpeedDpp = ( int16_t ) ( pHandle->PseudoFreqConv / wCaptBuf )*pHandle->Direction;
                        //	}
                        //	else 
                        // { /* Average speed allow to smooth the mechanical sensors misalignement */
                        pHandle->AvrElSpeedDpp =  (int16_t)(pHandle->PseudoFreqConv / (pHandle->ElPeriodSum / pHandle->SpeedBufferSize )) ; /* Average value */
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

/**
* @brief  Example of private method of the class HALL to implement an MC IRQ function
*         to be called when TIMx update event occurs
* @param  pHandle: handler of the current instance of the hall_speed_pos_fdbk component
* @retval none
*/
void * HALL_TIMx_UP_IRQHandler( void * pHandleVoid )
{
    HALL_Handle_t * pHandle = ( HALL_Handle_t * ) pHandleVoid;
    if (pHandle->SensorIsReliable)
    {
        /* an update event occured for this interrupt request generation */
        pHandle->OVFCounter++;
        if (pHandle->OVFCounter >= (pHandle->HALLMaxRatio))
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
                pHandle->SensorPeriod[bIndex]  = pHandle->MaxPeriod;
            }
            pHandle->BufferFilled = 0 ;
            pHandle->AvrElSpeedDpp = 0;
            pHandle->SpeedFIFOIdx = 0;
            pHandle->ElPeriodSum = pHandle->MaxPeriod * pHandle->SpeedBufferSize;
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
    if (pHandle->SensorPlacement == DEGREES_120)
    {
        pHandle->HallState  = (uint8_t) ( R_BSP_PinRead(pHandle->H3PortPin) << 2 | R_BSP_PinRead(pHandle->H2PortPin) << 1 | R_BSP_PinRead(pHandle->H1PortPin) );
    }
    else
    {
        pHandle->HallState  = (uint8_t) ( (R_BSP_PinRead(pHandle->H2PortPin) ^ 1 ) << 2  |  R_BSP_PinRead(pHandle->H3PortPin) << 1  |  R_BSP_PinRead(pHandle->H1PortPin) );
    }
    /* Set angle based on hall sensor states */
    switch (pHandle->HallState)
    {
        case STATE_5:
            pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift + S16_60_PHASE_SHIFT / 2 );
            break;
        case STATE_1:
            pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift + S16_60_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2 );
            break;
        case STATE_3:
            pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift + S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2 );
            break;
        case STATE_2:
            pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift - S16_120_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2 );
            break;
        case STATE_6:
            pHandle->_Super.hElAngle = ( int16_t )( pHandle->PhaseShift - S16_60_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2 );
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
