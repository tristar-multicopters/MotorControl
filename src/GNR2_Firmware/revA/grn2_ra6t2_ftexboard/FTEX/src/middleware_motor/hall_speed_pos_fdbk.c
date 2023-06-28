 /**
  * @file    hall_speed_pos_fdbk.c
  * @brief   This file provides firmware functions that implement the features of
  *          the Hall Speed & Position Feedback component.
*/

#include "speed_pos_fdbk.h"
#include "hall_speed_pos_fdbk.h"
#include "mc_type.h"
#include "stdlib.h"

// ========================== Private defines ============================== //

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

#define HALL_MAX_PSEUDO_SPEED        ((int16_t)0x7FFF)  // With digit-per-PWM unit (here 2*PI rad = 0xFFFF)

static void HALL_Init_Electrical_Angle(HallPosSensorHandle_t * pHandle);


void HallPosSensor_Init(HallPosSensorHandle_t * pHandle)
{
    // Calculate loacl variables and HALL handle variables
    uint16_t hMinReliableElSpeedUnit = pHandle->Super.hMinReliableMecSpeedUnit * pHandle->Super.bElToMecRatio;
    uint16_t hMaxReliableElSpeedUnit = pHandle->Super.hMaxReliableMecSpeedUnit * pHandle->Super.bElToMecRatio;
    uint8_t bSpeedBufferSize;
    uint8_t bIndex;
    /* Adjustment factor: minimum measurable speed is x time less than the minimum reliable speed */
    hMinReliableElSpeedUnit /= 4u;
    /* Adjustment factor: maximum measurable speed is x time greater than the maximum reliable speed */
    hMaxReliableElSpeedUnit *= 2u;
    pHandle->hOvfFreq = (uint16_t)(pHandle->wTIMClockFreq / 524287u);
    /* Initialize hall-timeout so that it can be used for further calculations */
    if (hMinReliableElSpeedUnit == 0u)
    {
        /* Set fixed to 150 ms */
        pHandle->hHallTimeout = 150u;
    }
    else
    {
        /* Set accordingly the min reliable speed */
        /* 1000 comes from mS
        * 6 comes from the fact that sensors are toggling each 60 deg = 360/6 deg */
        pHandle->hHallTimeout = 1000*SPEED_UNIT / (6u * hMinReliableElSpeedUnit);
    }
    /* Compute the prescaler to the closet value of the TimeOut (in mS)*/
    pHandle->hHallMaxRatio = (pHandle->hHallTimeout * pHandle->hOvfFreq) / 1000 ;					// prescaler so that counter counts 65536 in desired timeout value
    /* Align wMaxPeriod to a multiple of Overflow.*/
    pHandle->wMaxPeriod = (pHandle->hHallMaxRatio) * 524287uL;
    pHandle->wMaxElSum = pHandle->wMaxPeriod * pHandle->bSpeedBufferSize;
    pHandle->hSaturationSpeed = hMaxReliableElSpeedUnit;
    pHandle->wPseudoFreqConv = ((pHandle->wTIMClockFreq/6u)/(pHandle->Super.hMeasurementFrequency)) * (pHandle->Super.DPPConvFactor);
    pHandle->wMinPeriod = ((SPEED_UNIT * (pHandle->wTIMClockFreq /6uL)) / hMaxReliableElSpeedUnit);
    pHandle->hPWMNbrPSamplingFreq = ((pHandle->Super.hMeasurementFrequency * pHandle->bPWMFreqScaling) / pHandle->hSpeedSamplingFreqHz) - 1u;
    /* Reset speed reliability */
    pHandle->bSensorIsReliable = true;
    /* Erase speed buffer */
    bSpeedBufferSize = pHandle->bSpeedBufferSize;
    for (bIndex = 0u; bIndex < bSpeedBufferSize; bIndex++)
    {
        pHandle->SensorPeriod[bIndex]  = pHandle->wMaxPeriod;
    }
    /* Initialize Array used for angle confinement */
    pHandle->SectorStartAngle[0] = 0;
    pHandle->SectorStartAngle[1] = (int16_t) (pHandle->hPhaseShift + S16_60_PHASE_SHIFT);
    pHandle->SectorStartAngle[2] = (int16_t) (pHandle->hPhaseShift + S16_60_PHASE_SHIFT + S16_120_PHASE_SHIFT);
    pHandle->SectorStartAngle[3] = (int16_t) (pHandle->hPhaseShift + S16_120_PHASE_SHIFT);
    pHandle->SectorStartAngle[4] = (int16_t) (pHandle->hPhaseShift - S16_60_PHASE_SHIFT);
    pHandle->SectorStartAngle[5] = (int16_t) (pHandle->hPhaseShift);
    pHandle->SectorStartAngle[6] = (int16_t) (pHandle->hPhaseShift - S16_120_PHASE_SHIFT);
    pHandle->SectorDestinationAngle[0] = 0;
    pHandle->SectorDestinationAngle[1] = (int16_t) (pHandle->hPhaseShift  + S16_120_PHASE_SHIFT) ;
    pHandle->SectorDestinationAngle[2] = (int16_t) (pHandle->hPhaseShift - S16_120_PHASE_SHIFT);
    pHandle->SectorDestinationAngle[3] = (int16_t) (pHandle->hPhaseShift + S16_60_PHASE_SHIFT + S16_120_PHASE_SHIFT) ;
    pHandle->SectorDestinationAngle[4] = (int16_t) (pHandle->hPhaseShift);
    pHandle->SectorDestinationAngle[5] = (int16_t) (pHandle->hPhaseShift  + S16_60_PHASE_SHIFT);
    pHandle->SectorDestinationAngle[6] = (int16_t) (pHandle->hPhaseShift - S16_60_PHASE_SHIFT);
    pHandle->SectorMiddleAngle[0] = 0;
    pHandle->SectorMiddleAngle[1] = (int16_t) (pHandle->SectorStartAngle[1]  +  S16_30_PHASE_SHIFT);
    pHandle->SectorMiddleAngle[2] = (int16_t) (pHandle->SectorStartAngle[2]  +  S16_30_PHASE_SHIFT);
    pHandle->SectorMiddleAngle[3] = (int16_t)  (pHandle->SectorStartAngle[3]  +  S16_30_PHASE_SHIFT);
    pHandle->SectorMiddleAngle[4] = (int16_t) (pHandle->SectorStartAngle[4]  +  S16_30_PHASE_SHIFT);
    pHandle->SectorMiddleAngle[5] = (int16_t) (pHandle->SectorStartAngle[5]  +  S16_30_PHASE_SHIFT);
    pHandle->SectorMiddleAngle[6] = (int16_t) (pHandle->SectorStartAngle[6]  +  S16_30_PHASE_SHIFT);
    
    SignalFiltering_Init(&pHandle->SpeedFilter);
    SignalFiltering_ConfigureButterworthFOLP(&pHandle->SpeedFilter,
                                                pHandle->fFilterAlpha,
                                                    pHandle->fFilterBeta);
    
    /*  Enable timer0 interrupts and counter */
    R_GPT_Start(pHandle->TIMx->p_ctrl);
    R_GPT_Enable(pHandle->TIMx->p_ctrl);
}

void HallPosSensor_Clear(HallPosSensorHandle_t * pHandle)
{
    SignalFiltering_Clear(&pHandle->SpeedFilter);
    /* Disable timer0 interrupts and counter */
    R_GPT_Disable(pHandle->TIMx->p_ctrl);
    R_GPT_Stop(pHandle->TIMx->p_ctrl);
    /*   Clear the timer for reinitialization  */
    R_GPT_CounterSet(pHandle->TIMx->p_ctrl,HALL_COUNTER_RESET);
    /* Reinitialize variables in HALL handle */
    /* Reset speed reliability */
    pHandle->bSensorIsReliable = true;
    /* Acceleration measurement not implemented.*/
    pHandle->Super.hMecAccelUnitP = 0;
    pHandle->bFirstCapt = 0u;
    pHandle->bBufferFilled = 0u;
    pHandle->hOVFCounter = 0u;
    pHandle->hCompSpeed = 0;
    pHandle->bDirection = POSITIVE;
    pHandle->hFiltElSpeedDpp = 0;
    /* Initialize speed buffer index */
    pHandle->bSensorPeriod = 0u;
    /* Clear speed error counter */
    pHandle->Super.bSpeedErrorNumber = 0;
    HALL_Init_Electrical_Angle(pHandle);
    /*  Enable timer0 interrupts and counter */
    R_GPT_Start(pHandle->TIMx->p_ctrl);
    R_GPT_Enable(pHandle->TIMx->p_ctrl);
}


int16_t HallPosSensor_CalcElAngle(HallPosSensorHandle_t * pHandle)
{
    //HALL_Init_Electrical_Angle(pHandle);
    
    if (pHandle->bBufferFilled < pHandle->bSpeedBufferSize)
    {
        HALL_Init_Electrical_Angle(pHandle);
    }
    else
    {
        /*      Calculate angle by adding electrical speed in Dpp      */
        if (pHandle->Super.hElSpeedDpp != HALL_MAX_PSEUDO_SPEED)
        {
            pHandle->hMeasuredElAngle += pHandle->Super.hElSpeedDpp;
            pHandle->Super.hElAngle += pHandle->Super.hElSpeedDpp + pHandle->hCompSpeed;
            pHandle->hPrevRotorFreq = pHandle->Super.hElSpeedDpp;
        }
        else
        {
            pHandle->Super.hElAngle += pHandle->hPrevRotorFreq;
        }
        /* Confine angle calculation to 60-degrees */
		int32_t hAngle_Diff = abs(pHandle->Super.hElAngle - pHandle->SectorMiddleAngle[pHandle->bHallState]);
		if((hAngle_Diff>S16_40_PHASE_SHIFT))
		{
            uint16_t hAngle_Diffu = ((uint16_t) pHandle->Super.hElAngle) - ((uint16_t) pHandle->SectorMiddleAngle[pHandle->bHallState]);
			if(abs((int16_t)hAngle_Diffu)>S16_40_PHASE_SHIFT)
            {
                if(pHandle->bDirection == POSITIVE)
                {
                    pHandle->Super.hElAngle = pHandle->SectorDestinationAngle[pHandle->bHallState];
                }
                else
                {
                    pHandle->Super.hElAngle = pHandle->SectorStartAngle[pHandle->bHallState];
                }
            }
		}
    }
    return pHandle->Super.hElAngle;
}


bool HallPosSensor_CalcAvrgMecSpeedUnit(HallPosSensorHandle_t * pHandle, int16_t * pMecSpeedUnit)
{
    bool bReliability;
    
    //pHandle->hFiltElSpeedDpp = SignalFiltering_CalcOutputI16(&pHandle->SpeedFilter, pHandle->hAvrElSpeedDpp);
    pHandle->hFiltElSpeedDpp = pHandle->hAvrElSpeedDpp;
    
    if (pHandle->bSensorIsReliable)
    {
        // Compare sum of the period with maximum permissible period
        if(pHandle->wElPeriodSum >= pHandle->wMaxElSum)
        {
            /* At start-up or very low freq */
            /* Based on current prescaler value only */
            pHandle->Super.hElSpeedDpp = 0;
            *pMecSpeedUnit = 0;
        }
        else
        {
            pHandle->Super.hElSpeedDpp =  pHandle->hFiltElSpeedDpp;
            if (pHandle->hAvrElSpeedDpp == 0)
            {
                /* Speed is too low */
                *pMecSpeedUnit = 0;
            }
            else
            {
                /* Check if speed is not to fast */
                if (pHandle->hAvrElSpeedDpp != HALL_MAX_PSEUDO_SPEED)
                {
                    if (pHandle->bHallMtpa == true)
                    {
                        pHandle->hCompSpeed = 0;
                    }
                    else
                    {
                        pHandle->hDeltaAngle = pHandle->hMeasuredElAngle - pHandle->Super.hElAngle;
                        pHandle->hCompSpeed = (int16_t)((int32_t)(pHandle->hDeltaAngle)/(int32_t)(pHandle->hPWMNbrPSamplingFreq));
                    }
                    /* Convert el_dpp to MecUnit */
                    *pMecSpeedUnit = (int16_t)(( pHandle->hFiltElSpeedDpp * (int32_t)pHandle->Super.hMeasurementFrequency * (int32_t) SPEED_UNIT) /
                                                ((int32_t) pHandle->Super.DPPConvFactor * (int32_t)pHandle->Super.bElToMecRatio));
                }
                else
                {
                    *pMecSpeedUnit = (int16_t)pHandle->hSaturationSpeed;
                }
            }
        }
        bReliability = true;
    }
    else
    {
        bReliability = false;
        pHandle->Super.bSpeedErrorNumber = pHandle->Super.bMaximumSpeedErrorsNumber;
        /* If speed is not reliable the El and Mec speed is set to 0 */
        pHandle->Super.hElSpeedDpp = 0;
        *pMecSpeedUnit = 0;
    }
    pHandle->Super.hAvrMecSpeedUnit = *pMecSpeedUnit;
    return (bReliability);
}

void * HallPosSensor_TIMx_CC_IRQHandler(void * pHandleVoid , uint32_t * pCapture)
{
    HallPosSensorHandle_t * pHandle = (HallPosSensorHandle_t *) pHandleVoid;
    uint8_t bPrevHallState;
    int8_t PrevDirection;
    uint32_t wCaptBuf;
    bool bUnexpectedBehavior = false;
    bool bReliableDirectionChange = false;

    if (pHandle->bSensorIsReliable)
    {
        /* A capture event generated this interrupt */
        bPrevHallState = pHandle->bHallState;
        PrevDirection = pHandle->bDirection;
        if (pHandle->bSensorPlacement == DEGREES_120)
        {
            pHandle->bHallState  = (uint8_t) (R_BSP_PinRead(pHandle->H3PortPin) << 2 | R_BSP_PinRead(pHandle->H2PortPin) << 1 | R_BSP_PinRead(pHandle->H1PortPin)) ;
        }
        else
        {
            pHandle->bHallState  = (uint8_t) ((R_BSP_PinRead(pHandle->H2PortPin) ^ 1) << 2  |  R_BSP_PinRead(pHandle->H3PortPin) << 1  |  R_BSP_PinRead(pHandle->H1PortPin));
        }
        switch (pHandle->bHallState)
        {
            case STATE_5:
                if (bPrevHallState == STATE_4)
                {
                    pHandle->bDirection = POSITIVE;
                    pHandle->wDirectionChangePattern = (pHandle->wDirectionChangePattern << 1) | 1; //shift left with 1  to record POSITIVE direction in the history
                    pHandle->hMeasuredElAngle = pHandle->hPhaseShift;
                }
                else if (bPrevHallState == STATE_1)
                {
                    pHandle->bDirection = NEGATIVE;
                    pHandle->wDirectionChangePattern = (pHandle->wDirectionChangePattern << 1); //shift left with 0 to record NEGATIVE direction in the history
                    pHandle->hMeasuredElAngle = (int16_t)(pHandle->hPhaseShift + S16_60_PHASE_SHIFT);
                }
                else
                {
                    bUnexpectedBehavior = true;
                    HALL_Init_Electrical_Angle(pHandle);
                }
                break;
            case STATE_1:
                if (bPrevHallState == STATE_5)
                {
                    pHandle->bDirection = POSITIVE;
                    pHandle->wDirectionChangePattern = (pHandle->wDirectionChangePattern << 1) | 1; //shift left with 1  to record POSITIVE direction in the history 
                    pHandle->hMeasuredElAngle = pHandle->hPhaseShift + S16_60_PHASE_SHIFT;
                }
                else if (bPrevHallState == STATE_3)
                {
                    pHandle->bDirection = NEGATIVE;
                    pHandle->wDirectionChangePattern = (pHandle->wDirectionChangePattern << 1); //shift left with 0 to record NEGATIVE direction in the history
                    pHandle->hMeasuredElAngle = (int16_t)(pHandle->hPhaseShift + S16_120_PHASE_SHIFT);
                }
                else
                {
                    bUnexpectedBehavior = true;
                    HALL_Init_Electrical_Angle(pHandle);
                }
                break;
            case STATE_3:
                if (bPrevHallState == STATE_1)
                {
                    pHandle->bDirection = POSITIVE;
                    pHandle->wDirectionChangePattern = (pHandle->wDirectionChangePattern << 1) | 1; //shift left with 1  to record POSITIVE direction in the history
                    pHandle->hMeasuredElAngle = (int16_t)(pHandle->hPhaseShift + S16_120_PHASE_SHIFT);
                }
                else if (bPrevHallState == STATE_2)
                {
                    pHandle->bDirection = NEGATIVE;
                    pHandle->wDirectionChangePattern = (pHandle->wDirectionChangePattern << 1); //shift left with 0 to record NEGATIVE direction in the history
                    pHandle->hMeasuredElAngle = (int16_t)(pHandle->hPhaseShift + S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT);
                }
                else
                {
                    bUnexpectedBehavior = true;
                    HALL_Init_Electrical_Angle(pHandle);
                }
                break;
            case STATE_2:
                if (bPrevHallState == STATE_3)
                {
                    pHandle->bDirection = POSITIVE;
                    pHandle->wDirectionChangePattern = (pHandle->wDirectionChangePattern << 1) | 1; //shift left with 1  to record POSITIVE direction in the history
                    pHandle->hMeasuredElAngle = (int16_t)(pHandle->hPhaseShift + S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT);//
                }
                else if (bPrevHallState == STATE_6)
                {
                    pHandle->bDirection = NEGATIVE;
                    pHandle->wDirectionChangePattern = (pHandle->wDirectionChangePattern << 1); //shift left with 0 to record NEGATIVE direction in the history
                    pHandle->hMeasuredElAngle = (int16_t)(pHandle->hPhaseShift - S16_120_PHASE_SHIFT);
                }
                else
                {
                    bUnexpectedBehavior = true;
                    HALL_Init_Electrical_Angle(pHandle);
                }
                break;
            case STATE_6:
                if (bPrevHallState == STATE_2)
                {
                    pHandle->bDirection = POSITIVE;
                    pHandle->wDirectionChangePattern = (pHandle->wDirectionChangePattern << 1) | 1; //shift left with 1  to record POSITIVE direction in the history
                    pHandle->hMeasuredElAngle = (int16_t)(pHandle->hPhaseShift - S16_120_PHASE_SHIFT);
                }
                else if (bPrevHallState == STATE_4)
                {
                    pHandle->bDirection = NEGATIVE;
                    pHandle->wDirectionChangePattern = (pHandle->wDirectionChangePattern << 1); //shift left with 0 to record NEGATIVE direction in the history
                    pHandle->hMeasuredElAngle = (int16_t)(pHandle->hPhaseShift - S16_60_PHASE_SHIFT);
                }
                else
                {
                    bUnexpectedBehavior = true;
                    HALL_Init_Electrical_Angle(pHandle);
                }
                break;
            case STATE_4:
                if (bPrevHallState == STATE_6)
                {
                    pHandle->bDirection = POSITIVE;
                    pHandle->wDirectionChangePattern = (pHandle->wDirectionChangePattern << 1) | 1; //shift left with 1  to record POSITIVE direction in the history
                    pHandle->hMeasuredElAngle = (int16_t)(pHandle->hPhaseShift - S16_60_PHASE_SHIFT);//
                }
                else if (bPrevHallState == STATE_5)
                {
                    pHandle->bDirection = NEGATIVE;
                    pHandle->wDirectionChangePattern = (pHandle->wDirectionChangePattern << 1); //shift left with 0 to record NEGATIVE direction in the history
                    pHandle->hMeasuredElAngle = (int16_t)(pHandle->hPhaseShift);
                }
                else
                {
                    bUnexpectedBehavior = true;
                    HALL_Init_Electrical_Angle(pHandle);
                }
                break;

            default:
                pHandle->bHallState = bPrevHallState;
                /* Bad hall sensor configutarion so update the speed reliability */
                pHandle->bSensorIsReliable = false;
                bUnexpectedBehavior = true;
                break;
        }
        if (pHandle->bDirection != PrevDirection)
        {
            pHandle->bDirectionChangeCounter++;
        }
        else
        {
            if (pHandle->bDirectionChangeCounter != 0)
            {
                bReliableDirectionChange = true;
            }
            pHandle->bDirectionChangeCounter = 0;
        }

        /* We need to check that the direction has not changed.
        If it is the case, the sign of the current speed can be the opposite of the
        average speed, and the average time can be close to 0 which lead to a
        computed speed close to the infinite, and bring instability. */
        if (bReliableDirectionChange)
        {
            /* Setting bBufferFilled to 0 will prevent to compute the average speed based
            on the SpeedPeriod buffer values */
            pHandle->bBufferFilled = 0 ;
            pHandle->bSensorPeriod = 0;
        }

        if (!bUnexpectedBehavior && (pHandle->bDirectionChangeCounter == 0 || pHandle->bDirectionChangeCounter == 2))
        {
            /* We need to check that the direction has not changed.
            If it is the case, the sign of the current speed can be the opposite of the
            average speed, and the average time can be close to 0 which lead to a
            computed speed close to the infinite, and bring instability. */
            if (pHandle->bDirection != PrevDirection)
            {
                /* Setting bBufferFilled to 0 will prevent to compute the average speed based
                on the SpeedPeriod buffer values */
                pHandle->bBufferFilled = 0 ;
                pHandle->bSensorPeriod = 0;
            }
            if (pHandle->bHallMtpa == true)
            {
                pHandle->Super.hElAngle = pHandle->hMeasuredElAngle;
            }
            else
            {
                /* Nothing to do */
            }
            /* Discard first capture */
            if (pHandle->bFirstCapt == 0u)
            {
                pHandle->bFirstCapt++;
                // No need to update here
            }
            else
            {
                /* used to validate the average speed measurement */
                if (pHandle->bBufferFilled < pHandle->bSpeedBufferSize)
                {
                    pHandle->bBufferFilled++;
                }
                /* Store the latest speed acquisition */
                wCaptBuf = *pCapture;
                /* Add the numbers of overflow to the counter */
                if (pHandle->hOVFCounter != 0u)
                {
                    wCaptBuf += (uint32_t) pHandle->hOVFCounter*0x80000uL;
                }
                else
                {
                    // Do nothing here
                }
                /* the HALL_MAX_PSEUDO_SPEED is temporary in the buffer, and never included in average computation*/
                if (wCaptBuf < pHandle->wMinPeriod)
                {
                    pHandle->hAvrElSpeedDpp = HALL_MAX_PSEUDO_SPEED;
                }
                else
                {
                    pHandle->wElPeriodSum -= pHandle->SensorPeriod[pHandle->bSensorPeriod]; /* value we gonna removed from the accumulator */
                    if (wCaptBuf >= pHandle->wMaxPeriod)
                    {
                        pHandle->SensorPeriod[pHandle->bSensorPeriod] =  pHandle->wMaxPeriod;//*pHandle->bDirection;
                    }
                    else
                    {
                        pHandle->SensorPeriod[pHandle->bSensorPeriod] =  wCaptBuf;
                        pHandle->wElPeriodSum += pHandle->SensorPeriod[pHandle->bSensorPeriod];
                    }
                    /* Update pointers to speed buffer */
                    pHandle->bSensorPeriod++;
                    if (pHandle->bSensorPeriod == pHandle->bSpeedBufferSize)
                    {
                        pHandle->bSensorPeriod = 0u;
                    }
                    if (pHandle->bSensorIsReliable)
                    {
                        if (pHandle->bBufferFilled < pHandle->bSpeedBufferSize)
                        {
                            pHandle->hAvrElSpeedDpp = (int16_t) (pHandle->wPseudoFreqConv / wCaptBuf)*pHandle->bDirection;
                        }
                        else
                        { /* Average speed allow to smooth the mechanical sensors misalignement */
                            pHandle->hAvrElSpeedDpp =  (int16_t)(pHandle->wPseudoFreqConv / (pHandle->wElPeriodSum / pHandle->bSpeedBufferSize)) ; /* Average value */
                            pHandle->hAvrElSpeedDpp =  pHandle->hAvrElSpeedDpp * pHandle->bDirection;
                        }
                    }
                    else /* Sensor is not reliable */
                    {
                        pHandle->hAvrElSpeedDpp = 0;
                    }
                }
                /* Reset the number of overflow occurred */
                pHandle->hOVFCounter = 0u;
            }
        }
    }

    return (void *)(0x0);
}


void * HallPosSensor_TIMx_UP_IRQHandler(void * pHandleVoid)
{
    HallPosSensorHandle_t * pHandle = (HallPosSensorHandle_t *) pHandleVoid;
    if (pHandle->bSensorIsReliable)
    {
        /* an update event occured for this interrupt request generation */
        pHandle->hOVFCounter++;
        if (pHandle->hOVFCounter >= (pHandle->hHallMaxRatio))
        {
            /* Set rotor speed to zero */
            pHandle->Super.hElSpeedDpp = 0;
            /* Reset the electrical angle according the hall sensor configuration */
            HALL_Init_Electrical_Angle(pHandle);
            /* Reset the overflow counter */
            pHandle->hOVFCounter = 0u;
            /* Reset first capture flag */
            pHandle->bFirstCapt = 0u;
            /* Reset the SensorSpeed buffer*/
            uint8_t bIndex;
            for (bIndex = 0u; bIndex < pHandle->bSpeedBufferSize; bIndex++)
            {
                pHandle->SensorPeriod[bIndex]  = pHandle->wMaxPeriod;
            }
            pHandle->bBufferFilled = 0 ;
            pHandle->hAvrElSpeedDpp = 0;
            pHandle->bSensorPeriod = 0;
            pHandle->wElPeriodSum = pHandle->wMaxPeriod * pHandle->bSpeedBufferSize;
        }
    }
    return (void *)(0x0);				// MC_NULL
}

/**
* Read the logic level of the three Hall sensor and individuates in this way the position of the rotor (+/- 30???). Electrical angle is then initialized.
*/
static void HALL_Init_Electrical_Angle(HallPosSensorHandle_t * pHandle)
{
    if (pHandle->bSensorPlacement == DEGREES_120)
    {
        pHandle->bHallState  = (uint8_t) (R_BSP_PinRead(pHandle->H3PortPin) << 2 | R_BSP_PinRead(pHandle->H2PortPin) << 1 | R_BSP_PinRead(pHandle->H1PortPin));
    }
    else
    {
        pHandle->bHallState  = (uint8_t) ((R_BSP_PinRead(pHandle->H2PortPin) ^ 1) << 2  |  R_BSP_PinRead(pHandle->H3PortPin) << 1  |  R_BSP_PinRead(pHandle->H1PortPin));
    }
    /* Set angle based on hall sensor states */
    switch (pHandle->bHallState)
    {
        case STATE_5:
            pHandle->Super.hElAngle = (int16_t)(pHandle->hPhaseShift + S16_60_PHASE_SHIFT / 2);
            break;
        case STATE_1:
            pHandle->Super.hElAngle = (int16_t)(pHandle->hPhaseShift + S16_60_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2);
            break;
        case STATE_3:
            pHandle->Super.hElAngle = (int16_t)(pHandle->hPhaseShift + S16_120_PHASE_SHIFT + S16_60_PHASE_SHIFT / 2);
            break;
        case STATE_2:
            pHandle->Super.hElAngle = (int16_t)(pHandle->hPhaseShift - S16_120_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2);
            break;
        case STATE_6:
            pHandle->Super.hElAngle = (int16_t)(pHandle->hPhaseShift - S16_60_PHASE_SHIFT - S16_60_PHASE_SHIFT / 2);
            break;
        case STATE_4:
            pHandle->Super.hElAngle = (int16_t)(pHandle->hPhaseShift - S16_60_PHASE_SHIFT / 2);
            break;
        default:
            /* Bad hall sensor configutarion so update the speed reliability */
            pHandle->bSensorIsReliable = false;
            break;
    }
    /* Initialize the measured angle */
    pHandle->hMeasuredElAngle = pHandle->Super.hElAngle;
    pHandle->bDirectionChangeCounter = 0;
}


void HallPosSensor_SetMecAngle(HallPosSensorHandle_t * pHandle, int16_t hMecAngle)
{
	if(pHandle!= NULL)
	{
		hMecAngle = 0;
	}
    (void) hMecAngle; // Void line added to remove warning
}
