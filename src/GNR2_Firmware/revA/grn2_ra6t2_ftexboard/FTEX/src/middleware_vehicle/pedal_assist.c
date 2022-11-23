/**
  * @file    pedal_assist.c
  * @brief   This module handles management of pedal assist
  *
  */

#include "pedal_assist.h"
#include "ASSERT_FTEX.h"

/* Functions ---------------------------------------------------- */

/**
    * @brief  Module initialization, to be called once before using it
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_Init(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    		
    PedalSpdSensor_Init(pHandle->pPSS);
    PedalTorqSensor_Init(pHandle->pPTS);
    WheelSpdSensor_Init(pHandle->pWSS);
	
    // Enable slow motor Start for Pedal Assist cadence base
    Foldback_EnableSlowStart(pHandle->SpeedFoldbackStartupDualMotorPAS);
    
}

/**
    * @brief  Set PAS level
    * @param  Pedal Assist handle
    * @param  PAS level handle
    * @retval None
    */
void PedalAssist_SetAssistLevel(PAS_Handle_t * pHandle, PasLevel_t bLevel)
{
    ASSERT(pHandle != NULL);
    pHandle->bCurrentAssistLevel = bLevel;
}

/**
    * @brief  Get PAS level
    * @param  Pedal Assist handle
    * @retval Current Pedal assit level
    */
PasLevel_t PedalAssist_GetAssistLevel(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->bCurrentAssistLevel;
}

/**
    * @brief  Set Pedal Assist standard torque based on screen informations`
    * @param  Pedal Assist handle
    * @retval pRefTorque in int16
    */
int16_t PedalAssist_GetPASTorque(PAS_Handle_t * pHandle)
{
    int16_t hRefTorque;
    PasLevel_t Got_Level;
    // Get the used PAS level 
    Got_Level = PedalAssist_GetAssistLevel(pHandle);
    // Calculate the maximum given reference torque per level 
    hRefTorque = (pHandle->sParameters.hPASMaxTorque / pHandle->sParameters.bMaxLevel) * Got_Level;
   
    return hRefTorque;  
}
/**
    * @brief  Set Pedal Assist standard torque based on screen informations`
    *         for Cadence PAS base
    * @param  Pedal Assist handle
    * @retval pRefTorque in int16
    */
int16_t PedalAssist_GetPASTorqueSpeed(PAS_Handle_t * pHandle)
{
    int16_t hRefTorque;
    PasLevel_t Got_Level;
    // Get the used PAS level 
    Got_Level = PedalAssist_GetAssistLevel(pHandle);  
    switch(Got_Level)
    {
        case PAS_LEVEL_0:
            hRefTorque = 0;
            break;      
        case PAS_LEVEL_1:
            hRefTorque = (pHandle->sParameters.hPASMaxTorque * PAS_LEVEL_3) / PAS_LEVEL_5; // ratio of 3/5 from the max torque based on the feeling of the user
            break;
        case PAS_LEVEL_2:
            hRefTorque = (pHandle->sParameters.hPASMaxTorque * PAS_LEVEL_4) / PAS_LEVEL_6; // ratio of 4/6 from the max torque based on the feeling of the user 
            break;
        case PAS_LEVEL_3:
            hRefTorque = (pHandle->sParameters.hPASMaxTorque * PAS_LEVEL_4) / PAS_LEVEL_5; // ratio of 4/5 from the max torque based on the feeling of the user
            break;
        case PAS_LEVEL_4:
            hRefTorque = (pHandle->sParameters.hPASMaxTorque * PAS_LEVEL_7) / PAS_LEVEL_8; // ratio of 7/8 from the max torque based on the feeling of the user
            break;
        case PAS_LEVEL_5:
            hRefTorque = (pHandle->sParameters.hPASMaxTorque); // ratio of 1 from the max torque based on the feeling of the user
            break;
        case PAS_LEVEL_WALK:
            hRefTorque = (pHandle->sParameters.hPASMaxTorque * PAS_LEVEL_3) / PAS_LEVEL_5; // Initial ratio picked by the team, still not tested by a client
            break;        
        default:
            hRefTorque = 0;
            break;
    }  
    return hRefTorque;
}

/**
    * @brief  Set Pedal Assist standard speed based on screen informations
    * @param  Pedal Assist handle
    * @retval pRefTorque in int16
    */
void PedalAssist_PASSetMaxSpeed(PAS_Handle_t * pHandle)
{
    uint16_t hKmSpeedTemp;
    PasLevel_t Got_Level;
    Got_Level = PedalAssist_GetAssistLevel(pHandle);
    // Calculate the maximum speed for control 
    hKmSpeedTemp = ((pHandle->sParameters.hMaxSpeedRatio * pHandle->sParameters.hPASMaxSpeed) / PAS_PERCENTAGE) / pHandle->sParameters.hPASMaxKmSpeed;
    // Set Speed limitation range for motor control 
    switch(Got_Level)
    {
        case PAS_LEVEL_0:
            Foldback_SetDecreasingRange (pHandle->SpeedFoldbackStartupDualMotorPAS, (uint16_t)(round (hKmSpeedTemp * PAS_LEVEL_SPEED_0)));
            Foldback_SetDecreasingRangeEndValue (pHandle->SpeedFoldbackStartupDualMotorPAS,(int16_t)(round (hKmSpeedTemp * PAS_LEVEL_SPEED_0)));
            break;      
        case PAS_LEVEL_1:
            Foldback_SetDecreasingRange (pHandle->SpeedFoldbackStartupDualMotorPAS, (uint16_t)(round (hKmSpeedTemp * PAS_LEVEL_SPEED_1)));
            Foldback_SetDecreasingRangeEndValue (pHandle->SpeedFoldbackStartupDualMotorPAS,(int16_t)(round (hKmSpeedTemp * PAS_LEVEL_SPEED_1)));
            break;
        case PAS_LEVEL_2:
            Foldback_SetDecreasingRange (pHandle->SpeedFoldbackStartupDualMotorPAS, (uint16_t)(round (hKmSpeedTemp * PAS_LEVEL_SPEED_2)));
            Foldback_SetDecreasingRangeEndValue (pHandle->SpeedFoldbackStartupDualMotorPAS,(int16_t)(round (hKmSpeedTemp * PAS_LEVEL_SPEED_2)));
            break;
        case PAS_LEVEL_3:
            Foldback_SetDecreasingRange (pHandle->SpeedFoldbackStartupDualMotorPAS, (uint16_t)(round (hKmSpeedTemp * PAS_LEVEL_SPEED_3)));
            Foldback_SetDecreasingRangeEndValue (pHandle->SpeedFoldbackStartupDualMotorPAS,(int16_t)(round (hKmSpeedTemp * PAS_LEVEL_SPEED_3)));
            break;
        case PAS_LEVEL_4:
            Foldback_SetDecreasingRange (pHandle->SpeedFoldbackStartupDualMotorPAS, (uint16_t)(round (hKmSpeedTemp * PAS_LEVEL_SPEED_4)));
            Foldback_SetDecreasingRangeEndValue (pHandle->SpeedFoldbackStartupDualMotorPAS,(int16_t)(round (hKmSpeedTemp * PAS_LEVEL_SPEED_4)));
            break;
        case PAS_LEVEL_5:
            Foldback_SetDecreasingRange (pHandle->SpeedFoldbackStartupDualMotorPAS, (uint16_t)(round (hKmSpeedTemp * PAS_LEVEL_SPEED_5)));
            Foldback_SetDecreasingRangeEndValue (pHandle->SpeedFoldbackStartupDualMotorPAS,(int16_t)(round (hKmSpeedTemp * PAS_LEVEL_SPEED_5)));
            break;
        case PAS_LEVEL_WALK:
            Foldback_SetDecreasingRange (pHandle->SpeedFoldbackStartupDualMotorPAS, (uint16_t)(round (hKmSpeedTemp * PAS_LEVEL_SPEED_WALK)));
            Foldback_SetDecreasingRangeEndValue (pHandle->SpeedFoldbackStartupDualMotorPAS,(int16_t)(round (hKmSpeedTemp * PAS_LEVEL_SPEED_WALK)));
            break;
        default:
            Foldback_SetDecreasingRange (pHandle->SpeedFoldbackStartupDualMotorPAS, (uint16_t)(round (hKmSpeedTemp * PAS_LEVEL_SPEED_0)));
            Foldback_SetDecreasingRangeEndValue (pHandle->SpeedFoldbackStartupDualMotorPAS,(int16_t)(round (hKmSpeedTemp * PAS_LEVEL_SPEED_0)));
            break;
    }
}

/**
    * @brief  Set Pedal Assist torque based on the pedal Torque Sensor
    * @param  Pedal Assist handle
    * @retval pRefTorqueS in int16
    */
int16_t PedalAssist_GetTorqueFromTS(PAS_Handle_t * pHandle)
{
    int16_t hRefTorqueS, hReadTS, hMaxTorq_Temp;
    PasLevel_t Got_Level;
    /* Read the Pedal torque sensor */
    hReadTS = PedalTorqSensor_ToMotorTorque(pHandle->pPTS);
    /* Got the PAS from the screen */
    Got_Level = PedalAssist_GetAssistLevel(pHandle);
    /* Add Level cases for a better user feeling */
    switch(Got_Level)
    {
        case PAS_LEVEL_0:
            hRefTorqueS = 0;
            break;
        case PAS_LEVEL_1:
            /* Convert the PAS torque sensing in motor torque */
            hRefTorqueS = (hReadTS * pHandle->sParameters.bCoeffLevel * PAS_LEVEL_1) / pHandle->sParameters.bMaxLevel;
            /* Safety for not exceeding the maximum torque value */
            hMaxTorq_Temp = (pHandle->sParameters.hMaxTorqueRatio * pHandle->sParameters.hPASMaxTorque)/PAS_PERCENTAGE;
            if (hRefTorqueS > hMaxTorq_Temp)
            {
                hRefTorqueS = hMaxTorq_Temp;
            }
            break;
        case PAS_LEVEL_2:
            /* Convert the PAS torque sensing in motor torque */
            hRefTorqueS = (hReadTS * pHandle->sParameters.bCoeffLevel * PAS_LEVEL_2) / pHandle->sParameters.bMaxLevel;
            /* Safety for not exceeding the maximum torque value */
            hMaxTorq_Temp = (pHandle->sParameters.hMaxTorqueRatio * pHandle->sParameters.hPASMaxTorque)/PAS_PERCENTAGE;
            if (hRefTorqueS > hMaxTorq_Temp)
            {
                hRefTorqueS = hMaxTorq_Temp;
            }
            break;
        case PAS_LEVEL_3:
            /* Convert the PAS torque sensing in motor torque */
            hRefTorqueS = (hReadTS * pHandle->sParameters.bCoeffLevel * PAS_LEVEL_3) / pHandle->sParameters.bMaxLevel;
            /* Safety for not exceeding the maximum torque value */
            hMaxTorq_Temp = (pHandle->sParameters.hMaxTorqueRatio * pHandle->sParameters.hPASMaxTorque)/PAS_PERCENTAGE;
            if (hRefTorqueS > hMaxTorq_Temp)
            {
                hRefTorqueS = hMaxTorq_Temp;
            }
            break;
        case PAS_LEVEL_4:
            /* Convert the PAS torque sensing in motor torque */
            hRefTorqueS = (hReadTS * pHandle->sParameters.bCoeffLevel * PAS_LEVEL_4) / pHandle->sParameters.bMaxLevel;
            /* Safety for not exceeding the maximum torque value */
            hMaxTorq_Temp = (pHandle->sParameters.hMaxTorqueRatio * pHandle->sParameters.hPASMaxTorque)/PAS_PERCENTAGE;
            if (hRefTorqueS > hMaxTorq_Temp)
            {
                hRefTorqueS = hMaxTorq_Temp;
            }
            break;		
        case PAS_LEVEL_5:
            /* Convert the PAS torque sensing in motor torque */
            hRefTorqueS = (hReadTS * pHandle->sParameters.bCoeffLevel * PAS_LEVEL_5) / pHandle->sParameters.bMaxLevel;
            /* Convert the PAS torque sensing in motor torque */
            hMaxTorq_Temp = (pHandle->sParameters.hMaxTorqueRatio * pHandle->sParameters.hPASMaxTorque)/PAS_PERCENTAGE;
            if (hRefTorqueS > hMaxTorq_Temp)
            {
                hRefTorqueS = hMaxTorq_Temp;
            }
            break;
        default:
            hRefTorqueS = 0;
            break;
    }
    return hRefTorqueS;
}

/**
    * @brief  Check the PAS Presence Flag
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_UpdatePASDetection (PAS_Handle_t * pHandle) 
{
    uint32_t	wSpeedt;
    uint16_t	hTorqueSens;
    /* Calculate the offset based on ration percentage */
    uint16_t  hOffsetTemp = (pHandle->pPTS->hParameters.hOffsetMT * pHandle->pPTS->hParameters.hMax) / PAS_PERCENTAGE;
	
    wSpeedt = PedalSpdSensor_GetPeriodValue(pHandle->pPSS);
    hTorqueSens = PedalTorqSensor_GetAvValue(pHandle->pPTS);

    if ((pHandle->sParameters.bTorqueSensorUse) && (hTorqueSens > hOffsetTemp) )
        pHandle->bPASDetected = true;
    else if (wSpeedt > 0)
    {
        pHandle->sParameters.bPASCountSafe++;
        if ((pHandle->sParameters.bPASCountSafe > 1))
        {
            pHandle->bPASDetected = true;
            pHandle->sParameters.bPASCountSafe--;
        }
    }
    else
    {
        pHandle->sParameters.bPASCountSafe = 0;
        pHandle->bPASDetected = false;
    }
} 

/**
    * @brief  Return if pedals are moving or not
    * @param  Pedal Assist handle
    * @retval True if pedal movement is detected, false otherwise
    */
bool PedalAssist_IsPASDetected(PAS_Handle_t * pHandle) 
{
    return pHandle->bPASDetected;
}

/**
    * @brief  Return if walk mode is active
    * @param  Pedal Assist handle
    * @retval True if walk mode is detected, false otherwise
    */
bool PedalAssist_IsWalkModeDetected(PAS_Handle_t * pHandle)
{
    if(pHandle->bCurrentAssistLevel == PAS_LEVEL_WALK)
    {
        return true;  
    }
    else
    {
        return false;    
    }       
}
