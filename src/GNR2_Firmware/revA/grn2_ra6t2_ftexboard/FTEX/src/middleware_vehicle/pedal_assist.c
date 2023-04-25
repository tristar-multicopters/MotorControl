/**
  * @file    pedal_assist.c
  * @brief   This module handles management of pedal assist
  *
  */

#include "pedal_assist.h"
#include "ASSERT_FTEX.h"
// disable warning about user_config_task modifying the pragma pack value
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpragma-pack"
#include "user_config_task.h"
#pragma clang diagnostic pop

// todo: move those hardcoded levels to a configurable field
const int16_t PASTorqueRatiosInPercentage[6] = 
{
    0,  // PAS 0 has a ratio of 0%
    60, // PAS 1 has a ratio of 60% (3/5)
    67, // PAS 2 has a ratio of 67% (4/6)
    80, // PAS 3 has a ratio of 80% (4/5)
    88, // PAS 4 has a ratio of 88% (7/8)
    100 // PAS 5 has a ratio of 100%
};
const int16_t walkModeTorqueRatio = 70; // walkmode has a ratio of 70%

/* Variables ---------------------------------------------------- */

uint8_t bPASCounterAct = 0; // Slow cadence PAS activation loop couter

/* Functions ---------------------------------------------------- */

// Internal utility function to verify that the assist level we got is within the supported range
void AssertIsValidLevel(PasLevel_t level);

/**
    * @brief  Module initialization, to be called once before using it
    * @param  Pedal Assist handle & Delay Handle
    * @retval None
    */
void PedalAssist_Init(PAS_Handle_t * pHandle, Delay_Handle_t * pPTSstuckDelay)
{
    ASSERT(pHandle != NULL);

    PedalSpdSensor_Init(pHandle->pPSS);
    WheelSpdSensor_Init(pHandle->pWSS);
    PedalTorqSensor_Init(pHandle->pPTS, pPTSstuckDelay);
    
    
    //initalise pass to level 1. This is necessary to 
    //make the bike run if no screen is present
    //as on apollo. 
    pHandle->bCurrentAssistLevel = DEFAULT_PAS_LEVEL;
	
    // Enable slow motor Start for Pedal Assist cadence base
    Foldback_EnableSlowStart(pHandle->SpeedFoldbackVehiclePAS);
    
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
    AssertIsValidLevel(bLevel);
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
    int16_t PASRatio = 0;
    PasLevel_t currentLevel = PedalAssist_GetAssistLevel(pHandle); 

    // assert we're within range so that the array below doesn't go out of bounds
    AssertIsValidLevel(currentLevel);
    if (currentLevel == PAS_LEVEL_WALK)
    {
        PASRatio = walkModeTorqueRatio;
    }
    else
    {
        // The pedal_assist module only supports 5 PAS levels at the moment
        PASRatio = PASTorqueRatiosInPercentage[currentLevel];
    }
    
    // compute the torque using the ratio from the PAS level
    return (pHandle->sParameters.hPASMaxTorque * PASRatio)/100;
}

/**
    * @brief  Set Pedal Assist standard speed based on screen informations
    * @param  Pedal Assist handle
    * @retval pRefTorque in int16
    */
void PedalAssist_PASSetMaxSpeed(PAS_Handle_t * pHandle)
{
    uint16_t hKmSpeedTemp;
    PasLevel_t currentLevel = PedalAssist_GetAssistLevel(pHandle);
    
    // assert we're within range so that the code below doesn't go out of bounds
    AssertIsValidLevel(currentLevel);
    
    // Calculate the maximum speed for control 
    hKmSpeedTemp = ((pHandle->sParameters.hMaxSpeedRatio * pHandle->sParameters.hPASMaxSpeed) / PAS_PERCENTAGE) / pHandle->sParameters.hPASMaxKmSpeed;
    
    // Get the userconfig speed
    uint8_t userConfigSpeed = 0;
    if (currentLevel == PAS_LEVEL_WALK)
    {
        userConfigSpeed = UserConfigTask_GetWalkModeSpeed();
    }
    else
    {
        userConfigSpeed = UserConfigTask_GetCadenceHybridLevelSpeed(currentLevel);
    }
    
    // set the foldbacks
    uint16_t speed = (uint16_t)(round (hKmSpeedTemp * userConfigSpeed));
    Foldback_SetDecreasingRange(pHandle->SpeedFoldbackVehiclePAS, speed);
    Foldback_SetDecreasingRangeEndValue(pHandle->SpeedFoldbackVehiclePAS,(int16_t)(speed));
}

/**
    * @brief  Set Pedal Assist torque based on the pedal Torque Sensor
    * @param  Pedal Assist handle
    * @retval pRefTorqueS in int16
    */
int16_t PedalAssist_GetTorqueFromTS(PAS_Handle_t * pHandle)
{
    int16_t hRefTorqueS, hReadTS, hMaxTorq_Temp;
    /* Read the Pedal torque sensor */
    hReadTS = PedalTorqSensor_ToMotorTorque(pHandle->pPTS);
    /* Got the PAS from the screen */
    PasLevel_t currentLevel = PedalAssist_GetAssistLevel(pHandle);
    
    AssertIsValidLevel(currentLevel);
    
    if (currentLevel == PAS_LEVEL_WALK || currentLevel == PAS_LEVEL_0)
    {
        return 0;
    }
    
    // Convert the PAS torque sensing in motor torque
    hRefTorqueS = (hReadTS * pHandle->sParameters.bCoeffLevel * currentLevel) / pHandle->sParameters.bMaxLevel;
    
    // Safety for not exceeding the maximum torque value 
    hMaxTorq_Temp = (pHandle->sParameters.hMaxTorqueRatio * pHandle->sParameters.hPASMaxTorque)/PAS_PERCENTAGE;
    if (hRefTorqueS > hMaxTorq_Temp)
    {
        hRefTorqueS = hMaxTorq_Temp;
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
    uint32_t  wSpeedt;
    uint16_t  hTorqueSens;
    uint16_t  hOffsetTemp;
    uint16_t  hWheelRPM;
    
    hWheelRPM = (uint16_t) WheelSpdSensor_GetSpeedRPM(pHandle->pWSS);
    /* Calculate the offset based on ration percentage */
    
    if(hWheelRPM < pHandle->pPTS->hParameters.hStartupOffsetMTSpeed) // If going at low speed use the startup offset
    {
        hOffsetTemp = (pHandle->pPTS->hParameters.hOffsetMTStartup * pHandle->pPTS->hParameters.hMax) / PAS_PERCENTAGE;
    }
    else
    {
        hOffsetTemp = (pHandle->pPTS->hParameters.hOffsetMT * pHandle->pPTS->hParameters.hMax) / PAS_PERCENTAGE;
    }        
    
	
    wSpeedt = PedalSpdSensor_GetPeriodValue(pHandle->pPSS);
    hTorqueSens = PedalTorqSensor_GetAvValue(pHandle->pPTS);

    /* Torque Sensor use and the offset was detected */
    if ((pHandle->bCurrentPasAlgorithm == TorqueSensorUse) && (hTorqueSens > hOffsetTemp))
    {
        pHandle->bPASDetected = true;
    }
    /* Hybrid Algorithm use and the offset was detected */
    else if ((pHandle->bCurrentPasAlgorithm == HybridSensorUse) && (hTorqueSens > hOffsetTemp))
    {
		pHandle->bPASDetected = true;
    }
    /* Cadence Sensor use */
    else if (wSpeedt > 0)
    {
        /* Add security layer to miss the first PAS detect if there is any issue 
           with the pedal */
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
    * @brief  Check the PAS Presence Flag for slow PAS detection sensors
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_UpdatePASDetectionCall(PAS_Handle_t * pHandle) 
{
    // Check if the PAS presence is detected
    if ( PedalAssist_IsPASDetected(pHandle))
    {
        bPASCounterAct ++;
        // For Slow PAS sensor on cadence check
        if (bPASCounterAct > pHandle->sParameters.bPASCountActivation && (pHandle->bCurrentPasAlgorithm == CadenceSensorUse))
        {
            PedalAssist_UpdatePASDetection (pHandle);
            bPASCounterAct = 0;
        }
        // For normal use with hybrid or Torque sensor
        if ((pHandle->bCurrentPasAlgorithm == TorqueSensorUse) || (pHandle->bCurrentPasAlgorithm ==HybridSensorUse))
        {
            PedalAssist_UpdatePASDetection (pHandle);
        }
                   
    }
    // If the PAS presence is not detected the safe coefficient for PAS Safe detection remain
    else 
    {
        PedalAssist_UpdatePASDetection (pHandle); 
        bPASCounterAct = 0;
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

/**
    * @brief  Reset the PAS Prameters
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_ResetParameters (PAS_Handle_t * pHandle) 
{
    PedalTorqSensor_ResetAvValue(pHandle->pPTS);   
    PedalSpdSensor_ResetValue(pHandle->pPSS);   
}

// Internal utility function to verify that the assist level we got is within the supported range
void AssertIsValidLevel(PasLevel_t level)
{    
    if (level == PAS_LEVEL_WALK)
    {
        // valid
        return;
    }
    
    if (level >= PAS_LEVEL_0 && level <= PAS_LEVEL_5)
    {
        // valid
        return;
    }
    
    // invalid
    ASSERT(false);
}