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
    
    pHandle->bCurrentAssistLevel = DEFAULT_PAS_LEVEL;
    PedalAssist_PASUpdateMaxSpeed(pHandle);
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
    * @retval Current Pedal assist level
    */
PasLevel_t PedalAssist_GetAssistLevel(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->bCurrentAssistLevel;
}

/**
    * @brief  Set Pedal Assist standard torque based on screen informations
    * @param  Pedal Assist handle
    * @retval pRefTorque in int16
    */
int16_t PedalAssist_GetPASTorque(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    int32_t hRefTorque;
    PasLevel_t Got_Level;
    // Get the used PAS level 
    Got_Level = PedalAssist_GetAssistLevel(pHandle);
    // Calculate the maximum given reference torque per level 
    hRefTorque = (pHandle->sParameters.hPASMaxTorque/PAS_PERCENTAGE) * pHandle->sParameters.PASTTorqRatiosInPercentage[Got_Level];
   
    return (int16_t) hRefTorque;  
}
/**
    * @brief  Set Pedal Assist standard torque based on screen informations
    *         for Cadence PAS base
    * @param  Pedal Assist handle
    * @retval pRefTorque in int16
    */
int16_t PedalAssist_GetPASCadenceMotorTorque(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    int16_t PASRatio = 0;
    PasLevel_t currentLevel = PedalAssist_GetAssistLevel(pHandle); 

    // assert we're within range so that the array below doesn't go out of bounds
    AssertIsValidLevel(currentLevel);
    if (currentLevel == PAS_LEVEL_WALK)
    {
        PASRatio = pHandle->sParameters.walkModeTorqueRatio;        
    }
    else
    {
        // The pedal_assist module only supports 5 PAS levels at the moment
        PASRatio = pHandle->sParameters.PASCTorqRatiosInPercentage[currentLevel];
    }
    
    // compute the torque using the ratio from the PAS level
    return (pHandle->sParameters.hPASMaxTorque * PASRatio)/100;
}

/**
    * @brief  Set Pedal Assist standard speed based on screen informations
    * @param  Pedal Assist handle
    * @retval pRefTorque in int16
    */
void PedalAssist_PASUpdateMaxSpeed(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);   
    PasLevel_t currentLevel = PedalAssist_GetAssistLevel(pHandle);
    
    // assert we're within range so that the code below doesn't go out of bounds
    AssertIsValidLevel(currentLevel);
        
    // Get the userconfig speed
    uint16_t userConfigSpeed = 0;
    if (currentLevel == PAS_LEVEL_WALK)
    {
        userConfigSpeed = UserConfigTask_GetWalkModeSpeed();
    }
    else if (pHandle->bCurrentPasAlgorithm == CadenceSensorUse) // Set the level specific speed limit
    {
        userConfigSpeed = UserConfigTask_GetCadenceLevelSpeed(currentLevel);
        
        //check if the cadence max speed is inside of the max speed limit
        //torque and cadence must have the same max speed limit
        if (userConfigSpeed > pHandle->sParameters.TorquePasMaxSpeed)
        {
            userConfigSpeed = pHandle->sParameters.TorquePasMaxSpeed;
        }
    }
    else if(pHandle->bCurrentPasAlgorithm == TorqueSensorUse)  // Set the generic limit
    {
        userConfigSpeed = pHandle->sParameters.TorquePasMaxSpeed;
    }
    
    pHandle->sParameters.hPASMaxSpeed =  userConfigSpeed;
}

/**
    * @brief  Get the Pedal Assist standard speed based on pas level
    * @param  Pedal Assist handle
    * @retval current PAS speed limit
    */
uint16_t PedalAssist_GetPASMaxSpeed(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->sParameters.hPASMaxSpeed;
}

void PedalAssist_SetTorquePASMaxSpeed(PAS_Handle_t * pHandle, uint16_t topSpeed)
{
    ASSERT(pHandle != NULL);
    pHandle->sParameters.TorquePasMaxSpeed = topSpeed;
}

/**
    * @brief  Set Pedal Assist torque based on the pedal Torque Sensor
    * @param  Pedal Assist handle
    * @retval pRefTorqueS in int16
    */
int16_t PedalAssist_GetTorqueFromTS(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    int16_t hRefTorqueS, hReadTS, hMaxTorq_Temp, hMaxLevelTorq_Temp;
    
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
    hRefTorqueS = ((hReadTS * pHandle->sParameters.bTorqueGain)/PAS_PERCENTAGE);
    
    hRefTorqueS = (hRefTorqueS/PAS_PERCENTAGE) * pHandle->sParameters.PASTTorqRatiosInPercentage[pHandle->bCurrentAssistLevel];
    
    
    // Safety for not exceeding the maximum torque value for a specific pas level
    hMaxLevelTorq_Temp = (pHandle->sParameters.PASTTorqRatiosInPercentage[pHandle->bCurrentAssistLevel] * pHandle->sParameters.hPASMaxTorque)/PAS_PERCENTAGE;
    if (hRefTorqueS > hMaxLevelTorq_Temp)
    {
        hRefTorqueS = hMaxLevelTorq_Temp;
    }
        
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
    ASSERT(pHandle != NULL);
    uint32_t  wSpeedt;
    uint16_t  hTorqueSens;
    uint16_t  hOffsetTemp;
    uint16_t  hWheelRPM;
    uint16_t  hTorquePASThreshold;
    static uint16_t PulseCounter = 0;
    bool CalculateAverage = false;
    
    
    hWheelRPM = (uint16_t) WheelSpdSensor_GetSpeedRPM(pHandle->pWSS);
    /* Calculate the offset based on ration percentage */
    
    if(hWheelRPM < pHandle->pPTS->hParameters.hStartupOffsetMTSpeedRPM) // If going at low speed use the startup offset
    {     
        CalculateAverage = true;  // Make sure we calculated the average to check for the threshold       
        hOffsetTemp = (pHandle->pPTS->hParameters.hOffsetMTStartup * pHandle->pPTS->hParameters.hMax) / PAS_PERCENTAGE;        
    }
    else
    {
        CalculateAverage = false; // No need for the average      
        hOffsetTemp = (pHandle->pPTS->hParameters.hOffsetMT * pHandle->pPTS->hParameters.hMax) / PAS_PERCENTAGE;
    }        
    
	
    wSpeedt = PedalSpdSensor_GetPeriodValue(pHandle->pPSS);
    hTorqueSens = PedalTorqSensor_GetAvValue(pHandle->pPTS);
   
    static uint16_t hThresholdCheckAvg[TORQUE_THRESHOLD_AVG_NB];
    static uint8_t AvgIndex = 0; 
    uint32_t wThresholdAvg = 0;
    
    if(CalculateAverage) // If we need to calculate the average
    {
        hThresholdCheckAvg[AvgIndex] = hTorqueSens;
    
        if(AvgIndex >= TORQUE_THRESHOLD_AVG_NB - 1)
        {
            AvgIndex = 0;
        }    
        else
        {
            AvgIndex ++;
        }
    
        for(int i = 0; i < TORQUE_THRESHOLD_AVG_NB; i ++)
        {
            wThresholdAvg += hThresholdCheckAvg[i];
        }
    
        wThresholdAvg /= TORQUE_THRESHOLD_AVG_NB;
        
        hTorquePASThreshold = (uint16_t) wThresholdAvg;
    }
    else // if there is no need for an average
    {
        hTorquePASThreshold = hTorqueSens;
    }
    
    /* Torque Sensor use and the offset was detected */
    if ((pHandle->bCurrentPasAlgorithm == TorqueSensorUse) && (hTorquePASThreshold > hOffsetTemp))
    {      
        pHandle->bPASDetected = true;
    }
    /* Cadence Sensor use */
    else if (wSpeedt > 0)
    {
        /* Add security layer to miss the first PAS detect if there is any issue 
           with the pedal */
        
        PulseCounter++;
        if ((PulseCounter > pHandle->sParameters.bPASCountSafe))
        {
            pHandle->bPASDetected = true;
            PulseCounter--;
        }
    }
    else
    {
        PulseCounter = 0;
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
    ASSERT(pHandle != NULL);
    // Check if the PAS presence is detected
    if ( PedalAssist_IsPASDetected(pHandle))
    {
        bPASCounterAct ++;
        // For Slow PAS sensor on cadence check
        if (bPASCounterAct > pHandle->sParameters.bPASCountActivation && (pHandle->bCurrentPasAlgorithm == CadenceSensorUse))
        {
            PedalAssist_UpdatePASDetection(pHandle);
            bPASCounterAct = 0;
        }
        
        if(pHandle->bCurrentPasAlgorithm == TorqueSensorUse)
        {
            PedalAssist_UpdatePASDetection(pHandle);
        }
                   
    }
    // If the PAS presence is not detected the safe coefficient for PAS Safe detection remain
    else 
    {
        PedalAssist_UpdatePASDetection(pHandle); 
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
    ASSERT(pHandle != NULL);
    return pHandle->bPASDetected;
}

/**
    * @brief  Return if walk mode is active
    * @param  Pedal Assist handle
    * @retval True if walk mode is detected, false otherwise
    */
bool PedalAssist_IsWalkModeDetected(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
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
    ASSERT(pHandle != NULL);
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
    
    if ((level >= PAS_LEVEL_0) && (level <= PAS_LEVEL_9))
    {
        // valid
        return;
    }
    
    // invalid
    ASSERT(false);
}

/**
    * @brief  Get the PAS algorithm
    * @param  Pedal Assist handle,
    * @retval PasAlgorithm_t
    */
PasAlgorithm_t PedalAssist_GetPASAlgorithm(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->bCurrentPasAlgorithm;
}

/**
    * @brief  Reset the PAS algorithm
    * @param  Pedal Assist handle, new pas algorithm
    * @retval None
    */
void PedalAssist_SetPASAlgorithm(PAS_Handle_t * pHandle, PasAlgorithm_t aPASAlgo)
{
    ASSERT(pHandle != NULL);
    
    pHandle->bCurrentPasAlgorithm = aPASAlgo;
}

