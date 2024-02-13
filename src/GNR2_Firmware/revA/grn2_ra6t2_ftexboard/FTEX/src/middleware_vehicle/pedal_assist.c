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

//
static PasCadenceState_t PasCadenceState = CADENCE_DETECTION_STARTUP;

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
    PedalAssist_ResetPASDetected(pHandle);
    PedalAssist_ResetCadenceStartupPasDection(pHandle);
    PedalAssist_ResetCadenceRunningPasDection(pHandle);
    PedalAssist_ResetTorqueStartupPasDection(pHandle);
    PedalAssist_ResetTorqueRunningPasDection(pHandle);
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
    hRefTorque = (pHandle->sParameters.hPASMaxTorque/PAS_PERCENTAGE) * pHandle->sParameters.PASMaxTorqRatiosInPercentage[Got_Level];
   
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
        PASRatio = pHandle->sParameters.PASMaxTorqRatiosInPercentage[currentLevel];
    }
    
    // compute the torque using the ratio from the PAS level
    return (pHandle->sParameters.hPASMaxTorque * PASRatio)/100;
}

/**
    * @brief  Set Pedal Assist standard speed based on screen informations
    * @param  Pedal Assist handle
    * @retval current PAS speed limit
    */
uint16_t PedalAssist_PASUpdateMaxSpeed(PAS_Handle_t * pHandle)
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
    else
    {        
        //get max speed associated with the current pas level.
        userConfigSpeed = pHandle->sParameters.PASMaxSpeed[currentLevel];
          
        //check if the max speed is inside of the max speed limit
        //torque and cadence must have the same max speed limit
        if (userConfigSpeed > pHandle->sParameters.PasMaxSpeed)
        {
            userConfigSpeed = pHandle->sParameters.PasMaxSpeed;
        }
    }
    //set max pas speed.
    return userConfigSpeed;
}


void PedalAssist_SetPASMaxSpeed(PAS_Handle_t * pHandle, uint16_t topSpeed)
{
    ASSERT(pHandle != NULL);
    pHandle->sParameters.PasMaxSpeed = topSpeed;
}

/**
    * @brief  Set Pedal Assist torque based on the pedal Torque Sensor
    * @param  Pedal Assist handle
    * @retval pRefTorqueS in int16
    */
int16_t PedalAssist_GetTorqueFromTS(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    int16_t hRefTorqueS, hRefMinTorqueS, hReadTS, hMaxTorq_Temp, hMaxLevelTorq_Temp;
    
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
    hRefTorqueS = ((hReadTS * pHandle->sParameters.bTorqueGain[currentLevel])/PAS_PERCENTAGE);
    
    //get ref torque to min torque calculation.
    hRefMinTorqueS = hRefTorqueS;
    
    //caclulate the final torque to be applied.
    hRefTorqueS = (hRefTorqueS/PAS_PERCENTAGE) * pHandle->sParameters.PASMaxTorqRatiosInPercentage[pHandle->bCurrentAssistLevel];
    
    //calculate the min torque to be applied, once PAS is detected.
    hRefMinTorqueS = hRefMinTorqueS*pHandle->sParameters.PASMinTorqRatiosInPercentage[pHandle->bCurrentAssistLevel];
    
    //verify if the applied torque is less than the minimum torque
    //if true, apllied the minimum torque.
    if (hRefTorqueS < hRefMinTorqueS)
    {
        hRefTorqueS = hRefMinTorqueS;
    }
    
    // Safety for not exceeding the maximum torque value for a specific pas level
    hMaxLevelTorq_Temp = (pHandle->sParameters.PASMaxTorqRatiosInPercentage[pHandle->bCurrentAssistLevel] * pHandle->sParameters.hPASMaxTorque)/PAS_PERCENTAGE;
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
    * @brief  Check the PAS Presence Flag based on torque detection
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_TorquePASDetection (PAS_Handle_t * pHandle) 
{
    ASSERT(pHandle != NULL);
    uint16_t  hTorqueSens;
    uint16_t  hOffsetTemp;
    uint16_t  hWheelRPM;
    uint16_t  hTorquePASThreshold;
    bool CalculateAverage = false;
    
    hWheelRPM = (uint16_t) WheelSpdSensor_GetSpeedRPM(pHandle->pWSS);
    
    /* Calculate the offset based on ration percentage */
    if(hWheelRPM < pHandle->pPTS->hParameters.hStartupOffsetMTSpeedRPM) // If going at low speed use the startup offset
    {     
        CalculateAverage = true;  // Make sure we calculated the average to check for the threshold       
        hOffsetTemp = (pHandle->pPTS->hParameters.hOffsetMTStartup * pHandle->pPTS->hParameters.hMax) / PAS_PERCENTAGE;
        pHandle->bTorqueStartupPASDetected = true;
        pHandle->bTorqueRunningPASDetected = false;
    }
    else
    {
        CalculateAverage = false; // No need for the average      
        hOffsetTemp = (pHandle->pPTS->hParameters.hOffsetMT * pHandle->pPTS->hParameters.hMax) / PAS_PERCENTAGE;
        pHandle->bTorqueStartupPASDetected = false;
        pHandle->bTorqueRunningPASDetected = true;
    }        
    
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
        //Initialize the buffer used to avg the toque value on start and stop
        //condition.
        memset(hThresholdCheckAvg,0,TORQUE_THRESHOLD_AVG_NB);
            
        hTorquePASThreshold = hTorqueSens;
    }
    
    /* Torque Sensor use and the offset was not detected */
    //flags to startup and running state must to be cleared.
    if (hTorquePASThreshold <= hOffsetTemp)
    {      
        pHandle->bTorqueStartupPASDetected = false;
        pHandle->bTorqueRunningPASDetected = false;
    }
} 

/**
    * @brief  Detect the PAS based on cadence detection
    * @param  Pedal Assist handle
    * @param  Increment time used by the windowsDetectionLimite in ms.
    * @retval None
    */
void PedalAssist_CadencePASDetection (PAS_Handle_t * pHandle, uint16_t windowsIncrementTimeMs)
{
    //check input conditions
    ASSERT(pHandle != NULL);
    ASSERT(windowsIncrementTimeMs != 0);
    //variable used to detect the cadence windows time.
    static uint16_t windowsDetectionLimite = 0;
    //variable used to hold the number of pulses detected from the cadence sensor.
    uint16_t wNumberOfPulses = 0;
    
    //check if a windows value reset was requested by the system
    //or PAS level is zero. 
    //if true, clear all variables used on the PAS cadence detection.
    //if not, continue the windows detection increment.
    if ((PedalSpdSensor_GetWindowsFlag(pHandle->pPSS) == true) || (pHandle->bCurrentAssistLevel == 0))
    {
        //reset windows flag to enable a new clear to the windows detection time.
        PedalSpdSensor_ClearWindowsFlag(pHandle->pPSS);
        
        //initialize windows detection limite.
        windowsDetectionLimite = 0;
        
        //Reset wNumberOfPulses value to initilaze a
        //new windows of the detection.
        PedalSpdSensor_ResetValue(pHandle->pPSS);
    }
    else
    {
        //increment on each windowsIncrementTimeMs ms.
        windowsDetectionLimite = windowsDetectionLimite + windowsIncrementTimeMs;
    }
    
    //state macchine to handle PAS cadence detection
    //on differents scenarios, as start, running and stop.
    switch(PasCadenceState)
    {
        
        //used to detect PAS from cadence when PAS was not yet detected yet.
        //this happen when the pedal start to be used.
        case CADENCE_DETECTION_STARTUP:
            
            //check if the cadence detection limite arived.
            if (windowsDetectionLimite >= pHandle->pPSS->wPedalSpeedSens_WindowsStartup)
            {
                //initialize
                windowsDetectionLimite = 0;
    
                //Read the number of pulses detected from the cadence signal.
                //and reset detected number of pulses, in AGT timer.
                PedalSpdSensor_ReadNumberOfPulses(pHandle->pPSS);

                //get the number of pulses detected from the cadence signal.
                wNumberOfPulses = PedalSpdSensor_GetNumberOfPulses(pHandle->pPSS);
        
                //Reset wNumberOfPulses value to initilaze a
                //new windows of the detection.
                PedalSpdSensor_ResetValue(pHandle->pPSS);
            
                //check if the cadence signal was detected and if it is in the PAS
                //activation condition.
                //OBS: this function must to be called after PedalSpdSensor_ReadNumberOfPulses
                //     and inside of the THR_VC_MediumFreq task.
                if ((wNumberOfPulses >= pHandle->pPSS->hPedalSpeedSens_MinPulseStartup))
                {
                    pHandle->bCadenceStartupPASDetected = true;
                    pHandle->bCadenceRunningPASDetected = false;
                    //move to the run state detection
                    PasCadenceState = CADENCE_DETECTION_RUNNING;
                }
                else
                {
                    pHandle->bCadenceStartupPASDetected = false;
                    pHandle->bCadenceRunningPASDetected = false;
                }
            }
        
        break;
            
        //used to detect PAS from cadence when the bike is running
        case CADENCE_DETECTION_RUNNING:
            
            //check if the cadence detection limite arived.
            if (windowsDetectionLimite >= pHandle->pPSS->wPedalSpeedSens_WindowsRunning)
            {
                //initialize
                windowsDetectionLimite = 0;
    
                //Read the number of pulses detected from the cadence signal.
                //and reset detected number of pulses, in AGT timer.
                PedalSpdSensor_ReadNumberOfPulses(pHandle->pPSS);

                //get the number of pulses detected from the cadence signal.
                wNumberOfPulses = PedalSpdSensor_GetNumberOfPulses(pHandle->pPSS);
        
                //Reset wNumberOfPulses value to initilaze a
                //new windows of the detection.
                PedalSpdSensor_ResetValue(pHandle->pPSS);
            
                //check if the cadence signal was detected and if it is in the PAS
                //activation condition.
                //OBS: this function must to be called after PedalSpdSensor_ReadNumberOfPulses
                //     and inside of the THR_VC_MediumFreq task.
                if ((wNumberOfPulses >= pHandle->pPSS->hPedalSpeedSens_MinPulseRunning))
                {
                    pHandle->bCadenceStartupPASDetected = false;
                    pHandle->bCadenceRunningPASDetected = true;
                }
                else
                {
                    pHandle->bCadenceStartupPASDetected = false;
                    pHandle->bCadenceRunningPASDetected = false;
                    //move to the startup state detection
                    //because PAS was not detected anymore.
                    PasCadenceState = CADENCE_DETECTION_STARTUP;
                }
            }
        
        break;
        
        //must not go into this state.
        default:
            
            //reset the system using a software reset
            ASSERT(false);
        
        break;
        
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
    * @brief  Reset PAS detected flag
    * @param  Pedal Assist handle
    * @retval none
    */
void PedalAssist_ResetPASDetected(PAS_Handle_t * pHandle) 
{
    ASSERT(pHandle != NULL);
    pHandle->bPASDetected = false;
}

/**
    * @brief  Set PAS detected flag
    * @param  Pedal Assist handle
    * @retval none
    */
void PedalAssist_SetPASDetected(PAS_Handle_t * pHandle) 
{
    ASSERT(pHandle != NULL);
    pHandle->bPASDetected = true;
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
    //reset the number of pulses
    PedalSpdSensor_ResetValue(pHandle->pPSS);   
    //reset the pulse windows time.
    PedalSpdSensor_SetWindowsFlag(pHandle->pPSS);
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
    return pHandle->bPasPowerAlgorithm;
}

/**
    * @brief  Reset the PAS algorithm
    * @param  Pedal Assist handle, new pas algorithm
    * @retval None
    */
void PedalAssist_SetPASAlgorithm(PAS_Handle_t * pHandle, PasAlgorithm_t aPASAlgo)
{
    ASSERT(pHandle != NULL);
    
    pHandle->bPasPowerAlgorithm = aPASAlgo;
}

/**
    * @brief  Reset Cadence State Pas Dection
    * @param  None
    * @retval None
    */
void PedalAssist_ResetCadenceStatePasDection(void)
{
    PasCadenceState = CADENCE_DETECTION_STARTUP;
}

/**
    * @brief  Try to detect PAS
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_PasDetection(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    //check if the condition to PAS detection, at startup,
    //has been met.
    switch (pHandle->bStartupPasAlgorithm)
    {
        case noSensorUse:
            
            PedalAssist_ResetPASDetected(pHandle);
        
        break;
            
        case TorqueSensorUse:
            
            if (pHandle->bTorqueStartupPASDetected == true)
            {
                PedalAssist_SetPASDetected(pHandle);
            }
        
        break;
        
        case CadenceSensorUse:
            
            if (pHandle->bCadenceStartupPASDetected == true)
            {
                PedalAssist_SetPASDetected(pHandle);
            }
        
        break;
        
        case HybridAndSensorUse:
            
            if ((pHandle->bTorqueStartupPASDetected == true) && (pHandle->bCadenceStartupPASDetected == true))
            {
                PedalAssist_SetPASDetected(pHandle);
            }
        
        break;
        
        case HybridOrSensorUse:
            
            if ((pHandle->bTorqueStartupPASDetected == true) || (pHandle->bCadenceStartupPASDetected == true))
            {
                PedalAssist_SetPASDetected(pHandle);
            }
        
        break;
            
         //must not go into this state.
        default:
            
            //reset the system using a software reset
            ASSERT(false);
        
        break;
    }
    
    //check if the condition to PAS detection, at running state,
    //has been met.
    switch (pHandle->bRunningPasAlgorithm)
    {
        case noSensorUse:
            
            PedalAssist_ResetPASDetected(pHandle);
        
        break;
            
        case TorqueSensorUse:
            
            if (pHandle->bTorqueRunningPASDetected == true)
            {
                PedalAssist_SetPASDetected(pHandle);
            }
        
        break;
        
        case CadenceSensorUse:
            
            if (pHandle->bCadenceRunningPASDetected == true)
            {
                PedalAssist_SetPASDetected(pHandle);
            }
        
        break;
        
        case HybridAndSensorUse:
            
            if ((pHandle->bTorqueRunningPASDetected == true) && (pHandle->bCadenceRunningPASDetected == true))
            {
                PedalAssist_SetPASDetected(pHandle);
            }
        
        break;
        
        case HybridOrSensorUse:

            if ((pHandle->bTorqueRunningPASDetected == true) || (pHandle->bCadenceRunningPASDetected == true))
            {
                PedalAssist_SetPASDetected(pHandle);
            }
        
        break;
            
         //must not go into this state.
        default:
            
            //reset the system using a software reset
            ASSERT(false);
        
        break;
    }
}

/**
    * @brief  Reset Pas detection flag on Cadence on startup state.
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_ResetCadenceStartupPasDection(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    pHandle->bCadenceStartupPASDetected = false;
}

/**
    * @brief  Set Pas detection flag from Cadence on startup state.
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_SetCadenceStartupPasDection(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    pHandle->bCadenceStartupPASDetected = true;
}

/**
    * @brief  Reset Pas detection flag from Cadence on running state.
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_ResetCadenceRunningPasDection(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    pHandle->bCadenceRunningPASDetected = false;
}

/**
    * @brief  Set Pas detection flag from Cadence on running state.
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_SetCadenceRunningPasDection(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    pHandle->bCadenceRunningPASDetected = true;
}

/**
    * @brief  Reset Pas detection flag from Torque on startup state.
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_ResetTorqueStartupPasDection(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    pHandle->bTorqueStartupPASDetected = false;
}

/**
    * @brief  Set Pas detection flag from Torque on startup state.
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_SetTorqueStartupPasDection(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    pHandle->bTorqueStartupPASDetected = true;
}

/**
    * @brief  Reset Pas detection flag from Torque on running state.
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_ResetTorqueRunningPasDection(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    pHandle->bTorqueRunningPASDetected = false;
}

/**
    * @brief  Set Pas detection flag from Torque on running state.
    * @param  Pedal Assist handle
    * @retval None
    */
void PedalAssist_SetTorqueRunningPasDection(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    pHandle->bTorqueRunningPASDetected = true;
}
