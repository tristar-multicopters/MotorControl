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
void PedalAssist_Init(PAS_Handle_t * pHandle, Delay_Handle_t * pPTSstuckDelay, uint16_t maxTorque, uint8_t wheelSpdSensorNbrPerRotation)
{
    ASSERT(pHandle != NULL);
    
    //init pas torque ramps
	
    //init walkmode ramp amd pas max torque
    pHandle->sParameters.hPASMaxTorque = (int16_t)maxTorque;

    PedalSpdSensor_Init(pHandle->pPSS);
    WSSInit(wheelSpdSensorNbrPerRotation);
    PedalTorqSensor_Init(pHandle->pPTS, pPTSstuckDelay, maxTorque);
    
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
        userConfigSpeed = UserConfigTask_GetWalkmodeSpeed();
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
    
    //safe measure
    if (currentLevel == PAS_LEVEL_0)
    {
        return 0;
    }
    //return torque to walk mode.
    if (currentLevel == PAS_LEVEL_WALK)
    {
       return pHandle->sParameters.walkModeTorqueRatio;
    }
         
    // Multiple gain on the motor torque
    hRefTorqueS = ((hReadTS * pHandle->sParameters.bTorqueGain[currentLevel])/PAS_PERCENTAGE);
    
    //Caclulate the final torque to be applied based on the max torque percentage to the current PAS level.
    hRefTorqueS = (hRefTorqueS/PAS_PERCENTAGE) * pHandle->sParameters.PASMaxTorqRatiosInPercentage[pHandle->bCurrentAssistLevel];
    
    //Calculate the min motor torque to be applied, for a specific PAS level.
    hRefMinTorqueS = (pHandle->sParameters.hPASMaxTorque/PAS_PERCENTAGE)*pHandle->sParameters.PASMinTorqRatiosInPercentage[pHandle->bCurrentAssistLevel];
    
    //verify if the applied torque is less or equal than the minimum torque
    //if true, apllied the minimum torque.
    if (hRefTorqueS < hRefMinTorqueS)
    {
        hRefTorqueS = hRefMinTorqueS;
    }
    
    // Calculate the maximo safety motor torque for not exceeding the maximum torque value for a specific pas level
    hMaxLevelTorq_Temp = (pHandle->sParameters.PASMaxTorqRatiosInPercentage[pHandle->bCurrentAssistLevel] * pHandle->sParameters.hPASMaxTorque)/PAS_PERCENTAGE;
    
    //Verify if the current motor torque execeed the maximo torque value to current PAS level.
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
    * @brief  Get walkmode torque
    * @param  Pedal Assist handle
    * @retval pRefTorqueS in int16
    */
int16_t PedalAssist_GetWalkmodeTorque(PAS_Handle_t * pHandle)
{
    return (pHandle->sParameters.walkModeTorqueRatio * pHandle->sParameters.hPASMaxTorque)/100;
}

/**
    * @brief  Update the PAS Handle power according to Cadence and Torque overrides
    * @param  Pedal Assist handle
    */
void PedalAssist_PASPowerDetection(PAS_Handle_t *pHandle)
{
    ASSERT(pHandle != NULL);
    
    // Validate the PAS Power Enable depending on the value of CADENCE_AND_OR_TORQUE (PAS Power AND/OR)
    bool StartupPowerEnable = ((pHandle->bCadenceStartupPASDetected | pHandle->bTorqueStartupPASDetected) & ~(CADENCE_AND_OR_TORQUE)) | 
                              ((pHandle->bCadenceStartupPASDetected & pHandle->bTorqueStartupPASDetected) & CADENCE_AND_OR_TORQUE);
    
    bool RuntimePowerEnable = ((pHandle->bCadenceRunningPASDetected | pHandle->bTorqueRunningPASDetected) & ~(CADENCE_AND_OR_TORQUE)) | 
                              ((pHandle->bCadenceRunningPASDetected & pHandle->bTorqueRunningPASDetected) & CADENCE_AND_OR_TORQUE);
    
    bool PowerEnable = StartupPowerEnable | RuntimePowerEnable;

    if(PowerEnable) pHandle->bPASPowerEnable = true;
    else pHandle->bPASPowerEnable = false;
}

/**
    * @brief  Check if the user is currently in cadence state, is currently pedalling
    * @param  Pedal Assist handle
    * @retval True if cadence(pedalling activity) is detected
    */
bool PedalAssist_IsCadenceDetected(PAS_Handle_t *pHandle)
{
    ASSERT(pHandle != NULL);
    if(pHandle->bCadenceRunningPASDetected || pHandle->bCadenceStartupPASDetected)
        return true;
    return false;
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
    
    hWheelRPM = (uint16_t) WheelSpdSensor_GetSpeedRPM();
    
    /* Calculate the offset based on ration percentage */
    if(hWheelRPM <= pHandle->pPTS->hParameters.hStartupOffsetMTSpeedRPM) // If going at low speed use the startup offset
    {     
        CalculateAverage = true;  // Make sure we calculated the average to check for the threshold       
        hOffsetTemp = (pHandle->pPTS->hParameters.hOffsetMTStartup * pHandle->pPTS->hParameters.hMax) / PAS_PERCENTAGE;
        pHandle->bTorqueStartupPASDetected = true;
        pHandle->bTorqueRunningPASDetected = false;
        pHandle->bPASTorqueRunningOverride = true;
    }
    else
    {
        CalculateAverage = false; // No need for the average      
        hOffsetTemp = (pHandle->pPTS->hParameters.hOffsetMT * pHandle->pPTS->hParameters.hMax) / PAS_PERCENTAGE;
        pHandle->bTorqueStartupPASDetected = false;
        pHandle->bTorqueRunningPASDetected = true;
        pHandle->bPASTorqueRunningOverride = true;
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
        pHandle->bPASTorqueRunningOverride = false;
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
    
    //state machine to handle PAS cadence detection
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
                    pHandle->bPASCadenceRunningOverride = true;
                    //move to the run state detection
                    PasCadenceState = CADENCE_DETECTION_RUNNING;
                }
                else
                {
                    pHandle->bCadenceStartupPASDetected = false;
                    pHandle->bCadenceRunningPASDetected = false;
                    pHandle->bPASCadenceRunningOverride = false;
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
                    pHandle->bPASCadenceRunningOverride = true;
                }
                else
                {
                    pHandle->bCadenceStartupPASDetected = false;
                    pHandle->bCadenceRunningPASDetected = false;
                    pHandle->bPASCadenceRunningOverride = false;

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
    * @brief  Return if PAS Power Enable is active
    * @param  Pedal Assist handle
    * @retval True if PAS Power Enable is detected, false otherwise
    */
bool PedalAssist_IsPowerEnableDetected(PAS_Handle_t *pHandle)
{
    ASSERT(pHandle != NULL);
    if(pHandle->bPASPowerEnable) return true;
    return false;
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
    * @brief  Get the Startup Pas algorithm
    * @param  Pedal Assist handle,
    * @retval PasAlgorithm_t
    */
PasAlgorithm_t PedalAssist_GetStartupPasAlgorithm(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->bStartupPasAlgorithm;
}

/**
    * @brief  Get the Running Pas algorithm
    * @param  Pedal Assist handle,
    * @retval PasAlgorithm_t
    */
PasAlgorithm_t PedalAssist_GetRunningPasAlgorithm(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->bRunningPasAlgorithm;
}

/**
    * @brief  Set startup PAS algorithm
    * @param  Pedal Assist handle, new pas algorithm
    * @retval None
    */
void PedalAssist_SetStartupPASAlgorithm(PAS_Handle_t * pHandle, PasAlgorithm_t aPASAlgo)
{
    ASSERT(pHandle != NULL);
    
    pHandle->bStartupPasAlgorithm = aPASAlgo;
}

/**
    * @brief  Set running PAS algorithm
    * @param  Pedal Assist handle, new pas algorithm
    * @retval None
    */
void PedalAssist_SetRunningPASAlgorithm(PAS_Handle_t * pHandle, PasAlgorithm_t aPASAlgo)
{
    ASSERT(pHandle != NULL);
    
    pHandle->bRunningPasAlgorithm = aPASAlgo;
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
    bool PasDetected = false;
    
    //check if the condition to PAS detection, at startup,
    //has been met.
    switch (pHandle->bStartupPasAlgorithm)
    {
        case noSensorUse:            
            // Do nothing      
        break;
            
        case TorqueSensorUse:            
            if (pHandle->bTorqueStartupPASDetected == true)
            {
                PasDetected = true;
            }        
        break;
        
        case CadenceSensorUse:            
            if (pHandle->bCadenceStartupPASDetected == true)
            {
                PasDetected = true;
            }        
        break;
        
        case HybridAndSensorUse:           
            if ((pHandle->bTorqueStartupPASDetected == true) && (pHandle->bCadenceStartupPASDetected == true))
            {
                PasDetected = true;
            }        
        break;
        
        case HybridOrSensorUse:           
            if ((pHandle->bTorqueStartupPASDetected == true) || (pHandle->bCadenceStartupPASDetected == true))
            {
                PasDetected = true;
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
        //Do nothing                    
        break;
            
        case TorqueSensorUse:            
            if (pHandle->bTorqueRunningPASDetected == true)
            {
                PasDetected = true;
            }        
        break;
        
        case CadenceSensorUse:            
            if (pHandle->bCadenceRunningPASDetected == true)
            {
                PasDetected = true;
            }        
            break;
        
        case HybridAndSensorUse:            
            if ((pHandle->bTorqueRunningPASDetected == true) && (pHandle->bCadenceRunningPASDetected == true))
            {
                PasDetected = true;
            }        
        break;
        
        case HybridOrSensorUse:
            if ((pHandle->bTorqueRunningPASDetected == true) || (pHandle->bCadenceRunningPASDetected == true))
            {
                PasDetected = true;
            }        
        break;
            
        //must not go into this state.
        default:           
            //reset the system using a software reset
            ASSERT(false);        
        break;
    }

    if (PasDetected)
    {
        PedalAssist_SetPASDetected(pHandle);
    }
    else
    {
        PedalAssist_ResetPASDetected(pHandle);
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

/**
    * @brief  Set a new PAS algorithm
    * @param  Pedal Assist handle, new pas algorithm
    * @retval None
    */
void PedalAssist_SetPASAlgorithm(PAS_Handle_t * pHandle, PasAlgorithm_t aPASAlgo)
{

    ASSERT(pHandle != NULL);
    ASSERT((aPASAlgo == TorqueSensorUse) || (aPASAlgo == CadenceSensorUse));
    
    if (aPASAlgo == CadenceSensorUse)
    {
        //Cadence PAS detection algorithm
        pHandle->bStartupPasAlgorithm = CadenceSensorUse;
        pHandle->bRunningPasAlgorithm = CadenceSensorUse;
        
        //make max and min torque the same to create a cadence behavior(power caculation).
        for(uint8_t n = PAS_LEVEL_0;n <= PAS_LEVEL_9;n++)
        {
            pHandle->sParameters.PASMinTorqRatiosInPercentage[n] = pHandle->sParameters.PASMaxTorqRatiosInPercentage[n];
        }
    }
    else
    {
        //Torque PAS detection algorithm
        pHandle->bStartupPasAlgorithm = TorqueSensorUse;
        pHandle->bRunningPasAlgorithm = TorqueSensorUse; 
        
        //setup the system to make a torque behavior(power calculation)
        for(uint8_t n = PAS_LEVEL_0;n <= PAS_LEVEL_9;n++)
        {    
            pHandle->sParameters.PASMinTorqRatiosInPercentage[n] = UserConfigTask_GetPasLevelMinTorque(n); 
        }
    }
}

/**
    * @brief  Check if a torque sensor issue is detected
    * @param  Pedal Assist handle
    * @retval True if torque sensor issue is detected
    */
bool PedalAssist_TorqueSensorIssueDetected(PAS_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);

    // Calculate a torque sensor issue threshold by adding PETAL_TORQUE_SENSOR_ERROR_OFFSET % to the current hOffsetPTS
    uint16_t torqueSensorThreshold = pHandle->pPTS->hParameters.hOffsetPTS * (uint16_t)(1.0 + (float)PETAL_TORQUE_SENSOR_ERROR_OFFSET/100);

    // If the torque sensor value is smaller than threshold or we have pedal pulse detected
    if(PedalTorqSensor_GetAvValue(pHandle->pPTS) < torqueSensorThreshold || PedalSpdSensor_NewPedalPulsesDetected(pHandle->pPSS))
    {
        pHandle->torqueSensorIssueTimer = 0;
        return false;
    }

    // If the torque sensor issue timer is triggered by the set timeout value
    if(pHandle->torqueSensorIssueTimer >= TORQUE_SENSOR_TIMEOUT_THRESHOLD)
    {
        //pHandle->bTorqueSensorIssue = true;
        return true;
    }

    // If we have a high torque sensor value with no pedalling activity, we increment the timer
    if(PedalTorqSensor_GetAvValue(pHandle->pPTS) >= torqueSensorThreshold && !PedalSpdSensor_NewPedalPulsesDetected(pHandle->pPSS))
    {
        pHandle->torqueSensorIssueTimer++;
        //pHandle->bTorqueSensorIssue = false;
        return false;
    }

    return false;
}
