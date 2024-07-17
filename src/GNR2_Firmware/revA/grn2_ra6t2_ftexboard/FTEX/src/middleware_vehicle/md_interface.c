/**
  * @file    md_interface.h
  * @author  Sami Bouzid, FTEX
  * @brief   Module that provides an interface to control multiple motor drives.
  *          M1 is the local drive, whereas M2, M3, M4, ... can be controlled externally using this interface.
*/

#include "md_interface.h"
#include "ASSERT_FTEX.h"
#include "mc_interface.h"
#include "wheel.h"

/*
* see function definition
*/
void MDI_Init(MultipleDriveInterfaceHandle_t * pHandle, SlaveMotorHandle_t * pSlaveM2, MC_Setup_t MCSetup)
{
    ASSERT(pHandle != NULL);
    ASSERT(pSlaveM2 != NULL);

    pHandle->pSlaveM2 = pSlaveM2;
    
    //Power initialization is not supported with dual motor, will have to change default values in MC layer to initialize the parameters that come from the battery
    MCInterface_PowerInit(MCSetup);
    MCInterface_SpeedLimitEnInit(MCSetup);    
}

/*
* see function definition
*/
uint16_t MDI_GetBusVoltageInVoltx100()
{
    return MCInterface_GetBusVoltageInVoltx100();
}

/*
* see function definition
*/
void MDI_ExecSpeedRamp(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor, int16_t hFinalSpeed)
{
    ASSERT(pHandle != NULL);
    
    int16_t wheelRPM = (int16_t) Wheel_GetWheelRpmFromSpeed((uint16_t)hFinalSpeed);
    int16_t motorRPM = (int16_t) round(wheelRPM * MCInterface_GetMotorGearRatio());         
    
    switch (bMotor)
    {
        case M1:            
            MCInterface_ExecSpeedRamp(motorRPM);
            break;
        case M2:
            break;
        default:
            break;
    }
}

/*
* see function definition
*/
void MDI_ExecTorqueRamp(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor, int16_t hFinalTorque)
{
    ASSERT(pHandle != NULL);
    switch (bMotor)
    {
        case M1:
            MCInterface_ExecTorqueRamp(hFinalTorque);
            break;
        case M2:
            SlaveMCInterface_ExecTorqueRamp(pHandle->pSlaveM2, hFinalTorque);
            break;
        default:
            break;
    }
}

/*
* see function definition
*/
void MDI_SetCurrentReferences(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor, qd_t Iqdref)
{
    ASSERT(pHandle != NULL);
    switch (bMotor)
    {
        case M1:
            MCInterface_SetCurrentReferences(Iqdref);
            break;
        case M2:
            break;
        default:
            break;
    }
}

/*
* see function definition
*/
bool MDI_StartMotor(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    bool bReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_StartMotor();
            break;
        case M2:
            bReturnValue = SlaveMCInterface_StartMotor(pHandle->pSlaveM2);
            break;
        default:
            break;
    }

    return bReturnValue;
}

/*
* see function definition
*/
bool MDI_StopMotor(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    bool bReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_StopMotor();
            break;
        case M2:
            bReturnValue = SlaveMCInterface_StopMotor(pHandle->pSlaveM2);
            break;
        default:
            break;
    }

    return bReturnValue;
}

/*
* see function definition
*/
bool MDI_CriticalFaultAcknowledged(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    bool bReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_CriticalFaultAcknowledged();
            break;
        case M2:
            bReturnValue = SlaveMCInterface_CriticalFaultAcknowledged(pHandle->pSlaveM2);
            break;
        default:
            break;
    }

    return bReturnValue;
}

/*
* see function definition
*/
MotorState_t  MDI_GetSTMState(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    MotorState_t bReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_GetSTMState();
            break;
        case M2:
            bReturnValue = SlaveMCInterface_GetSTMState(pHandle->pSlaveM2);
            break;
        default:
            break;
    }

    return bReturnValue;
}

/*
* see function definition
*/
uint32_t MDI_GetOccurredCriticalFaults(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    uint32_t wReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            wReturnValue = MCInterface_GetOccurredCriticalFaults();
            break;
        case M2:
            wReturnValue = SlaveMCInterface_GetOccurredCriticalFaults(pHandle->pSlaveM2);
            break;
        default:
            break;
    }

    return wReturnValue;
}

/*
* see function definition
*/
uint32_t MDI_GetCurrentErrors(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    uint32_t wReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            wReturnValue = MCInterface_GetCurrentErrors();
            break;
        case M2:
            wReturnValue = SlaveMCInterface_GetCurrentErrors(pHandle->pSlaveM2);
            break;
        default:
            break;
    }

    return wReturnValue;
}

/*
* see function definition
*/
uint32_t MDI_GetOccurredErrors(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    uint32_t wReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            wReturnValue = MCInterface_GetOccurredErrors();
            break;
        case M2:
            wReturnValue = SlaveMCInterface_GetOccurredErrors(pHandle->pSlaveM2);
            break;
        default:
            break;
    }

    return wReturnValue;
}

/*
* see function definition
*/
uint32_t MDI_GetOccurredWarnings(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    uint32_t wReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            wReturnValue = MCInterface_GetOccurredWarning();
            break;
        case M2:
            wReturnValue = SlaveMCInterface_GetOccurredWarnings(pHandle->pSlaveM2);
            break;
        default:
            break;
    }

    return wReturnValue;
}

/*
* see function definition
*/
uint32_t MDI_GetCurrentCriticalFaults(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    uint32_t wReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            wReturnValue = MCInterface_GetCurrentCriticalFaults();
            break;
        case M2:
            wReturnValue = SlaveMCInterface_GetCurrentCriticalFaults(pHandle->pSlaveM2);
            break;
        default:
            break;
    }

    return wReturnValue;
}

/*
* see function definition
*/
STCModality_t MDI_GetControlMode(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    STCModality_t bReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_GetControlMode();
            break;
        case M2:
            break;
        default:
            break;
    }

    return bReturnValue;
}

/*
* see function definition
*/
int16_t MDI_GetImposedMotorDirection(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    int16_t bReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_GetImposedMotorDirection();
            break;
        case M2:
            break;
        default:
            break;
    }

    return bReturnValue;
}

/*
* see function definition
*/
int16_t MDI_GetLastRampFinalSpeed(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    int16_t bReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_GetLastRampFinalSpeed();
            break;
        case M2:
            break;
        default:
            break;
    }

    return bReturnValue;
}

/*
* see function definition
*/
bool MDI_IsRampCompleted(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    bool bReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_IsRampCompleted();
            break;
        case M2:
            break;
        default:
            break;
    }

    return bReturnValue;
}

/*
* see function definition
*/
void MDI_StopRamp(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    switch (bMotor)
    {
        case M1:
            MCInterface_StopRamp();
            break;
        case M2:
            break;
        default:
            break;
    }
}

/*
* see function definition
*/
bool MDI_GetSpdSensorReliability(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    bool bReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_GetSpdSensorReliability();
            break;
        case M2:
            break;
        default:
            break;
    }

    return bReturnValue;
}

/*
* see function definition
*/
int16_t MDI_GetAvrgMecSpeedUnit(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    int16_t hReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            hReturnValue = MCInterface_GetAvrgMecSpeedUnit();
            break;
        case M2:
            hReturnValue = SlaveMCInterface_GetAvrgMecSpeedUnit(pHandle->pSlaveM2);
            break;
        default:
            break;
    }

    return hReturnValue;
}

/*
* see function definition
*/
int16_t MDI_GetMecSpeedRefUnit(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    int16_t hReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            hReturnValue = MCInterface_GetMecSpeedRefUnit();
            break;
        case M2:
            break;
        default:
            break;
    }

    return hReturnValue;
}

/*
* see function definition
*/
ab_t MDI_GetIab(uint8_t bMotor)
{
    ab_t ReturnValue = {0};

    switch (bMotor)
    {
        case M1:
            ReturnValue = MCInterface_GetIab();
            break;
        case M2:
            break;
        default:
            break;
    }

    return ReturnValue;
}

/*
* see function definition
*/
AlphaBeta_t MDI_GetIalphabeta(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    AlphaBeta_t ReturnValue = {0};

    switch (bMotor)
    {
        case M1:
            ReturnValue = MCInterface_GetIalphabeta();
            break;
        case M2:
            break;
        default:
            break;
    }

    return ReturnValue;
}

/*
* see function definition
*/
qd_t MDI_GetIqd(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    qd_t ReturnValue = {0};

    switch (bMotor)
    {
        case M1:
            ReturnValue = MCInterface_GetIqd();
            break;
        case M2:
            break;
        default:
            break;
    }

    return ReturnValue;
}

/*
* see function definition
*/
qd_t MDI_GetIqdHF(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    qd_t ReturnValue = {0};

    switch (bMotor)
    {
        case M1:
            ReturnValue = MCInterface_GetIqdHF();
            break;
        case M2:
            break;
        default:
            break;
    }

    return ReturnValue;
}

/*
* see function definition
*/
qd_t MDI_GetIqdref(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    qd_t ReturnValue = {0};

    switch (bMotor)
    {
        case M1:
            ReturnValue = MCInterface_GetIqdref();
            break;
        case M2:
            break;
        default:
            break;
    }

    return ReturnValue;
}

/*
* see function definition
*/
qd_t MDI_GetVqd(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    qd_t ReturnValue = {0};

    switch (bMotor)
    {
        case M1:
            ReturnValue = MCInterface_GetVqd();
            break;
        case M2:
            break;
        default:
            break;
    }

    return ReturnValue;
}

/*
* see function definition
*/
AlphaBeta_t MDI_GetValphabeta(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    AlphaBeta_t ReturnValue = {0};

    switch (bMotor)
    {
        case M1:
            ReturnValue = MCInterface_GetValphabeta();
            break;
        case M2:
            break;
        default:
            break;
    }

    return ReturnValue;
}

/*
* see function definition
*/
int16_t MDI_GetElAngledpp(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    int16_t hReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            hReturnValue = MCInterface_GetElAngledpp();
            break;
        case M2:
            break;
        default:
            break;
    }

    return hReturnValue;
}

/*
* see function definition
*/
int16_t MDI_GetTeref(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    int16_t hReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            hReturnValue = MCInterface_GetTeref();
            break;
        case M2:
            break;
        default:
            break;
    }

    return hReturnValue;
}

/*
* see function definition
*/
int16_t MDI_GetPhaseCurrentAmplitude(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    int16_t hReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            hReturnValue = MCInterface_GetPhaseCurrentAmplitude();
            break;
        case M2:
            break;
        default:
            break;
    }

    return hReturnValue;
}

/*
* see function definition
*/
int16_t MDI_GetPhaseVoltageAmplitude(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    int16_t hReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            hReturnValue = MCInterface_GetPhaseVoltageAmplitude();
            break;
        case M2:
            break;
        default:
            break;
    }

    return hReturnValue;
}

/**
  *  Getting the controller NTC temperature value
  */
int16_t MDI_GetControllerTemp(MultipleDriveInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return MCInterface_GetControllerTemp();
}

/**
  *  Getting the controller NTC temperature value
  */
int16_t MDI_GetMotorTemp(MultipleDriveInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return MCInterface_GetMotorTemp();
}

/*
* see function definition
*/
void MDI_Clear_Iqdref(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    switch (bMotor)
    {
        case M1:
            MCInterface_GetCurrentCriticalFaults();
            break;
        case M2:
            break;
        default:
            break;
    }
}

/**
  *  Obtain the motor torque referenece for a motor
  */
uint16_t MDI_GetMotorTorqueReference(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    
    uint16_t MotorTorqueRef = 0;
    switch (bMotor)
    {
        case M1:            
            MotorTorqueRef = (uint16_t) abs(MCInterface_GetTorqueReference(M1));
            break;
        case M2:
            MotorTorqueRef = (uint16_t) abs(MCInterface_GetTorqueReference(M2));
            break;
        default:
            break;
    }
    
    return MotorTorqueRef;
}

/**
  *  Obtain the max application power
  */
uint16_t MDI_GetMaxPositivePower(MultipleDriveInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return MCInterface_GetMaxPositivePower();
}

/**
  *  Function sets the torque control speed limit does the conversion from desired 
  *  wheel speed in kmH to motor rpm.Is also the wrapper for the function SpdTorqCtrl_SetSpeedLimit
  */
void MDI_SetTorqueSpeedLimit(MultipleDriveInterfaceHandle_t * pHandle, uint16_t speedKMH)
{
    ASSERT(pHandle != NULL);
    
    int16_t desiredMotorRPM;
    float gearRatio = MCInterface_GetMotorGearRatio();
    uint16_t wheelRpm = (uint16_t) Wheel_GetWheelRpmFromSpeed(speedKMH);  // Convert desired wheel speed in kmH to desired wheel RPM
    
    MCInterface_SetSpeedLimitWheelRpm(wheelRpm);
    
    desiredMotorRPM = (int16_t) round(gearRatio * wheelRpm);  // Convert desired wheel rpm to desired motor rpm
    
    
    MCInterface_SetSpeedLimit(desiredMotorRPM); // Set the torque control speed limitation

}    

/**
  *  Get the Max safe current from motor control.
  */
int16_t MDI_GetMaxCurrent()
{
    return MCInterface_GetMaxCurrent();
}

/**
  *  Get the maximum ongoing current,
  */
int16_t MDI_GetOngoingMaxCurrent()
{
    return MCInterface_GetOngoingMaxCurrent();
}

/**
  *  Set the maximum ongoing current, 
  */
void MDI_SetOngoingMaxCurrent(int16_t aCurrent)
{
    MCInterface_SetOngoingMaxCurrent(aCurrent);
}

/**
  *  Function sets the wheel speed limit. Used for mid-drives. Is also the wrapper for the function SpdTorqCtrl_SetSpeedLimitWheelRpm
  */
void MDI_SetWheelRPM(MultipleDriveInterfaceHandle_t * pHandle, uint16_t aWheelRPM)
{
    ASSERT(pHandle != NULL);
    MCInterface_SetWheelRPM(aWheelRPM);
}  

/**
  *  Get the motor gear ratio
  */
float MDI_GetMotorGearRatio(MultipleDriveInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return MCInterface_GetMotorGearRatio();
}

/**
  *  Get the motor type
  */
MotorType_t MDI_GetMotorType(MultipleDriveInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return MCInterface_GetMotorType();
}

/**
  *  Get the nominal torque
  */
uint16_t MDI_GetNominalTorque(MultipleDriveInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return MCInterface_GetNominalTorque();
}

/**
  *  Get the starting torque
  */
uint16_t MDI_GetStartingTorque(MultipleDriveInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return MCInterface_GetStartingTorque();
}

/**
  *  Get the RS value
  */
float MDI_GetRS(MultipleDriveInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return MCInterface_GetRS();
}

/**
  *  Get the number of magnets on the wheel speed sensor
  */
uint8_t MDI_GetWheelSpdSensorNbrPerRotation(MultipleDriveInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return MCInterface_GetWheelSpdSensorNbrPerRotation();
}

/**
  *  Get whether the motor temp sensor is mixed
  */
bool MDI_GetMotorTempSensorMixed(MultipleDriveInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return MCInterface_GetMotorTempSensorMixed();
}

/**
  *  Set speed limit of the wheel 
  */
void MDI_SetSpeedLimitWheelRpm(uint16_t wheelRpm)
{    
    MCInterface_SetSpeedLimitWheelRpm(wheelRpm);
}
    
/**
  *  Set speed limit of the motor 
  */
void MDI_SetSpeedLimit(int16_t desiredMotorRPM)
{
    MCInterface_SetSpeedLimit(desiredMotorRPM);
}

/**
  *  Get max measurable current
  */
float MDI_GetMaxMeasurableCurrent()
{
    return MCInterface_GetMaxMeasurableCurrent();
}
    
/**
  *  Set max bus current
  */
void MDI_SetMaxBusCurrent(uint16_t maxBusCurrent)
{
    MCInterface_SetMaxBusCurrent(maxBusCurrent);
}
    
/**
  *  Set max continuous current
  */
void MDI_SetMaxContinuousCurrent(uint16_t maxContinuousCurrent)
{
    MCInterface_SetMaxContinuousCurrent(maxContinuousCurrent);
}

/**
  *  Set power foldback end value
  */
void MDI_SetPowerFoldbackEndValue(int32_t endValue)
{
    MCInterface_SetPowerFoldbackEndValue(endValue);
}

/**
  *  Set power foldback range
  */
void MDI_SetPowerFoldbackRange(uint16_t range)
{
    MCInterface_SetPowerFoldbackRange(range);
}

/**
  *  Set motor temp sensor type
  */
void MDI_SetMotorTempSensorType(uint8_t sensorType)
{
    MCInterface_SetMotorTempSensorType(sensorType);
}


/**
  *  Set motor NTC Beta coefficient
  */
void MDI_SetMotorNTCBetaCoef(uint16_t NTCBetaCoef)
{
    MCInterface_SetMotorNTCBetaCoef(NTCBetaCoef);
}

/**
  *  Set motor NTC resistance coefficient
  */
void MDI_SetMotorNTCResistanceCoef(float NTCResCoef)
{
    MCInterface_SetMotorNTCResistanceCoef(NTCResCoef);
}

/**
  *  Set is motor signal mixed
  */
void MDI_SetIsMotorSignalMixed(bool value)
{
    MCInterface_SetIsMotorSignalMixed(value);
}

/**
  *  Set is min signal threshold value for mixed WSS and temp signals
  */
void MDI_SetMinSignalThresholdValueMixed(uint16_t value)
{
    MCInterface_SetMinSignalThresholdValueMixed(value);
}

/**
  *  Set max wheel speed period for mixed WSS and temp signals
  */
void MDI_SetMaxWheelSpeedPeriodUsValueMixed(uint32_t value)
{
    MCInterface_SetMaxWheelSpeedPeriodUsValueMixed(value);
}

/**
  *  Get extracted wheel speed from mixed WSS and temp signals
  */
float MDI_GetExtractedWheelSpeedMixed(void)
{
    return MCInterface_GetExtractedWheelSpeedMixed();
}

/**
  *  Get extracted motor temp from mixed WSS and temp signals
  */
bool MDI_EnableRegen(uint8_t bMotor)
{
    switch (bMotor)
    {
    case M1:
        MCInterface_EnableRegen();
        return true;
        break;
    case M2:
        // SlaveInterface function to be called for dual
        break;
    }
    return false;
}

/**
  *  Get extracted motor temp from mixed WSS and temp signals
  */
bool MDI_DisableRegen(uint8_t bMotor)
{
    switch (bMotor)
    {
    case M1:
        MCInterface_DisableRegen();
        return true;
        break;
    case M2:
        // SlaveInterface function to be called for dual
        break;
    }
    return false;
}

/**
  *  Set maximum allowed current for regen operation
  */
bool MDI_SetRegenMaxCurrent(uint8_t bMotor, uint16_t maxCurrent)
{
    switch (bMotor)
    {
    case M1: 
        return MCInterface_SetRegenMaxCurrent(maxCurrent);
        break;
    case M2:
        // SlaveInterface function to be called for dual
        break; 
    }
    return false;
}

/**
  *  Get the maximum allowed current for regen operation
  */
int16_t MDI_GetRegenMaxCurrent(uint8_t bMotor)
{
    switch (bMotor)
    {
    case M1:
        return MCInterface_GetRegenMaxNegativeCurrent();
        break;
    case M2:
        // SlaveInterface function to be called for dual
        break;
    }
    return 0;
}

/**
  *  Set Regen minimum current
  */
bool MDI_SetRegenMinCurrent(uint8_t bMotor, uint16_t minCurrent)
{
    switch (bMotor)
    {
    case M1:
        return MCInterface_SetMinRegenCurrent(minCurrent);
        break;
    case M2:
        // SlaveInterface function to be called for dual
        break;
    }
    return false;
}

/**
  *  Get Regen minimum current
  */
int16_t MDI_GetRegenMinCurrent(uint8_t bMotor)
{
    switch (bMotor)
    {
    case M1:
        return MCInterface_GetMinNegativeCurrent();
        break;
    case M2:
        // SlaveInterface function to be called for dual
        break;
    }
    return 0;
}

/**
  *  Get extracted motor temp from mixed WSS and temp signals
  */
bool MDI_SetRegenMinSpeed(uint8_t bMotor, uint16_t minWheelRPM)
{
    int16_t motorRPM = (int16_t) round(minWheelRPM * MCInterface_GetMotorGearRatio()); 
    switch (bMotor)
    {
    case M1:
        return MCInterface_SetRegenMinSpeed(motorRPM);
        break;
    case M2:
        // SlaveInterface function to be called for dual
        break;
    }
    return false;
}

/**
  *  Get extracted motor temp from mixed WSS and temp signals
  */
bool MDI_SetRegenMaxVoltage(uint8_t bMotor, uint16_t maxVoltage)
{
    switch (bMotor)
    {
    case M1:
        return MCInterface_SetRegenMaxVoltage(maxVoltage);
        break;
    case M2:
        // SlaveInterface function to be called for dual
        break;
    }
    return false;
}

/**
  *  Get Regen max alloed voltage
  */
uint16_t MDI_GetRegenMaxVoltage(uint8_t bMotor)
{
    switch (bMotor)
    {
    case M1:
        return MCInterface_GetRegenMaxVoltage();
        break;
    case M2:
        // SlaveInterface function to be called for dual
        break;
    }
    return 0;
}

/**
  *  Set the regen Level Percent
  */
bool MDI_SetRegenLevelPercent(uint8_t bMotor, uint8_t hLevelPercent)
{
    switch (bMotor)
    {
    case M1:
        return MCInterface_SetRegenLevelPercent(hLevelPercent);
        break;
    case M2:
        // SlaveInterface function to be called for dual
        break;
    }
    return false;
}

/**
  *  Set the regen ramp Percent in milliseconds.
  */
bool MDI_SetRegenRampPercent(uint8_t bMotor, uint16_t hRampPercent)
{
    switch (bMotor)
    {
    case M1:
        return MCInterface_SetRegenRampPercent(hRampPercent);
        break;
    case M2:
        // SlaveInterface function to be called for dual
        break;
    }
    return false;
}

/**
  *  Get extracted motor temp from mixed WSS and temp signals
  */
uint16_t MDI_GetRegenRampPercent(uint8_t bMotor)
{
    switch (bMotor)
    {
    case M1:
        return MCInterface_GetRegenRampPercent();
        break;
    case M2:
        // SlaveInterface function to be called for dual
        break;
    }
    return 0;
}
