/**
  * @file    md_interface.h
  * @author  Sami Bouzid, FTEX
  * @brief   Module that provides an interface to control multiple motor drives.
  *          M1 is the local drive, whereas M2, M3, M4, ... can be controlled externally using this interface.
*/

#include "md_interface.h"
#include "ASSERT_FTEX.h"
#include "wheel.h"

/*
* see function definition
*/
void MDI_Init(MultipleDriveInterfaceHandle_t * pHandle, MotorControlInterfaceHandle_t * pMCI, SlaveMotorHandle_t * pSlaveM2, MC_Setup_t MCSetup)
{
    ASSERT(pHandle != NULL);
    ASSERT(pMCI != NULL);
    ASSERT(pSlaveM2 != NULL);

    pHandle->pMCI = pMCI;
    pHandle->pSlaveM2 = pSlaveM2;
    
    //Power initialization is not supported with dual motor, will have to change default values in MC layer to initialize the parameters that come from the battery
    MCInterface_PowerInit(pHandle->pMCI, MCSetup);
    MCInterface_SpeedLimitEnInit(pHandle->pMCI, MCSetup);
    
}

/*
* see function definition
*/
uint16_t MDI_GetBusVoltageInVoltx100(MotorControlInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return MCInterface_GetBusVoltageInVoltx100(pHandle);
}

/*
* see function definition
*/
void MDI_ExecSpeedRamp(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor, int16_t hFinalSpeed)
{
    ASSERT(pHandle != NULL);
    
    int16_t wheelRPM = (int16_t) Wheel_GetWheelRpmFromSpeed((uint16_t)hFinalSpeed);
    int16_t motorRPM = (int16_t) round(wheelRPM * pHandle->pMCI->pSpeedTorqCtrl->fGearRatio);         
    
    switch (bMotor)
    {
        case M1:            
            MCInterface_ExecSpeedRamp(pHandle->pMCI, motorRPM);
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
            MCInterface_ExecTorqueRamp(pHandle->pMCI, hFinalTorque);
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
            MCInterface_SetCurrentReferences(pHandle->pMCI, Iqdref);
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
            bReturnValue = MCInterface_StartMotor(pHandle->pMCI);
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
            bReturnValue = MCInterface_StopMotor(pHandle->pMCI);
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
            bReturnValue = MCInterface_CriticalFaultAcknowledged(pHandle->pMCI);
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
            bReturnValue = MCInterface_GetSTMState(pHandle->pMCI);
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
            wReturnValue = MCInterface_GetOccurredCriticalFaults(pHandle->pMCI);
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
            wReturnValue = MCInterface_GetCurrentErrors(pHandle->pMCI);
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
            wReturnValue = MCInterface_GetOccurredErrors(pHandle->pMCI);
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
            wReturnValue = MCInterface_GetOccurredWarning(pHandle->pMCI);
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
            wReturnValue = MCInterface_GetCurrentCriticalFaults(pHandle->pMCI);
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
            bReturnValue = MCInterface_GetControlMode(pHandle->pMCI);
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
            bReturnValue = MCInterface_GetImposedMotorDirection(pHandle->pMCI);
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
            bReturnValue = MCInterface_GetLastRampFinalSpeed(pHandle->pMCI);
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
            bReturnValue = MCInterface_IsRampCompleted(pHandle->pMCI);
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
            MCInterface_StopRamp(pHandle->pMCI);
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
            bReturnValue = MCInterface_GetSpdSensorReliability(pHandle->pMCI);
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
            hReturnValue = MCInterface_GetAvrgMecSpeedUnit(pHandle->pMCI);
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
            hReturnValue = MCInterface_GetMecSpeedRefUnit(pHandle->pMCI);
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
ab_t MDI_GetIab(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    ab_t ReturnValue = {0};

    switch (bMotor)
    {
        case M1:
            ReturnValue = MCInterface_GetIab(pHandle->pMCI);
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
            ReturnValue = MCInterface_GetIalphabeta(pHandle->pMCI);
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
            ReturnValue = MCInterface_GetIqd(pHandle->pMCI);
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
            ReturnValue = MCInterface_GetIqdHF(pHandle->pMCI);
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
            ReturnValue = MCInterface_GetIqdref(pHandle->pMCI);
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
            ReturnValue = MCInterface_GetVqd(pHandle->pMCI);
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
            ReturnValue = MCInterface_GetValphabeta(pHandle->pMCI);
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
            hReturnValue = MCInterface_GetElAngledpp(pHandle->pMCI);
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
            hReturnValue = MCInterface_GetTeref(pHandle->pMCI);
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
            hReturnValue = MCInterface_GetPhaseCurrentAmplitude(pHandle->pMCI);
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
            hReturnValue = MCInterface_GetPhaseVoltageAmplitude(pHandle->pMCI);
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
    return MCInterface_GetControllerTemp(pHandle->pMCI);
}

/**
  *  Getting the controller NTC temperature value
  */
int16_t MDI_GetMotorTemp(MultipleDriveInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return MCInterface_GetMotorTemp(pHandle->pMCI);
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
            MCInterface_GetCurrentCriticalFaults(pHandle->pMCI);
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
            MotorTorqueRef = (uint16_t) abs(MCInterface_GetTorqueReference(pHandle->pMCI, M1));
            break;
        case M2:
            MotorTorqueRef = (uint16_t) abs(MCInterface_GetTorqueReference(pHandle->pMCI, M2));
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
    return MCInterface_GetMaxPositivePower(pHandle->pMCI);
}

/**
  *  Function sets the torque control speed limit does the conversion from desired 
  *  wheel speed in kmH to motor rpm.Is also the wrapper for the function SpdTorqCtrl_SetSpeedLimit
  */
void MDI_SetTorqueSpeedLimit(MultipleDriveInterfaceHandle_t * pHandle, uint16_t speedKMH)
{
    ASSERT(pHandle != NULL);
    ASSERT(pHandle->pMCI->pSpeedTorqCtrl);
    
    int16_t desiredMotorRPM;
    float gearRatio = pHandle->pMCI->pSpeedTorqCtrl->fGearRatio;
    uint16_t wheelRpm = (uint16_t) Wheel_GetWheelRpmFromSpeed(speedKMH);  // Convert desired wheel speed in kmH to desired wheel RPM
    
    SpdTorqCtrl_SetSpeedLimitWheelRpm(pHandle->pMCI->pSpeedTorqCtrl, wheelRpm);
    
    desiredMotorRPM = (int16_t) round(gearRatio * wheelRpm);  // Convert desired wheel rpm to desired motor rpm
    
    
    SpdTorqCtrl_SetSpeedLimit(pHandle->pMCI->pSpeedTorqCtrl, desiredMotorRPM); // Set the torque control speed limitation

}    

/**
  *  Function sets the wheel speed limit. Used for mid-drives. Is also the wrapper for the function SpdTorqCtrl_SetSpeedLimitWheelRpm
  */
void MDI_SetWheelRPM(MultipleDriveInterfaceHandle_t * pHandle, uint16_t aWheelRPM)
{
    ASSERT(pHandle != NULL);
    MCInterface_SetWheelRPM(pHandle->pMCI, aWheelRPM);
}  

/**
  *  Get the motor gear ratio
  */
float MDI_GetMotorGearRatio(MultipleDriveInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return MCInterface_GetMotorGearRatio(pHandle->pMCI);
}

/**
  *  Get the motor type
  */
MotorType_t MDI_GetMotorType(MultipleDriveInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return MCInterface_GetMotorType(pHandle->pMCI);
}

/**
  *  Get the nominal torque
  */
uint16_t MDI_GetNominalTorque(MultipleDriveInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return MCInterface_GetNominalTorque(pHandle->pMCI);
}

/**
  *  Get the starting torque
  */
uint16_t MDI_GetStartingTorque(MultipleDriveInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return MCInterface_GetStartingTorque(pHandle->pMCI);
}

/**
  *  Get the RS value
  */
float MDI_GetRS(MultipleDriveInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return MCInterface_GetRS(pHandle->pMCI);
}

/**
  *  Get the number of magnets on the wheel speed sensor
  */
uint8_t MDI_GetWheelSpdSensorNbrPerRotation(MultipleDriveInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return MCInterface_GetWheelSpdSensorNbrPerRotation(pHandle->pMCI);
}

/**
  *  Get whether the motor temp sensor is mixed
  */
bool MDI_GetMotorTempSensorMixed(MultipleDriveInterfaceHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return MCInterface_GetMotorTempSensorMixed(pHandle->pMCI);
}
