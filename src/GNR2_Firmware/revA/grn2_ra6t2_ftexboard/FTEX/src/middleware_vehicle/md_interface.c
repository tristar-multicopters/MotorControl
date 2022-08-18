/**
  * @file    md_interface.h
  * @author  Sami Bouzid, FTEX
  * @brief   Module that provides an interface to control multiple motor drives.
  *          M1 is the local drive, whereas M2, M3, M4, ... can be controlled externally using this interface.
*/

#include "md_interface.h"
#include "ASSERT_FTEX.h"

void MDI_Init(MultipleDriveInterfaceHandle_t * pHandle,  MotorControlInterfaceHandle_t * pMCI)
{
    ASSERT(pHandle != NULL);
    ASSERT(pMCI != NULL); 
    pHandle->pMCI = pMCI;
}

void MDI_ExecSpeedRamp(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor, int16_t hFinalSpeed)
{
    ASSERT(pHandle != NULL); 
    switch (bMotor)
    {
        case M1:
            MCInterface_ExecSpeedRamp(pHandle->pMCI, hFinalSpeed);
            break;
        case M2:
            break;
        default:
            break;
    }
}

void MDI_ExecTorqueRamp(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor, int16_t hFinalTorque)
{
    ASSERT(pHandle != NULL);
    switch (bMotor)
    {
        case M1:
            MCInterface_ExecTorqueRamp(pHandle->pMCI, hFinalTorque);
            break;
        case M2:
            break;
        default:
            break;
    }
}

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
            break;
        default:
            break;
    }

    return bReturnValue;
}

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
            break;
        default:
            break;
    }

    return bReturnValue;
}

bool MDI_FaultAcknowledged(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    bool bReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_FaultAcknowledged(pHandle->pMCI);
            break;
        case M2:
            break;
        default:
            break;
    }

    return bReturnValue;
}

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
            break;
        default:
            break;
    }

    return bReturnValue;
}

uint16_t MDI_GetOccurredFaults(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    uint16_t bReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_GetOccurredFaults(pHandle->pMCI);
            break;
        case M2:
            break;
        default:
            break;
    }

    return bReturnValue;
}

uint16_t MDI_GetCurrentFaults(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    uint16_t bReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_GetCurrentFaults(pHandle->pMCI);
            break;
        case M2:
            break;
        default:
            break;
    }

    return bReturnValue;
}

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

int16_t MDI_GetAvrgMecSpeedUnit(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    int16_t bReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_GetAvrgMecSpeedUnit(pHandle->pMCI);
            break;
        case M2:
            break;
        default:
            break;
    }

    return bReturnValue;
}

int16_t MDI_GetMecSpeedRefUnit(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    int16_t bReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_GetMecSpeedRefUnit(pHandle->pMCI);
            break;
        case M2:
            break;
        default:
            break;
    }

    return bReturnValue;
}

ab_t MDI_GetIab(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    ab_t bReturnValue = {0};

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_GetIab(pHandle->pMCI);
            break;
        case M2:
            break;
        default:
            break;
    }

    return bReturnValue;
}

AlphaBeta_t MDI_GetIalphabeta(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    AlphaBeta_t bReturnValue = {0};

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_GetIalphabeta(pHandle->pMCI);
            break;
        case M2:
            break;
        default:
            break;
    }

    return bReturnValue;
}

qd_t MDI_GetIqd(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    qd_t bReturnValue = {0};

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_GetIqd(pHandle->pMCI);
            break;
        case M2:
            break;
        default:
            break;
    }

    return bReturnValue;
}

qd_t MDI_GetIqdHF(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    qd_t bReturnValue = {0};

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_GetIqdHF(pHandle->pMCI);
            break;
        case M2:
            break;
        default:
            break;
    }

    return bReturnValue;
}

qd_t MDI_GetIqdref(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    qd_t bReturnValue = {0};

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_GetIqdref(pHandle->pMCI);
            break;
        case M2:
            break;
        default:
            break;
    }

    return bReturnValue;
}

qd_t MDI_GetVqd(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    qd_t bReturnValue = {0};

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_GetVqd(pHandle->pMCI);
            break;
        case M2:
            break;
        default:
            break;
    }

    return bReturnValue;
}

AlphaBeta_t MDI_GetValphabeta(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    AlphaBeta_t bReturnValue = {0};

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_GetValphabeta(pHandle->pMCI);
            break;
        case M2:
            break;
        default:
            break;
    }

    return bReturnValue;
}

int16_t MDI_GetElAngledpp(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    int16_t bReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_GetElAngledpp(pHandle->pMCI);
            break;
        case M2:
            break;
        default:
            break;
    }

    return bReturnValue;
}

int16_t MDI_GetTeref(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    int16_t bReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_GetTeref(pHandle->pMCI);
            break;
        case M2:
            break;
        default:
            break;
    }

    return bReturnValue;
}

int16_t MDI_GetPhaseCurrentAmplitude(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    int16_t bReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_GetPhaseCurrentAmplitude(pHandle->pMCI);
            break;
        case M2:
            break;
        default:
            break;
    }

    return bReturnValue;
}

int16_t MDI_GetPhaseVoltageAmplitude(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    int16_t bReturnValue = 0;

    switch (bMotor)
    {
        case M1:
            bReturnValue = MCInterface_GetPhaseVoltageAmplitude(pHandle->pMCI);
            break;
        case M2:
            break;
        default:
            break;
    }

    return bReturnValue;
}

void MDI_Clear_Iqdref(MultipleDriveInterfaceHandle_t * pHandle, uint8_t bMotor)
{
    ASSERT(pHandle != NULL);
    switch (bMotor)
    {
        case M1:
            MCInterface_GetCurrentFaults(pHandle->pMCI);
            break;
        case M2:
            break;
        default:
            break;
    }
}
