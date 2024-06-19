/**
  * @file    slave_mc_interface.c
  * @author  Sami Bouzid, FTEX
  * @brief   Module that provides an interface to control a slave motor,
                i.e. motor controller not locally present on this ganrunner device.
*/

#include "slave_mc_interface.h"
#include "ASSERT_FTEX.h"

#define SDO_TIMEOUT_FAULT_ACK_IN_MS 200 // 200ms timeout when sending a fault ack SDO

/* The SDO transfer finalization callback when acknowledging a fault */
void FaultAckDownloadFinishCb(CO_CSDO *csdo, uint16_t index, uint8_t sub, uint32_t code)
{
    // suppress warning about unused variables
    (void) csdo;
    (void) index;
    (void) sub;
    
    if (code == 0)
    {
        /* SDO completed succesfully */
    }
    else
    {
    /* a timeout or abort is detected during SDO transfer  */
    }
}


/*
* see function definition
*/
void SlaveMCInterface_Init(SlaveMotorHandle_t * pHandle, CO_NODE * pNode, SlaveMotorRegisterAddr_t RegisterAddr)
{
    ASSERT(pHandle != NULL);
    ASSERT(pNode != NULL);

    pHandle->pCONode = pNode;
    pHandle->RegisterAddr = RegisterAddr;
}

/*
* see function definition
*/

void SlaveMCInterface_UpdateFeedback(SlaveMotorHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);

    COObjRdValue(CODictFind(&pHandle->pCONode->Dict, pHandle->RegisterAddr.wRegAddrState), pHandle->pCONode, &pHandle->Feedback.bState,  sizeof(uint16_t));
    COObjRdValue(CODictFind(&pHandle->pCONode->Dict, pHandle->RegisterAddr.wRegAddrOccurredFaults), pHandle->pCONode, &pHandle->Feedback.wOccurredFaults, sizeof(pHandle->Feedback.wOccurredFaults));
    COObjRdValue(CODictFind(&pHandle->pCONode->Dict, pHandle->RegisterAddr.wRegAddrCurrentFaults), pHandle->pCONode, &pHandle->Feedback.wCurrentFaults, sizeof(pHandle->Feedback.wCurrentFaults));
    COObjRdValue(CODictFind(&pHandle->pCONode->Dict, pHandle->RegisterAddr.wRegAddrCurrentErrorsNow), pHandle->pCONode, &pHandle->Feedback.wCurrentErrorsNow, sizeof(pHandle->Feedback.wCurrentErrorsNow));
    COObjRdValue(CODictFind(&pHandle->pCONode->Dict, pHandle->RegisterAddr.wRegAddrOccurredErrors), pHandle->pCONode, &pHandle->Feedback.wOccurredErrors, sizeof(pHandle->Feedback.wOccurredErrors));
    COObjRdValue(CODictFind(&pHandle->pCONode->Dict, pHandle->RegisterAddr.wRegAddrWarnings), pHandle->pCONode, &pHandle->Feedback.wWarnings, sizeof(pHandle->Feedback.wWarnings));
    COObjRdValue(CODictFind(&pHandle->pCONode->Dict, pHandle->RegisterAddr.wRegAddrMotorSpeed), pHandle->pCONode, &pHandle->Feedback.hMotorSpeed, sizeof(pHandle->Feedback.hMotorSpeed));
    COObjRdValue(CODictFind(&pHandle->pCONode->Dict, pHandle->RegisterAddr.wRegAddrIq), pHandle->pCONode, &pHandle->Feedback.hIq, sizeof(pHandle->Feedback.hIq));
    COObjRdValue(CODictFind(&pHandle->pCONode->Dict, pHandle->RegisterAddr.wRegAddrId), pHandle->pCONode, &pHandle->Feedback.hId, sizeof(pHandle->Feedback.hId));
    
}


/*
* see function definition
*/
void SlaveMCInterface_ExecTorqueRamp(SlaveMotorHandle_t * pHandle, int16_t hFinalTorque)
{
    ASSERT(pHandle != NULL);

    int16_t Tref = hFinalTorque;

    COObjWrValue(CODictFind(&pHandle->pCONode->Dict, pHandle->RegisterAddr.wRegAddrTorqueRef), pHandle->pCONode, &Tref, sizeof(Tref));
}


/*
* see function definition
*/
bool SlaveMCInterface_StartMotor(SlaveMotorHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    bool bRetVal = true;

    bool bStart = true;
    if (COObjWrValue(CODictFind(&pHandle->pCONode->Dict, pHandle->RegisterAddr.wRegAddrStartMotor), pHandle->pCONode, &bStart, sizeof(bStart)) != CO_ERR_NONE)
    {
        bRetVal = false;
    }

    return bRetVal;
}

/*
* see function definition
*/
bool SlaveMCInterface_StopMotor(SlaveMotorHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    bool bRetVal = true;

    bool bStart = false;
    if (COObjWrValue(CODictFind(&pHandle->pCONode->Dict, pHandle->RegisterAddr.wRegAddrStartMotor), pHandle->pCONode, &bStart, sizeof(bStart)) != CO_ERR_NONE)
    {
        bRetVal = false;
    }

    return bRetVal;
}

/*
* see function definition
*/
bool SlaveMCInterface_CriticalFaultAcknowledged(SlaveMotorHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    bool bRetVal = true;

    CO_CSDO *csdo;
    csdo = COCSdoFind(pHandle->pCONode, 0);
    uint8_t Data = true;
    if (COCSdoRequestDownload(csdo, pHandle->RegisterAddr.wRegAddrFaultAck, &Data, sizeof(Data), FaultAckDownloadFinishCb, SDO_TIMEOUT_FAULT_ACK_IN_MS) != CO_ERR_NONE)
    {
        bRetVal = false;
    }

    return bRetVal;
}

/*
* see function definition
*/
MotorState_t  SlaveMCInterface_GetSTMState(SlaveMotorHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    MotorState_t bReturnValue = 0;

    bReturnValue = pHandle->Feedback.bState;

    return bReturnValue;
}

/*
* see function definition
*/
uint32_t SlaveMCInterface_GetOccurredCriticalFaults(SlaveMotorHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    uint32_t wReturnValue = 0;

    wReturnValue = pHandle->Feedback.wOccurredFaults;

    return wReturnValue;
}

/*
* see function definition
*/
uint32_t SlaveMCInterface_GetCurrentErrors(SlaveMotorHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->Feedback.wCurrentErrorsNow;
}

/*
* see function definition
*/
uint32_t SlaveMCInterface_GetOccurredErrors(SlaveMotorHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->Feedback.wOccurredErrors;
}

/*
* see function definition
*/
uint32_t SlaveMCInterface_GetOccurredWarnings(SlaveMotorHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    uint32_t wReturnValue = 0;

    wReturnValue = pHandle->Feedback.wWarnings;

    return wReturnValue;
}

/*
* see function definition
*/
uint32_t SlaveMCInterface_GetCurrentCriticalFaults(SlaveMotorHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    uint32_t wReturnValue = 0;

    wReturnValue = pHandle->Feedback.wCurrentFaults;

    return wReturnValue;
}


/*
* see function definition
*/
int16_t SlaveMCInterface_GetAvrgMecSpeedUnit(SlaveMotorHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    int16_t hReturnValue = 0;

    hReturnValue = pHandle->Feedback.hMotorSpeed;

    return hReturnValue;
}
