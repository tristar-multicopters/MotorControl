/**
  * @file    slave_mc_interface.c
  * @author  Sami Bouzid, FTEX
  * @brief   Module that provides an interface to control a slave motor,
                i.e. motor controller not locally present on this ganrunner device.
*/

#include "slave_mc_interface.h"
#include "ASSERT_FTEX.h"


#define SDO_TIMEOUT_FAULT_ACK       200 // 200ms timeout when sending a fault ack SDO


/* The SDO transfer finalization callback when acknowledging a fault */
void FaultAckDownloadFinishCb(CO_CSDO *csdo, uint16_t index, uint8_t sub, uint32_t code)
{
    (void)csdo;
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

    COObjRdValue(CODictFind(&pHandle->pCONode->Dict, pHandle->RegisterAddr.wRegAddrState), pHandle->pCONode, &pHandle->Feedback.bState, 2, 0);
    COObjRdValue(CODictFind(&pHandle->pCONode->Dict, pHandle->RegisterAddr.wRegAddrOccuredFaults), pHandle->pCONode, &pHandle->Feedback.hOccuredFaults, 2, 0);
    COObjRdValue(CODictFind(&pHandle->pCONode->Dict, pHandle->RegisterAddr.wRegAddrCurrentFaults), pHandle->pCONode, &pHandle->Feedback.hCurrentFaults, 2, 0);
    COObjRdValue(CODictFind(&pHandle->pCONode->Dict, pHandle->RegisterAddr.wRegAddrMotorSpeed), pHandle->pCONode, &pHandle->Feedback.hMotorSpeed, 2, 0);
}


/*
* see function definition
*/
void SlaveMCInterface_ExecTorqueRamp(SlaveMotorHandle_t * pHandle, int16_t hFinalTorque)
{
    ASSERT(pHandle != NULL);
    
    int16_t Tref = hFinalTorque;

    COObjWrValue(CODictFind(&pHandle->pCONode->Dict, pHandle->RegisterAddr.wRegAddrTorqueRef), pHandle->pCONode, &Tref, 2, 0);
}


/*
* see function definition
*/
bool SlaveMCInterface_StartMotor(SlaveMotorHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    bool bRetVal = true;
    
    bool bStart = true;

    COObjWrValue(CODictFind(&pHandle->pCONode->Dict, pHandle->RegisterAddr.wRegAddrStartMotor), pHandle->pCONode, &bStart, 1, 0);
    
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

    COObjWrValue(CODictFind(&pHandle->pCONode->Dict, pHandle->RegisterAddr.wRegAddrStartMotor), pHandle->pCONode, &bStart, 1, 0);
    

    return bRetVal;
}

/*
* see function definition
*/
bool SlaveMCInterface_FaultAcknowledged(SlaveMotorHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    bool bRetVal = true;
    
    CO_CSDO *csdo;
    csdo = COCSdoFind(pHandle->pCONode, 0);
    uint8_t Data = true;
    COCSdoRequestDownload(csdo, pHandle->RegisterAddr.wRegAddrFaultAck, &Data, 1, FaultAckDownloadFinishCb, SDO_TIMEOUT_FAULT_ACK);

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
uint16_t SlaveMCInterface_GetOccurredFaults(SlaveMotorHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    uint16_t hReturnValue = 0;

    hReturnValue = pHandle->Feedback.hOccuredFaults;

    return hReturnValue;
}

/*
* see function definition
*/
uint16_t SlaveMCInterface_GetCurrentFaults(SlaveMotorHandle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    uint16_t hReturnValue = 0;

    hReturnValue = pHandle->Feedback.hCurrentFaults;

    return hReturnValue;
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

