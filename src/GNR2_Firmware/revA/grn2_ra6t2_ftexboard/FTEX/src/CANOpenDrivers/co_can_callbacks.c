/******************************************************************************
   Copyright 2020 Embedded Office GmbH & Co. KG
   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at
       http://www.apache.org/licenses/LICENSE-2.0
   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
******************************************************************************/

/******************************************************************************
* INCLUDES
******************************************************************************/

#include "co_core.h"

void COTmrLock  (void);
void COTmrUnlock(void);

/******************************************************************************
* MANDATORY CALLBACK FUNCTIONS
******************************************************************************/

void CONodeFatalError(void)
{
    volatile uint8_t debugExit = 0u;

    /* Place here your fatal error handling.
     * There is most likely a programming error.
     * !! Please don't ignore this errors. !!
     */
    for (;debugExit == 0u;);
}

void COTmrLock(void)
{
    /* This function helps to guarantee the consistancy
     * of the internal timer management while interrupted
     * by the used timer interrupt. Most likely you need
     * at this point on of the following mechanisms:
     * - disable the used hardware timer interrupt
     * - get a 'timer-mutex' from your RTOS (ensure to
     *   call COTmrService() in a timer triggered task)
     */
}

void COTmrUnlock(void)
{
    /* This function helps to guarantee the consistancy
     * of the internal timer management while interrupted
     * by the used timer interrupt. Most likely you need
     * at this point on of the following mechanisms:
     * - (re)enable the used hardware timer interrupt
     * - release the 'timer-mutex' from your RTOS (ensure
     *   to call COTmrService() in a timer triggered task)
     */
}

/******************************************************************************
* OPTIONAL CALLBACK FUNCTIONS
******************************************************************************/

void CONmtModeChange(CO_NMT *nmt, CO_MODE mode)
{
    (void)nmt;
    (void)mode;

    /* Optional: place here some code, which is called
     * when a NMT mode change is initiated.
     */
}

void CONmtResetRequest(CO_NMT *nmt, CO_NMT_RESET reset)
{
    (void)nmt;
    (void)reset;

    /* Optional: place here some code, which is called
     * when a NMT reset is requested by the network.
     */
}

void CONmtHbConsEvent(CO_NMT *nmt, uint8_t nodeId)
{
    (void)nmt;
    (void)nodeId;

    /* Optional: place here some code, which is called
     * called when heartbeat consumer is in use and
     * detects an error on monitored node(s).
     */
}

void CONmtHbConsChange(CO_NMT *nmt, uint8_t nodeId, CO_MODE mode)
{
    (void)nmt;
    (void)nodeId;
    (void)mode;

    /* Optional: place here some code, which is called
     * when heartbeat consumer is in use and detects a
     * NMT state change on monitored node(s).
     */
}

int16_t COLssLoad(uint32_t *baudrate, uint8_t *nodeId)
{
    (void)baudrate;
    (void)nodeId;

    /* Optional: place here some code, which is called
     * when LSS client is in use and the CANopen node
     * is initialized.
     */
    return (0u);
}

int16_t COLssStore(uint32_t baudrate, uint8_t nodeId)
{
    (void)baudrate;
    (void)nodeId;

    /* Optional: place here some code, which is called
     * when LSS client is in use and the CANopen node
     * needs to store updated values.
     */
    return (0u);
}

void COIfCanReceive(CO_IF_FRM *frm)
{
    (void)frm;

    /* Optional: place here some code, which is called
     * when you need to handle CAN messages, which are
     * not part of the CANopen protocol.
     */
}

void COPdoTransmit(CO_IF_FRM *frm)
{
    (void)frm;

    /* Optional: place here some code, which is called
     * just before a PDO is transmitted. You may adjust
     * the given CAN frame which is send afterwards.
     */
}

int16_t COPdoReceive(CO_IF_FRM *frm)
{
    (void)frm;

    /* Optional: place here some code, which is called
     * right after receiving a PDO. You may adjust
     * the given CAN frame which is written into the
     * object dictionary afterwards or suppress the
     * write operation.
     */
    return(0u);
}

void COPdoSyncUpdate(CO_RPDO *pdo)
{
    (void)pdo;

    /* Optional: place here some code, which is called
     * right after the object dictionary update due to
     * a synchronized PDO.
     */
}

int16_t COParaDefault(CO_PARA *pg)
{
    (void)pg;

    /* Optional: place here some code, which is called
     * when a parameter group is restored to factory
     * settings.
     */
    return (0u);
}