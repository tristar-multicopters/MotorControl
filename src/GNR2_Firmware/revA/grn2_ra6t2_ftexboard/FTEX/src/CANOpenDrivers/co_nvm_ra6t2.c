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
/**
*  "co_nvm_ra6t2.c"
*  Abstraction Layer module for NVM interface
*/

/******************************************************************************
* INCLUDES
******************************************************************************/

#include "co_nvm_ra6t2.h"

#include <cmsis_os2.h>

/******************************************************************************
* PRIVATE DEFINES
******************************************************************************/

/* TODO: place here your timer register definitions */

/******************************************************************************
* PRIVATE FUNCTIONS
******************************************************************************/

static void     CANo_NVM_Init  (void);
static uint32_t CANo_NVM_Read  (uint32_t start, uint8_t *buffer, uint32_t size);
static uint32_t CANo_NVM_Write (uint32_t start, uint8_t *buffer, uint32_t size);


/******************************************************************************
* PUBLIC VARIABLE
******************************************************************************/

/* TODO: rename the variable to match the naming convention:
 *   <device>NvmDriver
 */
const CO_IF_NVM_DRV CoNvmDriver = {
    CANo_NVM_Init,
    CANo_NVM_Read,
    CANo_NVM_Write
};

/******************************************************************************
* PRIVATE FUNCTIONS
******************************************************************************/

static void CANo_NVM_Init(void)
{
    /* TODO: initialize the non-volatile memory */
    //R_FLASH_HP_Open    (    &g_flash0_ctrl, &g_flash0_cfg );
}

static uint32_t CANo_NVM_Read(uint32_t start, uint8_t *buffer, uint32_t size)
{
    (void)start;
    (void)buffer;
    (void)size;

    /* TODO: read a memory block from non-volatile memory into given buffer */    
    return (0u);
}

static uint32_t CANo_NVM_Write(uint32_t start, uint8_t *buffer, uint32_t size)
{
    (void)start;
    (void)buffer;
    (void)size;

    /* TODO: write content of given buffer into non-volatile memory */
    return (0u);
}
