/**
*  @file firmware_update.c
*  @brief Application layer used to handle DFU process.
*/ 

/*********************************************
                Includes                       
*********************************************/

#include "firmware_update.h"

/*********************************************
                Defines
*********************************************/

/*********************************************
                Private Variables
*********************************************/

//strutc used to the state machine responsible to control firmware update.
static FirmwareUpdateStateMachine_t FirmwareUpdateStateMachine = CHECK_START;

//Initialize the struct used to control/implement firmware update.
static FirmwareUpdateControl_t FirmwareUpdateControl = {.otaCommand = NO_COMMAND,.otaStatus = NO_ERROS,.updating = false,
                                                        .firstDataFrameReceived = false,.frameCount = 0,.frameControl = 0,
                                                        .serialFlashInit = false,.dataFrameReceived = false};

//variable used to hold the crc value calculated on the fly(streaming).
static uint32_t crcState = (uint32_t)~0L;                                                        
/*********************************************
                Public Variables
*********************************************/


/****************************************************************
                Private Functions Prototype
*****************************************************************/
static void FirmwareUpdate_ReadDataFrame(CO_NODE  *pNode);
                                                        

/****************************************************************
                Private Functions 
*****************************************************************/

/**
  @brief function read, verify data integrity and copy data to the external memory.
  @param CO_NODE  *pNode pointer to the CAN node used on the CANOPEN communication.
  @param VCI_Handle_t * pVCI pointer to the struct responsible to handle vehicle control interface.
  @return void.
*/
static void FirmwareUpdate_ReadDataFrame(CO_NODE  *pNode)
{
    //check if the pointer is null
    ASSERT(pNode != NULL);
    
    //variable used on the crc calcule.
    uint8_t n;
    
    //Variable used to receive the calculated crc.
    uint32_t crc32 = 0;
    
    //flag used to inform if the received data frame was corrupted or not.
    static bool dataFrameCorrupted = false;
    
    //variable used to count the maximum numbers of retrys.
    static uint8_t dataFrameAttempts = 0;
    
    //variable used to hold the frame count received in the OD domaine.
    uint16_t tempFrameCount = 0;
    
    //varible uses hold the EXIT_SUCCESS or EXIT_FAILURE when 
    //opeing, closing, writing or reading on the external flash memory.
    FirmwareUpdateControl.serialFlashReponse = EXIT_FAILURE;
    
    //verify if all bytes of the data frame was received.
    if (FirmwareUpdateControl.dataFrameReceived == true)//pVCI->pFirmwareUpdateDomainObj->Offset >= NUMBER_OF_DATA_FRAME_BYTES)
    {
        //clear data frame received flag to wait for a new data frame.
        FirmwareUpdateControl.dataFrameReceived = false;
        
        //calculate crc to received data frame on whole frame. DATAFRAME_SIZE is not fixed, but DATAFRAME_HEADERSIZE is.
        FirmwareUpdateControl.crc = crc8(FirmwareUpdateControl.data, FirmwareUpdateControl.data[DATAFRAME_SIZE] + DATAFRAME_HEADERSIZE);
        
        //data frame is not corrupted.
        dataFrameCorrupted = false;
        
        //check if the data frama has the correct preambule and crc.
        //if yes, check the frame number and type.
        //if not, set status variable to TRANSFER_CORRUPTED_FRAME.
        //and move to 
        if ((FirmwareUpdateControl.data[PREAMB] == PREAMBULE_BYTE) && (FirmwareUpdateControl.data[FirmwareUpdateControl.data[DATAFRAME_SIZE] + DATAFRAME_HEADERSIZE] == FirmwareUpdateControl.crc))
        {
            //get frame control value from the data frame received.
            tempFrameCount = (uint16_t)((FirmwareUpdateControl.data[FRAMECOUNT_MSB] << 8) + FirmwareUpdateControl.data[FRAMECOUNT_LSB]);
            
            //verify if the first data frama of the file.
            //if yes, only aligne data frame number and type.
            //if not, check if is the next data frame.
            if ((tempFrameCount == 0) && (FirmwareUpdateControl.data[FRAME_CONTROL] == FIRST_FRAME))
            {
                //Set this flag to indicate the first data frame was received.
                FirmwareUpdateControl.firstDataFrameReceived = true;
                
                //first frame received.
                FirmwareUpdateControl.frameCount = 0;
                
                //control first frame received.
                FirmwareUpdateControl.frameControl = FIRST_FRAME;
                
                //Verify if the external flash memory was initialized correctly.
                if (FirmwareUpdateControl.serialFlashInit == true)
                {
                    //get size of the data frame payload.
                    FirmwareUpdateControl.frameDataSize = FirmwareUpdateControl.data[DATAFRAME_SIZE];
                        
                    //write the data frame payload received in the external flash memory.
                    if((SF_Storage_PutPackChunk(&FirmwareUpdateControl.data[DATAPAYLOAD_START], FirmwareUpdateControl.frameDataSize) == EXIT_SUCCESS) &&
                        (uCAL_SPI_IO_GetSpiFailedFlag(&SPI1Handle) == false))
                    {
                        //calculate crc 32 on the fly for the data payload of the received data frame.
                        for (n = 0; n < FirmwareUpdateControl.frameDataSize; n++)
                        {
                            //Calculate frimware CRC32 on the fly.
                            FirmwareUpdate_Crc32Update(FirmwareUpdateControl.data[DATAPAYLOAD_START + n]);
                        }
                            
                        //data payload was correctly wrote in the external memory.
                        FirmwareUpdateControl.serialFlashReponse = EXIT_SUCCESS;
  
                    }
                }
                
                //operation on the external flash memory failled.
                if (FirmwareUpdateControl.serialFlashReponse == EXIT_FAILURE)
                {
                    //
                    dataFrameCorrupted = true;
                    
                    //increment the number of retry.
                    dataFrameAttempts++;
                }
            }
            else
            {
                //check if the first data frame was received.
                //if not set a error to indicate the system didn't
                //received the first data frame.
                if (FirmwareUpdateControl.firstDataFrameReceived == true)
                {   
                    //increment to next data frame.
                    FirmwareUpdateControl.frameCount++;
                    
                    //verify if the next data frame is realy the next data frame.
                    //if is not set TRANSFER_MISSING_FRAME
                    if (tempFrameCount == FirmwareUpdateControl.frameCount)
                    {
                        //get frame control value.
                        FirmwareUpdateControl.frameControl = FirmwareUpdateControl.data[FRAME_CONTROL];
                    
                        //get size of the data frame payload.
                        FirmwareUpdateControl.frameDataSize = FirmwareUpdateControl.data[DATAFRAME_SIZE];
                        
                        //write the data frame payload received in the external flash memory.
                        if ((SF_Storage_PutPackChunk(&FirmwareUpdateControl.data[DATAPAYLOAD_START], FirmwareUpdateControl.frameDataSize) == EXIT_SUCCESS)
                            && (uCAL_SPI_IO_GetSpiFailedFlag(&SPI1Handle) == false))
                        {
                            //calculate crc 32 on the fly for the data payload of the received data frame.
                            for (n = 0; n < FirmwareUpdateControl.frameDataSize; n++)
                            {
                                //Calculate frimware CRC32 on the fly.
                                FirmwareUpdate_Crc32Update(FirmwareUpdateControl.data[DATAPAYLOAD_START + n]);
                            }
                            
                            //data payload was correctly wrote in the external memory.
                            FirmwareUpdateControl.serialFlashReponse = EXIT_SUCCESS;
                        }
                        else
                        {
                            //decrement the frame count to wait and compare with the next resent data frame.
                            FirmwareUpdateControl.frameCount--;
                    
                            //data frame is corrupted.
                            dataFrameCorrupted = true;
                    
                            //increment the number of retry.
                            dataFrameAttempts++;
                        }
                    
                        //check if is the last data frame.
                        if ((FirmwareUpdateControl.frameControl == LAST_FRAME) && (FirmwareUpdateControl.serialFlashReponse == EXIT_SUCCESS))
                        {
                            //Get crc 32 result.
                            crc32 = FirmwareUpdate_Crc32Result();
                            
                            //Copy crc32 from variable to buffer to be written at the end of firmware file.
                            memcpy(FirmwareUpdateControl.data,&crc32,sizeof(crc32));
                            
                            //Write crc32 at the end of firmware file.
                            SF_Storage_PutPackChunk(FirmwareUpdateControl.data, sizeof(uint32_t));
                            
                            //informr the IOT that file transfer was finished.
                            FirmwareUpdateControl.otaStatus = FILE_TRANSFER_COMPLETED;
                            
                            //move to FIRMWARE_UPDATE_FINISHED state.
                            FirmwareUpdateStateMachine = FIRMWARE_UPDATE_FINISHED; 
                        }
                    }
                    else
                    {
                        //decrement the frame count to wait and compare with the next resent data frame.
                        FirmwareUpdateControl.frameCount--;
                    
                        //data frame is corrupted.
                        dataFrameCorrupted = true;
                    
                        //increment the number of retry.
                        dataFrameAttempts++;
                    }
                }
                else
                {
                    //data frame is corrupted.
                    dataFrameCorrupted = true;
                    
                    //increment the number of retry.
                    dataFrameAttempts++;
                }
            }
        }
        else
        {
            //data frame is corrupted.
            dataFrameCorrupted = true;
            
            //increment the number of retry.
            dataFrameAttempts++;
        }
        
        //verify if data frame is corrupted.
        //if yes, set otastatus to inform the IOT module,
        //initialize the buffer off set to wait for a new data frame,
        //and increment the retry count variable.
        if (dataFrameCorrupted == true)
        {
            //change status to correspondent error.
            FirmwareUpdateControl.otaStatus = TRANSFER_CORRUPTED_FRAME;
            
            //verify of the retry count is more than the maxium trys, change the state machine to FIRMWARE_UPDATE_ERROR.
            if (dataFrameAttempts >= MAXIMUM_ATTEMPTS)
            {       
                //move to error state.dataFrameCorrupted = false;
                FirmwareUpdateStateMachine = FIRMWARE_UPDATE_ERROR;  
                
                //initialize the variable to not corrupted.
                dataFrameCorrupted = false;
                
                //initialize the variable.
                dataFrameAttempts = 0;
                
                //reset the maximum timeout to wait 3300 ms.
                FirmwareUpdateControl.otaTimeOut = OTA_TIMEOUT;
            }
        }
        else
        {
            //verify if the last data frame was received.
            if (FirmwareUpdateStateMachine != FIRMWARE_UPDATE_FINISHED)
            {
                //change status to no error.
                FirmwareUpdateControl.otaStatus = NO_ERROS;
            }
            
            //initialize the number of retry.
            dataFrameAttempts = 0;
            
            //reset the maximum timeout to wait 3300 ms.
            FirmwareUpdateControl.otaTimeOut = OTA_TIMEOUT;
        }
        
        //write the new status on the OD subindex 1.
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_FIRMWAREUPDATE_MEMORY, 1)), pNode, &FirmwareUpdateControl.otaStatus, sizeof(uint8_t));
        
        //write the new status on the OD subindex 3.
        COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_FIRMWAREUPDATE_MEMORY, 3)), pNode, &FirmwareUpdateControl.frameCount, sizeof(uint16_t));
    }        
}

/****************************************************************
                Public Functions
*****************************************************************/

/**
  @brief function used to manage the firmware update procedure.
  @param CO_NODE  *pNode pointer to the CAN node used on the CANOPEN communication.
  @param VCI_Handle_t * pVCI pointer to the struct responsible to handle vehicle control interface.
  @return void
*/
void FirmwareUpdate_Control (CO_NODE  *pNode, VCI_Handle_t * pVCI)
{   
    
    //check if the pointer is null
    ASSERT(pNode != NULL);
    ASSERT(pVCI != NULL);
    
    int err;
    //state machine responsible to control the firmware update.
    switch(FirmwareUpdateStateMachine)
    {
        //state to check if a PREPARE_FILE_TRANSFER command was received.
        case CHECK_START:
            
            //allowed to start a DFU if the battery charge is above the DFU_MINIMUMBATERRY_CHARGE.
            //this blocks only the start check. this means if DFU was started before the battery drops
            //below DFU_MINIMUMBATERRY_CHARGE the device will continues on the DFU procedure.
            if (BatMonitor_GetSOC(VCInterfaceHandle.pPowertrain->pBatMonitorHandle) >= DFU_MINIMUMBATERRY_CHARGE)
            {
                //Read the OD responsible to hold the firmware update command on subindex 0.
                COObjRdValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_FIRMWAREUPDATE_MEMORY, 0)), pNode, &FirmwareUpdateControl.otaCommand, sizeof(FirmwareUpdateControl.otaCommand));
            }
            
            //check if a PREPARE_FILE_TRANSFER command was received.
            //if was, move to the next state to prepare the system to the firmware update.
            if (FirmwareUpdateControl.otaCommand == PREPARE_FILE_TRANSFER)
            {
                //Initialize the varible responsible to hold crc 32 value 
                //calculated on the fly.
                FirmwareUpdate_Crc32Reset();
                
                //inform system that a firmware update will start.
                FirmwareUpdateControl.updating = true;
                
                //clear flag responsible to indicate the first data frame was received.
                FirmwareUpdateControl.firstDataFrameReceived = false;
                
                //initialize the number of the received data frame.
                FirmwareUpdateControl.frameCount = 0;
                
                //set first frame received.
                FirmwareUpdateControl.frameControl = FIRST_FRAME;
                
                //initialize the ota status to prepare it to the firmware update.
                FirmwareUpdateControl.otaStatus  = NO_ERROS;
                
                //initializa external flash memory response.
                FirmwareUpdateControl.serialFlashReponse = EXIT_FAILURE;
                
                //move to the next state.
                FirmwareUpdateStateMachine = PREPARE_SYSTEM_TO_UPDATE;
                
                //wait maximum 3300 ms to motors stop and go to idle state.
                FirmwareUpdateControl.otaTimeOut = OTA_TIMEOUT;
            }
        
        break;
            
        //state to stop motors and prepare to firmware update.
        case PREPARE_SYSTEM_TO_UPDATE:
        
            //wait maximum 500 ms to motors stop and go to idle state.
            if (MCInterface_GetSTMState(pVCI->pPowertrain->pMDI->pMCI) == M_IDLE)
            {
 
                //move to the next state.
                FirmwareUpdateStateMachine = OPEN_DFU_FILE;
                
                //reset the maximum timeout to wait 3300 ms.
                FirmwareUpdateControl.otaTimeOut = OTA_TIMEOUT;    
                    
                //initializa external flash memory response.
                FirmwareUpdateControl.serialFlashReponse = EXIT_FAILURE; 
            }
            else
            {
                //decrement timeout
                FirmwareUpdateControl.otaTimeOut--;
                
                //check if the procedure timeouts.
                if (FirmwareUpdateControl.otaTimeOut <= 0)
                {
                    //set status to internal error.
                    FirmwareUpdateControl.otaStatus = TRANSFER_INTERNAL_ERROR;
                    
                    //move to error state.
                    FirmwareUpdateStateMachine = FIRMWARE_UPDATE_ERROR;
                    
                    //reset the maximum timeout to wait 3300 ms.
                    FirmwareUpdateControl.otaTimeOut = OTA_TIMEOUT;
                }
            }
        
        break;
            
        //state used to open the external memory to a write procedure.
        case OPEN_DFU_FILE:
            
            //Open DFU File For writing Process
            if (SF_Storage_StartPackWrite() == EXIT_SUCCESS)
            {
                //change status to READY_FILE_TRANSFER.
                FirmwareUpdateControl.otaStatus  = READY_FILE_TRANSFER;
                
                //move to the next state.
                FirmwareUpdateStateMachine = READY_TO_FIRMWARE_UPDATE;
                
                //reset the maximum timeout to wait 3300 ms.
                FirmwareUpdateControl.otaTimeOut = OTA_TIMEOUT;    
                    
                //initializa external flash memory response.
                FirmwareUpdateControl.serialFlashReponse = EXIT_FAILURE;
                
                //close uart to reduce the volume of data stored in the FIFO queue buffer(RTOS).
                //GNR will lost the communication with the display.
                R_SCI_B_UART_Close(&g_uart9_ctrl);
                
                //Close ADC to reduce the number of interruption(ADC has hight priority)
                //ADC interruption can block the interruption used by the DFU.
                R_ADC_B_Close(g_adc0.p_ctrl);
                
                //Close theses timers interruption to give priority to interruptions
                //used during DFU.
                R_GPT_Close(g_timer0.p_ctrl);            
                R_GPT_Close(g_timer9.p_ctrl);
            }
            else
            {
                //set status to internal error.
                FirmwareUpdateControl.otaStatus = TRANSFER_INTERNAL_ERROR;
                    
                //move to error state.
                FirmwareUpdateStateMachine = FIRMWARE_UPDATE_ERROR;
                    
                //reset the maximum timeout to wait 3300 ms.
                FirmwareUpdateControl.otaTimeOut = OTA_TIMEOUT;
            }
            
            //write the new status on the OD subindex 1
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_FIRMWAREUPDATE_MEMORY, 1)), pNode, &FirmwareUpdateControl.otaStatus, sizeof(uint8_t));
            
            //write the new value of the FirmwareUpdateControl.frameCount on the OD subindex 3.
            COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_FIRMWAREUPDATE_MEMORY, 3)), pNode, &FirmwareUpdateControl.frameCount, sizeof(uint16_t));
            
        break;
            
        //state to ready firmware bytes, check the crc and copy into the external memory.
        case READY_TO_FIRMWARE_UPDATE:
        
            //call the function responsible to manage the firmware update.
            FirmwareUpdate_ReadDataFrame(pNode);
        
            //decrement timeout
            FirmwareUpdateControl.otaTimeOut--;
                
            //check if the procedure timeouts.
            if (FirmwareUpdateControl.otaTimeOut <= 0)
            {
                //set status to internal error.
                FirmwareUpdateControl.otaStatus = TRANSFER_MISSING_FRAME;
                
                //write the new status on the OD subindex 1.
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_FIRMWAREUPDATE_MEMORY, 1)), pNode, &FirmwareUpdateControl.otaStatus, sizeof(uint8_t));
                    
                //move to error state.
                FirmwareUpdateStateMachine = FIRMWARE_UPDATE_ERROR;
                
                //reset the maximum timeout to wait 3300 ms.
                FirmwareUpdateControl.otaTimeOut = OTA_TIMEOUT;
            }
        
        break;
            
        //
        case FIRMWARE_UPDATE_FINISHED:
                
            //decrement timeout
            FirmwareUpdateControl.otaTimeOut--;
                
            //check if the procedure timeouts.
            //if timeout reset the system to pass the control
            //to bootloader firmware.
            if (FirmwareUpdateControl.otaTimeOut <= 0)
            {   
                //close file to write procedure.
                err = SF_Storage_FinalizePackWrite();
                
                //check if the file was correctly closed.
                if ( err == EXIT_SUCCESS)
                {   
                    //Deinitializes the serial flash storage.
                    err = SF_Storage_Deinit();
                }
                
                //if any of teh procedures above failled, erase the external memory
                //to do not keep a corrupted firmware.
                if (err != EXIT_SUCCESS)
                {     
                    //erase external memory.
                    Serial_Flash_EraseChip(&EFlash_Storage_Handle.eFlashStorage);
                }
                
                //force a software reset
                NVIC_SystemReset();
            }
        
        break;
        
        //state to handle errors during the firmware update.
        case FIRMWARE_UPDATE_ERROR:
            
            //decrement timeout
            FirmwareUpdateControl.otaTimeOut--;
                
            //timeout to finish this state.
            if (FirmwareUpdateControl.otaTimeOut <= 0)
            {
                //update command register with NO_COMMAND(9xFF).
                FirmwareUpdateControl.otaCommand = NO_COMMAND;
                
                //wrire the command value into the OD subindex 0.
                COObjWrValue(CODictFind(&pNode->Dict, CO_DEV(CO_OD_REG_FIRMWAREUPDATE_MEMORY, 0)), pNode, &FirmwareUpdateControl.otaCommand, sizeof(FirmwareUpdateControl.otaCommand));
                
                //verify if the DFU file was closed correctly.
                SF_Storage_FinalizePackWrite();
                    
                //Deinitializes the serial flash storage.
                SF_Storage_Deinit();
                
                //erase external memory.
                Serial_Flash_EraseChip(&EFlash_Storage_Handle.eFlashStorage);
                
                //force a software reset
                NVIC_SystemReset();
            }
            
        break;
    }
}

/**
  @brief return a flag, true or false, to indicating if the system
         is doing a firmware update.
  @return bool return true if the system is doing a firmware upadte.
*/
bool FirmwareUpdate_Running(void)
{
    //return one flag indicating if a firmware upload is running.
    return FirmwareUpdateControl.updating;
}

/**
  @brief used to initialize the external flash memory.
         The flash memory must to be initialized at the 
         firmware start, because initialization process
         requires some seconds sometimes.
  @return void
*/
void FirmwareUpdate_SerialFlashMemoryInit(void)
{
    //Initializes the serial flash storage.
    if (SF_Storage_Init(&EFlash_Storage_Handle) == EXIT_SUCCESS)
    {
        //serial flash storage initialized correctly.
        FirmwareUpdateControl.serialFlashInit = true;
        
    }
    else
    {
        //error on external memory operation. was not possible 
        //initialize memory.
        FirmwareUpdateControl.serialFlashInit = false;
    }
}

/**
  @brief used to check if the array(domain),used to receive the data frame
         on the firmware update procedure, was 100% received.
         Copy the data frame received to the process buffer and
         set a flag to indicate a new data frame was received.
  @return void
*/
void FirmwareUpdate_CheckDataFrame(VCI_Handle_t * pVCI)
{
    //check if pointer is null
    ASSERT(pVCI != NULL);
    
    //verify if all bytes of the data frame was received.
    if (pVCI->pFirmwareUpdateDomainObj->Offset >= pVCI->pFirmwareUpdateDomainObj->Start[DATAFRAME_SIZE] + DATAFRAME_HEADERSIZE + DATAFRAME_CRCSIZE)
    {
        //Make the domain object to point again to the beginning to allow 
        //the device to detect the next data frame.
        pVCI->pFirmwareUpdateDomainObj->Offset = 0x00;
        //DATAFRAME_SIZE
        //copy data frama from the dictionary.
        memcpy(FirmwareUpdateControl.data,pVCI->pFirmwareUpdateDomainObj->Start,NUMBER_OF_DATA_FRAME_BYTES);
        
        //
        FirmwareUpdateControl.dataFrameReceived = true;
    }
}
/**
    Calculate CRC-8 using polynomial X^8 + X^2 + X + 1 (0b10001100)
*/
uint8_t crc8(const void* vptr, int len) 
{
    //check if pointer is null
    ASSERT(vptr != NULL);
    const uint8_t *data = vptr;
    uint8_t crc = 0;
    int i;

    for (i = 0; i < len; i++) 
	{
        crc = crc8_table[crc ^ data[i]];
    }

    return crc;
}

/**********************************************************************************************************************
 * @brief Reset crcState variable to 0xFFFFFFFF
 **********************************************************************************************************************/
void FirmwareUpdate_Crc32Reset(void) 
{
    crcState = (uint32_t)~0L;
}

/**********************************************************************************************************************
 * @brief Calculate CRC-32 using polynomial x32 + x26 + x23 + x22 + x16 + x12 + x11 + x10 + x8 + x7 + x5 + x4 + x2 + x1 + 1
 **********************************************************************************************************************/
void FirmwareUpdate_Crc32Update(const uint8_t data) 
{
    //
    uint8_t tableIndex = 0;
    
    //Calculate crc32 on the fly.
    tableIndex = (uint8_t)(crcState ^ (data >> (0 * 4)));
    crcState = FLASH_READ_DWORD(crc32_table + (tableIndex & 0x0f)) ^ (crcState >> 4);
    tableIndex = (uint8_t)(crcState ^ (data >> (1 * 4)));
    crcState = FLASH_READ_DWORD(crc32_table + (tableIndex & 0x0f)) ^ (crcState >> 4);
}

/**********************************************************************************************************************
 * @brief Return crc 32 value.
 *        This operation is done using a separate function because we are calculating the CRC on the fly
 **********************************************************************************************************************/
uint32_t FirmwareUpdate_Crc32Result(void) 
{
    return ~crcState;
}
