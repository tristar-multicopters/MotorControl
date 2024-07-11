/**
  ******************************************************************************
  * @file    log_high_speed.c
  * @author  Andy Beaudoin, FTEX
  * @brief   High level module that logs high speed motor control variables
  *
  ******************************************************************************
*/

#include "log_high_speed.h"
#include "ASSERT_FTEX.h"
#include <stdlib.h>
#include "gnr_parameters.h"
#include "md_interface.h"


volatile uint8_t HSLog_ResolutionCounter;



/**
  Initialises the handle variables that are specific to the module and
  links the generic UART interrupts of the specified UART instance to this module.
  VCI handle is used to access the information that needs to be logged
*/
void LogHS_Init(LogHighSpeed_Handle_t *pHandle,VCI_Handle_t *pVCIHandle, UART_Handle_t *pUARTHandle)
{    

    ASSERT(pHandle     != NULL);
    ASSERT(pVCIHandle  != NULL);
    ASSERT(pUARTHandle != NULL);
    
    #if USE_HSLOG
    static LogBuffer_t LogBuffer;   
    
    pHandle->pVController = pVCIHandle;               // Pointer to VController
    pHandle->pUART_handle = pUARTHandle;              // Pointer to UART instance  
    pHandle->pUART_handle->Super = pHandle;    // Initialise the super pointer that the UART needs   
    
    pHandle->pUART_handle->pRxCallback = &LogHS_RX_IRQ_Handler;   // Link the interrupts from the UART instance to this module
    pHandle->pUART_handle->pTxCallback = &LogHS_TX_IRQ_Handler; 
    
    pHandle->pUART_handle->UARTBaudrate = BAUD115200;
    uCAL_UART_Init(pHandle->pUART_handle);  // Initialise the UART module with the baudrate that we need.  
    uCAL_UART_Receive(pHandle->pUART_handle, &(pHandle->RxByte), sizeof(pHandle->RxByte));  // Start the first reception   
   
    pHandle->ByteCount = 0;
    pHandle->State     = 0;
    pHandle->LogHSBufferIndex = 0;   
    pHandle->TxComplete     = false; 
    pHandle->DumpInProgress = false;
    pHandle->pLogHSBuffer = &LogBuffer;   
                    
    //Initial behavior of the logger                
    pHandle->LogHSEnable      = true; // States if the logger is active by default
    pHandle->BufferWrapAround = true; // States if the buffer automatically wraps around
    
    // Reset resolution counter and initialize n-1 number of calls to skip
    HSLog_ResolutionCounter = 0;
    pHandle->LogHSResolution = LOGHS_RESOLUTION;  
    #endif    
}

/**
  Manages UART recepetion interrupts when the Log high speed is in use
*/
void LogHS_RX_IRQ_Handler(void *pVoidHandle)
{
    ASSERT(pVoidHandle != NULL);    
    LogHighSpeed_Handle_t *pHandle = pVoidHandle;  // Convert the void handle pointer to a handle pointer
    
    
    if(pHandle->ByteCount == 0) // If this is the first byte in a frame
    {
        pHandle->RxFrame = ((uint16_t)(pHandle->RxByte << 8)) & 0xFF00; // Store the byte
        pHandle->ByteCount++; 
        uCAL_UART_Receive(pHandle->pUART_handle, &(pHandle->RxByte), sizeof(pHandle->RxByte));         
    }
    else if(pHandle->ByteCount == 1) // If it's the second we have completed a frame
    {    
        pHandle->RxFrame += (uint16_t)(pHandle->RxByte & 0x00FF); // Store the byte 
        pHandle->ByteCount = 0;
        
        uCAL_UART_SetTaskFlag(pHandle->pUART_handle);  // Tell the task that we unblocked the task
        osThreadFlagsSet(COMM_Uart_handle, UART_FLAG); // Notify task that we have a frame
    }
    else
    {
        //We should never reach this
        pHandle->ByteCount = 0;
    }        
        
}    

/**
  Manages UART transmission interrupts when the Log high speed is in use
*/
void LogHS_TX_IRQ_Handler(void *pVoidHandle)
{
    ASSERT(pVoidHandle != NULL);    
    LogHighSpeed_Handle_t *pHandle = pVoidHandle;  // Convert the void handle pointer to a handle pointer
    
    pHandle->TxComplete = true;
} 

/**
  Function that is called once a complete UART frame is received. It process
    the frame and takes decisions according to the current state of the module and the 
    contents of the frame.
*/
void LogHS_ProcessFrame(LogHighSpeed_Handle_t *pHandle)
{
    ASSERT(pHandle != NULL);
    
    switch(pHandle->RxFrame)
    {
        case START_LOG: // PC asks us to start the log 
            if(!pHandle->DumpInProgress) // We cant start logging if we arre currently dumping the log
            {
                pHandle->LogHSEnable = true;
            }
            break;
        
        case END_LOG: // PC ask us to end/stop the log
            pHandle->LogHSEnable = false;
            break;
        
        case DUMP_LOG: // PC asks us to start the data dumping process            
            if(pHandle->State == STANDBY) // if we are in standby
            {
                pHandle->State = SEND_DUMP_INFO; // Tell the Pc how much data we will send
                LogHS_SendDataStateMachine(pHandle);
            }
            break;
            
        case READY_TO_RECEIVE:  // PC tells us it is ready to receive that data        
            if(pHandle->State == SEND_DUMP_INFO)
            {
                pHandle->State = SEND_DATA;
                LogHS_SendDataStateMachine(pHandle);
            }
            break;
        case FORMAT_LOG:       // PC tells us to wipe the contents of the buffer
            if(!pHandle->DumpInProgress)
            {
                LogHS_ResetBuffer(pHandle);                        
            }            
            break;
    } 
    uCAL_UART_Receive(pHandle->pUART_handle, &(pHandle->RxByte), sizeof(pHandle->RxByte));    //Set the UART for the next reception    
}



/**
  Function that is used to coordinate the transfer of data between the controller
    and the pyhton script running on a PC.It does so using a simple state machine
*/
void LogHS_SendDataStateMachine(LogHighSpeed_Handle_t *pHandle)
{
    ASSERT(pHandle != NULL);
    
    switch(pHandle->State)
    {
        case STANDBY: 
             // Waiting for pc to request a data dump          
           break; 
        
        case SEND_DUMP_INFO: 
             // Tell the PC how many int16 we will send (dump info)
             LogHS_SendDumpInfo(pHandle); 
           break;
        
        case SEND_DATA: 
             // Send the data that got logged
             LogHS_StartDumpLog(pHandle);
             pHandle->State = STANDBY;
           break;            
    };  

}

/**
  Function that is used to insert motor control data into the logging buffer.
    a new data point made up of a data set is inserted in the buffere everytime
    this function is called.
*/
void LogHS_LogMotorVals(LogHighSpeed_Handle_t *pHandle)
{
    ASSERT(pHandle != NULL);
      
    ASSERT(pHandle != NULL); 
    if(pHandle->LogHSEnable && pHandle->DumpInProgress == false)
    {
            //Write these values in the logging array 
            (*(pHandle->pLogHSBuffer))[pHandle->LogHSBufferIndex][0] = MDI_GetIab(M1).a;    // Current A    
            (*(pHandle->pLogHSBuffer))[pHandle->LogHSBufferIndex][1] = MDI_GetIab(M1).b;    // Current B
            (*(pHandle->pLogHSBuffer))[pHandle->LogHSBufferIndex][2] = MDI_GetIqd(pHandle->pVController->pPowertrain->pMDI, M1).d;    // Id actual 
            (*(pHandle->pLogHSBuffer))[pHandle->LogHSBufferIndex][3] = MDI_GetIqd(pHandle->pVController->pPowertrain->pMDI, M1).q;    // Iq Actual 
            (*(pHandle->pLogHSBuffer))[pHandle->LogHSBufferIndex][4] = MDI_GetIqdref(pHandle->pVController->pPowertrain->pMDI, M1).d; // Id target 
            (*(pHandle->pLogHSBuffer))[pHandle->LogHSBufferIndex][5] = MDI_GetIqdref(pHandle->pVController->pPowertrain->pMDI, M1).q; // Iq target 
            (*(pHandle->pLogHSBuffer))[pHandle->LogHSBufferIndex][6] = MDI_GetAvrgMecSpeedUnit(pHandle->pVController->pPowertrain->pMDI, M1); //FOCCopy->hElAngle; // Electrical position    
            (*(pHandle->pLogHSBuffer))[pHandle->LogHSBufferIndex][7] = (int16_t)pHandle->pVController->pStateMachine->hVFaultOccurred; // Fault    

            if (pHandle->LogHSBufferIndex >= LOGHS_NB_SAMPLE_POINT-1)
            {
                if(pHandle->BufferWrapAround)//Check if we have wrap around enabled
                {
                    pHandle->LogHSBufferIndex = 0;
                }
                else                         //If we dont want wrap around stop the buffer
                {
                    LogHS_StopLog(pHandle);
                }                            
            }
            else
            {
                pHandle->LogHSBufferIndex ++;
            }               
    }
}

/**
  Function that is used to insert motor control data into the logging buffer.
    a new data point made up of a data set is inserted in the buffere every nth time
    this function is called.
*/
void LogHS_LogMotorValsVarRes(LogHighSpeed_Handle_t *pHandle)
{
    ASSERT(pHandle != NULL);
    if (HSLog_ResolutionCounter >= pHandle->LogHSResolution)  // Log only resolution counter has counted nth time
    {
        LogHS_LogMotorVals(pHandle); 
        HSLog_ResolutionCounter = 0;    // Reset resloution counter
    }
    else
    {
        HSLog_ResolutionCounter ++;     // Increment resolution counter
    }             
}
/**
  Function that is used to dump the collected data on the UART.
    this is called once the python script running on a CP has confirmed 
    that it is ready for the data.
*/
void LogHS_DumpLog(LogHighSpeed_Handle_t *pHandle)
{
    ASSERT(pHandle != NULL);
      static uint32_t ReadIndex;
    
    if(pHandle->DumpInProgress)
    {    
         //Start from the oldest data     
               if(pHandle->LogHSBufferIndex > 0)
                 {
                     ReadIndex = (pHandle->LogHSBufferIndex - 1); 
                 }
                 else
                 {
                     ReadIndex = LOGHS_NB_SAMPLE_POINT-1;
                 }
                 
                 //Log each data point represented by a data set
         for(int i = 0; i < LOGHS_NB_SAMPLE_POINT; i++)
         {
             if(ReadIndex < LOGHS_NB_SAMPLE_POINT - 1)
             {
                 ReadIndex ++;  
             }
             else
             {
                 ReadIndex = 0; 
             }  
        
             LogHS_DumpDataSet(pHandle,ReadIndex); 
             R_BSP_SoftwareDelay (250,BSP_DELAY_UNITS_MICROSECONDS); //delai is necessary            
         }
    
         pHandle->DumpInProgress = false;
         LogHS_ResetBuffer(pHandle);
         HSLog_ResolutionCounter = 0;
    }     
}

/**
  Function that is used to dump a data set. Only called by the DumpLog function.
    It is called as many times as there are data points. The data set index received 
    tells the function which set of data does it need to send over UART
*/
void LogHS_DumpDataSet(LogHighSpeed_Handle_t *pHandle, uint32_t DataSetIndex)
{
     ASSERT(pHandle != NULL);
    
   uint8_t buffer[LOGHS_NB_DATA*2];  
        
   for(unsigned int i = 0; i < LOGHS_NB_DATA; i++) // Transfer the int16_t data into a char array
   {     
      buffer[i*2]   = ((*(pHandle->pLogHSBuffer))[DataSetIndex][i]) & 0x00FF;           
      buffer[i*2+1] = (((*(pHandle->pLogHSBuffer))[DataSetIndex][i]) & 0xFF00) >> 8;  
   }
   
   pHandle->TxComplete = false;
       
   //Transfer this data set      
   uCAL_UART_Transmit(pHandle->pUART_handle,buffer,LOGHS_NB_DATA * 2);
       
   uint32_t Counter = 0;
   while(pHandle->TxComplete != true)
   {
       Counter ++;        
       if(Counter > 300000) // Failing to get a TxComplete before this 
       {                    // counter reaches 300000 will result in a failed
          Counter = 0;      // Transfer
          break;  
       }        
   }     
   
}

/**
  Function that is used to start the data dump.
*/
void LogHS_StartDumpLog(LogHighSpeed_Handle_t *pHandle)
{   
    ASSERT(pHandle != NULL);
    
    pHandle->LogHSEnable    = false;
    pHandle->DumpInProgress = true;
    LogHS_DumpLog(pHandle);   
}

/**
  Function that is used to send the dump info to the python script
    running on a PC. It tells it how many data points we have and how   
    many data we have per set (each set is a data point)
*/
void LogHS_SendDumpInfo(LogHighSpeed_Handle_t *pHandle)
{
      ASSERT(pHandle != NULL);
    
    static uint8_t DIbuffer[6]; //Dump Info buffer
    uint16_t NumberDataPoints = LOGHS_NB_SAMPLE_POINT;
    uint16_t NumberDataPerSet = LOGHS_NB_DATA;
       
    
    DIbuffer[0] = 0x64; // 'd'
    DIbuffer[1] = 0x69; // 'i'
    DIbuffer[2] = (uint8_t) (NumberDataPoints & 0x00FF);         //Prepare the values to be sent to QT
    DIbuffer[3] = (uint8_t) ((NumberDataPoints & 0xFF00) >> 8);
    DIbuffer[4] = (uint8_t) (NumberDataPerSet & 0x00FF);
    DIbuffer[5] = (uint8_t) ((NumberDataPerSet & 0xFF00) >> 8);

    
    pHandle->TxComplete = false;
       
    //Transfer the data info    
    uCAL_UART_Transmit(pHandle->pUART_handle,DIbuffer,6); //buffer is volatile because optimisation broke this part of the code

}    

/**
  Function that resets the contents of the logging buffer
*/
void LogHS_ResetBuffer(LogHighSpeed_Handle_t *pHandle)
{
      ASSERT(pHandle != NULL);
    
      if(pHandle->DumpInProgress == false)
        {
          pHandle->LogHSEnable = false; //disable the log while we reset the array
           for(int i = 0; i < LOGHS_NB_SAMPLE_POINT; i++)
        {          
                for(int y = 0; y < LOGHS_NB_DATA; y++)
                {
                  (*(pHandle->pLogHSBuffer)) [i][y] = 0;
              }
        }
        }
}

/**
  Functions that starts the logger 
*/
void LogHS_StartLog(LogHighSpeed_Handle_t *pHandle)
{
      ASSERT(pHandle != NULL);
      
      if(pHandle->DumpInProgress == false)
        {    
        pHandle->LogHSEnable = true;
        }
}

/**
  Functions that stops the logger 
*/
void LogHS_StopLog(LogHighSpeed_Handle_t *pHandle)
{
      ASSERT(pHandle != NULL);
    
    pHandle->LogHSEnable = false;
}

/**
  Functions that returns the position of the cursor
    of the logger.Value between 0 and  LOGHS_NB_SAMPLE_POINT - 1
*/
uint32_t LogHS_GetCursorPos(LogHighSpeed_Handle_t *pHandle)    
{
      ASSERT(pHandle != NULL);
    
    return(pHandle->LogHSBufferIndex);
}

/**
  Functions that resets the position of the cursor
*/
void LogHS_ResetCursor(LogHighSpeed_Handle_t *pHandle)
{
      ASSERT(pHandle != NULL);
    
      if(pHandle->DumpInProgress == false)
        {
        pHandle->LogHSBufferIndex = 0;
        }
}

/**
  Functions that starts a one shot capture of the high speed logger.
    This means that the contents of the buffer is erased and the logger is then started 
    Once it is competly filled it automatically stops.
*/
void LogHS_StartOneShot(LogHighSpeed_Handle_t *pHandle)
{
    ASSERT(pHandle != NULL);
    
    if(pHandle->DumpInProgress == false)
    {
        LogHS_StopLog(pHandle);             //Stop the logger (if it was started)
        LogHS_ResetCursor(pHandle);         //Reset the postion of the cursor
        pHandle->BufferWrapAround = false; //Disable the wrap around (if it was enabled)
        LogHS_StartLog(pHandle);            //Start the logger
    }
}

