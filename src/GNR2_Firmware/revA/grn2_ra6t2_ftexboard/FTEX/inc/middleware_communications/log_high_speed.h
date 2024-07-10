/**
  ******************************************************************************
  * @file    log_high_speed.h
  * @author  Andy Beaudoin, FTEX
  * @brief   High level module that logs high speed motor control variables
  *
  ******************************************************************************
*/

#ifndef __LOG_HIGH_SPEED_H
#define __LOG_HIGH_SPEED_H

#include "stdbool.h"
#include "uCAL_UART.h"
#include "vc_interface.h"
#include "gnr_main.h"
 
//Defines the types of messages that can eb sent by the PC 
typedef enum
{
  START_LOG        = 0x736C, // 'sl' (start logging) Used to start the high speed logging
  END_LOG          = 0x656C, // 'el' (end logging)   Used to end the high speed logging
  DUMP_LOG         = 0x6472, // 'dr' (data request)  Data request from the PC 
  READY_TO_RECEIVE = 0x7272, // 'rr' (ready receive) PC is ready to receive the logged data
  FORMAT_LOG       = 0x6672, // 'fr' (format ram)    PC wants us to wipe the contents of the logger
}Log_HS_msg_t;

//Defines the states in the state mahcine used to coordinate data transfer
enum SendDataState
{
  STANDBY,
  SEND_DUMP_INFO,
  SEND_DATA        
};   

#define LOGHS_NB_DATA           8 // Number of int16_t data per sample point
#define LOGHS_NB_SAMPLE_POINT   1250 // Number of sample points in the buffer
#define LOGHS_RESOLUTION        4 // Number of times skip sampling when LogHS_LogMotorVals is called.


extern osThreadId_t COMM_Uart_handle; // Task Id for UART

//Definition of the poiinter type to point to a logging buffer
typedef int16_t LogBuffer_t[LOGHS_NB_SAMPLE_POINT][LOGHS_NB_DATA];

//Defines the necessary variables needed by the log high speed module
typedef struct
{
    UART_Handle_t *pUART_handle;   // Contains the callback that will be assigned 
                                   // to the event_handler 
    VCI_Handle_t *pVController;    // Pointer to vehicle handle
    
    bool LogHSEnable;            // Enable r disables data collection
    bool TxComplete;             // Tx complete flag
    bool DumpInProgress;         // Indicates if we are currently transmitting data
    bool BufferWrapAround;       // Indicates if the buffer wraps around automatically
    uint8_t RxByte;              // Used for byte-by-byte reception
    uint16_t RxFrame;            // Used to store the received frame
    uint8_t State;               // Used in the data transfer state machine  
    uint8_t ByteCount;           // Counts how many bytes we have received by UART 
    uint32_t LogHSBufferIndex;   // Cursor that indicates where is the current newest log values in the buffer
    LogBuffer_t *pLogHSBuffer;   // Pointer to the buffer where the data is logged    
    uint8_t LogHSResolution;     // Specifies that actual recording occurs for every nth call of LogHS_LogMotorValsVarRes function. 
    
}LogHighSpeed_Handle_t;




/**
  @brief Initialises the handle variables that are specific to the module and
         links the generic UART interrupts of the specified UART instance to this module.
         VCI handle is used to acces the information that needs to be logged
                
  @param Receives LogHS handle, VCI handle and UART handle
  @return void
*/
void LogHS_Init(LogHighSpeed_Handle_t *pHandle,VCI_Handle_t *pVCIHandle, UART_Handle_t *pUARTHandle);

/**
  @brief Manages UART recepetion interrupts when the Log high speed is in use
  
  @param Receives LogHS handle as void pointer
  @return void
*/
void LogHS_RX_IRQ_Handler(void *pVoidHandle);

/**
  @brief Manages UART transmission interrupts when the Log high speed is in use
  
  @param Receives LogHS handle as void pointer
  @return void
*/
void LogHS_TX_IRQ_Handler(void *pVoidHandle);


/**
  @brief Function that is called once a complete UART frame is received. It process
                 the frame and takes decisions according to the current state of the module and the 
                 contents of the frame.
                 
  @param Receives LogHS handle
  @return void
*/
void LogHS_ProcessFrame(LogHighSpeed_Handle_t *pHandle);

/**
  @brief Function that is used to coordinate the transfer of data between the controller
           and the pyhton script running on a PC.It does so using a simple state machine
                 
  @param Receives LogHS handle
  @return void
*/
void LogHS_SendDataStateMachine(LogHighSpeed_Handle_t *pHandle);

/**
  @brief Function that is used to insert motor control data into the logging buffer.
                 a new data point made up of a data set is inserted in the buffere everytime
                 this function is called.
                 
  @param Receives LogHS handle
  @return void
*/
void LogHS_LogMotorVals(LogHighSpeed_Handle_t *pHandle);


/**
  @brief Function that is used to insert motor control data into the logging buffer.
                 a new data point made up of a data set is inserted in the buffere every nth time
                 this function is called.
                 
  @param Receives LogHS handle
  @return void
*/
void LogHS_LogMotorValsVarRes(LogHighSpeed_Handle_t *pHandle);

/**
  @brief Function that is used to dump the collected data on the UART.
                 this is called once the python script running on a CP has confirmed 
                 that it is ready for the data.
                 
  @param Receives LogHS handle
  @return void
*/
void LogHS_DumpLog(LogHighSpeed_Handle_t *pHandle);

/**
  @brief Function that is used to dump a data set. Only called by the DumpLog function.
                 It is called as many times as there are data points. The data set index received 
                 tells the function which set of data does it need to send over UART
                 
  @param Receives LogHS handle and the data set index
  @return void
*/
void LogHS_DumpDataSet(LogHighSpeed_Handle_t *pHandle, uint32_t DataSetIndex);

/**
  @brief Function that is used to start the data dump.
                 
  @param Receives LogHS handle and the data set index
  @return void
*/
void LogHS_StartDumpLog(LogHighSpeed_Handle_t *pHandle);

/**
  @brief Function that is used to send the dump info to the python script
           running on a PC. It tells it how many data points we have and how   
                 many data we have per set (each set is a data point)
                 
  @param Receives LogHS handle and the data set index
  @return void
*/
void LogHS_SendDumpInfo(LogHighSpeed_Handle_t *pHandle);

/**
  @brief Function that resets the contents of the logging buffer
                 
  @param Receives LogHS handle and the data set index
  @return void
*/
void LogHS_ResetBuffer(LogHighSpeed_Handle_t *pHandle);

/**
  @brief Functions that starts the logger 
                 
  @param Receives LogHS handle and the data set index
  @return void
*/
void LogHS_StartLog(LogHighSpeed_Handle_t *pHandle);

/**
  @brief Functions that stops the logger 
                 
  @param Receives LogHS handle and the data set index
  @return void
*/
void LogHS_StopLog(LogHighSpeed_Handle_t *pHandle);

/**
  @brief Functions that returns the position of the cursor
           of the logger.Value between 0 and  LOGHS_NB_SAMPLE_POINT - 1
                 
  @param Receives LogHS handle and the data set index
  @return uint32_t position of the cursor
*/
uint32_t LogHS_GetCursorPos(LogHighSpeed_Handle_t *pHandle);    

/**
  @brief Functions that resets the position of the cursor
                 
  @param Receives LogHS handle and the data set index
  @return void
*/
void LogHS_ResetCursor(LogHighSpeed_Handle_t *pHandle);

/**
  @brief Functions that starts a one shot capture of the high speed logger.
           This means that the contents of the buffer is erased and the logger is then started 
                 Once it is competly filled it automatically stops.
                 
  @param Receives LogHS handle and the data set index
  @return void
*/
void LogHS_StartOneShot(LogHighSpeed_Handle_t *pHandle);

#endif
