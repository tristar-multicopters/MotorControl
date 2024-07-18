/**
  ******************************************************************************
  * @file    lcd_apt.c
  * @author  Andy Beaudoin, FTEX
  * @brief   High level module that describes APT LCD communication protocol
  *
  ******************************************************************************
*/
    
#include "lcd_apt.h"
#include "ASSERT_FTEX.h"
#include "wheel.h"
#include "gnr_main.h"
#include "vc_constants.h"
#include "vc_errors_management.h"
#include <stdint.h>
#include <stdbool.h>

extern osThreadId_t COMM_Uart_handle; // Task Id for UART

/**
 *  Function for initializing the LCD module with the APT protocol
 *  It links its own custom interrupt routines with the UART callbacks using
 *  a void pointer. 
 */
void LCD_APT_init(APT_Handle_t *pHandle,VCI_Handle_t *pVCIHandle, UART_Handle_t *pUARTHandle)
{
    ASSERT(pHandle     != NULL);
    ASSERT(pVCIHandle  != NULL);
    ASSERT(pUARTHandle != NULL);
    
    pHandle->pVController = pVCIHandle;        // Pointer to VController
    pHandle->pUART_handle = pUARTHandle;       // Pointer to UART instance  
    pHandle->pUART_handle->Super = pHandle;    // Initialise the super pointer that the UART needs   
    
    pHandle->pUART_handle->pRxCallback = &LCD_APT_RX_IRQ_Handler;   // Link the interrupts from the UART instance to this module
    pHandle->pUART_handle->pTxCallback = &LCD_APT_TX_IRQ_Handler; 
        
    
    VC_Errors_SetCycleLength(6);
    
    pHandle->pUART_handle->UARTBaudrate = BAUD9600;
    uCAL_UART_Init(pHandle->pUART_handle);  // Initialise the UART module with the baudrate that APT screen needs  
    uCAL_UART_Receive(pHandle->pUART_handle, &(pHandle->RxByte), sizeof(pHandle->RxByte));  // Start the first reception   
}

/** 
 *  Function for building a frame specific to the APT protocol
 *  It is interrupt driven until a frame is completed where it  
 *  unblocks a comm task to process the frame.
 *  It is based on a byte by byte reception 
 */
void LCD_APT_RX_IRQ_Handler(void *pVoidHandle)
{     
    ASSERT(pVoidHandle != NULL);    
    APT_Handle_t *pHandle = pVoidHandle;  // Convert the void handle pointer to a handle pointer
    
    if (pHandle->RxCount >= RX_BYTE_BUFFER_SIZE) //Overflow safety
    {
       pHandle->RxCount = 0;
    }
    
    
    pHandle->RxBuffer[pHandle->RxCount] = pHandle->RxByte;
    pHandle->RxCount++;
    
    uCAL_UART_SetTaskFlag(pHandle->pUART_handle);  // Tell the task that we unblocked the task
    osThreadFlagsSet(COMM_Uart_handle, UART_FLAG); // Notify task that a byte has been received 
    uCAL_UART_Receive(pHandle->pUART_handle, &(pHandle->RxByte), sizeof(pHandle->RxByte));    
}

/**
 *  Function for sending a frame specific to the APT protocol
 *  once a frame has been received and processed this function
 *  sends the response made byt the frame process function.  
 *  It is based on a byte by byte transmission.   
 */
void LCD_APT_TX_IRQ_Handler(void *pVoidHandle)
{
    ASSERT(pVoidHandle != NULL);    
    APT_Handle_t *pHandle = pVoidHandle;  

    uint8_t tx_data;

    if (pHandle->tx_frame.ByteCnt < pHandle->tx_frame.Size) // Do we hav bytes to send ?
    {
        tx_data = pHandle->tx_frame.Buffer[pHandle->tx_frame.ByteCnt];
        pHandle->tx_frame.ByteCnt ++;
        // Send the data byte 
        uCAL_UART_Transmit(pHandle->pUART_handle, &tx_data, 1); 
    }
    else //If not then resume recepetion for the next frame
    {             
        pHandle->tx_frame.ByteCnt = 0;
        uCAL_UART_Receive(pHandle->pUART_handle, &(pHandle->RxByte), sizeof(pHandle->RxByte));
    }
}

/**
 *  Task for the APT screen
 *
 *
 */
void LCD_APT_Task(APT_Handle_t *pHandle)
{
   
    uint8_t ByteCount = pHandle->rx_frame.ByteCnt;
    uint8_t NextByte;    
    uint8_t BytesReceived[16];
    uint8_t NumbByteRx;
    
    
    NumbByteRx = pHandle->RxCount; //Save the number of bytes we have received but reset the variable so we dont miss any new bytes
    pHandle->RxCount = 0;
    
    for(int BIdx = 0; BIdx < NumbByteRx; BIdx++)
    {
        BytesReceived[BIdx] = pHandle->RxBuffer[BIdx]; //Save a local copy of the bytes we have received       
    }    
    
    for(int i = 0; i < NumbByteRx; i++) //Continue building the frame with the bytes we've received   
    {
        NextByte = BytesReceived[i];
        
        if (ByteCount == 0) // Each case represents the number of the byte received in a frame.                    
        {
             if (NextByte == APT_START) // Is it the proper start byte ?
             {
                 pHandle->rx_frame.Buffer[ByteCount] = NextByte;
                 pHandle->rx_frame.ByteCnt ++;                 
             }
             else // If not, its a bad cmd
             {
                 pHandle->rx_frame.ByteCnt = 0;
                 uCAL_UART_SendDefault(pHandle->pUART_handle);
             }
             
        }
        else if (ByteCount > 0 && ByteCount < 8)
        {
            // Byte 1 to byte 7 contain various information
            // They dont need to be checked because as long as 
            // the checksum is good then they are valid
            // The checksum is verified in the frame process function
                 
            pHandle->rx_frame.Buffer[ByteCount] = NextByte;
            pHandle->rx_frame.ByteCnt ++;
                
        }
        else if (ByteCount == 8) // Every frame has a length of 9 bytes (0-8)
        {
                                        
            if (NextByte == APT_END) // Is it the proper end byte ?
            {
                pHandle->rx_frame.Buffer[ByteCount] = NextByte;
                
                LCD_APT_ProcessFrame(pHandle); //We have received a full frame so do the frame processing
            }
            pHandle->rx_frame.ByteCnt = 0;
        }
        else
        {            
            // We should never get here but if we do trash the frame
            pHandle->rx_frame.ByteCnt = 0; 
            uCAL_UART_SendDefault(pHandle->pUART_handle);            
        }
    }        
}


/**
 *  Function for decoding a received frame (previously built in the RxCallback function)
 *  according to the APT screen protocol.
 *  This is executed in a comm task that gets unblocked when a complete frame is received.
 *
 */
void LCD_APT_ProcessFrame(APT_Handle_t *pHandle)
{
    APT_frame_t replyFrame = {0};
    static bool walkmodeTransition = false;
    static bool DefaultPasInitiliased = false;
    static uint8_t pasLvlBeforeWalk = 0;
    int32_t  toSend      = 0;
    uint32_t Check       = 0;
    uint16_t Merge       = 0;
    uint8_t  pasLvl; 
    uint8_t  LightStatus = 0;
   
     
    
    //Verification of the checksum
    for(int i = 0; i < 6; i += 2) //Checksum is the sum of double bytes into a 16 bits
    {                             //Single bytes need to be paired into a single 16 bits before being summed    
        Merge = (uint16_t) (pHandle->rx_frame.Buffer[i] << 8) +  pHandle->rx_frame.Buffer[i+1];
        Check += Merge;
    }
     
    if (Check > 0x0000FFFF) // Small patch to accomodate bug in APT screen
    {
        Check += 0x0000FFFF & (Check >> 16);      
    }
     
    Check = (Check & 0x0000FFFF); //Protection in case of overflow
    
    //Check if the CRC is good
    if (Check == (uint32_t)(pHandle->rx_frame.Buffer[CHECK + 1] + (pHandle->rx_frame.Buffer[CHECK] << 8)))
    {
         
        LightStatus = ((pHandle->rx_frame.Buffer[PAS]) & 0x10);
         
        if (LightStatus) //Operate the light according to what the screne tells us
        {
            Light_Enable(pHandle->pVController->pPowertrain->pHeadLight);  
            Light_Enable(pHandle->pVController->pPowertrain->pTailLight); 
        }
        else
        {
            Light_Disable(pHandle->pVController->pPowertrain->pHeadLight); 
            Light_Disable(pHandle->pVController->pPowertrain->pTailLight);              
        }   
         
        //Reading the Pass
        pasLvl = (pHandle->rx_frame.Buffer[PAS] & 0x0F); //Only the 4 LSB contain the pass level
        uint8_t maxLevels = pHandle->pVController->pPowertrain->pPAS->sParameters.bMaxLevel;
        uint8_t currentPas = PedalAssist_GetAssistLevel(pHandle->pVController->pPowertrain->pPAS);
        uint8_t standardizedPas = LCD_APT_ConvertPASLevelFromAPT(pasLvl,maxLevels);
        
        if(standardizedPas == DEFAULT_PAS_LEVEL && DefaultPasInitiliased == false) 
        {
            DefaultPasInitiliased = true;
        }
        
        
        if ((pasLvl != PAS_UNCHANGED && (standardizedPas != pHandle->OldPAS)) || (standardizedPas == PAS_LEVEL_WALK))
        {                  
            if((pasLvl == PAS_LEVEL_WALK) && (walkmodeTransition == false)) // when we switch to walk mode we need to remember our previous state
            {
                walkmodeTransition = true;
                pasLvlBeforeWalk = currentPas;
            }                    
                
            if(DefaultPasInitiliased)
            {    
                PedalAssist_SetAssistLevel(pHandle->pVController->pPowertrain->pPAS,standardizedPas);   
                pHandle->OldPAS = standardizedPas;                 
                pHandle->APTChangePasFlag = true;
            }
         }
         else if (walkmodeTransition && (standardizedPas != PAS_LEVEL_WALK)) // If we were in walkmode we need to restore the previous pas level
         {
            walkmodeTransition = false;
                
            PedalAssist_SetAssistLevel(pHandle->pVController->pPowertrain->pPAS,pasLvlBeforeWalk);         
            pHandle->APTChangePasFlag = true;
         }                


    #if DYNAMIC_SPEED_LIMITATION   

        uint8_t speedLimit;
        //Reading the Speed limit
        speedLimit = pHandle->rx_frame.Buffer[SPEED];        
        
        // setting the max speed for any speed limits
        PWRT_SetScreenMaxSpeed(pHandle->pVController->pPowertrain,speedLimit);
        
    #endif 
        
    #ifdef SCREENPOWERCONTROL
        
        // Reading the Current limit          
        uint16_t CurrentLimit;
        CurrentLimit = pHandle->rx_frame.Buffer[CURRENTL];
        
        PWRT_SetOngoingMaxCurrent(pHandle->pVController->pPowertrain, CurrentLimit);
        
    #endif

        //Reading and updating the wheel diameter 
        uint8_t diameterFromScreen = LCD_APT_CalculateWheelDiameter(pHandle->rx_frame.Buffer[WHEELD]);
        Wheel_SetWheelDiameter(diameterFromScreen);
    
        //For APT protocol, LSB is sent first for multi-bytes values
        replyFrame.Size = 13;
    
        replyFrame.Buffer[ 0] = APT_START; //Start
      
        //Get the amount of amps we are currently pushing 
        toSend = PWRT_GetDCCurrent(pHandle->pVController->pPowertrain);
         
        toSend = LCD_APT_ApplyPowerFilter((uint16_t)toSend);  
          
        toSend = toSend * 2; //Covert from amps to 0.5 amps; 
        
        replyFrame.Buffer[1] = (toSend & 0x000000FF); //Power 0.5 A/unit         

        /* Condition use for wheel speed sensor rpm to send */
        toSend = WheelSpeedSensor_GetSpeedRPM(); // Getting RPM from Wheel Speed Module
                   
        toSend =  LCD_APT_ApplySpeedFilter((uint16_t) toSend);
                  
        toSend = toSend * 500; //Converion from RPM to period in ms, 500 is used as scaling to conserve precision          
         
        if (toSend != 0) //verify toSend value to avoid a divisiyeon by zero.
        {
            toSend = 500000/(toSend/60); // Descaling here with the 500000 to return to original unit            
        }
            
         
        replyFrame.Buffer[2] = (toSend & 0x00FF);      // Wheel speed Low half 
        replyFrame.Buffer[3] = (toSend & 0xFF00) >> 8; // Wheel speed High half
        
        
        replyFrame.Buffer[4] = LCD_APT_ErrorConversionFTEXToAPT(VC_Errors_CycleError()); // Error Code
      
        replyFrame.Buffer[5] = 0x04; // Brake Code bXXXX 0100 means the motor is working
                      
        replyFrame.Buffer[6] = 0x00;
         
        // If we want to change the PAS level we need to change it here
        replyFrame.Buffer[7] = PAS_UNCHANGED; // Send 0x0A unless we want to change the PAS on the screen  
        
        currentPas = PedalAssist_GetAssistLevel(pHandle->pVController->pPowertrain->pPAS);
        if ((!pHandle->APTChangePasFlag && (pasLvl == PAS_UNCHANGED) && currentPas != pHandle->OldPAS) || DefaultPasInitiliased == false)
        {           
           // framesSinceSync = 0;
            replyFrame.Buffer[7] = currentPas;
            pHandle->OldPAS = currentPas;
        }
                               
        replyFrame.Buffer[8] = 0x00;
        replyFrame.Buffer[9] = 0x00;
    
        //Calculate checksum
        //Sum of paired bytes
        Merge = 0;
        Check = 0;
        for(uint16_t i = 0; i < NUMBER_OF_CRC_BYTES; i += BYTE_PER_PAIR)
        {
             Merge = (uint16_t) (replyFrame.Buffer[i+1] << 8) + replyFrame.Buffer[i];
             Check += Merge;
        }
        
        replyFrame.Buffer[10] =  (0x000000FF & Check);       // Checksum Low  Half
        replyFrame.Buffer[11] = ((0x0000FF00 & Check) >> 8); // Checksum High Half    
            
        replyFrame.Buffer[12] = APT_END; // End
            
        replyFrame.ByteCnt = 0;   
        
        pHandle->tx_frame = replyFrame; 
             
        LCD_APT_TX_IRQ_Handler((void *) pHandle); // Start the transmission of the answer frame
            
    }// End of CRC check
       
}


/**  
 *  Function used to apply a filter on the speed we send to the screen 
 */
uint16_t LCD_APT_ApplySpeedFilter(uint16_t aInstantSpeedInRPM)
{
   static uint16_t SpeedAvg; 
   uint16_t Bandwidth = 3; //Bandwidth CANNOT be set to 0
    
   SpeedAvg = ((Bandwidth-1) * SpeedAvg + aInstantSpeedInRPM)/Bandwidth;
   
   return (SpeedAvg); 
}    

/**  
 *  Function used to apply a filter on the power we send to the screen 
 */
uint16_t LCD_APT_ApplyPowerFilter(uint16_t aInstantPowerInAmps)
{
   static uint16_t PowerAvg; 
   uint16_t Bandwidth = 3; //Bandwidth CANNOT be set to 0
        
   PowerAvg = ((Bandwidth-1) * PowerAvg + aInstantPowerInAmps)/Bandwidth;
   
   return (PowerAvg); 
} 

/** Function used to translate the PAS level received from the APT  
 *  screen standard to the FTEX standard
 */
uint8_t LCD_APT_ConvertPASLevelToAPT(PasLevel_t aPAS_Level, uint8_t maxLevel)
{
   uint8_t PAS_Out = PAS_LEVEL_0; 
   switch(aPAS_Level)
   {
        case PAS_LEVEL_0:
            PAS_Out = 0x0;
            break;
        case PAS_LEVEL_1:
            PAS_Out = 0x1;
            break;
        case PAS_LEVEL_2:
            if (maxLevel == 5)
            {
                PAS_Out = 0x03;
                break;
            }
            PAS_Out = 0x2;
            break;
        case PAS_LEVEL_3:
            if (maxLevel == 5)
            {
                PAS_Out = 0x05;
                break;
            }
            PAS_Out = 0x3;
            break;
        case PAS_LEVEL_4:
            if (maxLevel == 5)
            {
                PAS_Out = 0x07;
                break;
            }
            PAS_Out = 0x4;
            break;
        case PAS_LEVEL_5:
            if (maxLevel == 5)
            {
                PAS_Out = 0x09;
                break;
            }
            PAS_Out = 0x5;
            break;
        case PAS_LEVEL_6:
            PAS_Out = 0x6;
            break;
        case PAS_LEVEL_7:
            PAS_Out = 0x7;
            break;
        case PAS_LEVEL_8:
            PAS_Out = 0x8;
            break;
        case PAS_LEVEL_9:
            PAS_Out = 0x9;
            break;
        case PAS_LEVEL_WALK: // For now we cant control walk mode
        default:  
            PAS_Out = 0x0;           
            break;

    }
    return PAS_Out;
}


/** Function used to translate the FTEX standard PAS level to the APT  
 *  screen standard.(is not the same as when we receive a PAS level from the APT screen)
 */
PasLevel_t LCD_APT_ConvertPASLevelFromAPT(uint8_t aPAS_Level, uint8_t aNumberOfLevels)
{
   PasLevel_t PAS_Out = 0x0;
   
    bool ExtendedPASLevels;
    
    if (aNumberOfLevels == 5)  // Check if we need to use the 5 PAS config
    {
        ExtendedPASLevels = false;
    }    
    else if (aNumberOfLevels == 9) // Or the 9 PAS config
    {
        ExtendedPASLevels = true; 
    }       
    else
    {
        ASSERT(false); // Number of PAS levels not compatible with APT screen  
    }       
   
    //PAS level from APT 0x02,0x04,0x06,0x08 are only for 9 PAS levels mode

    switch(aPAS_Level)
    {
        case 0x0:
            PAS_Out = PAS_LEVEL_0;
            break;
        case 0x1:
            PAS_Out = PAS_LEVEL_1;
            break;
        case 0x2:
            if (ExtendedPASLevels)
                PAS_Out = PAS_LEVEL_2;    
            break;
        case 0x3:
            if (ExtendedPASLevels)
                PAS_Out = PAS_LEVEL_3;
            else
                PAS_Out = PAS_LEVEL_2;
            break;
        case 0x4:
            if (ExtendedPASLevels)
                PAS_Out = PAS_LEVEL_4;
            break;           
        case 0x5:
            if (ExtendedPASLevels)
                PAS_Out = PAS_LEVEL_5; 
            else    
                PAS_Out = PAS_LEVEL_3;
            break;
        case 0x6:
            if (ExtendedPASLevels)
                PAS_Out = PAS_LEVEL_6;            
            break;           
        case 0x7:
            if (ExtendedPASLevels)
                PAS_Out = PAS_LEVEL_7;
            else           
                PAS_Out = PAS_LEVEL_4;
            break;
        case 0x8:
            if (ExtendedPASLevels)
                PAS_Out = PAS_LEVEL_8; 
            break;
        case 0x9:
            if (ExtendedPASLevels)
                PAS_Out = PAS_LEVEL_9; 
            else    
                PAS_Out = PAS_LEVEL_5;
            break;
        case 0xF:
            PAS_Out = PAS_LEVEL_WALK;
            break;
        case 0xA:
            PAS_Out = PAS_UNCHANGED;
            break;
        default:
            PAS_Out = PAS_LEVEL_0;
            break;
    }

   return PAS_Out;
}

// get the value we got from the APT screen for the wheel size and convert to wheel diameter if need be
uint8_t LCD_APT_CalculateWheelDiameter(uint16_t aValue)
{
   uint8_t wheelDiameter; 
    
   if (aValue <= APT_WHEEL_DIAM_INCHES_MAX && aValue > 0)
   {
       wheelDiameter = (uint8_t) aValue;
   } 
   else if (aValue > APT_WHEEL_CIRCUMFERENCE_MIN)
   {
       wheelDiameter = (uint8_t) round(((float) (aValue/FTEX_PI)/ (float) FTEX_INCH_TO_CM)); // Convert from circumference in cm to diamater in inches     
   }
   else
   {
       wheelDiameter = WHEEL_DIAMETER_DEFAULT;   
   }   

   return wheelDiameter;
}

/**
 * Function used to convert from standard FTEX error codes to APT error codes    
 */
uint8_t LCD_APT_ErrorConversionFTEXToAPT(uint8_t aError)
{
    uint8_t ConvertedError;
    switch(aError)
    {
        case NO_ERROR:
            ConvertedError = APT_NO_ERROR; 
            break;
        case THROTTLE_STUCK:
            ConvertedError = APT_THROTTLE;
            break;
        case OV_PROTECTION:
            ConvertedError = APT_OV_PROTECTION;
            break;
        case UV_PROTECTION:
            ConvertedError = APT_UV_PROTECTION;
            break;
        case CONTROLLER_OT_PROTECT:
            ConvertedError = APT_CONTROLLER_OT_PROTECT;
            break;
         case CONTROLLER_FOLDBACK_TEMP:
            ConvertedError = APT_CONTROLLER_TEMP_FOLDBACK;
            break;
        case UT_PROTECTION:
            ConvertedError = APT_UT_PROTECTION;
            break;
        case IOT_COMM_ERROR: // Both theses errors transflat eto a generic comm error
        case DUAL_COMM_ERROR:    
            #if VC_IOT_ERROR_DISPLAY == 1
                ConvertedError = APT_COMM_ERROR;
            #else
                ConvertedError = APT_NO_ERROR;
            #endif
            break;      
        case MOTOR_PHASE_ERROR:
            ConvertedError = APT_THREE_PHASE_ERROR;
            break;
        case MOTOR_HALL_ERROR:
            ConvertedError = APT_HALL_ERROR;
            break;
        case OVER_CURRENT:
            ConvertedError = APT_OVER_CURRENT;
            break;         
        case BATT_LOW:
            ConvertedError = APT_BAT_LOW;
            break;         
        case PAS_BOOT_ERROR:
            ConvertedError = APT_PAS_BOOT_ERR;
            break;         
        case CONTROLLER_ERROR:
            ConvertedError = APT_CONTROLLER_ERR;
            break;         
        case BRAKE_ERROR:
            ConvertedError = APT_BRAKE_ERROR;
            break;        
        case TORQUE_SENSOR_ERROR:
            ConvertedError = APT_TORQUE_SENSOR_ISSUE; 
            break;
        case MOTOR_OT_PROTECT:
        case MOTOR_NTC_DISC_FREEZE:
        case MOTOR_FOLDBACK_TEMP:
            ConvertedError = APT_MOTOR_OT_PROTECT;
            break;              
        case UNMAPPED_ERROR: // Errors that APT has but that we currently don't flag            
            ConvertedError = APT_TURN_ERROR;
            ConvertedError = APT_CONTROL_PROTEC;
            break;        
       /* default: // If it's not a standard APT error just consider it a custom error or a debugging error
            
            if (aError < 0x9F && aError != 0x30) // For custom errors we need a value less or equal to 0x9F but that isn't 0x30,  
            {
                ConvertedError = aError;
            }
            else
            {
                ConvertedError = 0x9F;
            }  
          break; */
    }
    
    return ConvertedError;
}

