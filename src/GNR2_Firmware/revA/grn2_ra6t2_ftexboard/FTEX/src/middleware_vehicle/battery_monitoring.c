/**
  * @file    battery_monitoring.c
  * @brief   This module handles battery monitoring
  *
*/

#include "battery_monitoring.h"
#include "vc_errors_management.h"
#include "ASSERT_FTEX.h"
 

/* Functions ---------------------------------------------------- */

/**
 *  Initializes battery monitor module
 */
void BatMonitor_Init(BatMonitor_Handle_t * pHandle, MotorControlInterfaceHandle_t * pMCI)
{	
    ASSERT(pHandle != NULL);
    ASSERT(pMCI != NULL);    
    pHandle->pMCI = pMCI;
    pHandle->ValCount    = 0;
    pHandle->StartupDone = false;
    pHandle->VBatAvg     = 0;    
    pHandle->LowBattery  = false;
}

/**
 *  Updates the VBat average with a new value
 */
void BatMonitor_UpdateAvg(BatMonitor_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL); 
    uint32_t NbVals = 0;
    uint32_t Sum  = 0;
    
    pHandle->VBatLog[pHandle->ValCount] =  MCInterface_GetBusVoltageInVoltx100(pHandle->pMCI);// Get the VBat voltage
    
    if(pHandle->StartupDone == false) //Check if we are still in our first roud of filling the array
    {
       NbVals = pHandle->ValCount + 1; 
       if(pHandle->ValCount == NB_SAMPLES_4_AVG - 1)
       {
           pHandle->StartupDone = true;
       }           
    }   
    else
    {
        NbVals = NB_SAMPLES_4_AVG;  
    }
    
    for(uint16_t i = 0; i < NbVals; i++) //Add all the values together
    {
        Sum = Sum + pHandle->VBatLog[i];      
    }
    
    pHandle->VBatAvg = (uint16_t)(Sum/NbVals); //Compute the average 
    
    if(pHandle->ValCount == NB_SAMPLES_4_AVG - 1) //Check if the cursor needs to wrap around
    {
        pHandle->ValCount = 0;   
    }
    else
    {
        pHandle->ValCount ++;  
    }
    
}

/**
 *  Calculate the SOC based on the voltage average and checks the value over
 *  numerous calls of this function to determine if we need to update the SOC
 */
void BatMonitor_ComputeSOC(BatMonitor_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    uint16_t NewSOC = 0;
    uint32_t VoltPercent = 0;
    
    VoltPercent = (pHandle->VBatMax * 100u) - (pHandle->VBatMin * 100u); // Values are * 100 to upscale the calculation
                                                                         // So we have better accuracy
    VoltPercent =  VoltPercent/100u; // Calculate how many volts correpsond to 1 % charge
    
    if(pHandle->VBatAvg < (pHandle->VBatMin * 100))
    {
        pHandle->VBatAvg = pHandle->VBatMin;   
    }   
    
    NewSOC = ((pHandle->VBatAvg) - (pHandle->VBatMin * 100u))/VoltPercent; //Calculate the SOC
    
    // We need to get a new SOC value that is consistent enough
    // before we decided to change the actual SOC.
    // to ensure this we use Transition_couunt.
    if(NewSOC != pHandle->SOC)
    {
        pHandle->Transition_count ++; // Everytime we get a new value increment by 1,
                                                                                                                                                          
        if((pHandle->Transition_count == (NB_SAMPLES_4_AVG * 10)) || pHandle->StartupDone == false)
        {
           pHandle->Transition_count = 0;// once the counter reaches NB_SAMPLES_4_AVG * 10 we consider the new SOC 
                                         // value stable enough to become the actual SOC
           pHandle->SOC = NewSOC;
        }
    }   
    else
    {
       if(pHandle->Transition_count - NB_SAMPLES_4_AVG/4 > 0) // everytime we dont get a new value decrement by NB_SAMPLES_4_AVG/4
       {
          pHandle->Transition_count = pHandle->Transition_count - NB_SAMPLES_4_AVG/4;
       }
       else
       {
          pHandle->Transition_count = 0;
       }     
    }
    
    if(pHandle->SOC > 100u) //Basic check just to make sure value is always 0%-100%
    {
        pHandle->SOC = 100u;    
    }
    else if (pHandle->SOC <= 0u)
    {
        pHandle->SOC = 0u; 
    }
    
}

/**
 *  Updates the SOC of the battery
 *  (simply calls updateAvg and ComputeSOC)
 */
void BatMonitor_UpdateSOC(BatMonitor_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    
    BatMonitor_UpdateAvg(pHandle);
    BatMonitor_ComputeSOC(pHandle);
    
    // If the SOC is below the threshold 
    if ((pHandle->SOC <= pHandle->LowBatSOC) && (pHandle->LowBattery == false))
    {
        pHandle->LowBattery = true; // update the flag
        
    }
    // If the SOC is above the recharge threshold 
    else if((pHandle->SOC >= pHandle->RechargedBatSOC) && (pHandle->LowBattery == true))
    {
        
        pHandle->LowBattery = false; // update the flag
    }

    
    if(pHandle->LowBattery == true) // If we have a low battery
    {
        VC_Errors_RaiseError(BATT_LOW, HOLD_UNTIL_CLEARED); // raise the flag
    }
    else // If not
    {
        VC_Errors_ClearError(BATT_LOW); // clear the flag
    }        
}

/**
 *  Get the current SOC value
 */
uint16_t BatMonitor_GetSOC(BatMonitor_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->SOC;
}

/**
 *  Get the low battery flag
 */
uint16_t BatMonitor_GetLowBatFlag(BatMonitor_Handle_t * pHandle)
{
    ASSERT(pHandle != NULL);
    return pHandle->LowBattery;
}
