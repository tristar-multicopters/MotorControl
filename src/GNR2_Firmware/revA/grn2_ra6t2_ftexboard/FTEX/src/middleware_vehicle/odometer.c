/**
  * @file    odometer.c
  * @brief   This module handles odometer management
  *
  */

#include "odometer.h"
#include "gnr_information.h"
#include "ASSERT_FTEX.h"

OdometerHandle_t Odometer;
/* Functions ---------------------------------------------------- */
/**
   Initializes Odometer function
 */
void Odometer_Init(Delay_Handle_t * pOdometerDelay, uint16_t TimeIntervalMS)
{
    ASSERT(pOdometerDelay != NULL);
    ASSERT(TimeIntervalMS  > 0);
    
    Odometer.pOdometerDelay = pOdometerDelay;
    
    Delay_SetTime(Odometer.pOdometerDelay, 1, SEC);
    
    Odometer.TimeIntervalMS = TimeIntervalMS;
    
    // Read the memory to initialise the odometer with the distance already recorded
    Odometer.OdometerDistance = GnrInfo_GetOdometer();
}

void Odometer_Update(void)
{
   static uint16_t SpeedStart = 0; // Speed at the start of the time delay
   static uint16_t SpeedEnd   = 0; // Speed at the end of the time delay  
   const float DelayTimeInSec = Odometer.TimeIntervalMS/1000;
    
   float AvgSpeed = 0;   
   uint16_t MetersTravelled = 0; 

    
   if (Delay_Update(Odometer.pOdometerDelay)) // Check if it's time to compute the travelled distance
   {
        SpeedEnd = Wheel_GetVehicleSpeedFromWSS();
       
        if (SpeedStart > 0 || SpeedEnd > 0) // Check if the bike has moved at all
        {   
            // Compute the average speed over the odometer delay time            
            AvgSpeed = (SpeedStart + SpeedEnd)/2;
       
            // Conversion from km/h to m/s
            AvgSpeed = AvgSpeed/KM_H_TO_M_S;
       
            // Multiple the speed in m/s with time in s to obtain meters travelled
            MetersTravelled = (uint16_t)round(AvgSpeed * DelayTimeInSec);
            
            // Add the enw distance travelled to the odometer tally
            Odometer.OdometerDistance += MetersTravelled;
        }
        
        SpeedStart = SpeedEnd; 
    }
}

uint32_t Odometer_GetDistanceTravelled(void)
{
   return Odometer.OdometerDistance; 
}

void Odometer_Save(void)
{
   GnrInfo_DownloadOdometer(Odometer.OdometerDistance);
}
