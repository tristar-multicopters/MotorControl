/**
*  @file motor_signa_processing.c
*  @brief Middleware layer used to processing 
*         signal comming from the motor.
*/ 

/*********************************************
                Includes                       
*********************************************/

#include "motor_signal_processing.h"
#include "board_hardware.h"
#include "ASSERT_FTEX.h"
#include "ntc_table.h"


/*********************************************
                Defines
*********************************************/

/*********************************************
                Private Variables
*********************************************/

//initialize the struct responsible to handle the data/info in the processing motor siganl
static processingMotorSignal_t processingMotoSignal = 
{
    .adcChannelConfigured = false,
    .algorithmTick = ALGORITHM_TICK_US, 
    .motorTemperature = INITIAL_TEMP_MOTOR,
    .wheelSpeedPeriod = 0, 
    .maxWheelSpeedPeriodUs = MAX_WHEELSPEED_PERIOD_US, 
    .minSignalThreshold = MINIMUM_SIGNAL_THRESHOLD
};

                                                       
/*********************************************
                Public Variables
*********************************************/


/****************************************************************
                Private Functions Prototype
*****************************************************************/
/**
  @brief Function used to extract motor temperature signal
         from the mixed digitalized signal.
*/

static void extractMotorTemperature(uint16_t signal);  

/**
  @brief Function used to extract bike wheel speed value
         from the mixed digitalized signal.
  @param uint16_t signal mixed signal to be separated.
  @return void.
*/

static void extractWheelSpeed(uint16_t signal);


/****************************************************************
                Private Functions 
*****************************************************************/

/**
  @brief Function used to extract motor temperature raw signal
         from the mixed digitalized signal.
  @param uint16_t signal mixed signal to be separated.
  @return void.
*/

static void extractMotorTemperature(uint16_t signal)
{
    static uint32_t countToMaximum = 0;
    static uint16_t signalMaxValue = INITIAL_TEMP_MOTOR;
   
    // 
    if (signal > signalMaxValue)
    {
        signalMaxValue = signal;
    }
    
    // signal must to be, consecutivly times, above the
    // minimum value. This indicate that our signal is good
    //and is not having transition detection.
    if ((signalMaxValue > processingMotoSignal.minSignalThreshold) && (countToMaximum >= TEMPERATURE_MAX_PERIOD_US) && (processingMotoSignal.wheelSpeedPeriod > 0))
    {
        processingMotoSignal.motorTemperature = signalMaxValue;
        signalMaxValue = 0;
        countToMaximum = 0;
    }
    
    //
    countToMaximum = countToMaximum  + processingMotoSignal.algorithmTick;
}

/**
  @brief Function used to extract wheel speed period
         from the mixed digitalized signal.
  @param uint16_t signal mixed signal to be separated.
  @return void.
*/

static void extractWheelSpeed(uint16_t signal)
{
    static bool lowLevelDtectionFlag = false;
    static bool newLowLevelDtectionFlag = false;
    static bool startMeasureFlag = false;
    static bool endMeasureFlag = false;
    static uint32_t wheelSpeedPeriod = 0;
    static uint8_t avgFilterIndex = 0;
    static uint8_t avgFiltercoef = 0;
    static float filterArray[MAX_AVG_FILTER_SIZE] = {0};
    float accuWheelSpeedValue = 0;
    
    //
    if ((signal < processingMotoSignal.minSignalThreshold) && (lowLevelDtectionFlag == false))
    {
        lowLevelDtectionFlag = true;
    }
    
    //
    if ((signal >= processingMotoSignal.minSignalThreshold) && (lowLevelDtectionFlag == true) && (startMeasureFlag == false))
    {
        startMeasureFlag = true;
        wheelSpeedPeriod = 0;
    }
    
    //increment the period
    wheelSpeedPeriod = wheelSpeedPeriod + processingMotoSignal.algorithmTick;
    
    // new low detection condition test.
    if ((signal < processingMotoSignal.minSignalThreshold) && (newLowLevelDtectionFlag == false) && (startMeasureFlag == true))
    {
        newLowLevelDtectionFlag = true;
    }
    
    //low to high and a high to low condition was detected. 
    //wheel speed measure is done.
    if ((signal < processingMotoSignal.minSignalThreshold) && (newLowLevelDtectionFlag == true) && (startMeasureFlag == true))
    {
        endMeasureFlag = true;
    }
    
    //verify if wheel speed measure is finished.
    if (endMeasureFlag == true)
    {
        endMeasureFlag = false;
        startMeasureFlag = false;
        lowLevelDtectionFlag = false;
        newLowLevelDtectionFlag = false;
        
        //get the new wheel speed value.
        filterArray[avgFilterIndex] = (float)wheelSpeedPeriod;
        
        wheelSpeedPeriod = 0;
        
        //check max filter coeficent value.
        if (avgFiltercoef < MAX_AVG_FILTER_SIZE)
        {
            avgFiltercoef++;
        }
        
        //accumulate the value in the filter array.
        for (uint8_t n = 0; n <= (avgFiltercoef - 1); n++)
        {
            accuWheelSpeedValue = accuWheelSpeedValue + filterArray[n];
        }
        
        //get final avg value.
        processingMotoSignal.wheelSpeedPeriod = accuWheelSpeedValue/avgFiltercoef;
        
        // verify if array index is above the limit to reinitialize it.
        if (avgFilterIndex < (MAX_AVG_FILTER_SIZE - 1))
        {
            avgFilterIndex++;
        }
        else
        {
            avgFilterIndex = 0;
        }
    }
    
    //timeout detection to measure the signal.
    if (wheelSpeedPeriod >= processingMotoSignal.maxWheelSpeedPeriodUs)
    {
        processingMotoSignal.wheelSpeedPeriod = 0;
        wheelSpeedPeriod = 0;
        avgFilterIndex = 0;
        avgFiltercoef = 0;
        endMeasureFlag = false;
        startMeasureFlag = false;
        lowLevelDtectionFlag = false;
        newLowLevelDtectionFlag = false;
    }
}

/****************************************************************
                Public Functions 
*****************************************************************/

/**
  @brief Function used to get, process and extract motor temperature
         and wheel speed the mixed signal comming from the motor.
  @param none.
  @return void.
*/

void processingMotorMixedSignal(void)
{
    //verify if the right ADC channel was correctly assigned
    //to this function.
    if (processingMotoSignal.adcChannelConfigured == false)
    {
        //pass the adc channel used to digitalize the mixed motor signal
        processingMotoSignal.mixedSignalRegConv.hChannel = MOTOR_TEMP_ANALOG_CHANNEL;
        
        //get the index position where the digitalized mixed sensor will be.
        processingMotoSignal.mixedSignalIndex = RegConvMng_RegisterRegConv(&processingMotoSignal.mixedSignalRegConv);
        
        //adc channel correctly assigned now.
        processingMotoSignal.adcChannelConfigured = true;
    }
    
    //get the raw digitalized signal from the mixed signal
    uint16_t signal = RegConvMng_ReadConv(processingMotoSignal.mixedSignalIndex);
    
    //Call the algorithm responsible to extract and calculate wheel speed period value
    //from the mixed signal.
    extractWheelSpeed(signal);
    
    //Call the algorithm responsible to extract motor temperature raw signal information
    //from the mixed signal.
    extractMotorTemperature(signal);
}

/**
  @brief Function used to get, process and extract motor temperature
         and wheel speed the mixed signal comming from the motor.
  @param none.
  @return uint16_t raw motor temperature signal.
*/

uint16_t getExtractedMotorTemperature(void)
{
    return processingMotoSignal.motorTemperature;
}

/**
  @brief Function used to get the mixedSignalRegConv index.
  @param none.
  @return uint8_t mixedSignalIndex.
*/

uint8_t getMixedSignalRegConvIndex(void)
{
    return processingMotoSignal.mixedSignalIndex;
}

/**
  @brief Function used to get, process and extract wheel speed
         period on micro seconds from the mixed signal comming from the motor.
  @param none.
  @return float wheel speed period on seconds.
*/

float getExtractedWheelSpeed(void)
{
    return (((float)processingMotoSignal.wheelSpeedPeriod)*1.33f)/1000000.0f;
}