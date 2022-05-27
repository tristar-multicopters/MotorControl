/**
*  uCAL_GPIO.c
*  uController Abstraction Layer for General Purpose Inputs and Outputs
*/ 
 
#include "uCAL_GPIO.h"
#include "hal_data.h"

// ==================== Private function prototypes ======================== //

/**
  Function used to convert a a standardized pin number
  to a Renesas specific reference to a pin using BSP,
	
  @param Receives pin number
  @return Returns a bsp pin reference (bsp_io_port_pin_t)
*/
bsp_io_port_pin_t uCAL_GPIO_PinNumToBSP(uint32_t aPinNum);

/**
  Function used to convert a Renesas specific reference to a pin using BSP,
  to a standardized pin number
	
  @param Receives bsp pin reference (bsp_io_port_pin_t)
  @return uint32_t
*/
uint32_t uCAL_GPIO_BSPToPinNum(bsp_io_port_pin_t aBSPPin);

/**
  Function used to read the output state of a pin (different from the input state)
  @param Receives bsp pin reference (bsp_io_port_pin_t)
  @return uint32_t
*/
uint32_t uCAL_GPIO_ReadPinOutput(bsp_io_port_pin_t aBSPPin);

// ========================================================================= //
 
/**
  Function used to reinitialise the state of a GPIO
*/
/*void uCAL_GPIO_ReInit(uint32_t aGPIO, struct GPIOConfig aPinConfig)
{
    bsp_io_port_pin_t bspPin;
	
    bspPin = uCAL_GPIO_PinNumToBSP(aGPIO); // Convert to BSP pin standard
		
	
    // Direction input or output
    if (aPinConfig.PinDirection == Input) 
    {
        R_PFS->PORT[bspPin >> 8].PIN[bspPin & BSP_IO_PRV_8BIT_MASK].PmnPFS_b.PDR = 0; // Setting pin as input
    }
    else if (aPinConfig.PinDirection == Output)
    {
        R_PFS->PORT[bspPin >> 8].PIN[bspPin & BSP_IO_PRV_8BIT_MASK].PmnPFS_b.PDR = 1; // Setting pin as output
    }		 
	 
    // Pull up or none
    if (aPinConfig.PinPull == None)
    {
        R_PFS->PORT[bspPin >> 8].PIN[bspPin & BSP_IO_PRV_8BIT_MASK].PmnPFS_b.PCR = 0;  // Disable pull up
    }
    else if (aPinConfig.PinPull == Up)
    {
        R_PFS->PORT[bspPin >> 8].PIN[bspPin & BSP_IO_PRV_8BIT_MASK].PmnPFS_b.PCR = 1;  // Enable pull up
    } 
	 
    // Output open drain or push/pull
    if (aPinConfig.PinOutput == PushPull)
    { 
        R_PFS->PORT[bspPin >> 8].PIN[bspPin & BSP_IO_PRV_8BIT_MASK].PmnPFS_b.NCODR = 0; // Setting pin as PushPull
    }
    else if (aPinConfig.PinOutput == OpenDrain)
    {
	    R_PFS->PORT[bspPin >> 8].PIN[bspPin & BSP_IO_PRV_8BIT_MASK].PmnPFS_b.NCODR = 1; // Setting pin as OpenDrain
    }	 
}*/

/**
  Function used to read the state of a GPIO
*/
bool uCAL_GPIO_Read(uint32_t aGPIO)
{
    uint32_t pinState;
    bsp_io_port_pin_t bspPin;
	
    bspPin = uCAL_GPIO_PinNumToBSP(aGPIO); // Convert from FTEX pins to BSP
	
    if (R_PFS->PORT[bspPin >> 8].PIN[bspPin & BSP_IO_PRV_8BIT_MASK].PmnPFS_b.PDR) // Check the port direction 0 In,1 Out
    {                         
        pinState = R_BSP_PinRead((bsp_io_port_pin_t) aGPIO);
    }
    else
    {
        pinState = uCAL_GPIO_ReadPinOutput(bspPin);
    } 	 
	 
    return (bool) pinState; 
}

/**
  Function used to set a GPIO
*/
void uCAL_GPIO_Set(uint32_t aGPIO)
{
    bsp_io_port_pin_t bspPin;
	
    bspPin = uCAL_GPIO_PinNumToBSP(aGPIO); // Convert from FTEX pins to BSP
	
    R_BSP_PinAccessEnable();
    R_BSP_PinWrite((bsp_io_port_pin_t) aGPIO,(bsp_io_level_t) 1); // Set the pin
}

/**
  Function used to reset a GPIO
*/
void uCAL_GPIO_Reset(uint32_t aGPIO)
{
    bsp_io_port_pin_t bspPin;
	
    bspPin = uCAL_GPIO_PinNumToBSP(aGPIO); // Convert from FTEX pins to BSP
	
    R_BSP_PinAccessEnable();
    R_BSP_PinWrite(bspPin,(bsp_io_level_t) 0); // Reset the pin 
}

/**
  Function used to toggle a GPIO
*/
void uCAL_GPIO_Toggle(uint32_t aGPIO)
{
    bsp_io_port_pin_t bspPin;
  
    bspPin = uCAL_GPIO_PinNumToBSP(aGPIO); // Convert from FTEX pins to BSP
	
    R_BSP_PinAccessEnable();

    if (uCAL_GPIO_ReadPinOutput(bspPin)) // Check if the pin is high
    {	
        R_BSP_PinWrite(bspPin,(bsp_io_level_t) 0); // If it is then put it low
    }		
    else
    {
        R_BSP_PinWrite(bspPin,(bsp_io_level_t) 1);// If not, it is already low so put it high
    } 
}

/**
  Function used to convert pin reference from FTEX to BSP
*/
bsp_io_port_pin_t uCAL_GPIO_PinNumToBSP(uint32_t aPinNum)
{
    uint32_t pinRef;
	
    // Conversion from FTEX pin number to BSP means that we need to only shift the 
    // seconde quartet, one quartet to the left.Ex pin 177 0x00B1 becomes 0x0B01	
    pinRef = ((aPinNum & 0x000000F0) << 4) + (aPinNum & 0x0000000F);
	
    return (bsp_io_port_pin_t) pinRef;
}

/**
  Function used to convert pin reference from BSP to FTEX
*/
uint32_t uCAL_GPIO_BSPToPinNum(bsp_io_port_pin_t aBSPPin)
{
    uint32_t pinRef;
	
    // Conversion from BSP to FTEX pin number means that we need to only shift the 
    // seconde quartet, one quartet to the right.Ex Port 5 pin 7 0x0507 becomes 0x0057
    pinRef = (((uint32_t)aBSPPin & 0x000000F0) << 4) + ((uint32_t)aBSPPin & 0x0000000F);
	
    return (uint32_t) pinRef;
}

/**
  Function used to read the current output value of a pin
*/
uint32_t uCAL_GPIO_ReadPinOutput(bsp_io_port_pin_t aBSPPin)
{   
    return R_PFS->PORT[aBSPPin >> 8].PIN[aBSPPin & BSP_IO_PRV_8BIT_MASK].PmnPFS_b.PODR;
}