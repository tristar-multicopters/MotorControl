/**
  *    ASSERT_FTEX.c
  *    Andy B
  *    This file contaisn the FTEX assert wrapper.
  *    You must include ASSERT_FTEX.h to use asserts
  */

#include "ASSERT_FTEX.h"
#include <stdio.h>


#define LOGENABLE 1 //To be moved to the apropriate location when the CAN logger module is implemented

/**
  DO NOT DIRECLY CALL THIS FUNCTION
  
  @brief This is the FTEX assert wrapper.If you want to add an assert check in your code  
  do it like this : ASSERT(X > 25);
  After going through the macro this function will be called along with the file in 
  which the assert was called and the line at which is was called. 

  For now the line and file number arent used. Waiting on the CAN bus logging module.
*/
void InternalAssert(bool expression , const char* failedFile, int failedLine)
{ 
    #ifdef LOGENABLE //If we want to log asserts
        
        //TODO: add call to the CAN logger when module is integrated into this project
        UNUSED_VARIABLE(failedFile);
        UNUSED_VARIABLE(failedLine);    
        //Send data to log
        //log(e, file, line);
           
    #endif   
     
    //Call renesas assert
    assert(expression);  
}
