/**
* @file   ASSERT_FTEX.h
* @author Andy B
* @brief  This file contaisn the FTEX assert wrapper.
*
* This file contaisn the wrapper for assert checks which enables us to either call
* the renesas assert when we are using a debugger or to log asserts with the CAN
* logger. To add asserts checks to other files make sure to include this .h and use 
* ASSERT(expr); where expr is the expression you want to test.
*
*/


#ifndef __ASSERT_FTEX_H
#define __ASSERT_FTEX_H

/* Includes ------------------------------------------------------------------*/
#include "assert.h"
#include <stdbool.h>
#include <string.h>

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__) //Used to show only the name of the file in the debugger

#define ASSERT(e) InternalAssert(e, __FILENAME__,__LINE__) //Macro used to add the file name and line # 
                                                       //automatically to help debugging.
#define UNUSED_VARIABLE(X) ((void)(X))
#define UNUSED_PARAMETER(X) UNUSED_VARIABLE(X)
extern void InternalAssert(bool, const char *, int);

#endif /* __ASSERT_FTEX_H */