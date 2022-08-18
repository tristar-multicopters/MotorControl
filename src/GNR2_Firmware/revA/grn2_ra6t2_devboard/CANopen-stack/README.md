=====================================================================================================
FREE CANOPEN STACK
=====================================================================================================

This project is a free implementation of the CANopen protocol according to the free specification
CiA 301. You need to register at Can in Automation (CiA) to retrieve your copy of the specification.

The source code is compliant to the C99 standard and you must cross-compile the source files as
part of your project with the cross-compiler of your choice.

Note: The source code of this project is independent from the CAN controller and microcontroller
hardware. The hardware specific parts are called drivers. For a full featured CANopen stack, 
we need the drivers for hardware timer, CAN controller and a non-volatile storage media.

This stack was taken from https://github.com/embedded-office/canopen-stack.

=====================================================================================================
FILES LOCATION
=====================================================================================================

* CANOpen stack files can be found on the file path ...\GitHub\GNR_firmware\src\GNR2_Firmware\revA\grn2_ra6t2
* CAN open abstraction layer files will be found on ...\GitHub\GNR_firmware\src\GNR2_Firmware\revA\grn2_ra6t2\FTEX\inc\peripherals_abstraction\CANOpen
  and ...\GitHub\GNR_firmware\src\GNR2_Firmware\revA\grn2_ra6t2\FTEX\src\peripherals_abstraction\CANOpen 