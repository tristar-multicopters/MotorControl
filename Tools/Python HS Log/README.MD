Instruction to use FTEX high speed logger

- Installation of python from :https://www.python.org/downloads/  

   - Install with custom installation.

   - Check add python to environment variables.
 
   - Make sure it is installed in this location : C:\Python

- run "pip install pyserial" in cmd  

- Make sure the logger is setup and enabled in the controller

- Plug in your PC a USB to serial adapter in the controller and in your pc
- If the drivers for USB to UART are not present, install them from: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers?tab=downloads

- Launch the .bat file
- Launch FTEX_HSLogger. If you are unable to run that file, open python HS Log folder in the terminal and run the following command: python FTEX_HSLogger.py. 

PS resulting .csv will be placed in C:\FTEX_HSLogs
