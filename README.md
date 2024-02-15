# Cloning this repository
This repository has long paths on the files. Windows will not clone it cleanly without first setting it to allow long paths.

git config --global core.longpaths true

# Keil uVision installation
Keil uVision can be found here. Yes, you need to input all these fields to download it.
http://e.informer.com/s/keil.com/demo%2Feval%2Fc51.htm

# MDK installation
This MDK is required to make Keil uVision work with this firmware.
https://developer.arm.com/documentation/101407/0538/About-uVision/Installation

# GNR_firmware
Repository for Ganrunner firmware 
We decided to use the Flexible Software Package (FSP) for Renesas RA MCU Family, version 3.7.0.
The 1 reason is our MCUboot, from Renesas FSP, doesn't work on version 3.8.0 and change to version 4.0.0 or more 
require a lot of changes.
Version 3.7.0 has averything we need to continue our developement.

# ARM Compiler
The ARM compiler is version locked to V6.18.
Find a copy and instructions here:
https://tristarmulticopters.atlassian.net/wiki/spaces/ERR/pages/596410394/Software+environment+Configuration

# Renesas Advanced Smart Configurator
You'll need this specific Renesas Advanced Smart Configurator
https://github.com/renesas/fsp/releases/download/v3.7.0/setup_fsp_v3_7_0_rasc_v2022-04.exe

The tag for the repo of FSP 3.7.0 is here:
https://github.com/renesas/fsp/releases/tag/v3.7.0

# Configure the IDE
Once you've gotten all these things installed follow these steps to finally setup the IDE to use everything together properly.
https://tristarmulticopters.atlassian.net/wiki/spaces/DT/pages/410026781/CREATING+EDITING+Keil+PROJECT+ENVIRONMENT+FOR+RENESAS+uC

Note that creating a new project is unnecessary, you can generate with the Smart Configurator the code for this repository.

# Python Requirements
This repository uses a code signing process in python. If you don't have the python requirements no errors is risen except a failure to sign the package and create an image with the bootloader. To make sure this doesn't happen make sure to install imgtool.

pip install imgtool