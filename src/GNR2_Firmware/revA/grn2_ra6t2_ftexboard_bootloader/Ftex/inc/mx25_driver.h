/**
  * @file    mx25_driver.h
  * @brief   This file is the MX25 serial flash header
    */

#ifndef __MX25_DRIVER_H
#define __MX25_DRIVER_H

/* Includes ------------------------------------------------------------------*/
#include "hal_data.h"
#include "uCAL_SPI.h"
#include "stdint.h"

/* Defines --------------------------------------------------------------------*/

#define CLOCK_FOR_READ    0x55    // For generate read clock.
                              //This data appear on MOSI during read period.
                                                            
typedef enum
{
  MX25L3233F_OK                 =  0,
  MX25L3233F_ERROR_INIT         = -1,
  MX25L3233F_ERROR_COMMAND      = -2,
  MX25L3233F_ERROR_TRANSMIT     = -3,
  MX25L3233F_ERROR_RECEIVE      = -4,
  MX25L3233F_ERROR_AUTOPOLLING  = -5,
  MX25L3233F_ERROR_MEMORYMAPPED = -6,
  MX25L3233F_ERROR_PARAMETER    = -7,
} MX25L3233F_Error_t;

/* Device ID */
#define MX25L3233F_MANUFACTURER_ID             0xC2
#define MX25L3233F_MEMORY_TYPE                 0x20
#define MX25L3233F_MEMORY_CAPACITY             0x16

/* Device Size, Block, Sector, Page & Secure OTP Size */
#define MX25L3233F_FLASH_SIZE                  ((uint32_t)   32 * 1024 * 1024 / 8)  /* 32 MBits => Bytes */
#define MX25L3233F_BLOCK_64K                   ((uint32_t)  64 * 1024)             /* blocks of 64KBytes */
#define MX25L3233F_BLOCK_32K                   ((uint32_t)  32 * 1024)             /* blocks of 32KBytes */
#define MX25L3233F_SECTOR_4K                   ((uint32_t)   4 * 1024)             /* sectors of 4KBytes */
#define MX25L3233F_PAGE_SIZE                   ((uint32_t) 256)                    /* 256 Bytes per page */
#define MX25L3233F_SECURE_OTP_SIZE             ((uint32_t)   4 * 1024 / 8)         /*!< 4K-bit => Bytes secured OTP */

/* MX25L3233F Configuration */
#define MX25L3233F_DUMMY_CLOCK_READ_SFDP       8

#define MX25L3233F_GENERAL_TIME_OUT            5000     /* 5s time out enogh for general functions */
#define MX25L3233F_WRITE_REG_MAX_TIME          40       /* Write Status Register Cycle Time */
#define MX25L3233F_SECTOR_4K_ERASE_MAX_TIME    200      /* Sector Erase Cycle Time (4KB) */
#define MX25L3233F_BLOCK_32K_ERASE_MAX_TIME    600      /* Block Erase Cycle Time (32KB) */
#define MX25L3233F_BLOCK_64K_ERASE_MAX_TIME    1000     /* Block Erase Cycle Time (64KB) */
#define MX25L3233F_CHIP_ERASE_MAX_TIME         30000    /* Chip Erase Cycle Time */



/* MX25L3233F Commands */

/* Read Operations */
#define MX25L3233F_READ_CMD                             0x03   // READ, Normal Read 3/4 Byte Address; SPI 1-1-1
#define MX25L3233F_FAST_READ_CMD                        0x0B   // FAST READ, Fast Read 3/4 Byte Address; SPI 1-1-1
#define MX25L3233F_2IO_FAST_READ_CMD                    0xBB   // 2READ, 2 x I/O Read 3/4 Byte Address; SPI 1-2-2
#define MX25L3233F_1I2O_FAST_READ_CMD                   0x3B   // DREAD, 1I 2O Read 3/4 Byte Address; SPI 1-1-2
#define MX25L3233F_4IO_FAST_READ_BOTTOM_CMD             0xEB   // 4READ, 4 I/O Read Bottom 3/4 Byte Address; SPI/QPI 1-4-4/4-4-4
#define MX25L3233F_1I4O_FAST_READ_CMD                   0x6B   // QREAD, 1I 4O Read 3/4 Byte Address; SPI 1-1-4

/* Program Operations */
#define MX25L3233F_PAGE_PROG_CMD                        0x02   // PP, Page Program 3/4 Byte Address; SPI/QPI 1-1-1/4-4-4
#define MX25L3233F_QUAD_PAGE_PROG_CMD                   0x38   // 4PP, Quad Page Program 3/4 Byte Address; SPI 1-4-4

/* Erase Operations */
#define MX25L3233F_SECTOR_ERASE_4K_CMD                  0x20   // SE, Sector Erase 4KB 3/4 Byte Address; SPI/QPI 1-1-0/4-4-0
#define MX25L3233F_BLOCK_ERASE_32K_CMD                  0x52   // BE32K, Block Erase 32KB 3/4 Byte Address; SPI/QPI 1-1-0/4-4-0
#define MX25L3233F_BLOCK_ERASE_64K_CMD                  0xD8   // BE, Block Erase 64KB 3/4 Byte Address; SPI/QPI 1-1-0/4-4-0
#define MX25L3233F_CHIP_ERASE_CMD                       0x60   // CE, Chip Erase 0 Byte Address; SPI/QPI 1-0-0/4-0-0
#define MX25L3233F_CHIP_ERASE_CMD1                      0xC7   // CE, Chip Erase 0 Byte Address; SPI/QPI 1-0-0/4-0-0

/***** Register/Setting Commands */
#define MX25L3233F_WRITE_ENABLE_CMD                     0x06   // WREN, Write Enable; SPI/QPI 1-0-0/4-0-0
#define MX25L3233F_WRITE_DISABLE_CMD                    0x04   // WRDI, Write Disable; SPI/QPI 1-0-0/4-0-0

#define MX25L3233F_READ_STATUS_REG_CMD                  0x05   // RDSR, Read Status Register; SPI/QPI 1-0-1/4-0-4
#define MX25L3233F_READ_CFG_REG_CMD                     0x15   // RDCR, Read Configuration Register; SPI/QPI 1-0-1/4-0-4
#define MX25L3233F_WRITE_STATUS_CFG_REG_CMD             0x01   // WRSR, Write Status/Configuration Register; SPI/QPI 1-0-1/4-0-4

#define MX25L3233F_PROG_ERASE_SUSPEND_CMD               0xB0   // PGM/ERS Suspend, Suspends Program/Erase; SPI/QPI 1-0-0/4-0-0
#define MX25L3233F_PROG_ERASE_SUSPEND_CMD1              0x75   // PGM/ERS Suspend, Suspends Program/Erase; SPI/QPI 1-0-0/4-0-0
#define MX25L3233F_PROG_ERASE_RESUME_CMD                0x30   // PGM/ERS Resume, Resumes Program/Erase; SPI/QPI 1-0-0/4-0-0
#define MX25L3233F_PROG_ERASE_RESUME_CMD1               0x7A   // PGM/ERS Resume, Resumes Program/Erase; SPI/QPI 1-0-0/4-0-0

#define MX25L3233F_DEEP_POWER_DOWN_CMD                  0xB9   // DP, Deep power down; SPI/QPI 1-0-0/4-0-0
#define MX25L3233F_RELEASE_DEEP_POWER_DOWN_CMD              0xAB   // RDP, Release from Deep power down; SPI/QPI 1-0-0/4-0-0

#define MX25L3233F_SET_BURST_LENGTH_CMD                 0x77   // SBL, Set burst length; SPI/QPI 1-0-1/4-0-4
#define MX25L3233F_SET_BURST_LENGTH_CMD1                0xC0   // SBL, Set burst length; SPI/QPI 1-0-1/4-0-4

/* ID/Security Commands */
/* Identification Operations */
#define MX25L3233F_READ_ID_CMD                          0x9F   // RDID, Read IDentification; SPI 1-0-1
#define MX25L3233F_READ_ELECTRONIC_ID_CMD               0xAB   // RES, Read Electronic ID; SPI/QPI 1-1-1/4-4-4
#define MX25L3233F_READ_ELECTRONIC_MANFACTURER_ID_CMD   0x90   // REMS, Read Electronic Manufacturer ID & Device ID; SPI 1-1-1

#define MX25L3233F_ENTER_SECURED_OTP_CMD                0xB1   // ENSO, Enter Secured OTP; SPI/QPI 1-0-0/4-0-0
#define MX25L3233F_EXIT_SECURED_OTP_CMD                 0xC1   // EXSO, Exit Secured OTP; SPI/QPI 1-0-0/4-0-0

#define MX25L3233F_READ_SECURITY_REGISTER_CMD           0x2B   // RDSCUR, Read Security Register; SPI/QPI 1-0-1/4-0-4
#define MX25L3233F_WRITE_SECURITY_REGISTER_CMD          0x2F   // WRSCUR, Write Security Register; SPI/QPI 1-0-0/4-0-0

/* Reset Commands */
#define MX25L3233F_NO_OPERATION_CMD                     0x00   // NOP, No Operation; SPI/QPI 1-0-0/4-0-0
#define MX25L3233F_RESET_ENABLE_CMD                     0x66   // RSETEN, Reset Enable; SPI/QPI 1-0-0/4-0-0
#define MX25L3233F_RESET_MEMORY_CMD                     0x99   // RST, Reset Memory; SPI/QPI 1-0-0/4-0-0

/* MX25L3233F Registers */
/* Status Register */
#define MX25L3233F_SR_WIP                      ((uint8_t)0x01)    /*!< Write in progress */
#define MX25L3233F_SR_WEL                      ((uint8_t)0x02)    /*!< Write enable latch */
#define MX25L3233F_SR_BP                       ((uint8_t)0x3C)    /*!< Block protected against program and erase operations */
#define MX25L3233F_SR_QE                       ((uint8_t)0x40)    /*!< QE bit; Quad IO mode enabled if =1 */
#define MX25L3233F_SR_SRWD                     ((uint8_t)0x80)    /*!< Status register write enable/disable */

/* Configuration Register */
#define MX25L3233F_CR_ODS                      ((uint8_t)0x01)    /*!< Output Driver Strenght bit used to configure the output driver level */
#define MX25L3233F_CR_TB                       ((uint8_t)0x08)    /*!< Top/Bottom bit used to configure the block protect area */
#define MX25L3233F_CR_DC                       ((uint8_t)0x40)    /*!< Dummy Clock Cycles setting */

/* Security Register */
#define MX25L3233F_SCUR_OTP_INDICATOR          ((uint8_t)0x01)    /*!< Secured OTP indicator bit */
#define MX25L3233F_SCUR_LDSO                   ((uint8_t)0x02)    /*!< Indicate if Lock-down Secured OTP */
#define MX25L3233F_SCUR_PSB                    ((uint8_t)0x04)    /*!< Program Suspend bit */
#define MX25L3233F_SCUR_ESB                    ((uint8_t)0x08)    /*!< Erase Suspend bit */
#define MX25L3233F_SCUR_P_FAIL                 ((uint8_t)0x20)    /*!< Indicate Program failed */
#define MX25L3233F_SCUR_E_FAIL                 ((uint8_t)0x40)    /*!< Indicate Erase failed */

/* MX25L3233F_Exported_Types */
typedef enum
{
  MX25L3233F_SPI_NORMAL_MODE = 0,         // 1-1-1 read command with 0 dummy cycle
  MX25L3233F_SPI_MODE,                    // 1-1-1 commands, Power on H/W default setting
  MX25L3233F_SPI_2IO_MODE,                // 1-2-2 read command
  MX25L3233F_SPI_1I2O_MODE,               // 1-1-2 read command
  MX25L3233F_SPI_4IO_MODE,                // 1-4-4 read command
  MX25L3233F_SPI_1I4O_MODE,               // 1-1-4 read command
} MX25L3233F_Interface_t;

typedef enum
{
  MX25L3233F_ERASE_CHIP = 0,               // Whole chip erase
  MX25L3233F_ERASE_4K   =  4 * 1024,       // 4K size Sector erase
  MX25L3233F_ERASE_32K  = 32 * 1024,       // 32K size Block erase
  MX25L3233F_ERASE_64K  = 64 * 1024        // 64K size Block erase
} MX25L3233F_Erase_t;

typedef enum
{
  MX25L3233F_BURST_READ_WRAP_8 = 0,
  MX25L3233F_BURST_READ_WRAP_16,
  MX25L3233F_BURST_READ_WRAP_32,
  MX25L3233F_BURST_READ_WRAP_64,
  MX25L3233F_BURST_READ_WRAP_NONE = 0x1F   // Disable wrap function
} MX25L3233F_WrapLength_t;

/* Configuration Register ODS & Dummy clock cycle setting ******************************/
/* MX25L3233F Dummy Clock setting
 *                | DC, DUMMY Clock | Numbers of Dummy Cycles
 *   -------------+-----------------+--------------------------
 *    2READ 1-2-2 |       0         |        4 clocks
 *    0xBB        |       1         |        8 clocks
 *   -------------+-----------------+--------------------------
 *    4READ 1-4-4 |       0         |        6 clocks
 *    0xEB        |       1         |       10 clocks 
 */

/* Function Prototypes ------------------------------------------------------------*/

/**
  * @brief  Polling Write In Progress bit become to 0
  * @param  MX25_Handle_t: MX25 driver handler 
  * @retval SPI memory status
  */
int32_t MX25_AutoPollingMemReady(uint32_t Timeout);

/**
  * @brief  Reads an amount of data from the SPI memory on STR mode.
  * @param  pData: Pointer to data to be read
  * @param  Address: Read start address
  * @param  Size: Size of data to read
  * @retval SPI memory status
  */
int32_t MX25_Read(uint8_t *pData, uint32_t Address, uint32_t Size);

/**
  * @brief  Read Flash 3 Byte IDs.
  * @param  Component object pointer
  * @retval SPI memory status
  */
int32_t MX25_ReadID(uint8_t *ID);

/**
  * @brief  Flash Reset
  * @param    None
  * @retval SPI memory status
  */
int32_t MX25_ResetEnable(void);

/**
  * @brief  Flash Reset memory
  * @param    None
  * @retval SPI memory status
  */
int32_t MX25_ResetMemory(void);



#endif /* __MX25_DRIVER_H */