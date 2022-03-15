/**
 *******************************************************************************
 * @file  startup_RIN32M4.c
 * @brief sample program for R-IN32M4
 * 
 * @note 
 * Copyright (C) 2011-2013 Renesas Electronics Corporation 
 * Copyright (C) 2011-2013 Renesas Micro Systems Co., Ltd. 
 * 
 * @par
 *  This is a sample program. 
 *  Renesas Electronics assumes no responsibility for any losses incurred. 
 * 
 *******************************************************************************
 */

/*============================================================================*/
/* I N C L U D E                                                              */
/*============================================================================*/
#include "RIN32M4.h"
#ifndef OSLESS
#include "kernel.h"
#endif

/*============================================================================*/
/* D E F I N E                                                                */
/*============================================================================*/

/*============================================================================*/
/* E X T E R N                                                                */
/*============================================================================*/
extern uint32_t Image$$ARM_LIB_STACKHEAP$$ZI$$Limit;
extern uint32_t Image$$EXEC_VECT$$RO$$Base;
extern void __main(void);
extern void $Super$$main(void);
#ifndef OSLESS
extern ER hwos_setup(void);
#endif

/*============================================================================*/
/* P R O G R A M                                                              */
/*============================================================================*/
__attribute__ ((section("handlers_rom"))) void NMI_Handler_rom(void)
{
	while(1);
}

__attribute__ ((section("handlers_rom"))) void HardFault_Handler_rom(void)
{
	while(1);
}

//
// Dummy Interrupt Handlers
//
// The following functions are defined weakly to allow the user
// to override them at link time simply by declaring their own
// function of the same name.
//
// If no user function is provided, the weak function is used.
//
__attribute__((weak)) void NMI_Handler(void)            { while(1); }
__attribute__((weak)) void HardFault_Handler(void)      { while(1); }
__attribute__((weak)) void MemManageFault_Handler(void) { while(1); }
__attribute__((weak)) void BusFault_Handler(void)       { while(1); }
__attribute__((weak)) void UsageFault_Handler(void)     { while(1); }
__attribute__((weak)) void SVC_Handler(void)            { while(1); }
__attribute__((weak)) void DebugMon_Handler(void)       { while(1); }
__attribute__((weak)) void PendSV_Handler(void)         { while(1); }
__attribute__((weak)) void SysTick_Handler(void)        { while(1); }

__attribute__((weak)) void TAUJ2I0_IRQHandler(void)     { while(1); }
__attribute__((weak)) void TAUJ2I1_IRQHandler(void)     { while(1); }
__attribute__((weak)) void TAUJ2I2_IRQHandler(void)     { while(1); }
__attribute__((weak)) void TAUJ2I3_IRQHandler(void)     { while(1); }
__attribute__((weak)) void UAJ0TIT_IRQHandler(void)     { while(1); }
__attribute__((weak)) void UAJ0TIR_IRQHandler(void)     { while(1); }
__attribute__((weak)) void UAJ1TIT_IRQHandler(void)     { while(1); }
__attribute__((weak)) void UAJ1TIR_IRQHandler(void)     { while(1); }
__attribute__((weak)) void CSIH0IC_IRQHandler(void)     { while(1); }
__attribute__((weak)) void CSIH0IR_IRQHandler(void)     { while(1); }
__attribute__((weak)) void CSIH0IJC_IRQHandler(void)    { while(1); }
__attribute__((weak)) void CSIH1IC_IRQHandler(void)     { while(1); }
__attribute__((weak)) void CSIH1IR_IRQHandler(void)     { while(1); }
__attribute__((weak)) void CSIH1IJC_IRQHandler(void)    { while(1); }
__attribute__((weak)) void IICB0TIA_IRQHandler(void)    { while(1); }
__attribute__((weak)) void IICB1TIA_IRQHandler(void)    { while(1); }
__attribute__((weak)) void FCN0REC_IRQHandler(void)     { while(1); }
__attribute__((weak)) void FCN0TRX_IRQHandler(void)     { while(1); }
__attribute__((weak)) void FCN0WUP_IRQHandler(void)     { while(1); }
__attribute__((weak)) void FCN1REC_IRQHandler(void)     { while(1); }
__attribute__((weak)) void FCN1TRX_IRQHandler(void)     { while(1); }
__attribute__((weak)) void FCN1WUP_IRQHandler(void)     { while(1); }
__attribute__((weak)) void DMA00_IRQHandler(void)       { while(1); }
__attribute__((weak)) void DMA01_IRQHandler(void)       { while(1); }
__attribute__((weak)) void DMA02_IRQHandler(void)       { while(1); }
__attribute__((weak)) void DMA03_IRQHandler(void)       { while(1); }
__attribute__((weak)) void RTDMA_IRQHandler(void)       { while(1); }
__attribute__((weak)) void TAUDI0_IRQHandler(void)      { while(1); }
__attribute__((weak)) void TAUDI1_IRQHandler(void)      { while(1); }
__attribute__((weak)) void TAUDI2_IRQHandler(void)      { while(1); }
__attribute__((weak)) void TAUDI3_IRQHandler(void)      { while(1); }
__attribute__((weak)) void TAUDI4_IRQHandler(void)      { while(1); }
__attribute__((weak)) void BUFDMA_IRQHandler(void)      { while(1); }
__attribute__((weak)) void ETHPHY0_IRQHandler(void)     { while(1); }
__attribute__((weak)) void ETHPHY1_IRQHandler(void)     { while(1); }
__attribute__((weak)) void ETHMIICMP_IRQHandler(void)   { while(1); }
__attribute__((weak)) void ETHPAUSECMP_IRQHandler(void) { while(1); }
__attribute__((weak)) void ETHTXCMP_IRQHandler(void)    { while(1); }
__attribute__((weak)) void ETHSW_IRQHandler(void)       { while(1); }
__attribute__((weak)) void ETHSWDLR_IRQHandler(void)    { while(1); }
__attribute__((weak)) void ETHSWSYNC_IRQHandler(void)   { while(1); }
__attribute__((weak)) void ETHRXFIFO_IRQHandler(void)   { while(1); }
__attribute__((weak)) void ETHTXFIFO_IRQHandler(void)   { while(1); }
__attribute__((weak)) void ETHRXDMA_IRQHandler(void)    { while(1); }
__attribute__((weak)) void ETHTXDMA_IRQHandler(void)    { while(1); }
__attribute__((weak)) void MACDMARXFRM_IRQHandler(void) { while(1); }
__attribute__((weak)) void INTPZ0_IRQHandler(void)      { while(1); }
__attribute__((weak)) void INTPZ1_IRQHandler(void)      { while(1); }
__attribute__((weak)) void INTPZ2_IRQHandler(void)      { while(1); }
__attribute__((weak)) void INTPZ3_IRQHandler(void)      { while(1); }
__attribute__((weak)) void INTPZ4_IRQHandler(void)      { while(1); }
__attribute__((weak)) void INTPZ5_IRQHandler(void)      { while(1); }
__attribute__((weak)) void INTPZ6_IRQHandler(void)      { while(1); }
__attribute__((weak)) void INTPZ7_IRQHandler(void)      { while(1); }
__attribute__((weak)) void INTPZ8_IRQHandler(void)      { while(1); }
__attribute__((weak)) void INTPZ9_IRQHandler(void)      { while(1); }
__attribute__((weak)) void INTPZ10_IRQHandler(void)     { while(1); }
__attribute__((weak)) void INTPZ11_IRQHandler(void)     { while(1); }
__attribute__((weak)) void INTPZ12_IRQHandler(void)     { while(1); }
__attribute__((weak)) void INTPZ13_IRQHandler(void)     { while(1); }
__attribute__((weak)) void INTPZ14_IRQHandler(void)     { while(1); }
__attribute__((weak)) void INTPZ15_IRQHandler(void)     { while(1); }
__attribute__((weak)) void INTPZ16_IRQHandler(void)     { while(1); }
__attribute__((weak)) void INTPZ17_IRQHandler(void)     { while(1); }
__attribute__((weak)) void INTPZ18_IRQHandler(void)     { while(1); }
__attribute__((weak)) void INTPZ19_IRQHandler(void)     { while(1); }
__attribute__((weak)) void INTPZ20_IRQHandler(void)     { while(1); }
__attribute__((weak)) void INTPZ21_IRQHandler(void)     { while(1); }
__attribute__((weak)) void INTPZ22_IRQHandler(void)     { while(1); }
__attribute__((weak)) void INTPZ23_IRQHandler(void)     { while(1); }
__attribute__((weak)) void INTPZ24_IRQHandler(void)     { while(1); }
__attribute__((weak)) void INTPZ25_IRQHandler(void)     { while(1); }
__attribute__((weak)) void INTPZ26_IRQHandler(void)     { while(1); }
__attribute__((weak)) void INTPZ27_IRQHandler(void)     { while(1); }
__attribute__((weak)) void INTPZ28_IRQHandler(void)     { while(1); }
__attribute__((weak)) void HWRTOS_IRQHandler(void)      { while(1); }
__attribute__((weak)) void BRAMERR_IRQHandler(void)     { while(1); }
__attribute__((weak)) void IICB0TIS_IRQHandler(void)    { while(1); }
__attribute__((weak)) void IICB1TIS_IRQHandler(void)    { while(1); }
__attribute__((weak)) void WDTA_IRQHandler(void)        { while(1); }
__attribute__((weak)) void SFLASH_IRQHandler(void)      { while(1); }
__attribute__((weak)) void UAJ0TIS_IRQHandler(void)     { while(1); }
__attribute__((weak)) void UAJ1TIS_IRQHandler(void)     { while(1); }
__attribute__((weak)) void CSIH0IRE_IRQHandler(void)    { while(1); }
__attribute__((weak)) void CSIH1IRE_IRQHandler(void)    { while(1); }
__attribute__((weak)) void FCN0ERR_IRQHandler(void)     { while(1); }
__attribute__((weak)) void FCN1ERR_IRQHandler(void)     { while(1); }
__attribute__((weak)) void DERR0_IRQHandler(void)       { while(1); }
__attribute__((weak)) void DERR1_IRQHandler(void)       { while(1); }
__attribute__((weak)) void ETHTXFIFOERR_IRQHandler(void){ while(1); }
__attribute__((weak)) void ETHRXERR_IRQHandler(void)    { while(1); }
__attribute__((weak)) void ETHRXDERR_IRQHandler(void)   { while(1); }
__attribute__((weak)) void ETHTXDERR_IRQHandler(void)   { while(1); }
__attribute__((weak)) void BUFDMAERR_IRQHandler(void)   { while(1); }
__attribute__((weak)) void LED0PHY0_IRQHandler(void)    { while(1); }
__attribute__((weak)) void LED0PHY1_IRQHandler(void)    { while(1); }
__attribute__((weak)) void CSINMIZ_IRQHandler(void)     { while(1); }
__attribute__((weak)) void CSIWDTZ_IRQHandler(void)     { while(1); }
__attribute__((weak)) void CSIINTZ_IRQHandler(void)     { while(1); }
__attribute__((weak)) void CSICLKLOSSZ_IRQHandler(void) { while(1); }
__attribute__((weak)) void CCSIRZ_IRQHandler(void)      { while(1); }
__attribute__((weak)) void CCSREFSTB_IRQHandler(void)   { while(1); }
__attribute__((weak)) void CCSMON3_IRQHandler(void)     { while(1); }
__attribute__((weak)) void GBEPHYFLF_IRQHandler(void)   { while(1); }
__attribute__((weak)) void LED1PHY0_IRQHandler(void)    { while(1); }
__attribute__((weak)) void LED1PHY1_IRQHandler(void)    { while(1); }
__attribute__((weak)) void LED2PHY0_IRQHandler(void)    { while(1); }
__attribute__((weak)) void LED2PHY1_IRQHandler(void)    { while(1); }
__attribute__((weak)) void FPU_IRQHandler(void)         { while(1); }
__attribute__((weak)) void ADC_IRQHandler(void)         { while(1); }

/**
 ******************************************************************************
  @brief  Reset Handler
  @param  none
  @retval none
 ******************************************************************************
*/
__attribute__ ((section("handlers_rom")))
void Reset_Handler(void)
{
  SystemInit();
  __main();
}

/**
 ******************************************************************************
  @brief  Sub main
  @param  none
  @retval none
 ******************************************************************************
*/
__attribute__ ((section("handlers_rom")))
void $Sub$$main(void)
{
	//--------------------------
	// Replace vectors address
	//--------------------------
	SCB->VTOR = (uint32_t)&Image$$EXEC_VECT$$RO$$Base;

	//--------------------------
	// Main program
	//--------------------------
	$Super$$main();
}

/**
 ******************************************************************************
  @brief  Main program (dummy)
  @param  none
  @retval 0
 ******************************************************************************
*/
#ifndef OSLESS
__attribute__ ((section("handlers_rom")))
int main(void)
{
	//--------------------------
	// Setup HWRTOS
	//--------------------------
	hwos_setup();

	return 0;
}
#endif	// OSLESS


/*============================================================================*/
/* Vector Table                                                               */
/*============================================================================*/
typedef void (*const vect_t)(void);

__attribute__ ((section("vectors_rom")))
void (* const vectors_rom_tbl[])(void) = {
	(vect_t)&Image$$ARM_LIB_STACKHEAP$$ZI$$Limit,	// Top of Stack - Allowing 4 words for DEBUGDRIVERDATA
	Reset_Handler,									// Reset Handler
	NMI_Handler_rom,								// NMI Handler
	HardFault_Handler_rom,							// Hard Fault Handler
};

__attribute__ ((section("vectors")))
void (* const vectors_tbl[])(void) = {
	(vect_t)&Image$$ARM_LIB_STACKHEAP$$ZI$$Limit,	// Top of Stack - Allowing 4 words for DEBUGDRIVERDATA
	Reset_Handler,									// Reset Handler
	NMI_Handler,									// NMI Handler
	HardFault_Handler,								// Hard Fault Handler
	MemManageFault_Handler,							// MemManage fault Handler
	BusFault_Handler,								// BusFault Handler
	UsageFault_Handler,								// Usage Fault Handler
	0,												// Reserved
	0,												// Reserved
	0,												// Reserved
	0,												// Reserved
	SVC_Handler,									// SVCall Handler
	DebugMon_Handler,								// Debug Monitor Handler
	0,												// Reserved
	PendSV_Handler,									// PendSV Handler
	SysTick_Handler,								// SysTick Handler

	// External Interrupts 1 - 240
	// These are essentially unused, so will all
	// take the same default handler if invoked.
	TAUJ2I0_IRQHandler,
	TAUJ2I1_IRQHandler,
	TAUJ2I2_IRQHandler,
	TAUJ2I3_IRQHandler,
	UAJ0TIT_IRQHandler,
	UAJ0TIR_IRQHandler,
	UAJ1TIT_IRQHandler,
	UAJ1TIR_IRQHandler,
	CSIH0IC_IRQHandler,
	CSIH0IR_IRQHandler,
	CSIH0IJC_IRQHandler,
	CSIH1IC_IRQHandler,
	CSIH1IR_IRQHandler,
	CSIH1IJC_IRQHandler,
	IICB0TIA_IRQHandler,
	IICB1TIA_IRQHandler,
	FCN0REC_IRQHandler,
	FCN0TRX_IRQHandler,
	FCN0WUP_IRQHandler,
	FCN1REC_IRQHandler,
	FCN1TRX_IRQHandler,
	FCN1WUP_IRQHandler,
	DMA00_IRQHandler,
	DMA01_IRQHandler,
	DMA02_IRQHandler,
	DMA03_IRQHandler,
	RTDMA_IRQHandler,
	TAUDI0_IRQHandler,
	TAUDI1_IRQHandler,
	TAUDI2_IRQHandler,
	TAUDI3_IRQHandler,
	TAUDI4_IRQHandler,
	BUFDMA_IRQHandler,
	ETHPHY0_IRQHandler,
	ETHPHY1_IRQHandler,
	ETHMIICMP_IRQHandler,
	ETHPAUSECMP_IRQHandler,
	ETHTXCMP_IRQHandler,
	ETHSW_IRQHandler,
	ETHSWDLR_IRQHandler,
	ETHSWSYNC_IRQHandler,
	ETHRXFIFO_IRQHandler,
	ETHTXFIFO_IRQHandler,
	ETHRXDMA_IRQHandler,
	ETHTXDMA_IRQHandler,
	MACDMARXFRM_IRQHandler,
	0,
	INTPZ0_IRQHandler,
	INTPZ1_IRQHandler,
	INTPZ2_IRQHandler,
	INTPZ3_IRQHandler,
	INTPZ4_IRQHandler,
	INTPZ5_IRQHandler,
	INTPZ6_IRQHandler,
	INTPZ7_IRQHandler,
	INTPZ8_IRQHandler,
	INTPZ9_IRQHandler,
	INTPZ10_IRQHandler,
	INTPZ11_IRQHandler,
	INTPZ12_IRQHandler,
	INTPZ13_IRQHandler,
	INTPZ14_IRQHandler,
	INTPZ15_IRQHandler,
	INTPZ16_IRQHandler,
	INTPZ17_IRQHandler,
	INTPZ18_IRQHandler,
	INTPZ19_IRQHandler,
	INTPZ20_IRQHandler,
	INTPZ21_IRQHandler,
	INTPZ22_IRQHandler,
	INTPZ23_IRQHandler,
	INTPZ24_IRQHandler,
	INTPZ25_IRQHandler,
	INTPZ26_IRQHandler,
	INTPZ27_IRQHandler,
	INTPZ28_IRQHandler,
	HWRTOS_IRQHandler,
	BRAMERR_IRQHandler,
	IICB0TIS_IRQHandler,
	IICB1TIS_IRQHandler,
	WDTA_IRQHandler,
	SFLASH_IRQHandler,
	UAJ0TIS_IRQHandler,
	UAJ1TIS_IRQHandler,
	CSIH0IRE_IRQHandler,
	CSIH1IRE_IRQHandler,
	FCN0ERR_IRQHandler,
	FCN1ERR_IRQHandler,
	DERR0_IRQHandler,
	DERR1_IRQHandler,
	ETHTXFIFOERR_IRQHandler,
	ETHRXERR_IRQHandler,
	ETHRXDERR_IRQHandler,
	ETHTXDERR_IRQHandler,
	BUFDMAERR_IRQHandler,
	LED0PHY0_IRQHandler,
	LED0PHY1_IRQHandler,
	0,												// Reserved 113
	0,												// Reserved 114
	0,												// Reserved 115
	0,												// Reserved 116
	0,												// Reserved 117
	0,												// Reserved 118
	0,												// Reserved 119
	0,												// Reserved 120
	0,												// Reserved 121
	0,												// Reserved 122
	CSINMIZ_IRQHandler,
	CSIWDTZ_IRQHandler,
	CSIINTZ_IRQHandler,
	CSICLKLOSSZ_IRQHandler,
	0,												// Reserved 127
	0,												// Reserved 128
	0,												// Reserved 129
	0,												// Reserved 130
	0,												// Reserved 131
	CCSIRZ_IRQHandler,
	CCSREFSTB_IRQHandler,
	CCSMON3_IRQHandler,
	0,												// Reserved 135
	0,												// Reserved 136
	GBEPHYFLF_IRQHandler,
	LED1PHY0_IRQHandler,
	LED1PHY1_IRQHandler,
	LED2PHY0_IRQHandler,
	LED2PHY1_IRQHandler,
	FPU_IRQHandler,
	ADC_IRQHandler
	
};
