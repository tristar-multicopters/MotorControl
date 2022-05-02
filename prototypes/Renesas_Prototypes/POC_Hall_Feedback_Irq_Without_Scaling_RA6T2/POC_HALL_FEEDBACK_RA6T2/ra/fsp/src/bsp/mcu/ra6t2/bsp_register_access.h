/*******************************************************************************************************************//**
 * @defgroup BSP_IO BSP I/O access
 * @ingroup RENESAS_COMMON
 * @brief This module provides basic read/write access to port pins.
 *
 * @{
 **********************************************************************************************************************/

#ifndef BSP_REGISTER_ACCESS_H
#define BSP_REGISTER_ACCESS_H

/* Common macro for FSP header files. There is also a corresponding FSP_FOOTER macro at the end of this file. */

/***********************************************************************************************************************
 * Exported global functions (to be accessed by other files)
 **********************************************************************************************************************/

/*******************************************************************************************************************//**
 * Read the current input level of the pin.
 *
 * @param[in]  pin             The pin
 *
 * @retval     Current input level
 **********************************************************************************************************************/
__STATIC_INLINE uint8_t R_BSP_GetTimerPrescaler (timer_ctrl_t * const p_ctrl)
{
		gpt_instance_ctrl_t * p_instance_ctrl = (gpt_instance_ctrl_t *) p_ctrl;
		return (uint8_t) p_instance_ctrl->p_reg->GTCR_b.TPCS;
}

/*******************************************************************************************************************//**
 * Set a pin to output and set the output level to the level provided. If PFS protection is enabled, disable PFS
 * protection using R_BSP_PinAccessEnable() before calling this function.
 *
 * @param[in]  pin      The pin
 * @param[in]  level    The level
 **********************************************************************************************************************/

__STATIC_INLINE void BSP_SetTimerPrecaler (timer_ctrl_t* const p_ctrl, timer_source_div_t Precalser)
{
		if(Precalser == )





}	

//__STATIC_INLINE void R_BSP_PinWrite (bsp_io_port_pin_t pin, bsp_io_level_t level)
//{
//    /* Clear PMR, ASEL, ISEL and PODR bits. */
//   // uint32_t pfs_bits = R_PFS->PORT[pin >> 8].PIN[pin & BSP_IO_PRV_8BIT_MASK].PmnPFS;
//  //  pfs_bits &= BSP_IO_PRV_PIN_WRITE_MASK;

//    /* Set output level and pin direction to output. */
// // uint32_t lvl = ((uint32_t) level | pfs_bits);
//  //  R_PFS->PORT[pin >> 8].PIN[pin & BSP_IO_PRV_8BIT_MASK].PmnPFS = (BSP_IO_PFS_PDR_OUTPUT | lvl);
//}










/** @} (end addtogroup BSP_IO) */

/* Common macro for FSP header files. There is also a corresponding FSP_HEADER macro at the top of this file. */


#endif