/* generated vector source file - do not edit */
        #include "bsp_api.h"
        /* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
        #if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_MAX_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = poeg_event_isr, /* POEG3 EVENT (Port Output disable interrupt D) */
            [1] = adc_b_adi0_isr, /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
        };
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_MAX_ENTRIES] =
        {
            [0] = BSP_PRV_IELS_ENUM(EVENT_POEG3_EVENT), /* POEG3 EVENT (Port Output disable interrupt D) */
            [1] = BSP_PRV_IELS_ENUM(EVENT_ADC12_ADI0), /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
        };
        #endif