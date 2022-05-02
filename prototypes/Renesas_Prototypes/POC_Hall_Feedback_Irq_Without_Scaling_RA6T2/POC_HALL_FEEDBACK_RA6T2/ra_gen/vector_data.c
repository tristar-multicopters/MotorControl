/* generated vector source file - do not edit */
        #include "bsp_api.h"
        /* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
        #if VECTOR_DATA_IRQ_COUNT > 0
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_MAX_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = poeg_event_isr, /* POEG3 EVENT (Port Output disable interrupt D) */
            [1] = adc_b_adi0_isr, /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
            [2] = gpt_counter_overflow_isr, /* GPT9 COUNTER OVERFLOW (Overflow) */
            [3] = gpt_capture_a_isr, /* GPT9 CAPTURE COMPARE A (Compare match A) */
            [4] = r_icu_isr, /* ICU IRQ6 (External pin interrupt 6) */
            [5] = r_icu_isr, /* ICU IRQ7 (External pin interrupt 7) */
            [6] = r_icu_isr, /* ICU IRQ12 (External pin interrupt 12) */
        };
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_MAX_ENTRIES] =
        {
            [0] = BSP_PRV_IELS_ENUM(EVENT_POEG3_EVENT), /* POEG3 EVENT (Port Output disable interrupt D) */
            [1] = BSP_PRV_IELS_ENUM(EVENT_ADC12_ADI0), /* ADC0 ADI0 (End of A/D scanning operation(Gr.0)) */
            [2] = BSP_PRV_IELS_ENUM(EVENT_GPT9_COUNTER_OVERFLOW), /* GPT9 COUNTER OVERFLOW (Overflow) */
            [3] = BSP_PRV_IELS_ENUM(EVENT_GPT9_CAPTURE_COMPARE_A), /* GPT9 CAPTURE COMPARE A (Compare match A) */
            [4] = BSP_PRV_IELS_ENUM(EVENT_ICU_IRQ6), /* ICU IRQ6 (External pin interrupt 6) */
            [5] = BSP_PRV_IELS_ENUM(EVENT_ICU_IRQ7), /* ICU IRQ7 (External pin interrupt 7) */
            [6] = BSP_PRV_IELS_ENUM(EVENT_ICU_IRQ12), /* ICU IRQ12 (External pin interrupt 12) */
        };
        #endif