/* generated vector header file - do not edit */
        #ifndef VECTOR_DATA_H
        #define VECTOR_DATA_H
        #ifdef __cplusplus
        extern "C" {
        #endif
                /* Number of interrupts allocated */
        #ifndef VECTOR_DATA_IRQ_COUNT
        #define VECTOR_DATA_IRQ_COUNT    (3)
        #endif
        /* ISR prototypes */
        void gpt_counter_overflow_isr(void);
        void gpt_capture_a_isr(void);
        void r_icu_isr(void);

        /* Vector table allocations */
        #define VECTOR_NUMBER_GPT1_COUNTER_OVERFLOW ((IRQn_Type) 0) /* GPT1 COUNTER OVERFLOW (Overflow) */
        #define GPT1_COUNTER_OVERFLOW_IRQn          ((IRQn_Type) 0) /* GPT1 COUNTER OVERFLOW (Overflow) */
        #define VECTOR_NUMBER_GPT1_CAPTURE_COMPARE_A ((IRQn_Type) 1) /* GPT1 CAPTURE COMPARE A (Compare match A) */
        #define GPT1_CAPTURE_COMPARE_A_IRQn          ((IRQn_Type) 1) /* GPT1 CAPTURE COMPARE A (Compare match A) */
        #define VECTOR_NUMBER_ICU_IRQ10 ((IRQn_Type) 2) /* ICU IRQ10 (External pin interrupt 10) */
        #define ICU_IRQ10_IRQn          ((IRQn_Type) 2) /* ICU IRQ10 (External pin interrupt 10) */
        #ifdef __cplusplus
        }
        #endif
        #endif /* VECTOR_DATA_H */