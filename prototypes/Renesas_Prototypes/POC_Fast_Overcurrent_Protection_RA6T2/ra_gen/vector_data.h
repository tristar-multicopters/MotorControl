/* generated vector header file - do not edit */
        #ifndef VECTOR_DATA_H
        #define VECTOR_DATA_H
                /* Number of interrupts allocated */
        #ifndef VECTOR_DATA_IRQ_COUNT
        #define VECTOR_DATA_IRQ_COUNT    (1)
        #endif
        /* ISR prototypes */
        void poeg_event_isr(void);

        /* Vector table allocations */
        #define VECTOR_NUMBER_POEG3_EVENT ((IRQn_Type) 0) /* POEG3 EVENT (Port Output disable interrupt D) */
        #define POEG3_EVENT_IRQn          ((IRQn_Type) 0) /* POEG3 EVENT (Port Output disable interrupt D) */
        #endif /* VECTOR_DATA_H */