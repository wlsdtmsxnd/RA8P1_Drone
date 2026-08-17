/* generated vector header file - do not edit */
#ifndef VECTOR_DATA_H
#define VECTOR_DATA_H
#ifdef __cplusplus
        extern "C" {
        #endif
/* Number of interrupts allocated */
#ifndef VECTOR_DATA_IRQ_COUNT
#define VECTOR_DATA_IRQ_COUNT    (20)
#endif
/* ISR prototypes */
void sci_b_uart_rxi_isr(void);
void sci_b_uart_txi_isr(void);
void sci_b_uart_tei_isr(void);
void sci_b_uart_eri_isr(void);
void spi_b_rxi_isr(void);
void spi_b_txi_isr(void);
void spi_b_tei_isr(void);
void spi_b_eri_isr(void);

/* Vector table allocations */
#define VECTOR_NUMBER_SCI9_RXI ((IRQn_Type) 0) /* SCI9 RXI (Receive data full) */
#define SCI9_RXI_IRQn          ((IRQn_Type) 0) /* SCI9 RXI (Receive data full) */
#define VECTOR_NUMBER_SCI9_TXI ((IRQn_Type) 1) /* SCI9 TXI (Transmit data empty) */
#define SCI9_TXI_IRQn          ((IRQn_Type) 1) /* SCI9 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI9_TEI ((IRQn_Type) 2) /* SCI9 TEI (Transmit end) */
#define SCI9_TEI_IRQn          ((IRQn_Type) 2) /* SCI9 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI9_ERI ((IRQn_Type) 3) /* SCI9 ERI (Receive error) */
#define SCI9_ERI_IRQn          ((IRQn_Type) 3) /* SCI9 ERI (Receive error) */
#define VECTOR_NUMBER_SPI1_RXI ((IRQn_Type) 4) /* SPI1 RXI (Receive buffer full) */
#define SPI1_RXI_IRQn          ((IRQn_Type) 4) /* SPI1 RXI (Receive buffer full) */
#define VECTOR_NUMBER_SPI1_TXI ((IRQn_Type) 5) /* SPI1 TXI (Transmit buffer empty) */
#define SPI1_TXI_IRQn          ((IRQn_Type) 5) /* SPI1 TXI (Transmit buffer empty) */
#define VECTOR_NUMBER_SPI1_TEI ((IRQn_Type) 6) /* SPI1 TEI (Transmission complete event) */
#define SPI1_TEI_IRQn          ((IRQn_Type) 6) /* SPI1 TEI (Transmission complete event) */
#define VECTOR_NUMBER_SPI1_ERI ((IRQn_Type) 7) /* SPI1 ERI (Error) */
#define SPI1_ERI_IRQn          ((IRQn_Type) 7) /* SPI1 ERI (Error) */
#define VECTOR_NUMBER_SCI2_RXI ((IRQn_Type) 8) /* SCI2 RXI (Receive data full) */
#define SCI2_RXI_IRQn          ((IRQn_Type) 8) /* SCI2 RXI (Receive data full) */
#define VECTOR_NUMBER_SCI2_TXI ((IRQn_Type) 9) /* SCI2 TXI (Transmit data empty) */
#define SCI2_TXI_IRQn          ((IRQn_Type) 9) /* SCI2 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI2_TEI ((IRQn_Type) 10) /* SCI2 TEI (Transmit end) */
#define SCI2_TEI_IRQn          ((IRQn_Type) 10) /* SCI2 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI2_ERI ((IRQn_Type) 11) /* SCI2 ERI (Receive error) */
#define SCI2_ERI_IRQn          ((IRQn_Type) 11) /* SCI2 ERI (Receive error) */
#define VECTOR_NUMBER_SCI0_RXI ((IRQn_Type) 12) /* SCI0 RXI (Receive data full) */
#define SCI0_RXI_IRQn          ((IRQn_Type) 12) /* SCI0 RXI (Receive data full) */
#define VECTOR_NUMBER_SCI0_TXI ((IRQn_Type) 13) /* SCI0 TXI (Transmit data empty) */
#define SCI0_TXI_IRQn          ((IRQn_Type) 13) /* SCI0 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI0_TEI ((IRQn_Type) 14) /* SCI0 TEI (Transmit end) */
#define SCI0_TEI_IRQn          ((IRQn_Type) 14) /* SCI0 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI0_ERI ((IRQn_Type) 15) /* SCI0 ERI (Receive error) */
#define SCI0_ERI_IRQn          ((IRQn_Type) 15) /* SCI0 ERI (Receive error) */
#define VECTOR_NUMBER_SCI4_RXI ((IRQn_Type) 16) /* SCI4 RXI (Receive data full) */
#define SCI4_RXI_IRQn          ((IRQn_Type) 16) /* SCI4 RXI (Receive data full) */
#define VECTOR_NUMBER_SCI4_TXI ((IRQn_Type) 17) /* SCI4 TXI (Transmit data empty) */
#define SCI4_TXI_IRQn          ((IRQn_Type) 17) /* SCI4 TXI (Transmit data empty) */
#define VECTOR_NUMBER_SCI4_TEI ((IRQn_Type) 18) /* SCI4 TEI (Transmit end) */
#define SCI4_TEI_IRQn          ((IRQn_Type) 18) /* SCI4 TEI (Transmit end) */
#define VECTOR_NUMBER_SCI4_ERI ((IRQn_Type) 19) /* SCI4 ERI (Receive error) */
#define SCI4_ERI_IRQn          ((IRQn_Type) 19) /* SCI4 ERI (Receive error) */
/* The number of entries required for the ICU vector table. */
#define BSP_ICU_VECTOR_NUM_ENTRIES (20)

#ifdef __cplusplus
        }
        #endif
#endif /* VECTOR_DATA_H */
