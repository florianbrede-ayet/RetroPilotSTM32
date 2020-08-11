
#ifndef _UART_H_
#define _UART_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "core.h"
#include "critical.h"


// IRQs: USART1, USART3

// ***************************** Definitions *****************************
#define FIFO_SIZE_INT 0x400U

typedef struct uart_ring {
  volatile int bytes_avail;

  volatile uint16_t w_ptr_tx;
  volatile uint16_t r_ptr_tx;
  uint8_t *elems_tx;
  uint32_t tx_fifo_size;
  volatile uint16_t w_ptr_rx;
  volatile uint16_t r_ptr_rx;
  uint8_t *elems_rx;
  uint32_t rx_fifo_size;
  USART_TypeDef *uart;
  void (*callback)(struct uart_ring*);
} uart_ring;

#define UART_BUFFER(x, size_rx, size_tx, uart_ptr, callback_ptr) \
  uint8_t elems_rx_##x[size_rx]; \
  uint8_t elems_tx_##x[size_tx]; \
  uart_ring uart_ring_##x = {  \
    .bytes_avail = 0, \
    .w_ptr_tx = 0, \
    .r_ptr_tx = 0, \
    .elems_tx = ((uint8_t *)&elems_tx_##x), \
    .tx_fifo_size = size_tx, \
    .w_ptr_rx = 0, \
    .r_ptr_rx = 0, \
    .elems_rx = ((uint8_t *)&elems_rx_##x), \
    .rx_fifo_size = size_rx, \
    .uart = uart_ptr, \
    .callback = callback_ptr \
  };


// ***************************** Function prototypes *****************************

void USART1_IRQ_Handler(void);
void USART3_IRQ_Handler(void);


uint32_t serial_get_avail(uart_ring *q);

void serial_external_putch(const char a);

bool serial_getc(uart_ring *q, char *elem);
bool serial_putc(uart_ring *q, char elem);

void serial_debug_putui(uint32_t i);
void serial_debug_puts(const char *a);
void serial_debug_puth(unsigned int i);
void serial_debug_hexdump(const void *a, int l);

// ******************************** UART buffers ********************************

uart_ring *serial_get_ring_by_number(int a);

#ifdef __cplusplus
}
#endif

#endif