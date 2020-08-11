#include "uart.h"

// IRQs: USART1, USART3

// ******************************** UART buffers ********************************

// external serial = USART3
UART_BUFFER(external, FIFO_SIZE_INT, FIFO_SIZE_INT, USART3, NULL)

// debug = USART1
UART_BUFFER(debug, FIFO_SIZE_INT, FIFO_SIZE_INT, USART1, NULL)

uart_ring *serial_get_ring_by_number(int a) {
  uart_ring *ring = NULL;
  switch(a) {
    case 1:
      ring = &uart_ring_debug;
      break;
    case 3:
      ring = &uart_ring_external;
      break;
    default:
      ring = NULL;
      break;
  }
  return ring;
}


// ***************************** Interrupt handlers *****************************

void uart_tx_ring(uart_ring *q){
  ENTER_CRITICAL();
  // Send out next byte of TX buffer
  if (q->w_ptr_tx != q->r_ptr_tx) {
    // Only send if transmit register is empty (aka last byte has been sent)
    if ((q->uart->SR & USART_SR_TXE) != 0) {
      q->uart->DR = q->elems_tx[q->r_ptr_tx];   // This clears TXE
      q->r_ptr_tx = (q->r_ptr_tx + 1U) % q->tx_fifo_size;
    }


    // Enable TXE interrupt if there is still data to be sent
    if(q->r_ptr_tx != q->w_ptr_tx){
      q->uart->CR1 |= USART_CR1_TXEIE;
    } else {
      q->uart->CR1 &= ~USART_CR1_TXEIE;
    }
  }
  EXIT_CRITICAL();
}

void uart_rx_ring(uart_ring *q){
  // Read out RX buffer
  uint8_t c = q->uart->DR;  // This read after reading SR clears a bunch of interrupts

  uint16_t next_w_ptr = (q->w_ptr_rx + 1U) % q->rx_fifo_size;
  // Do not overwrite buffer data
  if (next_w_ptr != q->r_ptr_rx) {
    q->elems_rx[q->w_ptr_rx] = c;
    q->w_ptr_rx = next_w_ptr;
    q->bytes_avail++;
    if (q->callback != NULL) {
      q->callback(q);
    }
  }

}

uint32_t serial_get_avail(uart_ring *q) {
  return q->bytes_avail;
}


// This read after reading SR clears all error interrupts. We don't want compiler warnings, nor optimizations
#define UART_READ_DR(uart) volatile uint8_t t = (uart)->DR; UNUSED(t);

void uart_interrupt_handler(uart_ring *q) {

  // Read UART status. This is also the first step necessary in clearing most interrupts
  uint32_t status = q->uart->SR;

  // If RXNE is set, perform a read. This clears RXNE, ORE, IDLE, NF and FE
  if((status & USART_SR_RXNE) != 0U){
    uart_rx_ring(q);
  }

  // Detect errors and clear them
  uint32_t err = (status & USART_SR_ORE) | (status & USART_SR_NE) | (status & USART_SR_FE) | (status & USART_SR_PE);
  if(err != 0U){
    #ifdef DEBUG_UART
      puts("Encountered UART error: "); puth(err); puts("\n");
    #endif
    UART_READ_DR(q->uart)
  }
  // Send if necessary
  uart_tx_ring(q);

}

void USART1_IRQ_Handler(void) { uart_interrupt_handler(&uart_ring_debug); }
void USART3_IRQ_Handler(void) { uart_interrupt_handler(&uart_ring_external); }


// ***************************** Hardware setup *****************************


// ************************* Low-level buffer functions *************************

bool serial_getc(uart_ring *q, char *elem) {
  bool ret = false;

  ENTER_CRITICAL();
  if (q->w_ptr_rx != q->r_ptr_rx) {
    if (elem != NULL) *elem = q->elems_rx[q->r_ptr_rx];
    q->r_ptr_rx = (q->r_ptr_rx + 1U) % q->rx_fifo_size;
    q->bytes_avail--;
    ret = true;
  }
  EXIT_CRITICAL();

  return ret;
}

bool injectc(uart_ring *q, char elem) {
  int ret = false;
  uint16_t next_w_ptr;

  ENTER_CRITICAL();
  next_w_ptr = (q->w_ptr_rx + 1U) % q->tx_fifo_size;
  if (next_w_ptr != q->r_ptr_rx) {
    q->elems_rx[q->w_ptr_rx] = elem;
    q->w_ptr_rx = next_w_ptr;
    q->bytes_avail++;
    ret = true;
  }
  EXIT_CRITICAL();

  return ret;
}

bool serial_putc(uart_ring *q, char elem) {
  bool ret = false;
  uint16_t next_w_ptr;

  ENTER_CRITICAL();
  next_w_ptr = (q->w_ptr_tx + 1U) % q->tx_fifo_size;
  if (next_w_ptr != q->r_ptr_tx) {
    q->elems_tx[q->w_ptr_tx] = elem;
    q->w_ptr_tx = next_w_ptr;
    ret = true;
  }
  EXIT_CRITICAL();

  uart_tx_ring(q);

  return ret;
}


void uart_flush_sync(uart_ring *q) {
  // empty the TX buffer
  while (q->w_ptr_tx != q->r_ptr_tx) {
    uart_tx_ring(q);
  }
}


void serial_external_putch(const char a) {
    (void)injectc(&uart_ring_debug, a);

}

// ************************ High-level debug functions **********************
void serial_debug_putch(const char a) {
    // assuming debugging is important if there's external serial connected
    while (!serial_putc(&uart_ring_debug, a));
    //(void)injectc(&uart_ring_debug, a);
}

void serial_debug_puts(const char *a) {
  for (const char *in = a; *in; in++) {
    if (*in == '\n') serial_debug_putch('\r');
    serial_debug_putch(*in);
  }
}

void serial_debug_putui(uint32_t i) {
  uint32_t i_copy = i;
  char str[11];
  uint8_t idx = 10;
  str[idx] = '\0';
  idx--;
  do {
    str[idx] = (i_copy % 10U) + 0x30U;
    idx--;
    i_copy /= 10;
  } while (i_copy != 0U);
  serial_debug_puts(&str[idx + 1U]);
}

void serial_debug_puth(unsigned int i) {
  char c[] = "0123456789abcdef";
  for (int pos = 28; pos != -4; pos -= 4) {
    serial_debug_putch(c[(i >> (unsigned int)(pos)) & 0xFU]);
  }
}

void serial_debug_puth2(unsigned int i) {
  char c[] = "0123456789abcdef";
  for (int pos = 4; pos != -4; pos -= 4) {
    serial_debug_putch(c[(i >> (unsigned int)(pos)) & 0xFU]);
  }
}

void serial_debug_hexdump(const void *a, int l) {
  if (a != NULL) {
    for (int i=0; i < l; i++) {
      if ((i != 0) && ((i & 0xf) == 0)) serial_debug_puts("\n");
      serial_debug_puth2(((const unsigned char*)a)[i]);
      serial_debug_puts(" ");
    }
  }
  serial_debug_puts("\n");
}
