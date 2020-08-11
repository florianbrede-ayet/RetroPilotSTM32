#include "critical.h"


// ********************* Critical section helpers *********************
volatile bool interrupts_enabled = true;

void enable_interrupts(void) {
  interrupts_enabled = true;
  __enable_irq();
}

void disable_interrupts(void) {
  interrupts_enabled = false;
  __disable_irq();
}

uint8_t global_critical_depth = 0U;
