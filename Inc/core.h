#ifndef _CORE_H_
#define _CORE_H_

#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_hal_rcc.h"

#define DEBUG 1

#define MODULE_RETROPILOT_ECU


#define LOW 0
#define HIGH 1

#define COMPILE_TIME_ASSERT(pred) ((void)sizeof(char[1 - (2 * ((int)(!(pred))))]))

#define MIN(a,b) \
 ({ __typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
   (_a < _b) ? _a : _b; })

#define MAX(a,b) \
 ({ __typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
   (_a > _b) ? _a : _b; })

#define ABS(a) \
 ({ __typeof__ (a) _a = (a); \
   (_a > 0) ? _a : (-_a); })

void safe_delay(uint64_t a);
void micros_init();
uint32_t millis();
uint32_t micros();

void digitalWrite(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t state);
uint8_t digitalRead(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);
int32_t adcRead(uint16_t GPIO_Pin);

#endif
