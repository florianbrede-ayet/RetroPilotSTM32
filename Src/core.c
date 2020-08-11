#include "main.h"
#include "core.h"
#include "stm32f4xx_hal_gpio.h"


extern ADC_HandleTypeDef hadc1;


void safe_delay(uint64_t a) {
  volatile uint64_t i;
  a=a*160000/80; // 160000 cycles per 1ms delay (one loop seems to take 80 ticks in -O0?)
  for (i = 0; i < a; i++);  
}


uint32_t millis() {
  return HAL_GetTick();
}


uint32_t usTicks;

void micros_init(void)
{
    usTicks = HAL_RCC_GetSysClockFreq() / 1000000;
    SysTick_Config(HAL_RCC_GetHCLKFreq() / 1000); //SysTick turns on the system tick timer and initializes its interrupt, 1ms
}

 //return us (rollover in ~70 minutes)
uint32_t micros(void)
{
    uint32_t ms, cycle_cnt, usTicks_t;
    do {
        ms = HAL_GetTick();
        cycle_cnt = SysTick->VAL;
    } while (ms != HAL_GetTick());
    usTicks_t = usTicks;
    return (ms * 1000) + (usTicks_t * 1000 - cycle_cnt) / usTicks_t;;
}


void digitalWrite(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, uint8_t state) {
  HAL_GPIO_WritePin(GPIOx, GPIO_Pin, (state==0 ? GPIO_PIN_RESET : GPIO_PIN_SET));  
}

uint8_t digitalRead(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin) {
  return HAL_GPIO_ReadPin(GPIOx, GPIO_Pin)==GPIO_PIN_SET ? 1 : 0; 
}

int32_t adcRead(uint16_t GPIO_Pin) {
  if (GPIO_Pin==Actuator_1_Poti_Pin) {
    ADC_ChannelConfTypeDef sConfig;
    sConfig.Channel      = ADC_CHANNEL_7;
    sConfig.Rank         = 1;         
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    sConfig.Offset = 0;   

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
      return -1;
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
      return -1;
    }
    if (HAL_ADC_PollForConversion(&hadc1, 1) != HAL_OK)
    {
      return -1;
    }
    else
    {
      return HAL_ADC_GetValue(&hadc1);
    }
  }
  else if (GPIO_Pin==Actuator_2_Poti_Pin) {
    ADC_ChannelConfTypeDef sConfig;
    sConfig.Channel      = ADC_CHANNEL_8;
    sConfig.Rank         = 1;         
    sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
    sConfig.Offset = 0;  

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
    {
      return -1;
    }

    if (HAL_ADC_Start(&hadc1) != HAL_OK)
    {
      return -1;
    }
    if (HAL_ADC_PollForConversion(&hadc1, 1) != HAL_OK)
    {
      return -1;
    }
    else
    {
      return HAL_ADC_GetValue(&hadc1);
    }
  }

  return -1;
}

