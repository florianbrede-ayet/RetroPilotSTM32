/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */


/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define IN2_P5_VDIV_Pin GPIO_PIN_0
#define IN2_P5_VDIV_GPIO_Port GPIOC
#define IN2_P4_GAS_Pin GPIO_PIN_2
#define IN2_P4_GAS_GPIO_Port GPIOC
#define KEYPAD_BTN_1_Pin GPIO_PIN_0
#define KEYPAD_BTN_1_GPIO_Port GPIOA
#define KEYPAD_BTN_2_Pin GPIO_PIN_1
#define KEYPAD_BTN_2_GPIO_Port GPIOA
#define KEYPAD_BTN_3_Pin GPIO_PIN_2
#define KEYPAD_BTN_3_GPIO_Port GPIOA
#define KEYPAD_BTN_4_Pin GPIO_PIN_3
#define KEYPAD_BTN_4_GPIO_Port GPIOA
#define IN2_P7_Pin GPIO_PIN_4
#define IN2_P7_GPIO_Port GPIOA
#define IN2_P8_Pin GPIO_PIN_5
#define IN2_P8_GPIO_Port GPIOA
#define L2982_ENA_Pin GPIO_PIN_6
#define L2982_ENA_GPIO_Port GPIOA
#define Actuator_1_Poti_Pin GPIO_PIN_7
#define Actuator_1_Poti_GPIO_Port GPIOA
#define IN2_P6_VDIV_Pin GPIO_PIN_4
#define IN2_P6_VDIV_GPIO_Port GPIOC
#define Actuator_2_Poti_Pin GPIO_PIN_0
#define Actuator_2_Poti_GPIO_Port GPIOB
#define L2982_IN2_Pin GPIO_PIN_1
#define L2982_IN2_GPIO_Port GPIOB
#define SW_CLUTCH_Pin GPIO_PIN_12
#define SW_CLUTCH_GPIO_Port GPIOB
#define SW_BRAKE_Pin GPIO_PIN_13
#define SW_BRAKE_GPIO_Port GPIOB
#define SW_GAS_Pin GPIO_PIN_14
#define SW_GAS_GPIO_Port GPIOB
#define L2982_IN4_Pin GPIO_PIN_15
#define L2982_IN4_GPIO_Port GPIOB
#define L2981_ENA_Pin GPIO_PIN_6
#define L2981_ENA_GPIO_Port GPIOC
#define L2981_ENB_Pin GPIO_PIN_7
#define L2981_ENB_GPIO_Port GPIOC
#define IN2_P1_PWM_Pin GPIO_PIN_8
#define IN2_P1_PWM_GPIO_Port GPIOC
#define L2981_IN4_Pin GPIO_PIN_9
#define L2981_IN4_GPIO_Port GPIOC
#define L2982_ENB_Pin GPIO_PIN_8
#define L2982_ENB_GPIO_Port GPIOA
#define L2982_IN1_Pin GPIO_PIN_15
#define L2982_IN1_GPIO_Port GPIOA
#define L2981_IN3_Pin GPIO_PIN_10
#define L2981_IN3_GPIO_Port GPIOC
#define L2981_IN2_Pin GPIO_PIN_11
#define L2981_IN2_GPIO_Port GPIOC
#define L2981_IN1_Pin GPIO_PIN_12
#define L2981_IN1_GPIO_Port GPIOC
#define VSS_INT_Pin GPIO_PIN_4
#define VSS_INT_GPIO_Port GPIOB
#define VSS_INT_EXTI_IRQn EXTI4_IRQn
#define L2982_IN3_Pin GPIO_PIN_9
#define L2982_IN3_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
