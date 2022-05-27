/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#define KEY_Pin GPIO_PIN_0
#define KEY_GPIO_Port GPIOC
#define KEY_EXTI_IRQn EXTI0_IRQn
#define LCD_WR_Pin GPIO_PIN_2
#define LCD_WR_GPIO_Port GPIOC
#define LCD_RD_Pin GPIO_PIN_3
#define LCD_RD_GPIO_Port GPIOC
#define LCD_CS_Pin GPIO_PIN_4
#define LCD_CS_GPIO_Port GPIOC
#define LCD_NOE_Pin GPIO_PIN_5
#define LCD_NOE_GPIO_Port GPIOC
#define CH_NT_Pin GPIO_PIN_15
#define CH_NT_GPIO_Port GPIOB
#define E5E8_B_Pin GPIO_PIN_7
#define E5E8_B_GPIO_Port GPIOC
#define E5E8_A_Pin GPIO_PIN_8
#define E5E8_A_GPIO_Port GPIOC
#define E5E8_A_EXTI_IRQn EXTI9_5_IRQn
#define E5E8_K_Pin GPIO_PIN_9
#define E5E8_K_GPIO_Port GPIOC
#define E5E8_K_EXTI_IRQn EXTI9_5_IRQn
#define LCD_PWM_Pin GPIO_PIN_6
#define LCD_PWM_GPIO_Port GPIOB
#define MPU_SDA_Pin GPIO_PIN_7
#define MPU_SDA_GPIO_Port GPIOB
#define MPU_SCL_Pin GPIO_PIN_8
#define MPU_SCL_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_9
#define LCD_RST_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */
#define FREERTOS_S(x) (250*x)
#define FREERTOS_MS(x) ((int)(1.0*x/4+0.5))

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
