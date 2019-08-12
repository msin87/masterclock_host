/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#define OUT2_pos_Pin GPIO_PIN_2
#define OUT2_pos_GPIO_Port GPIOE
#define OUT3_pos_Pin GPIO_PIN_3
#define OUT3_pos_GPIO_Port GPIOE
#define OUT4_pos_Pin GPIO_PIN_4
#define OUT4_pos_GPIO_Port GPIOE
#define OUT5_pos_Pin GPIO_PIN_5
#define OUT5_pos_GPIO_Port GPIOE
#define OUT6_pos_Pin GPIO_PIN_6
#define OUT6_pos_GPIO_Port GPIOE
#define Isense9_Pin GPIO_PIN_0
#define Isense9_GPIO_Port GPIOC
#define Isense10_Pin GPIO_PIN_1
#define Isense10_GPIO_Port GPIOC
#define Isense11_Pin GPIO_PIN_2
#define Isense11_GPIO_Port GPIOC
#define UVLOsense_Pin GPIO_PIN_3
#define UVLOsense_GPIO_Port GPIOC
#define Isense0_Pin GPIO_PIN_0
#define Isense0_GPIO_Port GPIOA
#define Isense1_Pin GPIO_PIN_1
#define Isense1_GPIO_Port GPIOA
#define Isense2_Pin GPIO_PIN_2
#define Isense2_GPIO_Port GPIOA
#define Isense3_Pin GPIO_PIN_3
#define Isense3_GPIO_Port GPIOA
#define DTMFout_Pin GPIO_PIN_4
#define DTMFout_GPIO_Port GPIOA
#define Isense4_Pin GPIO_PIN_5
#define Isense4_GPIO_Port GPIOA
#define Isense5_Pin GPIO_PIN_6
#define Isense5_GPIO_Port GPIOA
#define Isense6_Pin GPIO_PIN_7
#define Isense6_GPIO_Port GPIOA
#define Isense7_Pin GPIO_PIN_0
#define Isense7_GPIO_Port GPIOB
#define Isense8_Pin GPIO_PIN_1
#define Isense8_GPIO_Port GPIOB
#define OUT7_pos_Pin GPIO_PIN_7
#define OUT7_pos_GPIO_Port GPIOE
#define OUT8_pos_Pin GPIO_PIN_8
#define OUT8_pos_GPIO_Port GPIOE
#define OUT9_pos_Pin GPIO_PIN_9
#define OUT9_pos_GPIO_Port GPIOE
#define OUT10_pos_Pin GPIO_PIN_10
#define OUT10_pos_GPIO_Port GPIOE
#define OUT11_pos_Pin GPIO_PIN_11
#define OUT11_pos_GPIO_Port GPIOE
#define OUT8_neg_Pin GPIO_PIN_8
#define OUT8_neg_GPIO_Port GPIOD
#define OUT9_neg_Pin GPIO_PIN_9
#define OUT9_neg_GPIO_Port GPIOD
#define OUT10_neg_Pin GPIO_PIN_10
#define OUT10_neg_GPIO_Port GPIOD
#define OUT11_neg_Pin GPIO_PIN_11
#define OUT11_neg_GPIO_Port GPIOD
#define ShutdownRPI_Pin GPIO_PIN_15
#define ShutdownRPI_GPIO_Port GPIOA
#define OUT0_neg_Pin GPIO_PIN_0
#define OUT0_neg_GPIO_Port GPIOD
#define OUT1_neg_Pin GPIO_PIN_1
#define OUT1_neg_GPIO_Port GPIOD
#define OUT2_neg_Pin GPIO_PIN_2
#define OUT2_neg_GPIO_Port GPIOD
#define OUT3_neg_Pin GPIO_PIN_3
#define OUT3_neg_GPIO_Port GPIOD
#define OUT4_neg_Pin GPIO_PIN_4
#define OUT4_neg_GPIO_Port GPIOD
#define OUT5_neg_Pin GPIO_PIN_5
#define OUT5_neg_GPIO_Port GPIOD
#define OUT6_neg_Pin GPIO_PIN_6
#define OUT6_neg_GPIO_Port GPIOD
#define OUT7_neg_Pin GPIO_PIN_7
#define OUT7_neg_GPIO_Port GPIOD
#define OUT0_pos_Pin GPIO_PIN_0
#define OUT0_pos_GPIO_Port GPIOE
#define OUT1_pos_Pin GPIO_PIN_1
#define OUT1_pos_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
