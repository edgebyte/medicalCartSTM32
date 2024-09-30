/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define DHT22_Pin GPIO_PIN_2
#define DHT22_GPIO_Port GPIOE
#define S06_Pin GPIO_PIN_2
#define S06_GPIO_Port GPIOH
#define LED06_Pin GPIO_PIN_3
#define LED06_GPIO_Port GPIOH
#define S05_Pin GPIO_PIN_4
#define S05_GPIO_Port GPIOH
#define LED05_Pin GPIO_PIN_5
#define LED05_GPIO_Port GPIOH
#define LOCK05_Pin GPIO_PIN_12
#define LOCK05_GPIO_Port GPIOB
#define LOCK04_Pin GPIO_PIN_13
#define LOCK04_GPIO_Port GPIOB
#define LOCK03_Pin GPIO_PIN_14
#define LOCK03_GPIO_Port GPIOB
#define LOCK02_Pin GPIO_PIN_15
#define LOCK02_GPIO_Port GPIOB
#define LOCK01_Pin GPIO_PIN_8
#define LOCK01_GPIO_Port GPIOD
#define S04_Pin GPIO_PIN_14
#define S04_GPIO_Port GPIOD
#define LED04_Pin GPIO_PIN_15
#define LED04_GPIO_Port GPIOD
#define S03_Pin GPIO_PIN_2
#define S03_GPIO_Port GPIOG
#define LED03_Pin GPIO_PIN_3
#define LED03_GPIO_Port GPIOG
#define S02_Pin GPIO_PIN_4
#define S02_GPIO_Port GPIOG
#define LED02_Pin GPIO_PIN_5
#define LED02_GPIO_Port GPIOG
#define S01_Pin GPIO_PIN_6
#define S01_GPIO_Port GPIOG
#define LED01_Pin GPIO_PIN_7
#define LED01_GPIO_Port GPIOG
#define BUZZER_Pin GPIO_PIN_8
#define BUZZER_GPIO_Port GPIOC
#define LOCK10_Pin GPIO_PIN_0
#define LOCK10_GPIO_Port GPIOD
#define LOCK09_Pin GPIO_PIN_1
#define LOCK09_GPIO_Port GPIOD
#define LOCK08_Pin GPIO_PIN_2
#define LOCK08_GPIO_Port GPIOD
#define LOCK07_Pin GPIO_PIN_3
#define LOCK07_GPIO_Port GPIOD
#define LOCK06_Pin GPIO_PIN_4
#define LOCK06_GPIO_Port GPIOD
#define S10_Pin GPIO_PIN_6
#define S10_GPIO_Port GPIOD
#define LED10_Pin GPIO_PIN_7
#define LED10_GPIO_Port GPIOD
#define S09_Pin GPIO_PIN_9
#define S09_GPIO_Port GPIOG
#define LED09_Pin GPIO_PIN_10
#define LED09_GPIO_Port GPIOG
#define S08_Pin GPIO_PIN_11
#define S08_GPIO_Port GPIOG
#define LED08_Pin GPIO_PIN_12
#define LED08_GPIO_Port GPIOG
#define S07_Pin GPIO_PIN_13
#define S07_GPIO_Port GPIOG
#define LED07_Pin GPIO_PIN_14
#define LED07_GPIO_Port GPIOG
#define ADD4_Pin GPIO_PIN_3
#define ADD4_GPIO_Port GPIOB
#define ADD3_Pin GPIO_PIN_4
#define ADD3_GPIO_Port GPIOB
#define ADD2_Pin GPIO_PIN_5
#define ADD2_GPIO_Port GPIOB
#define ADD1_Pin GPIO_PIN_6
#define ADD1_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
