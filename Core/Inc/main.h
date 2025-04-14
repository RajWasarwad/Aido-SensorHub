/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
#define MQ135_Pin GPIO_PIN_0
#define MQ135_GPIO_Port GPIOA
#define IR_Sensor_Pin GPIO_PIN_5
#define IR_Sensor_GPIO_Port GPIOA
#define MQ135_PWR_Pin GPIO_PIN_0
#define MQ135_PWR_GPIO_Port GPIOB
#define IR_PWR_Pin GPIO_PIN_1
#define IR_PWR_GPIO_Port GPIOB
#define DHT22_Pin GPIO_PIN_10
#define DHT22_GPIO_Port GPIOB
#define DHT22_PWR_Pin GPIO_PIN_11
#define DHT22_PWR_GPIO_Port GPIOB
#define BME688_PWR_Pin GPIO_PIN_13
#define BME688_PWR_GPIO_Port GPIOB
#define Ultrasonic_PWR_Pin GPIO_PIN_15
#define Ultrasonic_PWR_GPIO_Port GPIOB
#define Ultrasonic_Trigger_Pin GPIO_PIN_8
#define Ultrasonic_Trigger_GPIO_Port GPIOA
#define Ultrasonic_Echo_Pin GPIO_PIN_9
#define Ultrasonic_Echo_GPIO_Port GPIOA
#define BME688_SCL_Pin GPIO_PIN_6
#define BME688_SCL_GPIO_Port GPIOB
#define BME688_SDA_Pin GPIO_PIN_7
#define BME688_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

typedef struct {
    GPIO_TypeDef* powerPort;
    uint16_t powerPin;
    uint32_t lastPollTime;
    uint32_t pollInterval;     // in ms
    uint8_t isEnabled;
    float lastValue;
} sensor_t;


/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
