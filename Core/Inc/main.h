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
#include "stm32f0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef enum {
    TELEMETRY_PACKET = 0x10,
    COMMAND_PACKET = 0x20
} PACKET_TYPE;

typedef enum {
  LED_COMMAND = 0x01,
  VALVE_COMMAND = 0x02,
  PT_CALIBRATION_COMMAND = 0x03,
  MODE_COMMAND = 0x04
} COMMAND_TYPE;

typedef enum {
  VOLTAGE_TELEMETRY = 0x01,
  PRESSURE_TELEMETRY = 0x02,
  TEMPERATURE_TELEMETRY = 0x03,
  WEIGHT_TELEMETRY = 0x04
} TELEMETRY_TYPE;

typedef enum {
  MODE_VOLTAGE = 0,
  MODE_PRESSURE = 1,
  MODE_PRESSURE_CALIBRATED = 2,
  MODE_TEMPERATURE = 3,
  MODE_WEIGHT = 4
} CONVERSION_MODE;
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

/* Create CAN telemetry packet
 * @param label P&ID identifying label
 * @param buf 7 byte packet buffer
 */
void create_float_packet(uint8_t label, PACKET_TYPE, TELEMETRY_TYPE, float value, uint8_t* buf);

HAL_StatusTypeDef send_can_msg(const uint8_t *data, size_t len);

float temperature_code_to_temperature(int16_t temperature_code);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ADC_CS_Pin GPIO_PIN_15
#define ADC_CS_GPIO_Port GPIOA
#define STATUS_IND_Pin GPIO_PIN_6
#define STATUS_IND_GPIO_Port GPIOB
#define WARN_IND_Pin GPIO_PIN_7
#define WARN_IND_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
