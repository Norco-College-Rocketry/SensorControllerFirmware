/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include <memory.h>
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "conversions.h"
#include "ADS1118.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  Ads1118TypeDef adc;
  PT_Config pt;
  LC_Config lc;
  CONVERSION_MODE conv_mode;
} SensorController;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CAN_TX_ID 0x442
#define PID_LABEL 01
#define SENSOR_A_LABEL 1
#define SENSOR_B_LABEL 2

#define EXCITATION 5 // Sensor excitation voltage
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim14;

/* USER CODE BEGIN PV */
uint8_t start_read_adc = 0;
uint8_t adc_read_cplt = 0;

SensorController controller = {
  .pt = { 1000, 0, 4.5f, 0.5f },
  .lc = { 100, 0.002f, EXCITATION },
  .conv_mode = MODE_PRESSURE
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM14_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN_Init();
  MX_SPI1_Init();
  MX_TIM14_Init();
  /* USER CODE BEGIN 2 */
  CAN_FilterTypeDef filter;
  filter.FilterMaskIdHigh = 0x0;
  filter.FilterMaskIdLow = 0x0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterBank = 0;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  filter.FilterActivation = CAN_FILTER_ENABLE;
  if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK) {
      Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING)) {
    Error_Handler();
  };

  if (HAL_CAN_Start(&hcan) != HAL_OK) {
      Error_Handler();
  }

  // Configure ADC
  controller.adc.hspi = &hspi1;
  controller.adc.cs_gpio_port = ADC_CS_GPIO_Port;
  controller.adc.cs_pin = ADC_CS_Pin;
  // PT configuration
  controller.adc.config = (ADS1118_CONFIG_DEFAULT           |
               (0b111 << ADS1118_CONFIG_BIT_MUX) |
               (1 << ADS1118_CONFIG_BIT_SS)      |
               (0b000 << ADS1118_CONFIG_BIT_PGA) ) & 0xFBFF;
  // TC configuration
//  controller.adc.config = (ADS1118_CONFIG_DEFAULT           |
//               (1 << ADS1118_CONFIG_BIT_SS)      |
//               (0b111 << ADS1118_CONFIG_BIT_PGA) );
  Ads1118_Configure(&controller.adc);

  // Start peripherals
//  HAL_ADC_Start(&hadc);
  HAL_TIM_Base_Start_IT(&htim14);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  int16_t buf[] = { 0, 0 };

  while (1)
  {
      if (adc_read_cplt && (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4))) {
    	  // Update ADC config readback
    	  controller.adc.config_readback = buf[1];
        float voltage = Ads1118_output_code_to_voltage(&controller.adc, buf[0]);
        float res = 0.0f;

        switch(controller.conv_mode) {
          case(MODE_VOLTAGE): {
              res = voltage;
              const uint8_t packet_size = 8;
              uint8_t packet[packet_size];
              create_float_packet(PID_LABEL, TELEMETRY_PACKET, VOLTAGE_TELEMETRY, res, packet);
              send_can_msg(packet, packet_size);
          } break;

          case(MODE_PRESSURE): {
              res = convert_pressure(voltage, &controller.pt);
              const uint8_t packet_size = 8;
              uint8_t packet[packet_size];
              create_float_packet(PID_LABEL, TELEMETRY_PACKET, PRESSURE_TELEMETRY, res, packet);
              send_can_msg(packet, packet_size);
          } break;

          case(MODE_PRESSURE_CALIBRATED): {
              res = convert_pressure_calibrated(voltage, &controller.pt);
              const uint8_t packet_size = 8;
              uint8_t packet[packet_size];
              create_float_packet(PID_LABEL, TELEMETRY_PACKET, PRESSURE_TELEMETRY, res, packet);
              send_can_msg(packet, packet_size);
          } break;

          case (MODE_TEMPERATURE): {
            res = convert_thermocouple_K(voltage);
          } break;
        }

        // Send measurement over CAN
//        send_can_msg((uint8_t*)(&res), sizeof(res));
        adc_read_cplt = 0;
      }

      if (start_read_adc) {
    	  // Start new single shot
        if (Ads1118_Transmit(&controller.adc, (uint32_t*)&buf) != HAL_OK) {
          Error_Handler();
        }

         start_read_adc = 0;
      }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN;
  hcan.Init.Prescaler = 6;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 99;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 47999;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ADC_CS_GPIO_Port, ADC_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, STATUS_IND_Pin|WARN_IND_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ADC_CS_Pin */
  GPIO_InitStruct.Pin = ADC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(ADC_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : STATUS_IND_Pin WARN_IND_Pin */
  GPIO_InitStruct.Pin = STATUS_IND_Pin|WARN_IND_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
float temperature_code_to_temperature(int16_t temperature_code) {
    if (temperature_code & (1 << 14)) {
        temperature_code -= 1;
        temperature_code = ~temperature_code;
    }
    return temperature_code * 0.03125f;
}

HAL_StatusTypeDef send_can_msg(const uint8_t *data, size_t len) {
    CAN_TxHeaderTypeDef header;
    header.IDE = CAN_ID_STD;
    header.StdId = CAN_TX_ID;
    header.RTR = CAN_RTR_DATA;
    header.TransmitGlobalTime = DISABLE;
    header.DLC = len;

    uint32_t mailbox;

    HAL_StatusTypeDef status = HAL_CAN_AddTxMessage(&hcan, &header, data, &mailbox);
    if (status != HAL_OK) {

    }

    return status;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_RxHeaderTypeDef header;
    uint8_t data[64];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) {
        Error_Handler();
    }

    uint8_t packet_type = data[0];
    if (packet_type == COMMAND_PACKET) {
      uint8_t cmd_type = data[1],
              label = data[2];

      if (label != PID_LABEL) return;

      switch (cmd_type) {
        case MODE_COMMAND: {
          // TODO state transition - configure ADC
          controller.conv_mode = data[3];
        }
          break;

        case PT_CALIBRATION_COMMAND: {
          uint8_t pt_id = data[3];
          switch (pt_id) {
            case 1: {
              controller.pt.m = 263.1578f;
              controller.pt.b = -150.3421f;
            } break;
            case 2: {
              controller.pt.m = 263.1578f;
              controller.pt.b = -146.1052f;
            } break;
            case 3: {
              controller.pt.m = 256.4102f;
              controller.pt.b = -142.0256f;
            } break;
            case 4: {
              controller.pt.m = 277.7777f;
              controller.pt.b = -152.6388f;
            } break;
            case 5: {
              controller.pt.m = 277.7777f;
              controller.pt.b = -136.4166f;
            } break;
          }
        } break;

        case LED_COMMAND: {
          uint8_t state = data[3];
          if (state == 2) {
            HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);
          } else {
            HAL_GPIO_WritePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin, state);
          }
        }
          break;
      }
    }
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	adc_read_cplt = 1;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  start_read_adc = 1;
//    should_read_adc = 1;

//    float out[2];
//    uint32_t buf;

//    buf = HAL_ADC_GetValue(&hadc);
//    out[TS_MCU] = __LL_ADC_CALC_TEMPERATURE(3300, buf, LL_ADC_RESOLUTION_12B);

//  uint16_t adc_config = ADS1118_CONFIG_DEFAULT | (0b100 << ADS1118_CONFIG_BIT_MUX) | (1 << ADS1118_CONFIG_BIT_SS);
//    if (Ads1118_Transmit(&adc_config, &hspi1, &buf, 1000) != HAL_OK) {
//        Error_Handler();
//    }
//    out[TS_ADC] = temperature_code_to_temperature(buf);

//    send_can_msg((uint8_t*)(&buf), 4);
}

void create_float_packet(uint8_t label, PACKET_TYPE packet_type, TELEMETRY_TYPE telemetry_type, float value, uint8_t* buf) {
    buf[0] = packet_type;
    buf[1] = PID_LABEL;
    buf[2] = SENSOR_B_LABEL; // TODO
    buf[3] = telemetry_type;
    memcpy(buf+4,&value, sizeof(value));
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
	  HAL_GPIO_TogglePin(WARN_IND_GPIO_Port, WARN_IND_Pin);
	  HAL_GPIO_TogglePin(STATUS_IND_GPIO_Port, STATUS_IND_Pin);
	  HAL_Delay(100);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
