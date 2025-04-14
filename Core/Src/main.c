/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include "DHT22.h"
#include "aqi.h"       // Include the AQI module
#include "sensor_logger.h" // Include sensor logger for buffer management
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// Flag for entering and exiting low power mode
uint8_t low_power_mode = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */


/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
typedef struct {
    GPIO_TypeDef* powerPort;
    uint16_t powerPin;
    uint32_t lastPollTime;
    uint32_t pollInterval;     // in ms
    uint8_t isEnabled;
} sensor_t;

#define AQI_ALERT_THRESHOLD 150    // Set the threshold for AQI alert (example value)


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void poll_dht22(uint32_t current_time);
void poll_mq135(uint32_t current_time);
void poll_bme688(uint32_t current_time);
void adaptive_polling();

/* USER CODE END PFP */

/* Private user variables ---------------------------------------------------------*/
sensor_t dht22 = {DHT22_PWR_GPIO_Port, DHT22_PWR_Pin, 0, 10000, 1};   // Poll every 10s
sensor_t mq135 = {MQ135_PWR_GPIO_Port, MQ135_PWR_Pin, 0, 10000, 1};    // Poll every 10s
sensor_t bme688 = {BME688_PWR_GPIO_Port, BME688_PWR_Pin, 0, 10000, 1};   // Poll every 10s


int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  DHT22_Init(DHT22_GPIO_Port, DHT22_Pin);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();


  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  uint32_t now = HAL_GetTick();
    /* USER CODE END WHILE */
	  adaptive_polling(); // make it in seperate thread to poll contineuosly...
	  poll_dht22(now);
	  poll_mq135(now);
	  poll_bme688(now);

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void adaptive_polling(){
    // --- IR Sensor ---
    int ir_state = HAL_GPIO_ReadPin(IR_Sensor_GPIO_Port, IR_Sensor_Pin);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_Delay(0.01); // 10 us
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);

	uint32_t value = HAL_TIM_ReadCapturedValue(&htim1, TIM_CHANNEL_2);
	uint32_t distance = (value * 0.034) / 2;
}

void poll_dht22(uint32_t current_time) {
    if (!dht22.isEnabled) return;

    if ((current_time - dht22.lastPollTime) >= dht22.pollInterval) {
        dht22.lastPollTime = current_time;

        float temperature, humidity;
        if (DHT22_Get_Data(&temperature, &humidity)) {
            // Log the data
            log_dht22_data(temperature, humidity, current_time);

            // Adaptive logic (example)
            if (temperature > 30.0 || temperature < 18.0) {
                dht22.pollInterval = 5000; // Poll faster
            } else {
                dht22.pollInterval = 10000; // Normal rate
            }
        }
    }
}

void poll_mq135(uint32_t current_time) {
    if (!mq135.isEnabled) return;

    if ((current_time - mq135.lastPollTime) >= mq135.pollInterval) {
        mq135.lastPollTime = current_time;

        float mq_value = read_mq135(mq135);
        float co, no2;
        if (mq_value) {
            log_mq135_data(co, no2, current_time);
            update_aqi();

            if (co > 5.0 || no2 > 5.0) {
                mq135.pollInterval = 3000;
            } else {
                mq135.pollInterval = 10000;
            }
        }
    }
}


void poll_bme688(uint32_t current_time) {
    if (!bme688.isEnabled) return;

    if ((current_time - bme688.lastPollTime) >= bme688.pollInterval) {
        bme688.lastPollTime = current_time;

        float pm25, co, no2, temp, hum;
        if (read_bme688(&pm25, &co, &no2, &temp, &hum)) {
            log_bme688_data(pm25, co, no2, temp, hum, current_time);
            update_aqi();

            // Adaptive logic based on fluctuation
            if (pm25 > 100 || co > 5 || no2 > 5) {
                bme688.pollInterval = 5000;
            } else {
                bme688.pollInterval = 10000;
            }
        }
    }
}


// Dummy MQ135 Read Function
uint8_t MQ135_Read(float* ppm) {
    if (rand() % 100 < 95) { // 95% chance of success
        *ppm = 50 + (rand() % 251); // 50 to 300 ppm
        return 1;
    }
    return 0; // simulate failure
}

// Dummy BME688 Read Function
uint8_t BME688_Read(float* temperature, float* humidity, float* gasResistance) {
    if (rand() % 100 < 95) { // 95% chance of success
        *temperature = 20.0 + (rand() % 1500) / 100.0;      // 20.0 to 35.0 °C
        *humidity = 30.0 + (rand() % 5000) / 100.0;         // 30.0 to 80.0 %
        *gasResistance = 10.0 + (rand() % 5000) / 100.0;    // 10.0 to 60.0 kΩ
        return 1;
    }
    return 0; // simulate failure
}


void power_on(sensor_t* sensor) {
    HAL_GPIO_WritePin(sensor->powerPort, sensor->powerPin, GPIO_PIN_SET);
}

void power_off(sensor_t* sensor) {
    HAL_GPIO_WritePin(sensor->powerPort, sensor->powerPin, GPIO_PIN_RESET);
}

// Function to enter Sleep Mode
void enter_sleep_mode(void) {
    // Disable all non-essential peripherals here before entering sleep
    HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);  // Wait for Interrupt (WFI) mode
}

// Function to enter Stop Mode
void enter_stop_mode(void) {
    // You can stop the main system clock, reduce peripherals, etc.
    HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI); // Wait for Interrupt (WFI)
}

// Interrupt handler (sensor interrupt or external pin interrupt)
void EXTI0_IRQHandler(void) {
    // Handle interrupt (for example, from an IR sensor or a user button)
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);  // Clear interrupt flag for the pin

    // Wake up logic (if the controller was in sleep mode, exit sleep mode)
    if (low_power_mode) {
        low_power_mode = 0;
        printf("Waking up from sleep mode!\n");
        HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);  // Enable the wake-up pin again (if used)
    }
}


/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MQ135_PWR_Pin|IR_PWR_Pin|DHT22_PWR_Pin|BME688_PWR_Pin
                          |Ultrasonic_PWR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Ultrasonic_Trigger_GPIO_Port, Ultrasonic_Trigger_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IR_Sensor_Pin */
  GPIO_InitStruct.Pin = IR_Sensor_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IR_Sensor_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MQ135_PWR_Pin IR_PWR_Pin DHT22_PWR_Pin BME688_PWR_Pin
                           Ultrasonic_PWR_Pin */
  GPIO_InitStruct.Pin = MQ135_PWR_Pin|IR_PWR_Pin|DHT22_PWR_Pin|BME688_PWR_Pin
                          |Ultrasonic_PWR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : DHT22_Pin */
  GPIO_InitStruct.Pin = DHT22_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(DHT22_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Ultrasonic_Trigger_Pin */
  GPIO_InitStruct.Pin = Ultrasonic_Trigger_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Ultrasonic_Trigger_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
