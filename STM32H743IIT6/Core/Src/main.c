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
#include "main.h"
#include "cmsis_os.h"
#include "dht22.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi5;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart6;
DHT22_Data_t sensorData;
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = { .name = "defaultTask",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityNormal, };
/* Definitions for buzzerTask */
osThreadId_t buzzerTaskHandle;
const osThreadAttr_t buzzerTask_attributes = { .name = "buzzerTask",
		.stack_size = 128 * 4, .priority = (osPriority_t) osPriorityLow, };
/* USER CODE BEGIN PV */
int timeDelay = 1;
int solenoidTimeDelay = 300;
uint8_t add1[1];
uint8_t add2[1];
uint8_t add3[1];
uint8_t add4[1];
uint8_t address[1];
uint8_t buzzerStatus[1];
uint8_t pinStatus[10] = { 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38,
		0x39 };
uint8_t solenoidStatus[10] = { };
uint8_t play[6] = { 0x41, 0x42, 0x43, 0x44, 0x45, 0x46 };
uint8_t rxBuffer[8];
uint8_t txBuffer[15] = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x03 };

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI5_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void solenoidControl(uint8_t solenoidNo);
void readLockStatus();
void replyProtocol();
uint8_t checkSum();
uint8_t checkSumReply();
uint8_t readAddress();

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
	HAL_Init();
	SystemClock_Config();
	MX_GPIO_Init();
	DHT22_Init();
	//MX_TIM1_Init(); // Initialize TIM1 for microsecond delay
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MPU Configuration--------------------------------------------------------*/
	MPU_Config();

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
	MX_UART4_Init();
	MX_USART6_UART_Init();
	MX_SPI1_Init();
	MX_SPI5_Init();
	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* Init scheduler */
	osKernelInitialize();

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/* Create the thread(s) */
	/* creation of defaultTask */
	defaultTaskHandle = osThreadNew(StartDefaultTask, NULL,
			&defaultTask_attributes);

	/* creation of buzzerTask */
	buzzerTaskHandle = osThreadNew(StartTask02, NULL, &buzzerTask_attributes);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
	/* USER CODE END RTOS_THREADS */

	/* USER CODE BEGIN RTOS_EVENTS */
	/* add events, ... */
	/* USER CODE END RTOS_EVENTS */
	vTaskSuspend(buzzerTaskHandle);
	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Supply configuration update enable
	 */
	HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

	/** Configure the main internal regulator output voltage
	 */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	__HAL_RCC_SYSCFG_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

	while (!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {
	}

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLN = 60;
	RCC_OscInitStruct.PLL.PLLP = 2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	RCC_OscInitStruct.PLL.PLLR = 2;
	RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
	RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
	RCC_OscInitStruct.PLL.PLLFRACN = 0;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 | RCC_CLOCKTYPE_D3PCLK1
			| RCC_CLOCKTYPE_D1PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
	RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 0x0;
	hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
	hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	hspi1.Init.TxCRCInitializationPattern =
	SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	hspi1.Init.RxCRCInitializationPattern =
	SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
	hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI1_Init 2 */

	/* USER CODE END SPI1_Init 2 */

}

/**
 * @brief SPI5 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI5_Init(void) {

	/* USER CODE BEGIN SPI5_Init 0 */

	/* USER CODE END SPI5_Init 0 */

	/* USER CODE BEGIN SPI5_Init 1 */

	/* USER CODE END SPI5_Init 1 */
	/* SPI5 parameter configuration*/
	hspi5.Instance = SPI5;
	hspi5.Init.Mode = SPI_MODE_MASTER;
	hspi5.Init.Direction = SPI_DIRECTION_2LINES;
	hspi5.Init.DataSize = SPI_DATASIZE_4BIT;
	hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi5.Init.NSS = SPI_NSS_SOFT;
	hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
	hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi5.Init.CRCPolynomial = 0x0;
	hspi5.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
	hspi5.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
	hspi5.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
	hspi5.Init.TxCRCInitializationPattern =
	SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	hspi5.Init.RxCRCInitializationPattern =
	SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
	hspi5.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
	hspi5.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
	hspi5.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
	hspi5.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
	hspi5.Init.IOSwap = SPI_IO_SWAP_DISABLE;
	if (HAL_SPI_Init(&hspi5) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN SPI5_Init 2 */

	/* USER CODE END SPI5_Init 2 */

}

/**
 * @brief UART4 Initialization Function
 * @param None
 * @retval None
 */
static void MX_UART4_Init(void) {

	/* USER CODE BEGIN UART4_Init 0 */

	/* USER CODE END UART4_Init 0 */

	/* USER CODE BEGIN UART4_Init 1 */

	/* USER CODE END UART4_Init 1 */
	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart4) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN UART4_Init 2 */

	/* USER CODE END UART4_Init 2 */

}

/**
 * @brief USART6 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART6_UART_Init(void) {

	/* USER CODE BEGIN USART6_Init 0 */

	/* USER CODE END USART6_Init 0 */

	/* USER CODE BEGIN USART6_Init 1 */

	/* USER CODE END USART6_Init 1 */
	huart6.Instance = USART6;
	huart6.Init.BaudRate = 115200;
	huart6.Init.WordLength = UART_WORDLENGTH_8B;
	huart6.Init.StopBits = UART_STOPBITS_1;
	huart6.Init.Parity = UART_PARITY_NONE;
	huart6.Init.Mode = UART_MODE_TX_RX;
	huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart6.Init.OverSampling = UART_OVERSAMPLING_16;
	huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart6.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_RS485Ex_Init(&huart6, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart6, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart6, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart6) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART6_Init 2 */

	/* USER CODE END USART6_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOD_CLK_ENABLE();
	__HAL_RCC_GPIOG_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOH, S06_Pin | LED06_Pin | S05_Pin | LED05_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOD,
			S04_Pin | LED04_Pin | LOCK10_Pin | LOCK09_Pin | LOCK08_Pin
					| LOCK07_Pin | LOCK06_Pin | S10_Pin | LED10_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG,
			S03_Pin | LED03_Pin | S02_Pin | LED02_Pin | S01_Pin | LED01_Pin
					| S09_Pin | LED09_Pin | S08_Pin | LED08_Pin | S07_Pin
					| LED07_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : DHT22_Pin */
	GPIO_InitStruct.Pin = DHT22_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(DHT22_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : S06_Pin LED06_Pin S05_Pin LED05_Pin */
	GPIO_InitStruct.Pin = S06_Pin | LED06_Pin | S05_Pin | LED05_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

	/*Configure GPIO pins : LOCK05_Pin LOCK04_Pin LOCK03_Pin LOCK02_Pin
	 ADD4_Pin ADD3_Pin ADD2_Pin ADD1_Pin */
	GPIO_InitStruct.Pin = LOCK05_Pin | LOCK04_Pin | LOCK03_Pin | LOCK02_Pin
			| ADD4_Pin | ADD3_Pin | ADD2_Pin | ADD1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pin : LOCK01_Pin */
	GPIO_InitStruct.Pin = LOCK01_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LOCK01_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : S04_Pin LED04_Pin LOCK10_Pin LOCK09_Pin
	 LOCK08_Pin LOCK07_Pin LOCK06_Pin S10_Pin
	 LED10_Pin */
	GPIO_InitStruct.Pin = S04_Pin | LED04_Pin | LOCK10_Pin | LOCK09_Pin
			| LOCK08_Pin | LOCK07_Pin | LOCK06_Pin | S10_Pin | LED10_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pins : S03_Pin LED03_Pin S02_Pin LED02_Pin
	 S01_Pin LED01_Pin S09_Pin LED09_Pin
	 S08_Pin LED08_Pin S07_Pin LED07_Pin */
	GPIO_InitStruct.Pin = S03_Pin | LED03_Pin | S02_Pin | LED02_Pin | S01_Pin
			| LED01_Pin | S09_Pin | LED09_Pin | S08_Pin | LED08_Pin | S07_Pin
			| LED07_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pin : BUZZER_Pin */
	GPIO_InitStruct.Pin = BUZZER_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(BUZZER_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument) {
	/* USER CODE BEGIN 5 */
	/* Infinite loop */
	for (;;) {
		if (HAL_UART_Receive(&huart4, rxBuffer, 8, 10) == HAL_OK) {
			// Data received successfully, transmit it back
			//HAL_GPIO_WritePin(S01_GPIO_Port, S01_Pin, 1);
			//HAL_UART_Transmit(&huart4, (uint8_t*) rxBuffer, 8, 10);
			if (rxBuffer[6] == checkSum() && rxBuffer[1] == 0x01) {
				//Control
				if (rxBuffer[2] == 0x43) {
					//Control Buzzer
					if (rxBuffer[3] == 0x42) {
						if (rxBuffer[5] == 0x31) {
							vTaskResume(buzzerTaskHandle);
							buzzerStatus[0] = 0x31;
						}
						if (rxBuffer[5] == 0x30) {
							vTaskSuspend(buzzerTaskHandle);
							HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin,
									GPIO_PIN_RESET);
							buzzerStatus[0] = 0x30;
						}

					}
					//Control Solenoid
					if (rxBuffer[3] == 0x53) {
						solenoidControl(rxBuffer[4]);
						replyProtocol();
					}
				}
				//Read
				if (rxBuffer[2] == 0x52) {
					//Buzzer
					if (rxBuffer[3] == 0x42) {

					}
					//Humid
					if (rxBuffer[3] == 0x48) {

					}
					//Solenoid
					if (rxBuffer[3] == 0x53) {

					}
					//Temp
					if (rxBuffer[3] == 0x54) {

					}

				}
				if (rxBuffer[2] == 0x43) {
					//pinStatusCheck();
					//checkSumProtocol();
					//add1[0] = readAddress();
					//HAL_UART_Transmit(&huart4, (uint8_t*) "TEST", 4, 10);
					//HAL_UART_Transmit(&huart4, (uint8_t*) pinStatus, 10, 10);
					//HAL_UART_Transmit(&huart4, (uint8_t*) address, 1, 10);
				}
				//HAL_UART_Transmit(&huart4, (uint8_t*) play, 8, 10);
			}
		}
		osDelay(1);
	}

	/* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
 * @brief Function implementing the myTask02 thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument) {
	/* USER CODE BEGIN StartTask02 */
	/* Infinite loop */
	for (;;) {
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_SET);
		osDelay(500);
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, GPIO_PIN_RESET);
		osDelay(500);
	}
	/* USER CODE END StartTask02 */
}

void readLockStatus() {
	solenoidStatus[0] = (
			(HAL_GPIO_ReadPin(LOCK01_GPIO_Port, LOCK01_Pin) == GPIO_PIN_SET) ?
					0x31 : 0x30);
	solenoidStatus[1] = (
			(HAL_GPIO_ReadPin(LOCK02_GPIO_Port, LOCK02_Pin) == GPIO_PIN_SET) ?
					0x31 : 0x30);
	solenoidStatus[2] = (
			(HAL_GPIO_ReadPin(LOCK03_GPIO_Port, LOCK03_Pin) == GPIO_PIN_SET) ?
					0x31 : 0x30);
	solenoidStatus[3] = (
			(HAL_GPIO_ReadPin(LOCK04_GPIO_Port, LOCK04_Pin) == GPIO_PIN_SET) ?
					0x31 : 0x30);
	solenoidStatus[4] = (
			(HAL_GPIO_ReadPin(LOCK05_GPIO_Port, LOCK05_Pin) == GPIO_PIN_SET) ?
					0x31 : 0x30);
	solenoidStatus[5] = (
			(HAL_GPIO_ReadPin(LOCK06_GPIO_Port, LOCK06_Pin) == GPIO_PIN_SET) ?
					0x31 : 0x30);
	solenoidStatus[6] = (
			(HAL_GPIO_ReadPin(LOCK07_GPIO_Port, LOCK07_Pin) == GPIO_PIN_SET) ?
					0x31 : 0x30);
	solenoidStatus[7] = (
			(HAL_GPIO_ReadPin(LOCK08_GPIO_Port, LOCK08_Pin) == GPIO_PIN_SET) ?
					0x31 : 0x30);
	solenoidStatus[8] = (
			(HAL_GPIO_ReadPin(LOCK09_GPIO_Port, LOCK09_Pin) == GPIO_PIN_SET) ?
					0x31 : 0x30);
	solenoidStatus[9] = (
			(HAL_GPIO_ReadPin(LOCK10_GPIO_Port, LOCK10_Pin) == GPIO_PIN_SET) ?
					0x31 : 0x30);
}

void replyProtocol() {
	txBuffer[0] = 0x02;
	txBuffer[1] = readAddress();
	txBuffer[2] = rxBuffer[2];
	readLockStatus();
	txBuffer[3] = solenoidStatus[0];
	if (txBuffer[2] == 0x42) {
		txBuffer[3] = buzzerStatus[0];
	}
	txBuffer[4] = solenoidStatus[1];
	txBuffer[5] = solenoidStatus[2];
	txBuffer[6] = solenoidStatus[3];
	txBuffer[7] = solenoidStatus[4];
	txBuffer[8] = solenoidStatus[5];
	txBuffer[9] = solenoidStatus[6];
	txBuffer[10] = solenoidStatus[7];
	txBuffer[11] = solenoidStatus[8];
	txBuffer[12] = solenoidStatus[9];
	txBuffer[13] = checkSum();
	txBuffer[14] = 0x03;
	HAL_UART_Transmit(&huart4, (uint8_t*) txBuffer, sizeof(txBuffer), 10);
}

uint8_t checkSum() {
	return (rxBuffer[1] ^ rxBuffer[2] ^ rxBuffer[3] ^ rxBuffer[4] ^ rxBuffer[5]);
}
uint8_t readAddress() {
	address[0] = 0x00;
	address[0] |= (HAL_GPIO_ReadPin(ADD1_GPIO_Port, ADD1_Pin) == GPIO_PIN_SET)
			<< 0;
	address[0] |= (HAL_GPIO_ReadPin(ADD2_GPIO_Port, ADD2_Pin) == GPIO_PIN_SET)
			<< 1;
	address[0] |= (HAL_GPIO_ReadPin(ADD3_GPIO_Port, ADD3_Pin) == GPIO_PIN_SET)
			<< 2;
	address[0] |= (HAL_GPIO_ReadPin(ADD4_GPIO_Port, ADD4_Pin) == GPIO_PIN_SET)
			<< 3;
	return address[0];
}
uint8_t checkSumReply() {
	return (txBuffer[1] ^ txBuffer[2] ^ txBuffer[3] ^ txBuffer[4] ^ txBuffer[5]
			^ txBuffer[6] ^ txBuffer[7] ^ txBuffer[8] ^ txBuffer[9]
			^ txBuffer[10] ^ txBuffer[11] ^ txBuffer[12]);
}

void solenoidControl(uint8_t solenoidNo) {
	switch (solenoidNo) {
	case 0x31:
		HAL_GPIO_WritePin(S01_GPIO_Port, S01_Pin, GPIO_PIN_SET);
		osDelay(solenoidTimeDelay);
		HAL_GPIO_WritePin(S01_GPIO_Port, S01_Pin, GPIO_PIN_RESET);
		break;
	case 0x32:
		HAL_GPIO_WritePin(S02_GPIO_Port, S02_Pin, GPIO_PIN_SET);
		osDelay(solenoidTimeDelay);
		HAL_GPIO_WritePin(S02_GPIO_Port, S02_Pin, GPIO_PIN_RESET);
		break;
	case 0x33:
		HAL_GPIO_WritePin(S03_GPIO_Port, S03_Pin, GPIO_PIN_SET);
		osDelay(solenoidTimeDelay);
		HAL_GPIO_WritePin(S03_GPIO_Port, S03_Pin, GPIO_PIN_RESET);
		break;
	case 0x34:
		HAL_GPIO_WritePin(S04_GPIO_Port, S04_Pin, GPIO_PIN_SET);
		osDelay(solenoidTimeDelay);
		HAL_GPIO_WritePin(S04_GPIO_Port, S04_Pin, GPIO_PIN_RESET);
		break;
	case 0x35:
		HAL_GPIO_WritePin(S05_GPIO_Port, S05_Pin, GPIO_PIN_SET);
		osDelay(solenoidTimeDelay);
		HAL_GPIO_WritePin(S05_GPIO_Port, S05_Pin, GPIO_PIN_RESET);
		break;
	case 0x36:
		HAL_GPIO_WritePin(S06_GPIO_Port, S06_Pin, GPIO_PIN_SET);
		osDelay(solenoidTimeDelay);
		HAL_GPIO_WritePin(S06_GPIO_Port, S06_Pin, GPIO_PIN_RESET);
		break;
	case 0x37:
		HAL_GPIO_WritePin(S07_GPIO_Port, S07_Pin, GPIO_PIN_SET);
		osDelay(solenoidTimeDelay);
		HAL_GPIO_WritePin(S07_GPIO_Port, S07_Pin, GPIO_PIN_RESET);
		break;
	case 0x38:
		HAL_GPIO_WritePin(S08_GPIO_Port, S08_Pin, GPIO_PIN_SET);
		osDelay(solenoidTimeDelay);
		HAL_GPIO_WritePin(S08_GPIO_Port, S08_Pin, GPIO_PIN_RESET);
		break;
	case 0x39:
		HAL_GPIO_WritePin(S09_GPIO_Port, S09_Pin, GPIO_PIN_SET);
		osDelay(solenoidTimeDelay);
		HAL_GPIO_WritePin(S09_GPIO_Port, S09_Pin, GPIO_PIN_RESET);
		break;
	case 0x3A:
		HAL_GPIO_WritePin(S10_GPIO_Port, S10_Pin, GPIO_PIN_SET);
		osDelay(solenoidTimeDelay);
		HAL_GPIO_WritePin(S10_GPIO_Port, S10_Pin, GPIO_PIN_RESET);
		break;
	}

}

/* MPU Configuration */

void MPU_Config(void) {
	MPU_Region_InitTypeDef MPU_InitStruct = { 0 };

	/* Disables the MPU */
	HAL_MPU_Disable();

	/** Initializes and configures the Region and the memory to be protected
	 */
	MPU_InitStruct.Enable = MPU_REGION_ENABLE;
	MPU_InitStruct.Number = MPU_REGION_NUMBER0;
	MPU_InitStruct.BaseAddress = 0x0;
	MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
	MPU_InitStruct.SubRegionDisable = 0x87;
	MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
	MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
	MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
	MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
	MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
	MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

	HAL_MPU_ConfigRegion(&MPU_InitStruct);
	/* Enables the MPU */
	HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
 * @brief  Period elapsed callback in non blocking mode
 * @note   This function is called  when TIM6 interrupt took place, inside
 * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
 * a global variable "uwTick" used as application time base.
 * @param  htim : TIM handle
 * @retval None
 */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	/* USER CODE BEGIN Callback 0 */

	/* USER CODE END Callback 0 */
	if (htim->Instance == TIM6) {
		HAL_IncTick();
	}
	/* USER CODE BEGIN Callback 1 */

	/* USER CODE END Callback 1 */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
