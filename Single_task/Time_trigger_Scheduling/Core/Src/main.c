/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
#include <stdio.h>
#include "lcd_i2cModule.h"
#include "keypad_4x4.h"
#include "sr04.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
sr04_t sr04;
float khoang_cach = 0;
char key_val, buffer[20];
uint8_t sound = 0;

char buffer_sr04[20];
char buffer_kp[20];
char buffer_sound[20];

#define HCSR04_PERIOD_MS 50
#define SOUNDSS_PERIOD_MS 50
#define KEYBOARD_PERIOD_MS 500
#define LCD_PERIOD_MS 250
#define UART_PERIOD_MS 50
#define TOTAL_CYCLE_MS 500

#define T1 50
#define T2 50
#define T3 500
#define T4 250
#define T5 50
#define T6 500

uint32_t start_time_task_1 = 0, end_time_task_1 = 0;
uint32_t start_time_task_2 = 0, end_time_task_2 = 0;
uint32_t start_time_task_3 = 0, end_time_task_3 = 0;
uint32_t start_time_task_4 = 0, end_time_task_4 = 0;
uint32_t start_time_task_5 = 0, end_time_task_5 = 0;


uint32_t last_tick_t1 = 0;
uint32_t last_tick_t2 = 0; 
uint32_t last_tick_t3 = 0;
uint32_t last_tick_t4 = 0;
uint32_t last_tick_t5 = 0;


/* Task intervals (in milliseconds) */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

void Task_HCSR04(void);
void Task_SoundSensor(void);
void Task_Keypad(void);
void time_HCSR04(void);
void time_SoundSensor(void);
void time_Keypad(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/*
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    UNUSED(huart);
    if(huart->Instance == USART1){
        if(u8_RxData != 13) {
            u8_RxBuff[_rxIndex++] = u8_RxData;
        }
        else if(u8_RxData == 13) {
            _rxIndex = 0;
            Tx_Flag = 1;
        }
        HAL_UART_Receive_IT(&huart2, &u8_RxData, 1);  
    }
}*/
 //Handle HCSR04
void handleHCSR04(void)
{
  static uint32_t lastHCSR04Time = 0;
  uint32_t currentTime = HAL_GetTick();
  
	//Kiem tra thoi gian tre de goi lai cam bien HCSR04 
  if (currentTime - lastHCSR04Time >= HCSR04_PERIOD_MS)
  {
    sr04_trigger(&sr04);
    khoang_cach = sr04.distance;
    snprintf(buffer_sr04, 50, "Distance: %.1fmm", khoang_cach);
    lastHCSR04Time = currentTime;
  }
}

//Handle Sound Sensor
	void handleSoundSensor(void)
{
  static uint32_t lastSoundTime = 0;
  uint32_t currentTime = HAL_GetTick();

  // Kiem tra thoi gian tre de doc c?m bien am thanh
  if (currentTime - lastSoundTime >= SOUNDSS_PERIOD_MS)
  {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET) {
      strcpy(buffer_sound, "YES ");
    }
    else {
      strcpy(buffer_sound, "NO ");
    }
    lastSoundTime = currentTime;
  }
}

//Handle keypad
void handleKeypad(void)
{
  static uint32_t lastKeypadTime = 0;
  uint32_t currentTime = HAL_GetTick();

  // Kiem tra thoi gian tre de doc ban phim
  if (currentTime - lastKeypadTime >= KEYBOARD_PERIOD_MS)
  {
    key_val = keypad_get_key_value();
    sprintf(buffer_kp, "Key: %c", key_val);
    lastKeypadTime = currentTime;
  }
}

//Handle LCD
void handleLCD(void)
{
	static uint32_t lastLCDTime = 0;
  uint32_t currentTime = HAL_GetTick();
	
	//Kiem tra thoi gian de hien thi LCD
	if (currentTime - lastLCDTime >= LCD_PERIOD_MS){
		//Hien thi khoang cach
		LCD_SetCursor(1, 1);
		LCD_Send_String(buffer_sr04, STR_NOSLIDE);
	
		//Hien thi trang thai am thanh
		LCD_SetCursor(2, 1);
		LCD_Send_String(buffer_sound, STR_NOSLIDE);
	
		//Hien thi keypad
		LCD_SetCursor(2, 7);
		LCD_Send_String(buffer_kp, STR_NOSLIDE);
		
		lastLCDTime = currentTime;
	}
}

//Handle UART
void handleUART(void)
{
	static uint32_t lastUARTTime = 0;
  uint32_t currentTime = HAL_GetTick();
	
	if (currentTime - lastUARTTime >= UART_PERIOD_MS){
		//Gui thong tin khoang cach len may tinh
		strcat(buffer_sr04, "\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)buffer_sr04, strlen(buffer_sr04), HAL_MAX_DELAY);
		
		//Gui trang thai am thanh len may tinh
		strcat(buffer_sound, "\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)buffer_sound, strlen(buffer_sound), HAL_MAX_DELAY);
		
		//Gui thong tin phim nhan keypad len may tinh
		strcat(buffer_kp, "\n");
		HAL_UART_Transmit(&huart2, (uint8_t*)buffer_kp, strlen(buffer_kp), HAL_MAX_DELAY);
		
		lastUARTTime = currentTime;
	}
}


/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint32_t last_execution_time = HAL_GetTick();
	uint32_t current_time;
	uint16_t f = 50;
	//uint32_t task_start_time; 
	//uint32_t task_execution_time; 
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
/* Init HCSR04 */
  sr04.trig_port = GPIOA;
  sr04.trig_pin = GPIO_PIN_9;
  sr04.echo_htim = &htim1;
  sr04.echo_channel = TIM_CHANNEL_1;
  sr04_init(&sr04);

  /* Init LCD */
  LCD_i2cDeviceCheck();
  LCD_Init();
  LCD_BackLight(LCD_BL_ON);

  /* Init keypad */
  keypad_init();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {  
		current_time = HAL_GetTick();
		if (current_time < T1) {
            // T1: HCSR04
            handleHCSR04();
        } else if (current_time >= T1 && current_time < T1 + T2) {
            // T2: SoundSensor
            handleSoundSensor();
        } else if (current_time >= T1 + T2 && current_time < T1 + T2 + T5) {
            // T5: UART
					handleUART();
				} else if (current_time >= T1 + T2 + T5 && current_time < T1 + T2 + T5 + T4) {
            // T4: LCD
            handleLCD();
				} else if (current_time >= T1 + T2 + T5 + T4 && current_time < 50) {
            HAL_Delay(19); }
				// 50
					else if (current_time >= f && current_time < f+ T1 ) {
            // T1: HCSR04
            handleHCSR04();
        } else if (current_time >= f + T1 && current_time < f + T1 + T2) {
            // T2: SoundSensor
            handleSoundSensor();
        } else if (current_time >= f +T1 + T2 && current_time < f + T1 + T2 + T5) {
            // T5: UART
					handleUART();
				} else if (current_time >= f + T1 + T2 + T5 && current_time < 2*f) {
            HAL_Delay(35); }
        //100 
        else if (current_time >= 2*f && current_time < 2*f+ T1 ) {
            // T1: HCSR04
            handleHCSR04();
        } else if (current_time >= 2*f + T1 && current_time < 2*f + T1 + T2) {
            // T2: SoundSensor
            handleSoundSensor();
        } else if (current_time >= 2*f +T1 + T2 && current_time < 2*f + T1 + T2 + T5) {
            // T5: UART
					  handleUART();
				} else if (current_time >= 2*f + T1 + T2 + T5 && current_time < 2*f + T1 + T2 + T5 +T3) {
            // T3: Keypad
            handleKeypad();
				} else if (current_time >= 2*f + T1 + T2 + T5 +T3 && current_time < 2*f + T1 + T2 + T5 +T3 + 126) {
            HAL_Delay(126); 
	      } else if (current_time >= 2*f + T1 + T2 + T5 +T3 + 126 && current_time < 2*f + 2*T1 + T2 + T5 +T3 + 126 ) {
            // T1: HCSR04
            handleHCSR04(); 
        } else if (current_time >= 2*f + 2*T1 + T2 + T5 +T3 + 126 && current_time < 2*f + 2*T1 + 2*T2 + T5 +T3 + 126) {
            // T2: SoundSensor
            handleSoundSensor();
        } else if (current_time >= 2*f + 2*T1 + 2*T2 + T5 +T3 + 126 && current_time < 2*f + 2*T1 + 2*T2 + 2*T5 +T3 + 126) {
            // T5: UART
					  handleUART();
				//200
				} else if (current_time >= 4*f && current_time < 4*f+ T1 ) {
            // T1: HCSR04
            handleHCSR04();
        } else if (current_time >= 4*f + T1 && current_time < 4*f + T1 + T2) {
            // T2: SoundSensor
            handleSoundSensor();
        } else if (current_time >= 4*f +T1 + T2 && current_time < 4*f + T1 + T2 + T5) {
            // T5: UART
					handleUART();
				} else if (current_time >= 4*f + T1 + T2 + T5 && current_time < 5*f) {
            HAL_Delay(35);
			  //250
				} else if (current_time < 5*f+T1) {
            // T1: HCSR04
            handleHCSR04();
        } else if (current_time >= 5*f+T1 && current_time < 5*f+T1 + T2) {
            // T2: SoundSensor
            handleSoundSensor();
        } else if (current_time >= 5*f+T1 + T2 && current_time < 5*f+T1 + T2 + T5) {
            // T5: UART
					handleUART();
				} else if (current_time >= 5*f+T1 + T2 + T5 && current_time < 5*f+T1 + T2 + T5 + T4) {
            // T4: LCD
            handleLCD();
				} else if (current_time >= 5*f+T1 + T2 + T5 + T4 && current_time < 6*f) {
            HAL_Delay(19);

				//300
					
				} else if (current_time >= 6*f && current_time < 6*f+ T1 ) {
            // T1: HCSR04
            handleHCSR04();
        } else if (current_time >= 6*f + T1 && current_time < 6*f + T1 + T2) {
            // T2: SoundSensor
            handleSoundSensor();
        } else if (current_time >= 6*f +T1 + T2 && current_time < 6*f + T1 + T2 + T5) {
            // T5: UART
					handleUART();
				} else if (current_time >= 6*f + T1 + T2 + T5 && current_time < 7*f) {
            HAL_Delay(35);
				//350
				} else if (current_time >= 7*f && current_time < 7*f+ T1 ) {
            // T1: HCSR04
            handleHCSR04();
        } else if (current_time >=7*f + T1 && current_time < 7*f + T1 + T2) {
            // T2: SoundSensor
            handleSoundSensor();
        } else if (current_time >= 7*f +T1 + T2 && current_time < 7*f + T1 + T2 + T5) {
            // T5: UART
					handleUART();
				} else if (current_time >= 7*f + T1 + T2 + T5 && current_time < 8*f) {
            HAL_Delay(35);	
				//400
				} else if (current_time >= 8*f && current_time < 8*f+ T1 ) {
            // T1: HCSR04
            handleHCSR04();
        } else if (current_time >=8*f + T1 && current_time < 8*f + T1 + T2) {
            // T2: SoundSensor
            handleSoundSensor();
        } else if (current_time >= 8*f +T1 + T2 && current_time < 8*f + T1 + T2 + T5) {
            // T5: UART
					handleUART();
				} else if (current_time >= 8*f + T1 + T2 + T5 && current_time < 9*f) {
            HAL_Delay(35);	
				//450
				} else if (current_time >= 9*f && current_time < 9*f+ T1 ) {
            // T1: HCSR04
            handleHCSR04();
        } else if (current_time >=9*f + T1 && current_time < 9*f + T1 + T2) {
            // T2: SoundSensor
            handleSoundSensor();
        } else if (current_time >= 9*f +T1 + T2 && current_time < 9*f + T1 + T2 + T5) {
            // T5: UART
					handleUART();
				} else if (current_time >= 9*f + T1 + T2 + T5 && current_time < TOTAL_CYCLE_MS) {
            HAL_Delay(35);
        }					
				//500
				if (current_time >= TOTAL_CYCLE_MS) {
            current_time = 0;  // Reset current_time về 0 để bắt đầu chu kỳ mới
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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 71;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 PB2 PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
    * @brief Task to handle HCSR04 distance measurement
    */
void Task_HCSR04(void)
  {
    sr04_trigger(&sr04);
    khoang_cach = sr04.distance;
  
    snprintf(buffer, 50, "Distance: %.1fmm", khoang_cach);
    LCD_SetCursor(1, 1);
    LCD_Send_String(buffer, STR_NOSLIDE);
  }
  
  /**
    * @brief Task to handle sound sensor
    */
void Task_SoundSensor(void)
  {
    if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == GPIO_PIN_RESET) {
      strcpy(buffer, "Sound: YES ");
      sound = 3;
    } else {
      strcpy(buffer, "Sound: NO ");
      sound = 8;
    }
  
    LCD_SetCursor(2, 1);
    LCD_Send_String(buffer, STR_NOSLIDE);
  }
  
  /**
    * @brief Task to handle keypad scanning
    */
void Task_Keypad(void)
  {
    key_val = keypad_get_key_value();
    if (key_val != 0) {
      sprintf(buffer, "Key: %c", key_val);
      LCD_SetCursor(2, 7);
      LCD_Send_String(buffer, STR_NOSLIDE);
    }
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
