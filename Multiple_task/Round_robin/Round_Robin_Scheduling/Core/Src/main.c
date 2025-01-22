/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2025 STMicroelectronics.
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "sr04.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "lcd_i2cModule.h"
#include "keypad_4x4.h"
#include "string.h"
#include "uart.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HCSR04_PERIOD_MS 50
#define SOUNDSS_PERIOD_MS 50
#define KEYBOARD_PERIOD_MS 500
#define LCD_PERIOD_MS 250
#define UART_PERIOD_MS 50

#define HCSR04_DEADLINE_MS 50
#define SOUNDSS_DEADLINE_MS 50
#define KEYBOARD_DEADLINE_MS 100
#define LCD_DEADLINE_MS 250
#define UART_DEADLINE_MS 50

#define HCSR04_EXCUTION_MS 3
#define SOUNDSS_EXCUTION_MS 1
#define KEYBOARD_EXCUTION_MS 3
#define LCD_EXCUTION_MS 10
#define UART_EXCUTION_MS 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* Define constants */
#define TIME_QUANTUM_MS 10 // Th?i gian quantum cho m?i task (ms)
#define TASK_COUNT 5       // S? lu?ng task

/* Global variables */
uint32_t currentTask = 0; // Task hi?n t?i trong vòng l?p round robin
uint32_t taskStartTime = 0;
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart2;

osThreadId HCSR04_TaskHandle;
osThreadId task_SoundSSHandle;
osThreadId Keypad_TaskHandle;
osThreadId LCD_TaskHandle;
osThreadId UART_TaskHandle;
/* USER CODE BEGIN PV */
sr04_t sr04;

float khoang_cach = 0;

uint8_t sound = 0;
uint8_t check = 0;

QueueHandle_t QueueHCSR04Handle; // khoi tao mot Queue (tuong tu de khoi tao mot task thi ta co lenh: TaskHandle_t)
QueueHandle_t QueueSoundSSHandle;
QueueHandle_t QueueKeyboardHandle;
QueueHandle_t QueueUARTHandle;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART2_UART_Init(void);
void HCSR04(void const * argument);
void Soundss(void const * argument);
void Keypad(void const * argument);
void LCD(void const * argument);
void UART(void const * argument);

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
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

	//Init HCSR04
  sr04.trig_port = GPIOA;
  sr04.trig_pin = GPIO_PIN_9;
  sr04.echo_htim = &htim1;
  sr04.echo_channel = TIM_CHANNEL_1;
  sr04_init(&sr04);
	
	  //Init lcd
		LCD_i2cDeviceCheck();
		LCD_Init();
		LCD_BackLight(LCD_BL_ON);
		
		// Init Keypad
		keypad_init();
  /* USER CODE END 2 */

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
		QueueHCSR04Handle = xQueueCreate(10, sizeof(uint32_t)*10); // tao mot queue moi
	QueueSoundSSHandle = xQueueCreate(10, sizeof(char) * 10);
	QueueKeyboardHandle = xQueueCreate(10, sizeof(char) * 10);
	QueueUARTHandle = xQueueCreate(10, sizeof(uint32_t) * 10);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of HCSR04_Task */
  osThreadDef(HCSR04_Task, HCSR04, osPriorityNormal, 0, 128);
  HCSR04_TaskHandle = osThreadCreate(osThread(HCSR04_Task), NULL);

  /* definition and creation of task_SoundSS */
  osThreadDef(task_SoundSS, Soundss, osPriorityNormal, 0, 128);
  task_SoundSSHandle = osThreadCreate(osThread(task_SoundSS), NULL);

  /* definition and creation of Keypad_Task */
  osThreadDef(Keypad_Task, Keypad, osPriorityNormal, 0, 128);
  Keypad_TaskHandle = osThreadCreate(osThread(Keypad_Task), NULL);

  /* definition and creation of LCD_Task */
  osThreadDef(LCD_Task, LCD, osPriorityNormal, 0, 128);
  LCD_TaskHandle = osThreadCreate(osThread(LCD_Task), NULL);

  /* definition and creation of UART_Task */
  osThreadDef(UART_Task, UART, osPriorityNormal, 0, 128);
  UART_TaskHandle = osThreadCreate(osThread(UART_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB0 PB1 PB10 PB11 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
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
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_HCSR04 */
/**
  * @brief  Function implementing the HCSR04_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_HCSR04 */
void HCSR04(void const * argument)
{
  /* USER CODE BEGIN 5 */
	char message[50];
  /* Infinite loop */
  for(;;)
  {
    sr04_trigger(&sr04);
    khoang_cach = sr04.distance;
		snprintf(message, 50, "Distance: %.1fmm", khoang_cach);
		xQueueSend(QueueHCSR04Handle, &message, 100);
		xQueueSend(QueueUARTHandle, &message, 100);
    osDelay(pdMS_TO_TICKS(50));
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Soundss */
/**
* @brief Function implementing the task_SoundSS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Soundss */
void Soundss(void const * argument)
{
  /* USER CODE BEGIN Soundss */
	char message[10];
  /* Infinite loop */
  for(;;)
  {
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_10) == 0){
			// co am thanh
		xQueueSend(QueueSoundSSHandle, &message, 100);
			xQueueSend(QueueUARTHandle, &message, 100);
			strcpy(message, "YES ");
			
		}
		else {
			// khong co am thanh
			strcpy(message, "NO  ");
			xQueueSend(QueueSoundSSHandle, &message, 100);
			xQueueSend(QueueUARTHandle, &message, 100);
		}
    osDelay(pdMS_TO_TICKS(SOUNDSS_PERIOD_MS));
  }
  /* USER CODE END Soundss */
}

/* USER CODE BEGIN Header_Keypad */
/**
* @brief Function implementing the Keypad_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Keypad */
void Keypad(void const * argument)
{
  /* USER CODE BEGIN Keypad */
	char key_val;
	char buffer[20];
  /* Infinite loop */
  for(;;)
  {
    // Xu ly task doc Keyboard
		key_val = keypad_get_key_value();
		sprintf(buffer, "Ky tu: %c", key_val);
		xQueueSend(QueueKeyboardHandle, &buffer, 100);
		xQueueSend(QueueUARTHandle, &buffer, 100);
    osDelay(pdMS_TO_TICKS(KEYBOARD_PERIOD_MS));
  }
  /* USER CODE END Keypad */
}

/* USER CODE BEGIN Header_LCD */
/**
* @brief Function implementing the LCD_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_LCD */
void LCD(void const * argument)
{
  /* USER CODE BEGIN LCD */
  /* Infinite loop */
  for(;;)
  {
    char buf[50];
		char buf1[10] = "    ";
		char buf2[10];
		// Xu ly task hien thi LCD
		xQueueReceive(QueueHCSR04Handle, buf, portMAX_DELAY);
		xQueueReceive(QueueSoundSSHandle, buf1, portMAX_DELAY);
		xQueueReceive(QueueKeyboardHandle, buf2, portMAX_DELAY);
		LCD_SetCursor(1, 1);
		LCD_Send_String(buf, STR_NOSLIDE);
		LCD_SetCursor(2, 1);
		LCD_Send_String(buf1, STR_NOSLIDE);
		LCD_SetCursor(2, 7);
		LCD_Send_String(buf2, STR_NOSLIDE);  
		
    osDelay(pdMS_TO_TICKS(LCD_PERIOD_MS));
  }
  /* USER CODE END LCD */
}

/* USER CODE BEGIN Header_UART */
/**
* @brief Function implementing the UART_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UART */
void UART(void const * argument)
{
  /* USER CODE BEGIN UART */
	char uart_buf[100];
  /* Infinite loop */
  for(;;)
  {
    // Xu ly task truyen thong tin len may tinh su dung USB to TTL CP2102
		 if (xQueueReceive(QueueUARTHandle, uart_buf, portMAX_DELAY) == pdPASS)
        {
					strcat(uart_buf, "\n");
            HAL_UART_Transmit(&huart2, (uint8_t *)uart_buf, strlen(uart_buf), HAL_MAX_DELAY);
        }
    osDelay(pdMS_TO_TICKS(UART_PERIOD_MS));
  }
  /* USER CODE END UART */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
