/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticQueue_t osStaticMessageQDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
/* USER CODE BEGIN PTD */

//Создаем наш тип данных для передачи по UART
typedef struct {
	char Buf[128];
	} QUEUE_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
uint32_t defaultTaskBuffer[ 128 ];
osStaticThreadDef_t defaultTaskControlBlock;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .cb_mem = &defaultTaskControlBlock,
  .cb_size = sizeof(defaultTaskControlBlock),
  .stack_mem = &defaultTaskBuffer[0],
  .stack_size = sizeof(defaultTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for Led1_Task */
osThreadId_t Led1_TaskHandle;
uint32_t Led1_TaskBuffer[ 128 ];
osStaticThreadDef_t Led1_TaskControlBlock;
const osThreadAttr_t Led1_Task_attributes = {
  .name = "Led1_Task",
  .cb_mem = &Led1_TaskControlBlock,
  .cb_size = sizeof(Led1_TaskControlBlock),
  .stack_mem = &Led1_TaskBuffer[0],
  .stack_size = sizeof(Led1_TaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Led2_Task */
osThreadId_t Led2_TaskHandle;
uint32_t Led2_TaskBuffer[ 128 ];
osStaticThreadDef_t Led2_TaskControlBlock;
const osThreadAttr_t Led2_Task_attributes = {
  .name = "Led2_Task",
  .cb_mem = &Led2_TaskControlBlock,
  .cb_size = sizeof(Led2_TaskControlBlock),
  .stack_mem = &Led2_TaskBuffer[0],
  .stack_size = sizeof(Led2_TaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ReadBtn_Task */
osThreadId_t ReadBtn_TaskHandle;
uint32_t ReadBtn_TaskBuffer[ 128 ];
osStaticThreadDef_t ReadBtn_TaskControlBlock;
const osThreadAttr_t ReadBtn_Task_attributes = {
  .name = "ReadBtn_Task",
  .cb_mem = &ReadBtn_TaskControlBlock,
  .cb_size = sizeof(ReadBtn_TaskControlBlock),
  .stack_mem = &ReadBtn_TaskBuffer[0],
  .stack_size = sizeof(ReadBtn_TaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Led3_Task */
osThreadId_t Led3_TaskHandle;
uint32_t Led3_TaskBuffer[ 128 ];
osStaticThreadDef_t Led3_TaskControlBlock;
const osThreadAttr_t Led3_Task_attributes = {
  .name = "Led3_Task",
  .cb_mem = &Led3_TaskControlBlock,
  .cb_size = sizeof(Led3_TaskControlBlock),
  .stack_mem = &Led3_TaskBuffer[0],
  .stack_size = sizeof(Led3_TaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for ADC_Task */
osThreadId_t ADC_TaskHandle;
uint32_t ADC_TaskBuffer[ 128 ];
osStaticThreadDef_t ADC_TaskControlBlock;
const osThreadAttr_t ADC_Task_attributes = {
  .name = "ADC_Task",
  .cb_mem = &ADC_TaskControlBlock,
  .cb_size = sizeof(ADC_TaskControlBlock),
  .stack_mem = &ADC_TaskBuffer[0],
  .stack_size = sizeof(ADC_TaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for UART_Task */
osThreadId_t UART_TaskHandle;
uint32_t UART_TaskBuffer[ 128 ];
osStaticThreadDef_t UART_TaskControlBlock;
const osThreadAttr_t UART_Task_attributes = {
  .name = "UART_Task",
  .cb_mem = &UART_TaskControlBlock,
  .cb_size = sizeof(UART_TaskControlBlock),
  .stack_mem = &UART_TaskBuffer[0],
  .stack_size = sizeof(UART_TaskBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Uart_Queue */
osMessageQueueId_t Uart_QueueHandle;
uint8_t Uart_QueueBuffer[ 10 * sizeof( QUEUE_t ) ];
osStaticMessageQDef_t Uart_QueueControlBlock;
const osMessageQueueAttr_t Uart_Queue_attributes = {
  .name = "Uart_Queue",
  .cb_mem = &Uart_QueueControlBlock,
  .cb_size = sizeof(Uart_QueueControlBlock),
  .mq_mem = &Uart_QueueBuffer,
  .mq_size = sizeof(Uart_QueueBuffer)
};
/* Definitions for BtnSem */
osSemaphoreId_t BtnSemHandle;
osStaticSemaphoreDef_t BtnSemControlBlock;
const osSemaphoreAttr_t BtnSem_attributes = {
  .name = "BtnSem",
  .cb_mem = &BtnSemControlBlock,
  .cb_size = sizeof(BtnSemControlBlock),
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
void StartDefaultTask(void *argument);
void StartLed1_Task(void *argument);
void StartLed2_Task(void *argument);
void StartReadBtn_Task(void *argument);
void StartLed3_Task(void *argument);
void StartADC_Task(void *argument);
void StartUART_Task(void *argument);

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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_ADC_Start(&hadc1);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of BtnSem */
  BtnSemHandle = osSemaphoreNew(1, 0, &BtnSem_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of Uart_Queue */
  Uart_QueueHandle = osMessageQueueNew (10, sizeof(QUEUE_t), &Uart_Queue_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of Led1_Task */
  Led1_TaskHandle = osThreadNew(StartLed1_Task, NULL, &Led1_Task_attributes);

  /* creation of Led2_Task */
  Led2_TaskHandle = osThreadNew(StartLed2_Task, NULL, &Led2_Task_attributes);

  /* creation of ReadBtn_Task */
  ReadBtn_TaskHandle = osThreadNew(StartReadBtn_Task, NULL, &ReadBtn_Task_attributes);

  /* creation of Led3_Task */
  Led3_TaskHandle = osThreadNew(StartLed3_Task, NULL, &Led3_Task_attributes);

  /* creation of ADC_Task */
  ADC_TaskHandle = osThreadNew(StartADC_Task, NULL, &ADC_Task_attributes);

  /* creation of UART_Task */
  UART_TaskHandle = osThreadNew(StartUART_Task, NULL, &UART_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

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
  hadc1.Init.ContinuousConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_8;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 300;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4095;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  HAL_GPIO_WritePin(GPIOA, Led1_Pin|Led2_Pin|Led3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : Btn_Pin */
  GPIO_InitStruct.Pin = Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Led1_Pin Led2_Pin Led3_Pin */
  GPIO_InitStruct.Pin = Led1_Pin|Led2_Pin|Led3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLed1_Task */
/**
* @brief Function implementing the Led1_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLed1_Task */
void StartLed1_Task(void *argument)
{
  /* USER CODE BEGIN StartLed1_Task */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(Led1_GPIO_Port, Led1_Pin);
    osDelay(100);
  }
  /* USER CODE END StartLed1_Task */
}

/* USER CODE BEGIN Header_StartLed2_Task */
/**
* @brief Function implementing the Led2_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLed2_Task */
void StartLed2_Task(void *argument)
{
  /* USER CODE BEGIN StartLed2_Task */
	QUEUE_t msg;
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(Led2_GPIO_Port, Led2_Pin);
	  strcpy(msg.Buf,"Led2 blink with delay 100mc\r\n\0");
	  osMessageQueuePut(Uart_QueueHandle, &msg,0 , osWaitForever);				//Отправляем строку в очередь
	  osDelay(100);
  }
  /* USER CODE END StartLed2_Task */
}

/* USER CODE BEGIN Header_StartReadBtn_Task */
/**
* @brief Function implementing the ReadBtn_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReadBtn_Task */
void StartReadBtn_Task(void *argument)
{
  /* USER CODE BEGIN StartReadBtn_Task */
	QUEUE_t message;
  /* Infinite loop */
  for(;;)
  {
	if(HAL_GPIO_ReadPin(Btn_GPIO_Port, Btn_Pin))
		{
			osSemaphoreRelease(BtnSemHandle); //Даем семафору 1
			strcpy(message.Buf, "Btn presses\r\n\0"); //
			osMessageQueuePut(Uart_QueueHandle, &message,0 , osWaitForever);		//Отправляем строку в очередь
		}
    osDelay(100);
  }
  /* USER CODE END StartReadBtn_Task */
}

/* USER CODE BEGIN Header_StartLed3_Task */
/**
* @brief Function implementing the Led3_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLed3_Task */
void StartLed3_Task(void *argument)
{
  /* USER CODE BEGIN StartLed3_Task */
  /* Infinite loop */
  for(;;)
  {
	if(osSemaphoreAcquire(BtnSemHandle, osWaitForever) == osOK) // osWaitForever -  бесконечное ожидание
		{
		HAL_GPIO_TogglePin(Led3_GPIO_Port, Led3_Pin);
		}
    osDelay(1000);
  }
  /* USER CODE END StartLed3_Task */
}

/* USER CODE BEGIN Header_StartADC_Task */
/**
* @brief Function implementing the ADC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartADC_Task */
void StartADC_Task(void *argument)
{
  /* USER CODE BEGIN StartADC_Task */
	QUEUE_t adc_msg;
  /* Infinite loop */
  for(;;)
  {
	uint16_t adc_res = HAL_ADC_GetValue(&hadc1); 			// записывае значение с АЦП
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3,adc_res);	// передаем значение adc_res в модуль ШИМ (Таймер 2, канал 3)

	if(adc_res == 0)
		{
			strcpy(adc_msg.Buf, "Value ADC = 0\r\n\0");
			osMessageQueuePut(Uart_QueueHandle, &adc_msg,0 , osWaitForever);//Отправляем строку в очередь
			osDelay(200);
		}
	else if(adc_res >= 4000)
		{
			strcpy(adc_msg.Buf, "Value ADC is max\r\n\0");
			osMessageQueuePut(Uart_QueueHandle, &adc_msg,0 , osWaitForever);//Отправляем строку в очередь
			osDelay(200);
		}
    osDelay(100);
  }
  /* USER CODE END StartADC_Task */
}

/* USER CODE BEGIN Header_StartUART_Task */
/**
* @brief Function implementing the UART_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartUART_Task */
void StartUART_Task(void *argument)
{
  /* USER CODE BEGIN StartUART_Task */
	QUEUE_t message;
  /* Infinite loop */
  for(;;)
  {
	osMessageQueueGet(Uart_QueueHandle, &message, 0 , osWaitForever);	//
	HAL_UART_Transmit(&huart1, (uint8_t*)message.Buf, strlen(message.Buf), osWaitForever);//Отправляем полученные данные по UART
    osDelay(1);
  }
  /* USER CODE END StartUART_Task */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
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
