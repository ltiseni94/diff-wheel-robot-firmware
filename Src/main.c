/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "lwip.h"

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

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* Definitions for Main_Task */
osThreadId_t Main_TaskHandle;
const osThreadAttr_t Main_Task_attributes = {
  .name = "Main_Task",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 512 * 4
};
/* Definitions for Blink_LED */
osThreadId_t Blink_LEDHandle;
const osThreadAttr_t Blink_LED_attributes = {
  .name = "Blink_LED",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 256 * 4
};
/* Definitions for UDP_Communicati */
osThreadId_t UDP_CommunicatiHandle;
const osThreadAttr_t UDP_Communicati_attributes = {
  .name = "UDP_Communicati",
  .priority = (osPriority_t) osPriorityHigh,
  .stack_size = 1024 * 4
};
/* Definitions for Comp_Wheel_Spds */
osThreadId_t Comp_Wheel_SpdsHandle;
const osThreadAttr_t Comp_Wheel_Spds_attributes = {
  .name = "Comp_Wheel_Spds",
  .priority = (osPriority_t) osPriorityRealtime1,
  .stack_size = 1024 * 4
};
/* USER CODE BEGIN PV */

uint8_t V_R_hB=0;
uint8_t V_R_lB=0;
uint8_t V_L_hB=0;
uint8_t V_L_lB=0;

uint8_t MotorEnable=0;
uint8_t WriteMotorState=0;
uint8_t LampEnable=0;

long int mywatchdog=500;

UART_HandleTypeDef *SERIALDEBUG = &huart3;
UART_HandleTypeDef *MOTOR_R = &huart4;
UART_HandleTypeDef *MOTOR_L = &huart5;

float speed_R;
float speed_L;

/*
 * Motor R -> UART 4 -> TIM 1
 * Motor L -> UART 5 -> TIM 4
 */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM4_Init(void);
static void MX_UART4_Init(void);
static void MX_UART5_Init(void);
void MainTask(void *argument);
void BlinkLED(void *argument);
void UDPcommunication(void *argument);
void CompWheelSpeeds(void *argument);

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
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM1_Init();
  MX_TIM4_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, 0);
  HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);

  uint8_t numofstartupblinks;
  for (numofstartupblinks=0; numofstartupblinks<3; numofstartupblinks++) {
  	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
  	  HAL_Delay(50);
  	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
  	  HAL_Delay(50);

  	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1);
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
  	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
  	  HAL_Delay(100);
  	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
  	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
  	  HAL_Delay(100);

  	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1);
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 1);
  	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 1);
  	  HAL_Delay(100);
  	  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
  	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
  	  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);
  	  HAL_Delay(100);
  }


  HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 0);
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, 0);
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, 0);


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
  /* creation of Main_Task */
  Main_TaskHandle = osThreadNew(MainTask, NULL, &Main_Task_attributes);

  /* creation of Blink_LED */
  Blink_LEDHandle = osThreadNew(BlinkLED, NULL, &Blink_LED_attributes);

  /* creation of UDP_Communicati */
  UDP_CommunicatiHandle = osThreadNew(UDPcommunication, NULL, &UDP_Communicati_attributes);

  /* creation of Comp_Wheel_Spds */
  Comp_Wheel_SpdsHandle = osThreadNew(CompWheelSpeeds, NULL, &Comp_Wheel_Spds_attributes);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 400;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3|RCC_PERIPHCLK_UART4
                              |RCC_PERIPHCLK_UART5|RCC_PERIPHCLK_CLK48;
  PeriphClkInitStruct.PLLSAI.PLLSAIN = 192;
  PeriphClkInitStruct.PLLSAI.PLLSAIR = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
  PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV4;
  PeriphClkInitStruct.PLLSAIDivQ = 1;
  PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_2;
  PeriphClkInitStruct.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart4ClockSelection = RCC_UART4CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Uart5ClockSelection = RCC_UART5CLKSOURCE_PCLK1;
  PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLLSAIP;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 5;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 5;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 57600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 57600;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_OTG_FS.Init.dma_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD1_Pin|LD3_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USB_PowerSwitchOn_GPIO_Port, USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RELAY_GPIO_Port, RELAY_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : USER_Btn_Pin */
  GPIO_InitStruct.Pin = USER_Btn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USER_Btn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD1_Pin LD3_Pin LD2_Pin */
  GPIO_InitStruct.Pin = LD1_Pin|LD3_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USB_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RELAY_Pin */
  GPIO_InitStruct.Pin = RELAY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RELAY_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void recv_fun(void *arg, struct udp_pcb *pcb, struct pbuf *datarecv, const ip_addr_t *addr, uint16_t port){
	mywatchdog=500;
	V_R_lB=pbuf_get_at(datarecv,0);
	V_R_hB=pbuf_get_at(datarecv,1);
	V_L_lB=pbuf_get_at(datarecv,2);
	V_L_hB=pbuf_get_at(datarecv,3);
	LampEnable=(pbuf_get_at(datarecv,4)&0x08)>>3;
	MotorEnable=(pbuf_get_at(datarecv,4)&0x02)>>1;
	WriteMotorState=(pbuf_get_at(datarecv,4)&0x01);
	pbuf_free(datarecv);
}

char checksumbyte(char addressbyte, char highvaluebyte, char lowvaluebyte){
	return addressbyte+highvaluebyte+lowvaluebyte;
}

void start_motor(UART_HandleTypeDef *huart){
	/*SENDING MESSAGE*/
	uint8_t address=0x00;
	uint8_t message[4]={address,0x00,0x01,0x01};
	HAL_UART_Transmit(huart,message,4,3);
}

void stop_motor(UART_HandleTypeDef *huart){
	/*SENDING MESSAGE*/
	uint8_t address=0x00;
	uint8_t message[4]={address,0x00,0x00,0x00};
	HAL_UART_Transmit(huart,message,4,3);
}

void motor_setPCspeedcontrolmode(UART_HandleTypeDef *huart){
	/*SENDING MESSAGE*/
	uint8_t address=0x02;
	uint8_t message[4]={address,0x00,0xc4,0xc6};
	HAL_UART_Transmit(huart,message,4,3);
}

void set_accelerationdeceleration(UART_HandleTypeDef *huart, char accel, char decel){
	/*SENDING MESSAGE*/
	uint8_t address=0x0a;
	uint8_t message[4]={address,accel,decel,checksumbyte(address,accel,decel)};
	HAL_UART_Transmit(huart,message,4,3);
}

void set_motorspeed(UART_HandleTypeDef *huart, char highvaluebyte, char lowvaluebyte){
	/*SENDING MESSAGE*/
	uint8_t address=0x06;
	uint8_t message[4]={address,highvaluebyte,lowvaluebyte,checksumbyte(address,highvaluebyte,lowvaluebyte)};
	HAL_UART_Transmit(huart,message,4,3);
}

void set_speedlimitvalue(UART_HandleTypeDef *huart, char highvaluebyte, char lowvaluebyte){
	/*SENDING MESSAGE*/
	uint8_t address=0x1d;
	uint8_t message[4]={address,highvaluebyte,lowvaluebyte,checksumbyte(address,highvaluebyte,lowvaluebyte)};
	HAL_UART_Transmit(huart,message,4,3);
}

void cleardriverfault(UART_HandleTypeDef *huart){
	/*SENDING MESSAGE*/
	uint8_t address=0x4a;
	uint8_t message[4]={address,0x00,0x00,address};
	HAL_UART_Transmit(huart,message,4,3);
}

void debugwithserial(UART_HandleTypeDef *huart){
	  uint8_t buffer[128];
	  short int dig_command_R=(short int)((V_R_hB<<8)|V_R_lB);
	  short int dig_command_L=(short int)((V_L_hB<<8)|V_L_lB);
	  int len=sprintf((char*)(buffer),"debug:\t%hi\t%hi\t%i\t%lu\t%lu\n",dig_command_R,dig_command_L,MotorEnable,htim1.Instance->CNT,htim4.Instance->CNT);
	  HAL_UART_Transmit(huart,buffer,len,2*len);
}


/* USER CODE END 4 */

/* USER CODE BEGIN Header_MainTask */
/**
  * @brief  Function implementing the Main_Task thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_MainTask */
__weak void MainTask(void *argument)
{
  /* init code for LWIP */
  MX_LWIP_Init();
  /* USER CODE BEGIN 5 */

  stop_motor(MOTOR_R);
  stop_motor(MOTOR_L);

  uint8_t RelayPinValue;
  uint32_t mytick;
  mytick = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  if (mywatchdog<0){
		  HAL_GPIO_WritePin(RELAY_GPIO_Port,RELAY_Pin,0);
		  stop_motor(MOTOR_R);
		  stop_motor(MOTOR_L);
		  osDelay(2);
		  set_motorspeed(MOTOR_R,0,0);
		  set_motorspeed(MOTOR_L,0,0);
		  MotorEnable=0;
		  LampEnable=0;

	  }
	  else{
		  if (MotorEnable){
			  if (WriteMotorState){
				  start_motor(MOTOR_R);
				  start_motor(MOTOR_L);
				  osDelay(2);
			  }
			  set_motorspeed(MOTOR_R,V_R_hB,V_R_lB);
			  set_motorspeed(MOTOR_L,V_L_hB,V_L_lB);
		  }
		  else{
			  stop_motor(MOTOR_R);
			  stop_motor(MOTOR_L);
		  }
		  RelayPinValue=LampEnable;
		  HAL_GPIO_WritePin(RELAY_GPIO_Port,RELAY_Pin,RelayPinValue);
	  }
	  /*debugwithserial(SERIALDEBUG);*/
      mytick += 25U;
      osDelayUntil(mytick);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_BlinkLED */
/**6
* @brief Function implementing the Blink_LED thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_BlinkLED */
__weak void BlinkLED(void *argument)
{
  /* USER CODE BEGIN BlinkLED */
	uint32_t mytick3;
	mytick3 = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	  mytick3+=500U;
	  HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin,1);
	  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,1);
	  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,1);
	  osDelayUntil(mytick3);
	  mytick3+=500U;
	  HAL_GPIO_WritePin(LD1_GPIO_Port,LD1_Pin,0);
	  HAL_GPIO_WritePin(LD2_GPIO_Port,LD2_Pin,0);
	  HAL_GPIO_WritePin(LD3_GPIO_Port,LD3_Pin,0);
	  osDelayUntil(mytick3);
  }
  /* USER CODE END BlinkLED */
}

/* USER CODE BEGIN Header_UDPcommunication */
/**
* @brief Function implementing the UDP_Communicati thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_UDPcommunication */
__weak void UDPcommunication(void *argument)
{
  /* USER CODE BEGIN UDPcommunication */
	  /* REMOTE ADDRESS DEFINITION */
	  struct ip4_addr computer_address;
	  IP4_ADDR(&computer_address,192,168,0,100);

	  /* UDP INITIALIZATION */
	  struct udp_pcb *hudp;
	  hudp=udp_new();
	  udp_bind(hudp,IP4_ADDR_ANY,9999);
	  udp_recv(hudp,recv_fun,NULL);

	  /* INITIALIZE TIME VARIABLE mytickN (TICK UNITS ms) */
	  uint32_t mytick2;
	  mytick2 = osKernelGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  struct pbuf *datasend;
	  datasend = pbuf_alloc(PBUF_TRANSPORT,10,PBUF_RAM);
	  char dati[10];

	  memcpy(&dati[0],(uint8_t*)&speed_R,4);
	  memcpy(&dati[4],(uint8_t*)&speed_L,4);
	  dati[8]=MotorEnable;
	  dati[9]=LampEnable;

	  memcpy(datasend->payload,dati,10);
	  udp_sendto(hudp,datasend,&computer_address,11111);
      pbuf_free(datasend);
      mywatchdog-=10;
      mytick2 += 10U;
      osDelayUntil(mytick2);
  }
  /* USER CODE END UDPcommunication */
}

/* USER CODE BEGIN Header_CompWheelSpeeds */
/**
* @brief Function implementing the Comp_Wheel_Spds thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_CompWheelSpeeds */
__weak void CompWheelSpeeds(void *argument)
{
  /* USER CODE BEGIN CompWheelSpeeds */

	/* INITIALIZE TIME VARIABLE mytickN (TICK UNITS ms) */
	uint32_t mytick4;

	/* DEFINE CUSTOM ENCODER STRUCTURE */
	struct my_encoder {
		int32_t Old_Angle;
		int32_t New_Angle;
		int32_t Old_Count;
		int32_t New_Count;
		int32_t Rep_Counter;
	};

	/* INITIALIZE ENCODER STRUCTURES */
	struct my_encoder enc_R = {0, 0, 0, 0, 0};
	struct my_encoder enc_L = {0, 0, 0, 0, 0};

	/* INITIALIZE AUXILIARY VARIABLES */
	float speed_R_old=0;
	float speed_L_old=0;

	/* FILTER PARAMETERS DEFINITION */
	float Ts=0.01;
	float Tf=0.05;
	float a = Ts/(Ts+Tf);
	float b = Tf/(Ts+Tf);

	/* GET ACTUAL TIME */
	mytick4 = osKernelGetTickCount();

  /* Infinite loop */
  for(;;)
  {
	/* STORE ENCODER READING INTO VARIABLE */
    enc_R.New_Count=(int32_t)(htim1.Instance->CNT);
    enc_L.New_Count=(int32_t)(htim4.Instance->CNT);

    /* REPETITION COUNTER */
    if ((enc_R.New_Count-enc_R.Old_Count)<(-32768)) {
    	(enc_R.Rep_Counter)++;
    }
    if ((enc_R.New_Count-enc_R.Old_Count)>(32768)) {
    	(enc_R.Rep_Counter)--;
    }
    if ((enc_L.New_Count-enc_L.Old_Count)<(-32768)) {
    	(enc_L.Rep_Counter)++;
    }
    if ((enc_L.New_Count-enc_L.Old_Count)>(32768)) {
        (enc_L.Rep_Counter)--;
    }

    /* CALCULATE NEW WHEEL COUNTS */
    enc_R.New_Angle = enc_R.Rep_Counter*65536 + enc_R.New_Count;
    enc_L.New_Angle = enc_L.Rep_Counter*65536 + enc_L.New_Count;

    /* CALCULATE FILTERED SPEEDS */
    speed_R = a*speed_R_old + b*((float)(enc_R.New_Angle-enc_R.Old_Angle))/Ts;
    speed_L = a*speed_L_old + b*((float)(enc_L.New_Angle-enc_L.Old_Angle))/Ts;

    /* UPDATE OLD VARIABLES */
    enc_R.Old_Angle=enc_R.New_Angle;
    enc_R.Old_Count=enc_R.New_Count;
    enc_L.Old_Angle=enc_L.New_Angle;
    enc_L.Old_Count=enc_L.New_Count;
    speed_R_old=speed_R;
    speed_L_old=speed_L;

	/*REALTIME*/
    mytick4 += 10U;
    osDelayUntil(mytick4);
  }
  /* USER CODE END CompWheelSpeeds */
}

 /**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
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
