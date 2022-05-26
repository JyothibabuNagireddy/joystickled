/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#define LED1 1
#define LED2 2
#define LED3 3
#define LED4 4
#define queue_length 3
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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

osThreadId CLIHandle;
osThreadId joyHandle;
osThreadId ledHandle;
/* USER CODE BEGIN PV */
#define RX_BUFF_SIZE	20
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
void c_l_i(void const * argument);
void joystick(void const * argument);
void leds(void const * argument);
xQueueHandle simplequeue;
xQueueHandle cli_queue;
/* USER CODE BEGIN PFP */
struct UART3_Rx
{
	bool fullFlag;
	bool cmdReady;
	uint8_t count;
	uint8_t buff[RX_BUFF_SIZE];
};
struct jtos
{
	uint8_t value;
    uint32_t delay;
};
struct jtos jtosBuff;
HAL_StatusTypeDef status = HAL_OK;
struct UART3_Rx tempBuff;
uint8_t* ptrTempBuff = &tempBuff.buff[0];
char *ptr;

char strBuff[128];
uint8_t rx_buff[RX_BUFF_SIZE];
uint8_t temp[RX_BUFF_SIZE];
uint16_t temp1;
uint8_t temp5;
bool softReset = false;
bool pinReset = false;

uint8_t goCmd[] = "d";
uint8_t rstCmd[] = "reset";
uint8_t reply[] = "\r\nTemp value passed to joystick...Now press any Joy button\r\n>";
uint8_t prompt[] = "\r\n>";
uint8_t cmdFail[] = "\r\nCommand not recognised...\r\n>";

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void printInfo()
{
  uint16_t flashSize = READ_REG(*((uint16_t *)FLASHSIZE_BASE));
  int HalVersion = HAL_GetHalVersion();
  int RevID = HAL_GetREVID();
  int DevID = HAL_GetDEVID();
  int UIDw0 = HAL_GetUIDw0();
  int UIDw1 = HAL_GetUIDw1();
  int UIDw2 = HAL_GetUIDw2();
  int HCLKF = HAL_RCC_GetHCLKFreq();

  // Wait for connection to Tera Term (or whatever) before printing
  HAL_Delay(500);

  if(pinReset)
  {
	 sprintf(&strBuff[0], "\r\nPin triggered reset occurred...");
	 status = HAL_UART_Transmit(&huart3, (uint8_t*)&strBuff[0], strlen(strBuff),100);
	 assert_param(status == HAL_OK);
  }

  if(softReset)
  {
	 sprintf(&strBuff[0], "\r\nSoftware triggered reset occurred...");
	 status = HAL_UART_Transmit(&huart3, (uint8_t*)&strBuff[0], strlen(strBuff),100);
	 assert_param(status == HAL_OK);
  }
  sprintf(&strBuff[0], "\r\n\r\nSTM32 HAL : L0_V%d.%d.%d (RC-%d)\r\n",
		   (HalVersion >> 24),
		   (HalVersion >> 16) & 0xFF,
		   (HalVersion >> 8) & 0xFF,
		    HalVersion & 0xFF);

	// Blocking calls used deliberately
  status = HAL_UART_Transmit(&huart3, (uint8_t*)&strBuff[0], strlen(strBuff),100);           //prints the STM32 HAL : L0_V1.4.4 (RC-0)
  assert_param(status == HAL_OK);
  sprintf(&strBuff[0], "Flash     : %d Kbytes\r\n", flashSize);
  status = HAL_UART_Transmit(&huart3, (uint8_t*)&strBuff[0], strlen(strBuff),100);           //prints the size of Flash : 128 Kbytes
  assert_param(status == HAL_OK);
  sprintf(&strBuff[0], "UID       : %d%d%d\r\n", UIDw2, UIDw1, UIDw0);
  status = HAL_UART_Transmit(&huart3, (uint8_t*)&strBuff[0], strlen(strBuff),100);           //prints the UID   : 5405532888256430194194424
  assert_param(status == HAL_OK);
  sprintf(&strBuff[0], "Dev ID    : 0x%03x\r\n", DevID);
  status = HAL_UART_Transmit(&huart3, (uint8_t*)&strBuff[0], strlen(strBuff),100);           //prints the Dev ID  : 0x460
  assert_param(status == HAL_OK);
  sprintf(&strBuff[0], "Rev ID    : 0x%04x\r\n", RevID);
  status = HAL_UART_Transmit(&huart3, (uint8_t*)&strBuff[0], strlen(strBuff),100);           //prints the Rev ID   : 0x2000
  assert_param(status == HAL_OK);
  sprintf(&strBuff[0], "HCLK      : %d Hz\r\n\n>", HCLKF);
  status = HAL_UART_Transmit(&huart3, (uint8_t*)&strBuff[0], strlen(strBuff),100);           //prints the clock HCLK  : 64000000 Hz
  assert_param(status == HAL_OK);
}
    //When data reception is finished, it will be called by interrupt handle function
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART3) {
    // We can accept the character
	if((tempBuff.count < RX_BUFF_SIZE) && (tempBuff.fullFlag == false)) {
	  // It's the end of a command
	  if(tempBuff.buff[tempBuff.count] == '\r') {
	    // Disable interrupts until we have processed the command
		status = HAL_UART_Abort(huart);
		assert_param(status == HAL_OK);

			    // Signal we have a command and reset pointer
		tempBuff.cmdReady = true;
		ptrTempBuff = &tempBuff.buff[0];
	  }
	  else // Assume character is good
	  {
		tempBuff.count += 1;
		ptrTempBuff++;
		// Enable reception again
		status = HAL_UART_Receive_IT(huart, ptrTempBuff, sizeof(uint8_t));
		assert_param(status == HAL_OK);
	  }
	}
	else // We can't accept it so start again
	{
	  // We ran out of buffer before getting a command!
	  status = HAL_UART_Abort(huart);
	  assert_param(status == HAL_OK);

	  // Signal an error condition, reset pointer
	  tempBuff.fullFlag = true;
	  ptrTempBuff = &tempBuff.buff[0];
	}
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
  tempBuff.fullFlag = false;                                         // we assign the fullflag=false,cmdready=false and count=0;
  tempBuff.cmdReady = false;
  tempBuff.count = 0;
  memset(&tempBuff.buff, 0x00, sizeof(tempBuff.buff));               //clear the buffer data otherwise garbage is there.

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  if __HAL_RCC_GET_FLAG(RCC_FLAG_SFTRST)
  {
    softReset = true;
  }
  if __HAL_RCC_GET_FLAG(RCC_FLAG_PINRST)
  {
    pinReset = true;
  }
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_UART_Transmit(&huart3,(uint8_t*)"Welcome to Joystick to Led\r\n",27,HAL_MAX_DELAY);
  memset(&tempBuff.buff, 0x00, sizeof(tempBuff.buff));
  HAL_Delay(200);
  simplequeue=xQueueCreate(queue_length,sizeof(int));                                                //create the message queue for transfer the information joystick to led.
  cli_queue=xQueueCreate(queue_length,sizeof(tempBuff));                                              //create the message queue for transfer the information command line interface to joystick.
  /*if(simplequeue==0)
  {
	char *str="unable to create queue\n";
	HAL_UART_Transmit(&huart3,(uint8_t*)str,strlen(str),HAL_MAX_DELAY);
  }
  else
  {
	char *str1="create queue\n";
	HAL_UART_Transmit(&huart3,(uint8_t*)str1,strlen(str1),HAL_MAX_DELAY);
  }*/

  status = HAL_UART_Receive_IT(&huart3, &tempBuff.buff[0], sizeof(uint8_t));
  assert_param(status == HAL_OK);

  // printInfo();

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
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of CLI */
  osThreadDef(CLI, c_l_i, osPriorityHigh, 0, 128);
  CLIHandle = osThreadCreate(osThread(CLI), NULL);

  /* definition and creation of joy */
  osThreadDef(joy, joystick, osPriorityAboveNormal, 0, 128);
  joyHandle = osThreadCreate(osThread(joy), NULL);

  /* definition and creation of led */
  osThreadDef(led, leds, osPriorityNormal, 0, 128);
  ledHandle = osThreadCreate(osThread(led), NULL);

  /* USER CODE BEGIN RTOS_THREADS */

  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();                                                                   //start the thread scheduler

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 // HAL_Delay(5);
    /* USER CODE END WHILE */
  }
    /* USER CODE BEGIN 3 */

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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SPI1_MOSI_DIR_Pin|EX_RESET_OD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MC_DissipativeBrake_Pin|LCD_CS_OD_Pin|MC_NTC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(MicroSD_CS_OD_GPIO_Port, MicroSD_CS_OD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SPI1_MOSI_DIR_Pin */
  GPIO_InitStruct.Pin = SPI1_MOSI_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_MOSI_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EX_RESET_OD_Pin */
  GPIO_InitStruct.Pin = EX_RESET_OD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(EX_RESET_OD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MC_PFCsync1_Pin */
  GPIO_InitStruct.Pin = MC_PFCsync1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_LPTIM1;
  HAL_GPIO_Init(MC_PFCsync1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 PC7 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : JOY_SEL_Pin */
  GPIO_InitStruct.Pin = JOY_SEL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(JOY_SEL_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MC_BusVoltage_Pin MC_CurrentA_Pin PA3 Audio_OUT_L_Pin
                           Audio_OUT_R_Pin Audio_IN_Pin */
  GPIO_InitStruct.Pin = MC_BusVoltage_Pin|MC_CurrentA_Pin|GPIO_PIN_3|Audio_OUT_L_Pin
                          |Audio_OUT_R_Pin|Audio_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_MOSI_Pin */
  GPIO_InitStruct.Pin = SPI1_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
  HAL_GPIO_Init(SPI1_MOSI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MC_EnIndex_Pin Potentiometer_Pin MC_CurrentC_Pin */
  GPIO_InitStruct.Pin = MC_EnIndex_Pin|Potentiometer_Pin|MC_CurrentC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MC_PFCpwm_Pin MC_EnB_Pin */
  GPIO_InitStruct.Pin = MC_PFCpwm_Pin|MC_EnB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MC_EmergencySTOP_Pin */
  GPIO_InitStruct.Pin = MC_EmergencySTOP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
  HAL_GPIO_Init(MC_EmergencySTOP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MC_DissipativeBrake_Pin MC_NTC_Pin */
  GPIO_InitStruct.Pin = MC_DissipativeBrake_Pin|MC_NTC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MC_UH_Pin MC_VH_Pin MC_WH_Pin */
  GPIO_InitStruct.Pin = MC_UH_Pin|MC_VH_Pin|MC_WH_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : MC_EnA_Pin */
  GPIO_InitStruct.Pin = MC_EnA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF1_TIM3;
  HAL_GPIO_Init(MC_EnA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD5 PD6 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : SDcard_detect_Pin */
  GPIO_InitStruct.Pin = SDcard_detect_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDcard_detect_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MC_PFCsync2_Pin */
  GPIO_InitStruct.Pin = MC_PFCsync2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM16;
  HAL_GPIO_Init(MC_PFCsync2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : MicroSD_CS_OD_Pin */
  GPIO_InitStruct.Pin = MicroSD_CS_OD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(MicroSD_CS_OD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : MC_UL_Pin MC_VL_Pin MC_WL_Pin */
  GPIO_InitStruct.Pin = MC_UL_Pin|MC_VL_Pin|MC_WL_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF2_TIM1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI1_SCK_Pin SPI1_MISO_Pin */
  GPIO_InitStruct.Pin = SPI1_SCK_Pin|SPI1_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF0_SPI1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : I2C1_SCL_Pin I2C1_SDA_Pin */
  GPIO_InitStruct.Pin = I2C1_SCL_Pin|I2C1_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LCD_CS_OD_Pin */
  GPIO_InitStruct.Pin = LCD_CS_OD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LCD_CS_OD_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_c_l_i */
/**
  * @brief  Function implementing the CLI thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_c_l_i */
void c_l_i(void const * argument)
{
  /* USER CODE BEGIN 5 */
  uint8_t response[] = "\r\nbuffer overflow\r\n>";
  printInfo();
  char *str="Give the delay like this  d-XXXX\r\n>";
  HAL_UART_Transmit(&huart3,(uint8_t*)str,strlen(str),HAL_MAX_DELAY);
  uint32_t TickDelay=pdMS_TO_TICKS(1000);

  /* Infinite loop */
  for(;;)
  {
	if(tempBuff.fullFlag == true) {
	// By this point the UART interrupts have been disabled
	// Send error message
	status = HAL_UART_Transmit_IT(&huart3, &response[0], sizeof(response));
	assert_param(status == HAL_OK);

	// Clear full flag, count, and buffer
	tempBuff.count = 0;
	tempBuff.fullFlag = false;
	memset(&tempBuff.buff, 0x00, sizeof(tempBuff.buff));

	// Enable reception again
	status = HAL_UART_Receive_IT(&huart3, &tempBuff.buff[0],
	sizeof(uint8_t));
	assert_param(status == HAL_OK);
    }
    if(tempBuff.cmdReady == true) {
	  // By this point the UART interrupts have been disabled
	  // We know the count so turn the end to a null terminator
	  tempBuff.buff[tempBuff.count] = '\0';
	  if(tempBuff.count > 0) {
	  // We have at least one character plus CR, so process

      if(tempBuff.buff[0]=='d') {                                                           //if you give d-xxxx command like this to tempBuff.buff, check first character
		  int i = 0, j = 0;
		  int n = sizeof(tempBuff.buff);                                                    //if it is true goes the inside loop, and perform operation

		  while (i < n && tempBuff.buff[i] != '-') {                                        //we want only this xxxx , thats why we are using this code.
			++i;
		  }
		  ++i;
		  while (i < n && tempBuff.buff[i] != ' ') {
			temp[j] = tempBuff.buff[i];
			++i, ++j;
		  }
			temp[j] = '\0';
	      //HAL_UART_Transmit(&huart3,temp,sizeof(temp),HAL_MAX_DELAY);
	      xQueueSend(cli_queue,&temp,portMAX_DELAY);                                        //Pass the temp value to Joystick using message queue IPC
          status = HAL_UART_Transmit_IT(&huart3, &reply[0], sizeof(reply));
          assert_param(status == HAL_OK);
	  }
	  else if(strcmp((const char*)rstCmd, (const char*)tempBuff.buff) == 0)
	  {
	  	sprintf(&strBuff[0], "\r\nResetting MCU...\r\n");
	  	// Deliberately blocking so we don't reset half way through the message
	  	status = HAL_UART_Transmit(&huart3, (uint8_t*)&strBuff[0], strlen(strBuff),100);   //Resetting MCU
	  	assert_param(status == HAL_OK);
	  	NVIC_SystemReset();
	  }
	  else
	  {
	  	 // Inform that the command is not recognised
	  	 status = HAL_UART_Transmit_IT(&huart3, &cmdFail[0], sizeof(cmdFail));    //prints the Command not recognised
	  	 assert_param(status == HAL_OK);
	  }
	}
	else
	{
	  // Nothing to process, so just give a prompt
	  status = HAL_UART_Transmit_IT(&huart3, &prompt[0], sizeof(prompt));         // Just give a prompt
	  assert_param(status == HAL_OK);
	}

	  // Clear ready flag, count, and buffer
	  tempBuff.count = 0;
	  tempBuff.cmdReady = false;
	  memset(&tempBuff.buff, 0x00, sizeof(tempBuff.buff));

	  // Enable reception again
	  status = HAL_UART_Receive_IT(&huart3, &tempBuff.buff[0], sizeof(uint8_t));
	  assert_param(status == HAL_OK);
    }
	  vTaskDelay(TickDelay);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_joystick */
/**
* @brief Function implementing the joy thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_joystick */
void joystick(void const * argument)
{
  /* USER CODE BEGIN joystick */

  xQueueReceive(cli_queue,&temp,portMAX_DELAY);      //Receive the data from CLI through message queue
  jtosBuff.delay=atoi((const char *)temp);                         //It is for converting ASCII to integer and assigns to jtosBuff.delay.
  uint32_t TickDelay=pdMS_TO_TICKS(500);
  /* Infinite loop */
  for(;;)
  {
	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2)) {        //Read the status from Joy_up, if true goes to the inside loop,else goes to next condition
		jtosBuff.value = LED1;                        //if you press joy_up assign value to jtosBuff.value
	}
	else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3)) {   //Read the status from Joy_down, if true goes to the inside loop,else goes to next condition
		jtosBuff.value = LED2;                        //if you press joy_down assign value to jtosBuff.value
	}
	else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7)) {   //Read the status from Joy_left, if true goes to the inside loop,else goes to next condition
		jtosBuff.value = LED3;                        //if you press joy_left assign value to jtosBuff.value
	}
	else if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8)) {   //Read the status from Joy_right, if true goes to the inside loop,else goes to next condition
		jtosBuff.value = LED4;                        //if you press joy_right assign value to jtosBuff.value
	}
	else {
		jtosBuff.value = 0;                           //if you not press anything, simply assign 0
	}
	xQueueSend(simplequeue,&jtosBuff,portMAX_DELAY);
    vTaskDelay(TickDelay);
	  //osDelay(1);
   }
  /* USER CODE END joystick */
}

/* USER CODE BEGIN Header_leds */
/**
* @brief Function implementing the led thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_leds */
void leds(void const * argument)
{
  /* USER CODE BEGIN leds */
  /* Infinite loop */
  for(;;)
  {
    if (xQueueReceive(simplequeue, &jtosBuff, portMAX_DELAY) == pdPASS) {
		if (jtosBuff.value == LED1) {                     //If you receive Joy_up , led1 is toggle
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_5);
			HAL_Delay(jtosBuff.delay);                    //Based on received Delay,HAL_Delay works
		}
        else if (jtosBuff.value == LED2) {                //If you receive Joy_down , led2 is toggle
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_6);
			HAL_Delay(jtosBuff.delay);                    //Based on received Delay,HAL_Delay works
        }
		else if (jtosBuff.value == LED3) {                //If you receive Joy_left , led3 is toggle
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_8);
			HAL_Delay(jtosBuff.delay);                    //Based on received Delay,HAL_Delay works
		}
		else if (jtosBuff.value == LED4) {                //If you receive Joy_right , led4 is toggle
			HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_9);
			HAL_Delay(jtosBuff.delay);                    //Based on received Delay,HAL_Delay works
		}
		else {
		    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_5, GPIO_PIN_RESET);   //If you not receive anything, Led1 is reset i.e Turn ON led1
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_6, GPIO_PIN_RESET);   //If you not receive anything, Led2 is reset i.e Turn ON led2
		    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8, GPIO_PIN_RESET);   //If you not receive anything, Led3 is reset i.e Turn ON led3
			HAL_GPIO_WritePin(GPIOD, GPIO_PIN_9, GPIO_PIN_RESET);   //If you not receive anything, Led4 is reset i.e Turn ON led4
		}
	 }
    osDelay(1);
  }
  /* USER CODE END leds */
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
