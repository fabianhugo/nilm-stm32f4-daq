/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "wifi_bglib.h"
#include "wf121.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/*
 * wf112.h
 *
 *  Created on: May 7, 2019
 *      Author: burr
 */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define wifi
#define BUFFERSIZE 64

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;
DMA_HandleTypeDef hdma_spi1_tx;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

/* USER CODE BEGIN PV */


typedef struct {
	uint8_t bytes[4];
}sample_t;

typedef struct {
     volatile sample_t buf[BUFFERSIZE]; //constant pointer, value can change (!= const int)
     volatile int16_t head;
     volatile int16_t full;
     volatile int16_t size;
} pp_buffer_t;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rxbuffer[BGLIB_MSG_MAXLEN]={0};

volatile uint8_t spibuffer1[4]={0};
volatile uint8_t spibuffer2[4]={0};
volatile uint8_t avgswitch=0;

volatile pp_buffer_t txbufferping;
volatile pp_buffer_t txbufferpong;
volatile uint8_t spirxcplt=1;

volatile uint8_t buffertowrite=0;
volatile uint8_t setmarker=0;


/** Length of message payload data. */
volatile uint16_t msg_length=0;
volatile uint8_t rxcplt=0;
volatile uint8_t awaitmore=0;
volatile uint8_t txcplt;


uint8_t started_measurement=0;

uint8_t saved_endpoint=0;
uint8_t connected = 0;
uint8_t errorcounter=0;

// Define BGLIB library
BGLIB_DEFINE();

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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */


  txbufferping.head = 0;
  txbufferping.full = 0;
  txbufferping.size = BUFFERSIZE;

  txbufferpong.head = 0;
  txbufferpong.full = 0;
  txbufferpong.size = BUFFERSIZE;
  #ifdef wifi

  wifi_init(rxbuffer);
  uint8_t streamingset=0;

  while (!streamingset){

	  if (rxcplt){
		  wifi_connect(rxbuffer, &streamingset); //Establishes the WiFi & TCPIP/UDP Connection. Type of Connection can be set in wf121.c by Macros.
	  }
  }
  //CONNECTION SUCESSFUL
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2); //for triggering CONV at NISO
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3); //for triggering CONV at ISO (current and voltage)
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1); //triggers spi read
  //The 2 commands below guarantee that operation isn't disturbed by the UART IRQ, can be uncommented if status updates shall be further received.
  HAL_UART_AbortReceive_IT(&huart1);
  rxcplt=0; //

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  if(rxcplt){ //Check for status updates of WiFi module
//		  wifi_check(rxbuffer, &streamingset);
//	  }


	  if((HAL_UART_GetState(&huart1)==HAL_UART_STATE_READY) && (txbufferping.full==1) && spirxcplt){
		  if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*) &txbufferping.buf[0].bytes[0], BUFFERSIZE*4/2)!=HAL_OK){
			  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);}
		  else
		  {
			  txbufferping.head=0;
			  txbufferping.full=0;
		  }
	  }

	  if((HAL_UART_GetState(&huart1)==HAL_UART_STATE_READY) && (txbufferpong.full==1) && spirxcplt){

		  if(HAL_UART_Transmit_DMA(&huart1, (uint8_t*) &txbufferpong.buf[0].bytes[0], BUFFERSIZE*4/2)!=HAL_OK){
			  HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);}
		  else
		  {
			  txbufferpong.head=0;
			  txbufferpong.full=0;
		  }
	  }

  }


#else


  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2); //for triggering CONV at NISO
  HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3); //for triggering CONV at ISO (current and voltage)
  HAL_TIM_OC_Start_IT(&htim1, TIM_CHANNEL_1); //triggers spi read
  //  char voltagestring[6];
while (1)
{
	HAL_Delay(100);
	voltage= (int16_t)((((spibuffer[0]&0b00111111)<<8)|(spibuffer[1]))<<2)/4;
	sprintf(voltagestring, "%u", voltage);
}
#endif
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

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 15;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1608;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 1608-3-440;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM2;
  sConfigOC.Pulse = 1608-1;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  huart1.Init.BaudRate = 1000000;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_RTS_CTS;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
  /* DMA2_Stream3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream3_IRQn);
  /* DMA2_Stream7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

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

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, MCLR_WF_Pin|PROG_PIN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PH0 PH1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PA0 PA3 PA4 PA8 
                           PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_8 
                          |GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED2_Pin */
  GPIO_InitStruct.Pin = LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SEL_MICRO_Pin */
  GPIO_InitStruct.Pin = SEL_MICRO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SEL_MICRO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB13 PB14 
                           PB15 PB4 PB5 PB6 
                           PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_13|GPIO_PIN_14 
                          |GPIO_PIN_15|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6 
                          |GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : MCLR_WF_Pin PROG_PIN_Pin */
  GPIO_InitStruct.Pin = MCLR_WF_Pin|PROG_PIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){ //receive complete
	//UART NVIC Preemption Priority is 4
	if (huart->Instance==USART1){
		if (!awaitmore){
			msg_length = BGLIB_MSG_LEN(rxbuffer);
			if(msg_length){
				awaitmore=1;
				rxcplt=0;
				HAL_UART_Receive_IT(&huart1, &rxbuffer[BGLIB_MSG_HEADER_LEN], msg_length);
			}
			else{
				rxcplt=1;
			}
		}
		else{
			awaitmore=0;
			rxcplt=1;
		}
	}

}


void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim){
	//TIM1 NVIC Preemption Priority is 0

	if (htim->Instance==TIM1){
		if (!avgswitch){
			HAL_SPI_Receive_DMA(&hspi1, (uint8_t*) spibuffer1, 4);
		}
		else{
			HAL_SPI_Receive_DMA(&hspi1, (uint8_t*) spibuffer2, 4);
		}
		avgswitch=!avgswitch;
	}
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi){
	//SPI NVIC Preemption Priority is 1, SPI is triggered in TIM1 IRQ therefore happens later
	spirxcplt=1;
	if (!avgswitch){
		HAL_GPIO_WritePin(PROG_PIN_GPIO_Port, PROG_PIN_Pin, 1);

	switch(buffertowrite){
	case 0:
		if (txbufferping.full==0){
			spirxcplt=0;

			// Averaging over two consecutive measurments, since values are in 14bit twos comp shifting and typecasting is necessary before adding them
			int16_t avgch1 =  ((( (int16_t)((((spibuffer1[0] & 0x003F)<<8) | spibuffer1[1])<<2)  +  (int16_t)((((spibuffer2[0] & 0x003F)<<8) | spibuffer2[1])<<2))>>2)/2);
			int16_t avgch2 =  ((( (int16_t)((((spibuffer1[2] & 0x003F)<<8) | spibuffer1[3])<<2)  +  (int16_t)((((spibuffer2[2] & 0x003F)<<8) | spibuffer2[3])<<2))>>2)/2);

			txbufferping.buf[txbufferping.head].bytes[0]= (avgch1 & 0x3F00)>>8;
			txbufferping.buf[txbufferping.head].bytes[1]= (avgch1 & 0x00FF);
			txbufferping.buf[txbufferping.head].bytes[2]= (avgch2 & 0x3F00)>>8;
			txbufferping.buf[txbufferping.head].bytes[3]= (avgch2 & 0x00FF);
			if (setmarker){
				txbufferping.buf[txbufferping.head].bytes[0]=txbufferping.buf[txbufferping.head].bytes[0] | 0b10000000;
				setmarker=0;
			}

			txbufferping.head++;
			if (txbufferping.head==txbufferping.size/2){
				buffertowrite= 1;
				txbufferping.full=1;
			}
		}
		else{
			//BUFFEROVERFLOW
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
			setmarker++;
		}
		break;


	case 1:
		if (txbufferpong.full==0){
			spirxcplt=0;

			int16_t avgch1 =  ((( (int16_t)((((spibuffer1[0] & 0x003F)<<8) | spibuffer1[1])<<2)  +  (int16_t)((((spibuffer2[0] & 0x003F)<<8) | spibuffer2[1])<<2))>>2)/2);
			int16_t avgch2 =  ((( (int16_t)((((spibuffer1[2] & 0x003F)<<8) | spibuffer1[3])<<2)  +  (int16_t)((((spibuffer2[2] & 0x003F)<<8) | spibuffer2[3])<<2))>>2)/2);

			txbufferpong.buf[txbufferpong.head].bytes[0]= (avgch1 & 0x3F00)>>8;
			txbufferpong.buf[txbufferpong.head].bytes[1]= (avgch1 & 0x00FF);
			txbufferpong.buf[txbufferpong.head].bytes[2]= (avgch2 & 0x3F00)>>8;
			txbufferpong.buf[txbufferpong.head].bytes[3]= (avgch2 & 0x00FF);
			if (setmarker){
				txbufferpong.buf[txbufferpong.head].bytes[0]=txbufferpong.buf[txbufferpong.head].bytes[0] | 0b01000000;
				setmarker=0;
			}
			txbufferpong.head++;
			if (txbufferpong.head==txbufferpong.size/2){
				buffertowrite= 0;
				txbufferpong.full=1;
			}
		}
		else{
			//BUFFEROVERFLOW
			HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, 1);
			setmarker++;
		}
		break;

	}
	HAL_GPIO_WritePin(PROG_PIN_GPIO_Port, PROG_PIN_Pin, 0);
	}

}


//void HAL_UART_TxHalfCpltCallback(UART_HandleTypeDef *huart){
//
//}

//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
////	huart->gState=HAL_UART_STATE_READY;
//	HAL_GPIO_WritePin(PROG_PIN_GPIO_Port, PROG_PIN_Pin, 0);
//}


/* USER CODE END 4 */

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
