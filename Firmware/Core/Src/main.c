/* USER CODE BEGIN Header */
/*
 * Main C file used for demos. Most of this is generated HAL, but the important stuff
 * can be found in USER CODE sections, especially after USER CODE BEGIN 4
 */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sx1276.h"
#include "polkadot.h"
#include <stdlib.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define PI_NODE 0		//this  determines whether or not this node is "special" or "stock"
#define DATA_REQ 55
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c3;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;

/* USER CODE BEGIN PV */

uint8_t data[16] = "hello world!\n\r";			//UART Transmit Buffer
uint8_t cmd_rx_buf[16];				//receive buffer for mesh commands from Pi
uint8_t cmd_tx_buf[16];
volatile uint8_t tx_len = 5;

volatile unsigned char xdone_flag = 0;
volatile unsigned char rx_flag = 0;
volatile unsigned char spi_flag = 0;
volatile unsigned char spi_rx_tx = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C3_Init(void);
/* USER CODE BEGIN PFP */

//prototypes for functions used directly in main
uint8_t spi_transfer(uint8_t data[], uint8_t len);
uint16_t readTemp();

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
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_ADC1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  SX1276_Init(SYNC_WORD, 24);  //Initialize the SX1276

  //PHASE 3 Demo:
  if(PI_NODE){
	  polkadot_init(4); //the hard-coded address of the Pi Node is 4
  } else {
	  uint32_t unique_seed = get_UID();	//initialize with a random ID
	  DEBUG_PRINT("device id: %d\n\r", unique_seed);
	  srand(unique_seed);
	  uint32_t new_id;
	  new_id = rand();
	  while(new_id < 11){
		  new_id = rand();
	  }
	  DEBUG_PRINT("New ID: %d\n\r", new_id);
	  polkadot_init(new_id);
	  rand_delay();
	  mesh_send_hello();
  }

  //MAIN DEMO2 TEST
//	  polkadot_init(50); //For data-sender
//	  HAL_Delay(100);
//	  mesh_transmit(10, data, 3);

//	  polkadot_init(10);   //For Receiver
//	  polkadot_init(25);  //Intermediate


  //RANGE TEST
//  polkadot_init(1);  //transmitter
//  polkadot_init(10); //receiver



//  //EMI TEST
//  HAL_Delay(100);
//  SX1276_Set_Mode(FSTX);  //set mode to FSTX so PLL is turned on
  SX1276_Start_Receive();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

//	  //RANGE TEST  -- Transmitter
//	  mesh_transmit(10, data, 16);
//	  SX1276_Start_Receive();
//	  HAL_Delay(1000);

//	  //TRANSMIT TEST
//	  HAL_Delay(2000);
//	  DEBUG_PRINT("Transmitting a Packet\n\r");
//	  HAL_Delay(10);
//	  SX1276_Transmit(data, 20);
//	  DEBUG_PRINT("\n\r");


//	  //RECEIVE TEST
//	  uint8_t length;
//	  SX1276_Receive(nrx_data, &length);
//	  DEBUG_PRINT("Packet Data: ");
//	  DEBUG_PRINT((char *) nrx_data);
//	  DEBUG_PRINT("\n\r\n\r");
//
//	  //RSSI TEST
//	  uint8_t rssivalue = SX1276_Get_RSSI();
//	  DEBUG_PRINT("RSSI Value = %ddB\n\r", rssivalue);

//	  //MESH TEST - Source
//	  //Expected packet - [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 0, 0, 0, 3, 0, 0, 0, 0, 104, 101, 108, 108, 111, 32, 119, 111, 114, 108, 100, 33, 10, 13, 0, 0]
//	  my_id = 0;
//	  uint8_t cast_result = mesh_send_data(3, 0, data, 2, 0, my_id, 16);
//	  if(cast_result){
//		  DEBUG_PRINT("Cast Successful\n\r");
//	  } else {
//		  DEBUG_PRINT("Cast Failed\n\r");
//	  }
//	  HAL_Delay(10000);

//	  //Temperature sensor test
//	  uint16_t temp = readTemp();
//	  DEBUG_PRINT("Temp: %d\n\r", temp);
//	  HAL_Delay(1000);

	  //process incoming requests from the Pi
//	  if(spi_flag && !spi_rx_tx && PI_NODE){ //!HAL_GPIO_ReadPin(SPI2_AS_GPIO_Port, SPI2_AS_Pin) && PI_NODE){		//wait for AS pin
//		  spi_flag = 0;
//		  DEBUG_PRINT("Received command:");
//		  printarr(cmd_rx_buf, 5);
//		  DEBUG_PRINT("\n\r");
//		  if(cmd_rx_buf[0] == DATA_REQ){
//			  uint32_t node_addr = cmd_rx_buf[1] << 24 | cmd_rx_buf[2] << 16 | cmd_rx_buf[3] << 8 | cmd_rx_buf[4];
//			  uint8_t data_req_arr[1] = {DATA_REQ};
//			  DEBUG_PRINT("Requesting data from node #%d\n\r", node_addr);
//			  mesh_transmit(node_addr, data_req_arr, 1);
//		  }
//		  SX1276_Start_Receive();		//go for a new packet
//	  }
	  if(xdone_flag){
		  xdone_flag = 0;
		  if(rx_flag){
		      uint8_t len = SX1276_Read_Register(REG_RX_NB_BYTES); //get packet length
		      uint8_t rxbuf[len];
		      SX1276_Read_Burst(REG_FIFO, rxbuf, len);
		      SX1276_Set_Mode(STDBY);
		      DEBUG_PRINT("Entering Packet Handler\n\r");
		      receive_packet_handler(rxbuf, len);
		      SX1276_Start_Receive();
		  }
	  }

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.ClockSpeed = 100000;
  hi2c3.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */
  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

//GPIO callback. Handles interrupts from the SX1276 and the raspberry pi.
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == XDONE_Pin){
		xdone_flag = 1;
	}
	if(GPIO_Pin == SPI2_AS_Pin && PI_NODE){
		HAL_SPI_TransmitReceive(&hspi2, cmd_tx_buf, cmd_rx_buf, tx_len, 1000);
		while (HAL_SPI_GetState(&hspi2) != HAL_SPI_STATE_READY){}
		if(!spi_rx_tx && PI_NODE){ //!HAL_GPIO_ReadPin(SPI2_AS_GPIO_Port, SPI2_AS_Pin) && PI_NODE){		//wait for AS pin
			spi_flag = 0;
			DEBUG_PRINT("Received command:");
			printarr(cmd_rx_buf, 5);
			DEBUG_PRINT("\n\r");
			if(cmd_rx_buf[0] == DATA_REQ){
				uint32_t node_addr = cmd_rx_buf[1] << 24 | cmd_rx_buf[2] << 16 | cmd_rx_buf[3] << 8 | cmd_rx_buf[4];
				uint8_t data_req_arr[1] = {DATA_REQ};
				DEBUG_PRINT("Requesting data from node #%d\n\r", node_addr);
				mesh_transmit(node_addr, data_req_arr, 1);
			  }
		  }
		SX1276_Start_Receive();
	}
}

//read the temperature sensor for the STM32
uint16_t readTemp(){
	DEBUG_PRINT("\tReading Temperature\n\r");
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1);
	uint16_t value = HAL_ADC_GetValue(&hadc1);
	return value;
}

//handshaking protocol for communicating with the raspberry pi
uint8_t spi_transfer(uint8_t data[], uint8_t length){
	DEBUG_PRINT("transferring data to the Pi over SPI\n\r");
	spi_rx_tx = 1;
	for(int i = 0; i < length; i ++){
		cmd_tx_buf[i] = data[i];
	}
	tx_len = length;
	HAL_GPIO_WritePin(DATA_RDY_GPIO_Port, DATA_RDY_Pin, GPIO_PIN_SET);
    while (!spi_flag){
    }
    HAL_GPIO_WritePin(DATA_RDY_GPIO_Port, DATA_RDY_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi2, data, length,100);
    HAL_Delay(50);
    spi_rx_tx = 0;
    tx_len = 5;
    return SUCCESS;
}

//FUNCTION FOR DOING STUFF WITH DATA
uint8_t DATA_RX_HANDLER(struct data_packet rx_pkt){
	//Print the received data to the console
	DEBUG_PRINT("Received Data: ");
	DEBUG_PRINT((char *) rx_pkt.packet_data);
	DEBUG_PRINT("\n\r");
	if(PI_NODE){
		uint8_t spi_data[6];
		spi_data[0] = rx_pkt.packet_data[0];
		spi_data[1] = rx_pkt.packet_data[1];
		spi_data[2] = (rx_pkt.source_id & 0xFF);
		spi_data[3] = (rx_pkt.source_id >> 8) & 0xFF;
		spi_data[4] = (rx_pkt.source_id >> 16) & 0xFF;
		spi_data[5] = (rx_pkt.source_id >> 24) & 0xFF;
		spi_transfer(spi_data, 6);
		SX1276_Start_Receive();
	} else {
		if(rx_pkt.packet_data[0] == DATA_REQ){		//reply with my temperature if that was a data request
			uint16_t mytemp = readTemp();
			uint8_t temppack[2] = {(mytemp & 0xFF), (mytemp >> 8)};
			mesh_transmit(4, temppack, 2);
		}
	}
	return SUCCESS;
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
  /* User can add their own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
