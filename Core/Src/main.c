/* USER CODE BEGIN Header */
/**
 *  BMD device ICCID: 134973
 *  Wake-up implementation
 *  https://github.com/STMicroelectronics/STMems_Standard_C_drivers/blob/master/iis2dlpc_STdC/examples/iis2dlpc_wake_up.c
 *
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
#include <stdio.h>
#include <string.h>
#include "iis2dlpc_reg.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define IIS2DLPC_BUS hspi1
#define IIS2DLPC_WAKEUP_THS 1
#define IIS2DLPC_WAKEUP_DUR 1
#define IIS2DLPC_LIR 1

#define DBG huart5

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart5;

/* USER CODE BEGIN PV */
static uint8_t iis2dlpc_whoami_reg = 0, rst = 0 , reg8bit = 0 ;
static int16_t iis2dlpc_temp_reg = 0 ;
static uint8_t dbg_tx_buff[300] ;
static stmdev_ctx_t iis2dlpc_ctx;
static iis2dlpc_reg_t iis2dlpc_int_route;
static iis2dlpc_all_sources_t all_source;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART5_UART_Init(void);
/* USER CODE BEGIN PFP */

static int32_t	platform_write			( void *handle , uint8_t reg , const uint8_t *bufp , uint16_t len ) ;
static int32_t	platform_read			( void *handle , uint8_t reg , uint8_t *bufp , uint16_t len ) ;
static void		dbg_tx					( uint8_t* tx_buff , uint16_t len ) ;
static void		iis2dlpc_int1_print		( void ) ;
static void		iis2dlpc_conf_print		( stmdev_ctx_t* ) ;

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
  MX_SPI1_Init();
  MX_USART5_UART_Init();
  /* USER CODE BEGIN 2 */
	iis2dlpc_ctx.write_reg = platform_write;
	iis2dlpc_ctx.read_reg = platform_read;
	iis2dlpc_ctx.handle = &IIS2DLPC_BUS;

	iis2dlpc_device_id_get ( &iis2dlpc_ctx , &iis2dlpc_whoami_reg ) ;
	if ( iis2dlpc_whoami_reg == IIS2DLPC_ID )
	{
		sprintf ( (char*)dbg_tx_buff , "Hello! My name is %d\n", iis2dlpc_whoami_reg ) ;
		dbg_tx ( dbg_tx_buff, strlen ( (char const*)dbg_tx_buff) ) ;
	}
	else
	{
		/* manage here device not found */
	}
	/*Restore default configuration */
	iis2dlpc_reset_set ( &iis2dlpc_ctx , PROPERTY_ENABLE ) ;
	do {
		iis2dlpc_reset_get ( &iis2dlpc_ctx, &rst ) ;
	} while ( rst ) ;
	/*Set full scale */
	iis2dlpc_full_scale_set ( &iis2dlpc_ctx , IIS2DLPC_2g ) ;
	/*Configure power mode */
	iis2dlpc_power_mode_set ( &iis2dlpc_ctx , IIS2DLPC_CONT_LOW_PWR_LOW_NOISE_12bit ) ;
	/*Set Output Data Rate */
	iis2dlpc_data_rate_set ( &iis2dlpc_ctx , IIS2DLPC_XL_ODR_200Hz );
	/*Apply high-pass digital filter on Wake-Up function */
	iis2dlpc_filter_path_set ( &iis2dlpc_ctx , IIS2DLPC_HIGH_PASS_ON_OUT ) ;
	/*Apply high-pass digital filter on Wake-Up function
	 * Duration time is set to zero so Wake-Up interrupt signal
	 * is generated for each X,Y,Z filtered data exceeding the
	 * configured threshold
	*/
	// default iis2dlpc_wkup_dur_set(&dev_ctx, 0);
	// range is 0-3
	iis2dlpc_wkup_dur_set ( &iis2dlpc_ctx , IIS2DLPC_WAKEUP_DUR ) ;

	/* Set wake-up threshold
	 * Set Wake-Up threshold: 1 LSb corresponds to FS_XL/2^6
	 */
	// default iis2dlpc_wkup_threshold_set ( &iis2dlpc_ctx , 2 ) ;
	// range is 0-63
	iis2dlpc_wkup_threshold_set ( &iis2dlpc_ctx , IIS2DLPC_WAKEUP_THS ) ;
	/*Enable interrupt generation on Wake-Up INT1 pin */
	iis2dlpc_pin_int1_route_get ( &iis2dlpc_ctx , &iis2dlpc_int_route.ctrl4_int1_pad_ctrl ) ;
	iis2dlpc_int_route.ctrl4_int1_pad_ctrl.int1_wu = PROPERTY_ENABLE ;
	iis2dlpc_pin_int1_route_set ( &iis2dlpc_ctx , &iis2dlpc_int_route.ctrl4_int1_pad_ctrl ) ;

	/* get IIS2DLPC raw temp */
	iis2dlpc_temperature_raw_get ( &iis2dlpc_ctx , &iis2dlpc_temp_reg ) ;
	sprintf ( (char *)dbg_tx_buff , "IIS2DLPC temp is %d\r\n" , iis2dlpc_temp_reg ) ;
	dbg_tx ( dbg_tx_buff , strlen ( (char const*)dbg_tx_buff ) ) ;

	iis2dlpc_int_notification_set ( &iis2dlpc_ctx , IIS2DLPC_LIR ) ;

	iis2dlpc_conf_print ( &iis2dlpc_ctx );

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		//iis2dlpc_all_sources_t all_source;

		/* Check Wake-Up events */
		iis2dlpc_all_sources_get ( &iis2dlpc_ctx , &all_source ) ;

		if ( all_source.wake_up_src.wu_ia )
		{
			sprintf ( (char *)dbg_tx_buff , "Wake-Up event on " ) ;
			if ( all_source.wake_up_src.x_wu )
				strcat ( (char*)dbg_tx_buff , "X" ) ;
			if ( all_source.wake_up_src.y_wu )
				strcat ( (char *)dbg_tx_buff, "Y" ) ;
			if ( all_source.wake_up_src.z_wu )
				strcat ( (char*)dbg_tx_buff , "Z" ) ;
			strcat ( (char *)dbg_tx_buff , " direction\r\n" ) ;
		    dbg_tx ( dbg_tx_buff , strlen ( (char const*)dbg_tx_buff ) ) ;
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_5;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART5_UART_Init(void)
{

  /* USER CODE BEGIN USART5_Init 0 */

  /* USER CODE END USART5_Init 0 */

  /* USER CODE BEGIN USART5_Init 1 */

  /* USER CODE END USART5_Init 1 */
  huart5.Instance = USART5;
  huart5.Init.BaudRate = 9600;
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
  /* USER CODE BEGIN USART5_Init 2 */

  /* USER CODE END USART5_Init 2 */

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
  HAL_GPIO_WritePin(IIS2DLPC_CS_GPIO_Port, IIS2DLPC_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(IIS2DLPC_SHDN_GPIO_Port, IIS2DLPC_SHDN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IIS2DLPC_CS_Pin */
  GPIO_InitStruct.Pin = IIS2DLPC_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IIS2DLPC_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IIS2DLPC_SHDN_Pin */
  GPIO_InitStruct.Pin = IIS2DLPC_SHDN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IIS2DLPC_SHDN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IIS2DLPC_INT1_Pin */
  GPIO_InitStruct.Pin = IIS2DLPC_INT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IIS2DLPC_INT1_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write ( void *handle , uint8_t reg , const uint8_t *bufp , uint16_t len )
{
	HAL_GPIO_WritePin	( IIS2DLPC_CS_GPIO_Port , IIS2DLPC_CS_Pin , GPIO_PIN_RESET ) ;
	HAL_Delay ( 20 ) ;
	HAL_SPI_Transmit	( handle , &reg , 1 , 1000 ) ;
	HAL_SPI_Transmit	( handle , (uint8_t*) bufp , len , 1000 ) ;
	HAL_GPIO_WritePin	( IIS2DLPC_CS_GPIO_Port , IIS2DLPC_CS_Pin , GPIO_PIN_SET) ;

	return 0;
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read ( void *handle , uint8_t reg , uint8_t *bufp , uint16_t len )
{
	reg |= 0x80;
	HAL_GPIO_WritePin ( IIS2DLPC_CS_GPIO_Port , IIS2DLPC_CS_Pin , GPIO_PIN_RESET) ;
	HAL_Delay ( 20 ) ;
	HAL_SPI_Transmit ( handle , &reg , 1 , 1000 ) ;
	HAL_SPI_Receive ( handle , bufp , len , 1000 ) ;
	HAL_GPIO_WritePin ( IIS2DLPC_CS_GPIO_Port , IIS2DLPC_CS_Pin , GPIO_PIN_SET) ;

	return 0;
}

static void iis2dlpc_int1_print (void)
{
	HAL_GPIO_ReadPin ( IIS2DLPC_INT1_GPIO_Port , IIS2DLPC_INT1_Pin ) ;
	sprintf ( (char *)dbg_tx_buff , "IIS2DLPC_INT1_Pin is: %d\r\n" , (uint8_t)HAL_GPIO_ReadPin ( IIS2DLPC_INT1_GPIO_Port , IIS2DLPC_INT1_Pin ) ) ;
	dbg_tx ( dbg_tx_buff , strlen ( (char const*)dbg_tx_buff ) ) ;
}

static void	iis2dlpc_conf_print	( stmdev_ctx_t* iis2dlpc_ctx )
{
	iis2dlpc_wkup_threshold_get ( iis2dlpc_ctx , &reg8bit ) ;
	sprintf ( (char *)dbg_tx_buff , "WAKE_UP_THS: %d\r\n" , reg8bit ) ;
	dbg_tx ( dbg_tx_buff , strlen ( (char const*)dbg_tx_buff ) ) ;

	iis2dlpc_read_reg ( iis2dlpc_ctx , IIS2DLPC_CTRL1 , &reg8bit , 1 ) ;
	sprintf ( (char *)dbg_tx_buff , "CTRL1: %d\r\n" , reg8bit ) ;
	dbg_tx ( dbg_tx_buff , strlen ( (char const*)dbg_tx_buff ) ) ;

	iis2dlpc_read_reg ( iis2dlpc_ctx , IIS2DLPC_CTRL3 , &reg8bit , 1 ) ;
	sprintf ( (char *)dbg_tx_buff , "CTRL3: %d\r\n" , reg8bit ) ;
	dbg_tx ( dbg_tx_buff , strlen ( (char const*)dbg_tx_buff ) ) ;

	iis2dlpc_read_reg ( iis2dlpc_ctx , IIS2DLPC_CTRL4_INT1_PAD_CTRL , &reg8bit , 1 ) ;
	sprintf ( (char *)dbg_tx_buff , "CTRL4: %d\r\n" , reg8bit ) ;
	dbg_tx ( dbg_tx_buff , strlen ( (char const*)dbg_tx_buff ) ) ;

	iis2dlpc_read_reg ( iis2dlpc_ctx , IIS2DLPC_CTRL5_INT2_PAD_CTRL , &reg8bit , 1 ) ;
	sprintf ( (char *)dbg_tx_buff , "CTRL5: %d\r\n" , reg8bit ) ;
	dbg_tx ( dbg_tx_buff , strlen ( (char const*)dbg_tx_buff ) ) ;

	iis2dlpc_read_reg ( iis2dlpc_ctx , IIS2DLPC_CTRL6 , &reg8bit , 1 ) ;
	sprintf ( (char *)dbg_tx_buff , "CTRL6: %d\r\n" , reg8bit ) ;
	dbg_tx ( dbg_tx_buff , strlen ( (char const*)dbg_tx_buff ) ) ;

	iis2dlpc_read_reg ( iis2dlpc_ctx , IIS2DLPC_STATUS , &reg8bit , 1 ) ;
	sprintf ( (char *)dbg_tx_buff , "STATUS: %d\r\n" , reg8bit ) ;
	dbg_tx ( dbg_tx_buff , strlen ( (char const*)dbg_tx_buff ) ) ;

	iis2dlpc_read_reg ( iis2dlpc_ctx , IIS2DLPC_WAKE_UP_SRC , &reg8bit , 1 ) ;
	sprintf ( (char *)dbg_tx_buff , "WAKE_UP_SRC: %d\r\n" , reg8bit ) ;
	dbg_tx ( dbg_tx_buff , strlen ( (char const*)dbg_tx_buff ) ) ;
}
/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void dbg_tx ( uint8_t* tx_buff , uint16_t len ) {
	HAL_UART_Transmit ( &DBG , tx_buff , len , 1000 ); }

void HAL_GPIO_EXTI_Callback ( uint16_t GPIO_Pin )
{
	sprintf ( (char*)dbg_tx_buff , "INT1 happened!\n" ) ;
	dbg_tx ( dbg_tx_buff, strlen ( (char const*)dbg_tx_buff) ) ;

	iis2dlpc_int1_print();
	//iis2dlpc_all_sources_get ( &iis2dlpc_ctx , &all_source ) ;
	//iis2dlpc_read_reg ( &iis2dlpc_ctx , IIS2DLPC_WAKE_UP_SRC , &reg8bit , 1 ) ;
	//iis2dlpc_int1_print();

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
