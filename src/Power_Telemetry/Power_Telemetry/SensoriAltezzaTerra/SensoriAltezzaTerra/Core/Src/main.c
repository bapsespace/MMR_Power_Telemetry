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
#include "can.h"
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define map(x, in_min, in_max, out_min, out_max) ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)

#define ADS_ADDRESS 						(0b01001001	<< 1)	//the last 2 bits depends from the electrical connection (see datasheet)
#define ADS_CONVERSION_REGISTER				0b00000000
#define ADS_CONFIGURATION_REGISTER			0b00000001
#define ADS_LO_THRESH_REGISTER				0b00000010
#define ADS_HI_THRESH_REGISTER				0b00000011

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t adsBuffer[8];
int16_t adsReading1;
int16_t adsReading2;
float adsVread1;
float adsVread2;
float adsVmod1;
float adsVmod2;
float adsDistance1;
float adsDistance2;

#define voltageConv ((float) (6.114 / 32768.0));

#define BOARD_ID	1

#define BOARD1_CAN_ID	0x318
#define BOARD2_CAN_ID	0x314

#if BOARD_ID == 1

#define BOARD_CAN_ID	BOARD1_CAN_ID

#elif BOARD_ID == 2

#define BOARD_CAN_ID	BOARD2_CAN_ID

#endif


CAN_TxHeaderTypeDef 	pHeader; 						//declare a specific header for message transmissions
CAN_RxHeaderTypeDef 	pRxHeader; 						//declare header for message reception
uint32_t 				TxMailbox;
CAN_FilterTypeDef 		sFilterConfig1;

uint32_t millisCanTransmission = 0;
#define  canTrasmissionDelay			5				//200Hz

uint8_t canTx[8] = {0, 0, 0, 0, 0, 0, 0, 0};
uint8_t canRx[8] = {0, 0, 0, 0, 0, 0, 0, 0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
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
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
  uint8_t addr[128] = {0x00};

  for (uint8_t i = 0; i < 128; i++){
	  addr[i] = HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(i << 1), 3, HAL_MAX_DELAY);
	  HAL_Delay(1);
  }



  pHeader.DLC = 8; 						//give message size of 1 byte
  pHeader.IDE = CAN_ID_STD; 			//set identifier to standard
  pHeader.RTR = CAN_RTR_DATA; 			//set data type to remote transmission request?
  pHeader.StdId = BOARD_CAN_ID; 		//define a standard identifier, used for message identification by filters (switch this for the other microcontroller)

  HAL_CAN_Start(&hcan);

  adsBuffer[0] = ADS_CONFIGURATION_REGISTER;
  adsBuffer[1] = 0b11000001; //MSB					//select ch0
  adsBuffer[2] = 0b10100011; //LSB

  HAL_Delay(1000);

  uint8_t pippo = HAL_I2C_Master_Transmit(&hi2c1, ADS_ADDRESS, adsBuffer, 3, 100);

  if (pippo && 1)
  {
	  while(1)
	  {
		  static uint32_t millisLedBlink = 0;
		  if(millis >= millisLedBlink + 100 && pippo == HAL_ERROR)
		  {
			  HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
			  HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
			  HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);

			  millisLedBlink = millis;
		  }

		  if(millis >= millisLedBlink + 1000 && pippo == HAL_BUSY)
		  {
			  HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
			  HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
			  HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);

			  millisLedBlink = millis;
		  }
	  }
  }

  HAL_Delay(50);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  static uint32_t millisAdcRead = 0;
	  if(millis >= millisAdcRead + 5) //200Hz sampling
	  {
		  static uint8_t sensorToggle = 0;

		  if(sensorToggle == 0)	//read ch0 = SENSOR 1 @ 100Hz
		  {
			  HAL_I2C_Mem_Read(&hi2c1, ADS_ADDRESS, ADS_CONVERSION_REGISTER, 1, adsBuffer, 2, 10);
			  adsReading1 = (adsBuffer[0] << 8 | adsBuffer[1]);
			  adsVread1 = adsReading1 * voltageConv;

#if BOARD_ID == 1
			  adsVmod1 = map(adsVread1, 0.6373, 5.03679, 0.63637, 5.0617);		//taratura con AGILENT 34405A @ 03/05/2023
			  adsDistance1 = map(adsVmod1, 0, 5, 16, 120);
#elif BOARD_ID == 2
			  adsVmod1 = map(adsVread1, 0.62219, 5.0758, 0.62510, 5.0989);		//taratura con AGILENT 34405A @ 03/05/2023
			  adsDistance1 = map(adsVmod1, 0, 5, 16, 120) - 0.12;
#else
			  adsVmod1 = 0;
			  adsDistance1 = map(adsVread1, 0, 5, 16, 120);
#endif

			  adsBuffer[0] = ADS_CONFIGURATION_REGISTER;
			  adsBuffer[1] = 0b11010001; //MSB					//select ch1
			  adsBuffer[2] = 0b10100011; //LSB

			  HAL_I2C_Master_Transmit(&hi2c1, ADS_ADDRESS, adsBuffer, 3, 10);

			  //adsBuffer[0] = 0x00;
			  //HAL_I2C_Master_Transmit(&hi2c1, ADS_ADDRESS, adsBuffer, 1, 100);

			  sensorToggle = 1;
		  }
		  else					//read ch1 = SENSOR 2 @ 100Hz
		  {
			  HAL_I2C_Mem_Read(&hi2c1, ADS_ADDRESS, ADS_CONVERSION_REGISTER, 1, adsBuffer, 2, 10);
			  adsReading2 = (adsBuffer[0] << 8 | adsBuffer[1]);
			  adsVread2 = adsReading2 * voltageConv;

#if BOARD_ID == 1
			  adsVmod2 = map(adsVread2, 0.0987, 5.0053, 0.0990, 5.0295);	//taratura con AGILENT 34405A @ 03/05/2023
			  adsDistance2 = map(adsVmod2, 0, 5, 16, 120) - 0.09;
#elif BOARD_ID == 2
			  adsVmod2 = map(adsVread2, 0.58997, 5.023413, 0.59256, 5.0463);
			  adsDistance2 = map(adsVmod2, 0, 5, 16, 120) - 0.12;
#else
			  adsVmod2 = 0;
			  adsDistance2 = map(adsVread2, 0, 5, 16, 120);					//taratura con AGILENT 34405A @ 03/05/2023
#endif

			  adsBuffer[0] = ADS_CONFIGURATION_REGISTER;
			  adsBuffer[1] = 0b11000001; //MSB					//select ch0
			  adsBuffer[2] = 0b10100011; //LSB

			  HAL_I2C_Master_Transmit(&hi2c1, ADS_ADDRESS, adsBuffer, 3, 10);

			  sensorToggle = 0;
		  }

		  millisAdcRead = millis;
	  }



	  if(millis >= millisCanTransmission + canTrasmissionDelay)					//CAN data transmission
	  {
		  static uint16_t distance1_can = 0;
		  static uint16_t distance2_can = 0;

		  if(adsDistance1 != 0)
			  distance1_can = ((uint16_t) (adsDistance1 * 100));

		  if(adsDistance2 != 0)
			  distance2_can = ((uint16_t) (adsDistance2 * 100));

		  canTx[0] = distance1_can >> 8;
		  canTx[1] = distance1_can;
		  canTx[2] = distance2_can >> 8;
		  canTx[3] = distance2_can;

		  canTx[4] = 0;
		  canTx[5] = 0;
		  canTx[6] = 0;
		  canTx[7] = 0;

		  HAL_CAN_AddTxMessage(&hcan, &pHeader, canTx, &TxMailbox);
		  millisCanTransmission = millis;
	  }




	  if(1)
	  {
		  static uint32_t millisLedBlink = 0;
		  if(millis >= millisLedBlink + 500)
		  {
			  HAL_GPIO_TogglePin(LED_R_GPIO_Port, LED_R_Pin);
			  HAL_GPIO_TogglePin(LED_G_GPIO_Port, LED_G_Pin);
			  HAL_GPIO_TogglePin(LED_B_GPIO_Port, LED_B_Pin);

			  millisLedBlink = millis;
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
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
