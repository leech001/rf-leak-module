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
#include "adc.h"
#include "dma.h"
#include "rtc.h"
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "nrf24l01.h"
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

/* USER CODE BEGIN PV */
static uint8_t *Unique_ID = (uint8_t *)UID_BASE;
static uint16_t *vrefint_cal = (uint16_t *)VREFINT_CAL_ADDR;

uint16_t up_count = 0;
volatile uint16_t adc[2] = {
	0,
};

uint16_t water = 0;
uint16_t voltage = 0;

uint8_t nrf_data[32] = {
	0,
};

volatile uint8_t flag = 0;

//        === NRF Setup ===
//        const uint64_t pipe0 = 0x7878787878LL;
const uint64_t pipe1 = 0xF0F0F0F0A1LL;
//        const uint64_t pipe2 = 0xF0F0F0F0B1LL;
//        const uint64_t pipe3 = 0xF0F0F0F0C1LL;
//        const uint64_t pipe4 = 0xF0F0F0F0D1LL;
//        const uint64_t pipe5 = 0xF0F0F0F0E1LL;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void NRF_Prepare(void);
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
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_RTC_Init();
	MX_SPI1_Init();
	/* USER CODE BEGIN 2 */

	// Clear standby flag
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF);

	// Read from backup register up count
	up_count = HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR0);

	// ADC
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)&adc, 2);

	// Wait DMA
	while (!flag)
	{
	}

	water = adc[0];
	voltage = VREFINT_CAL_VREF * (*vrefint_cal) / adc[1];

	// Send a status every 360 min
	if (up_count == 0 || up_count >= 360)
	{
		NRF_Prepare();

		write(&nrf_data, 32);
		up_count = 1;
	}

	// Send a notification if the humidity level is above the permissible level 
	if (water > 1000)
	{
		NRF_Prepare();

		for (uint8_t i = 0; i < 5; i++)
		{
			write(&nrf_data, 32);
			HAL_Delay(1000);
		}
	}

	up_count++;
	HAL_PWR_EnableBkUpAccess();
	HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR0, up_count);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		HAL_PWR_EnterSTANDBYMode();

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
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI | RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV16;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if (hadc->Instance == ADC1)
	{
		HAL_ADC_Stop_DMA(&hadc1);
		HAL_ADC_Stop(&hadc1);
		flag = 1;
	}
}

void NRF_Prepare(void)
{
	HAL_GPIO_WritePin(NRF_PWR_GPIO_Port, NRF_PWR_Pin, GPIO_PIN_SET);
	while (!isChipConnected())
		;
	NRF_Init();
	setDataRate(RF24_250KBPS);
	setChannel(76);
	openWritingPipe(pipe1);
	maskIRQ(true, true, true);

	// Add message
	memcpy(nrf_data, Unique_ID, 12);
	nrf_data[12] = water >> 8;
	nrf_data[13] = water & 0xff;
	nrf_data[14] = voltage >> 8;
	nrf_data[15] = voltage & 0xff;
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

#ifdef USE_FULL_ASSERT
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
