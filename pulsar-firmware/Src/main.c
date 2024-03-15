/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
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
#include "comp.h"
#include "dac.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pid.h"
#include "ssd1306.h"
#include "ssd1306_tests.h"
#include "ssd1306_fonts.h"
#include <string.h>
#include <stdio.h>
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

volatile uint32_t voltage;
volatile uint32_t current;
volatile uint32_t inputVoltage;

volatile uint32_t pwm = 0;
volatile uint32_t pwmMin = 0;
volatile uint32_t pwmMax = 800;

volatile uint32_t targetVoltage = 0;
volatile uint32_t targetCurrent = 0;

volatile uint32_t adcActualChannel;

volatile uint32_t CCmode = 0;

volatile uint32_t compVoltageState = 0;
volatile uint32_t compCurrentState = 0;

char buffer[15];

int32_t trap;

PID_TypeDef voltagePID;
PID_TypeDef currentPID;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void HAL_ADC_SetChannel(ADC_HandleTypeDef *hadc, uint32_t channel);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_I2C1_Init();
	MX_COMP1_Init();
	MX_COMP2_Init();
	MX_DAC1_Init();
	/* USER CODE BEGIN 2 */

	ssd1306_Init();

	HAL_ADCEx_Calibration_Start(&hadc1);
	adcActualChannel = ADC_CHANNEL_0;
	HAL_ADC_SetChannel(&hadc1, adcActualChannel);
	HAL_ADC_Start_IT(&hadc1);

	HAL_DAC_Start(&hdac1, DAC_CHANNEL_1);
	HAL_DAC_Start(&hdac1, DAC_CHANNEL_2);

	HAL_COMP_Start(&hcomp1);
	HAL_COMP_Start(&hcomp2);

	//boost
//	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
//	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 10);

//buck
	HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_2);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);

	targetVoltage = 4200;
	targetCurrent = 500;

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_1, DAC_ALIGN_12B_R,
				targetVoltage / 18.5);
		HAL_DAC_SetValue(&hdac1, DAC_CHANNEL_2, DAC_ALIGN_12B_R,
				targetCurrent / 2);

		memset(buffer, ' ', sizeof(buffer));
		sprintf(buffer, "%d mV     ", (int) voltage);
		ssd1306_SetCursor(0, 0);
		ssd1306_WriteString(buffer, Font_11x18, White);

		memset(buffer, ' ', sizeof(buffer));
		sprintf(buffer, "%d mA     ", (int) current);
		ssd1306_SetCursor(0, 25);
		ssd1306_WriteString(buffer, Font_11x18, White);

		ssd1306_UpdateScreen();

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

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
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {

	if (adcActualChannel == ADC_CHANNEL_0) {

		voltage = HAL_ADC_GetValue(&hadc1) * (18.5);

		adcActualChannel = ADC_CHANNEL_2;

	} else if (adcActualChannel == ADC_CHANNEL_2) {

		current = HAL_ADC_GetValue(&hadc1) * (2);
		adcActualChannel = ADC_CHANNEL_10;

	} else if (adcActualChannel == ADC_CHANNEL_10) {

		inputVoltage = HAL_ADC_GetValue(&hadc1) * (8.9);
		adcActualChannel = ADC_CHANNEL_0;
	}

	if (current > (targetCurrent))
		CCmode = 1;
	else if (voltage > (targetVoltage))
		CCmode = 0;

	HAL_ADC_SetChannel(&hadc1, adcActualChannel);

	if (CCmode)
		HAL_GPIO_WritePin(CC_MODE_GPIO_Port, CC_MODE_Pin, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(CC_MODE_GPIO_Port, CC_MODE_Pin, GPIO_PIN_RESET);

	HAL_ADC_Start_IT(&hadc1);

}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM1) {

		compVoltageState = HAL_COMP_GetOutputLevel(&hcomp1);
		compCurrentState = HAL_COMP_GetOutputLevel(&hcomp2);

//		if (compVoltageState == COMP_OUTPUT_LEVEL_LOW
//				&& compCurrentState == COMP_OUTPUT_LEVEL_LOW && pwm < pwmMax) {
//			pwm++;
//		} else if (pwm > pwmMin) {
//			pwm--;
//		}
		if (compCurrentState == COMP_OUTPUT_LEVEL_LOW) {
			if (compVoltageState == COMP_OUTPUT_LEVEL_LOW && pwm < pwmMax) {
				pwm++;
			} else if (compVoltageState == COMP_OUTPUT_LEVEL_HIGH
					&& pwm > pwmMin) {
				pwm--;
			}
		} else if(compCurrentState == COMP_OUTPUT_LEVEL_HIGH && pwm > pwmMin){
			pwm--;
		}

			__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, pwm);

	}
}

void HAL_ADC_SetChannel(ADC_HandleTypeDef *hadc, uint32_t channel) {

	ADC_ChannelConfTypeDef sConfig = { 0 };

	sConfig.Channel = channel;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(hadc, &sConfig) != HAL_OK) {
		Error_Handler();
	}

}

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
