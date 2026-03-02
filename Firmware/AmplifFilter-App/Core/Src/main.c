/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "crc.h"
#include "dac.h"
#include "dma.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "common.h"
#include "usbd_cdc_if.h"
#include "system_msp.h"
#include "configuration.h"

#include "arm_math.h"

#include "meas.h"

// for random number gen
#include "stdlib.h"
#include "stdint.h"

//#include "stdbool.h"
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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc);

float FIRCalc(float* fir_buff, float* fir_coef);

float random_float(void) {
    // rand() vrací 0..RAND_MAX
    float r = (float)rand() / (float)RAND_MAX;  // 0..1
    return r*0.2f-0.1f;;                             // -0.5 .. 0.5
}

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
	uint32_t tick;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
	/* Disable IWDG in debug mode */
	DBGMCU->APB4FZ1 = DBGMCU->APB4FZ1 | DBGMCU_APB4FZ1_DBG_IWDG1;

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  MX_DAC1_Init();
  MX_USART3_UART_Init();
  MX_CRC_Init();
  MX_TIM5_Init();
  MX_TIM16_Init();
  MX_IWDG1_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */

	Config_Init();

	systemParamInit();	// Init system default state
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	tick = HAL_GetTick();

	uint32_t tm = 0;				// Time counter

/*
	float hysteresis = 0.1;
	float threshold = 1.0;
	conf.meas.hysteresis = hysteresis;
	conf.meas.threshlod = threshold;

	uint32_t tm_analog = 0;
*/
	while (1)
	{
		CDC_PacketReceived();
		conf.sys.tick = HAL_GetTick();

		/* Check and update parameter settings every 1 sec */
		if (conf.sys.tick > tm)
		{
			tm = conf.sys.tick + 1000;

			setParams((uint8_t)conf.meas.offset,(uint8_t)conf.meas.gain);
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		}

		/* After completed ADC conversion */
		/* Read and store new value, calculate average and send via USB, set new DAC value */
		if (new_raw_data)
		{
			if(measProcess(adc_value) == STATUS_OK)
				new_raw_data = 0;		// Turn flag off --> data processed
		}

		/* Every 2s change threshold value - depend on average */
		/*if (conf.sys.tick > tm_analog)
		{
			tm_analog = conf.sys.tick + 2000;

			// Only when fifo is full (100 samples after restart)
			if (fifo_loaded)
			{
				if (raw_average > threshold + hysteresis)
				{
					threshold += threshold / 10;
				}
				else if (raw_average < threshold - hysteresis)
				{
					threshold -= threshold / 10;
				}

				hysteresis = threshold / 10;

				conf.meas.hysteresis = hysteresis;
				conf.meas.threshlod = threshold;

				uint32_t dac_val = (uint32_t)(((threshold / 8.0) + 1.25) * 4095) / 2.5 ;
				HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, dac_val );
			}

		}*/

		if (TICK_EXPIRED(tick))
		{
			tick += 100;

			System_ReloadIwdg();
			Control_Handle();
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

  /*AXI clock gating */
  RCC->CKGAENR = 0xFFFFFFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_DIRECT_SMPS_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 70;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */



/* FIR filter implementation on moving float buff */
float FIRCalc(float* fir_buff, float* fir_coef)
{
	float fir = 0.0;
	for(int i=0;i<FIR_SIZE;i++)
	{
		fir += fir_buff[BUFF_SIZE-1-i] * fir_coef[i];
	}
	return fir;
}

/*	Callback handlers	*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM7)
	{
		//HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		HAL_ADC_Start_DMA(&hadc1, &adc_value, 1);
	}
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc->Instance == ADC1)
	{
		new_raw_data = 1;
	}
}

/* Analog watchdog IRQ callback */
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc)
{
	if(hadc->Instance == ADC1)
	{
		uint32_t isr = hadc->Instance->ISR;

		/* Need to turn of IRQ flag in ISR */
		    if (isr & ADC_ISR_AWD1)
		    {
		        __HAL_ADC_CLEAR_FLAG(hadc, ADC_FLAG_AWD1);
		    }
	}
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* User can add his own implementation to report the HAL error return state */

//  System_Reset();
  while(1)
  {
  }
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
