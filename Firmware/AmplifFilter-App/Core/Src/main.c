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
#define BUFF_SIZE			100						// Size of buffer for average calculation
//#define k_16b				(3.3/pow(2,16))			// For calculate voltage on ADC input
#define k_16b				(2.5/(pow(2,16) - 1))	// For calculate voltage on ADC input (External 2.5 V reference)

#define ADC_16b_HALF_RANGE	32768					// Half of ADC range --> ~equals to 0 V
#define ADC_16b_hysteresis	10						// Hysteresis of ADC for zero-crossing

#define FIR_SIZE			101						// Size of digital fir filter
#define FIR_BLOCK_SIZE		1						// Size of block processed by FIR
#define STAT_BLOCK_SIZE		100						// Block size for statistic calculations

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_ADC_LevelOutOfWindowCallback(ADC_HandleTypeDef *hadc);

Status_t setParams();
void cleanBuffer(float* buff);
void insertNewVal(float* buff);
float average(float* buff);
float FIRCalc(float* fir_buff, float* fir_coef);

float random_float(void) {
    // rand() vrací 0..RAND_MAX
    float r = (float)rand() / (float)RAND_MAX;  // 0..1
    return r*0.2f-0.1f;;                             // -0.5 .. 0.5
}

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

float raw_value = 0.0;
uint32_t adc_value = 0;
uint8_t new_raw_data = 0;

uint8_t servo_on = 0;

uint8_t fifo_loaded = 0;

float data_buffer[2000];
uint16_t data_buff_cnt = 0;
float data_filt[2000];

float raw_filt = 0.0;

static arm_fir_instance_f32 S;
static float firState32[FIR_BLOCK_SIZE + FIR_SIZE -1];

float dsp_mean = 0.0;		// Mean of measured data
float dsp_var = 0.0;		// Variance of measured data
float dsp_rms = 0.0;		// RMS of measured data
float dsp_std = 0.0;		// Standard deviation of measured data

/* fsample=2khz, fcut=10hz 6dB window blackmann */
const float fir_coef[FIR_SIZE] = {
    0.0f, 0.000005732734735f, 0.000023473445253f, 0.000054183281463f,
    0.000099026801763f, 0.000159375398653f, 0.000236804160522f, 0.000333081901772f,
    0.000450154358987f, 0.000590120442212f, 0.000755202025175f, 0.000947707216255f,
    0.001169988303445f, 0.001424394315109f, 0.001713219913654f, 0.002038650214672f,
    0.002402704209089f, 0.002807176904753f, 0.003253581002355f, 0.003743091132492f,
    0.004276488907635f, 0.004854113794863f, 0.005475818179548f, 0.006140925921500f,
    0.006848204880953f, 0.007595838978887f, 0.008381414227188f, 0.009201915934682f,
    0.010053729638457f, 0.010932656936347f, 0.011833940632641f, 0.012752301059663f,
    0.013681978918612f, 0.014616790227592f, 0.015550190582871f, 0.016475344076753f,
    0.017385203391314f, 0.018272593617439f, 0.019130295142531f, 0.019951138645411f,
    0.020728098228574f, 0.021454378962517f, 0.022123515605927f, 0.022729448974133f,
    0.023266619071364f, 0.023730035871267f, 0.024115353822708f, 0.024418924003839f,
    0.024637861177325f, 0.024770069867373f, 0.024814281612635f, 0.024770069867373f,
    0.024637861177325f, 0.024418924003839f, 0.024115353822708f, 0.023730035871267f,
    0.023266619071364f, 0.022729448974133f, 0.022123515605927f, 0.021454378962517f,
    0.020728098228574f, 0.019951138645411f, 0.019130295142531f, 0.018272593617439f,
    0.017385203391314f, 0.016475344076753f, 0.015550190582871f, 0.014616790227592f,
    0.013681978918612f, 0.012752301059663f, 0.011833940632641f, 0.010932656936347f,
    0.010053729638457f, 0.009201915934682f, 0.008381414227188f, 0.007595838978887f,
    0.006848204880953f, 0.006140925921500f, 0.005475818179548f, 0.004854113794863f,
    0.004276488907635f, 0.003743091132492f, 0.003253581002355f, 0.002807176904753f,
    0.002402704209089f, 0.002038650214672f, 0.001713219913654f, 0.001424394315109f,
    0.001169988303445f, 0.000947707216255f, 0.000755202025175f, 0.000590120442212f,
    0.000450154358987f, 0.000333081901772f, 0.000236804160522f, 0.000159375398653f,
    0.000099026801763f, 0.000054183281463f, 0.000023473445253f, 0.000005732734735f,
    0.0f
};



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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

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
	HAL_TIM_Base_Start_IT(&htim7);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	tick = HAL_GetTick();

	uint32_t tm = 0;				// Time counter

	float buff[BUFF_SIZE];		// Raw values buffer implemented as FIFO
	cleanBuffer(buff);

	float raw_average = 0.0;
	float hysteresis = 0.1;
	float threshold = 1.0;
	conf.meas.hysteresis = hysteresis;
	conf.meas.threshlod = threshold;

	uint32_t tm_analog = 0;

	HAL_GPIO_WritePin(MUX_ON_GPIO_Port, MUX_ON_Pin, 1);
	HAL_DAC_Start(&hdac1, DAC1_CHANNEL_1);
	HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 2048);

	/* FIR Init */
	arm_fir_init_f32(&S, FIR_SIZE, fir_coef, firState32, FIR_BLOCK_SIZE);

	// ToDo - rozdelit zpracovani dat podle obsahu registru --> raw x fir filtering
	// ToDo - vytvořit funkce na zpracovani dat, později rozdělit na moduly
	// ToDo - zjistit vzorkovacku a navrhnout spravny FIR vcetne implementace - je spravne?
	// ToDo - nakonec kontrola fci na statistiku - na jak velkem bufferu pocitat?
	// ToDo - pro overeni funkcnosti firu generovat sum pricist k raw a co bude na vystupu

	while (1)
	{
		CDC_PacketReceived();
		conf.sys.tick = HAL_GetTick();

		/* Check and update parameter settings every 1 sec */
		if (conf.sys.tick > tm)
		{
			tm = conf.sys.tick + 1000;

			setParams();
			HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
		}



		/* After completed ADC conversion */
		/* Read and store new value, calculate average and send via USB, set new DAC value */
		if (new_raw_data)
		{
			raw_value = (((float) adc_value * k_16b) - 1.25) * 8;

			// Add noise in range -0.5--0.5 V
			//raw_value+=3.0f * random_float();

			conf.meas.raw = raw_value;
			insertNewVal(buff);
			raw_average = average(buff);
			conf.meas.average = raw_average;



			/* FIR filter implementation on moving float buff */
			//raw_filt = FIRCalc(buff, fir_coef);
			arm_fir_f32(&S, &raw_value, &raw_filt, FIR_BLOCK_SIZE);
			conf.dsp.fir = raw_filt;

			/* Add to 2000 size buff */
			//data_buffer[data_buff_cnt] = adc_value;
			data_buffer[data_buff_cnt] = raw_value;
			data_filt[data_buff_cnt] = raw_filt;
			if (data_buff_cnt >= 2000)
			{
				data_buff_cnt = 0;
			}
			else
			{
				data_buff_cnt++;
			}

			/* Statistic functions */
			arm_mean_f32(buff, STAT_BLOCK_SIZE, &dsp_mean);
			arm_var_f32(buff, STAT_BLOCK_SIZE, &dsp_var);
			arm_std_f32(buff, STAT_BLOCK_SIZE, &dsp_std);
			arm_rms_f32(buff, STAT_BLOCK_SIZE, &dsp_rms);

			/* Save to internal registers */
			conf.dsp.mean = dsp_mean;
			conf.dsp.var = dsp_var;
			conf.dsp.std = dsp_std;
			conf.dsp.rms = dsp_rms;


			new_raw_data = 0;		// Turn flag off --> data processed
		}


		/* Every 2s change threshold value - depend on average */
		if (conf.sys.tick > tm_analog)
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

		}




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

  /** Macro to configure the PLL clock source
  */
  __HAL_RCC_PLL_PLLSOURCE_CONFIG(RCC_PLLSOURCE_HSE);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInitStruct.PLL3.PLL3M = 1;
  PeriphClkInitStruct.PLL3.PLL3N = 24;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 4;
  PeriphClkInitStruct.PLL3.PLL3R = 1;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_3;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_PLL3;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL3;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*This function sets parameters for amplifier gain and turn ON/OFF DC compensation circuit (DC servo)*/
Status_t setParams()
{
	Status_t ret = STATUS_OK;
	/*DC Offset settings*/
	if(conf.meas.offset)
		HAL_GPIO_WritePin(SERVO_GPIO_Port, SERVO_Pin, 1);
	else
		HAL_GPIO_WritePin(SERVO_GPIO_Port, SERVO_Pin, 0);

	/*AMP Gain settings*/
	switch (conf.meas.gain)
	{
	case 0:
		HAL_GPIO_WritePin(GAIN_1_GPIO_Port, GAIN_1_Pin, 1);
		HAL_GPIO_WritePin(GAIN_3_GPIO_Port, GAIN_3_Pin, 0);
		HAL_GPIO_WritePin(GAIN_10_GPIO_Port, GAIN_10_Pin, 0);
		HAL_GPIO_WritePin(GAIN_20_GPIO_Port, GAIN_20_Pin, 0);
		break;
	case 1:
		HAL_GPIO_WritePin(GAIN_1_GPIO_Port, GAIN_1_Pin, 0);
		HAL_GPIO_WritePin(GAIN_3_GPIO_Port, GAIN_3_Pin, 1);
		HAL_GPIO_WritePin(GAIN_10_GPIO_Port, GAIN_10_Pin, 0);
		HAL_GPIO_WritePin(GAIN_20_GPIO_Port, GAIN_20_Pin, 0);
		break;
	case 2:
		HAL_GPIO_WritePin(GAIN_1_GPIO_Port, GAIN_1_Pin, 0);
		HAL_GPIO_WritePin(GAIN_3_GPIO_Port, GAIN_3_Pin, 0);
		HAL_GPIO_WritePin(GAIN_10_GPIO_Port, GAIN_10_Pin, 1);
		HAL_GPIO_WritePin(GAIN_20_GPIO_Port, GAIN_20_Pin, 0);
		break;
	case 3:
		HAL_GPIO_WritePin(GAIN_1_GPIO_Port, GAIN_1_Pin, 0);
		HAL_GPIO_WritePin(GAIN_3_GPIO_Port, GAIN_3_Pin, 0);
		HAL_GPIO_WritePin(GAIN_10_GPIO_Port, GAIN_10_Pin, 0);
		HAL_GPIO_WritePin(GAIN_20_GPIO_Port, GAIN_20_Pin, 1);
		break;
	default:
		ret = STATUS_ERROR;	// Gain not defined - real gain == 100k/1k (feedback bypass through 100k resistor)
	}

	return ret;
}


void cleanBuffer(float* buff)
{
	for(int i=0;i<BUFF_SIZE;i++)
		buff[i] = 0.0;
}

/* Insert new item into FIFO buffer */
void insertNewVal(float* buff)
{
	/* Counter for loading fifo */
	static uint8_t cnt = 0;
	if(cnt >= BUFF_SIZE-1)
		fifo_loaded = 1;
	else
	{
		fifo_loaded = 0;
		cnt++;
	}
	/* Insert new value and push old ones */
	for(int i=0;i<BUFF_SIZE;i++)
		buff[i] = buff[i+1];
	buff[BUFF_SIZE-1] = raw_value;
}

/* Average of float array */
float average(float* buff)
{
	float sum = 0.0;
	for(int i=0;i<BUFF_SIZE;i++)
		sum += buff[i];

	return sum/BUFF_SIZE;
}

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
