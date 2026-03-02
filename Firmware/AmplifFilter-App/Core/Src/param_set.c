/*
 * param_set.c
 *
 *  Created on: Feb 24, 2026
 *      Author: Martin Stastny
 */

#include "param_set.h"

#include "tim.h"
#include "stm32_hal_legacy.h"
#include "dac.h"

Status_t systemParamInit(void)
{
	Status_t ret = STATUS_OK;

	// Clear buffer for average and statistics calc
	cleanBuffer(buff);

	// Turns on MUX_EN pin to high
	HAL_GPIO_WritePin(MUX_ON_GPIO_Port, MUX_ON_Pin, 1);

	// Turns on DAC and set output on half range
	HAL_DAC_Start(&hdac1, DAC1_CHANNEL_1);
	HAL_DAC_SetValue(&hdac1, DAC1_CHANNEL_1, DAC_ALIGN_12B_R, 2048);

	/* FIR Init */
	arm_fir_init_f32(&S, FIR_SIZE, fir_coef, firState32, FIR_BLOCK_SIZE);

	// Start TIM7 with period 500us (2kHz) which start ADC in IRQ (HAL_TIM_PeriodElapsedCallback)
	HAL_TIM_Base_Start_IT(&htim7);

	return ret;
}

/*This function sets parameters for amplifier gain and turn ON/OFF DC compensation circuit (DC servo)*/
Status_t setParams(uint8_t servo_on,uint8_t gain)
{
	Status_t ret = STATUS_OK;
	/*DC Offset settings*/
	if(servo_on)
		HAL_GPIO_WritePin(SERVO_GPIO_Port, SERVO_Pin, 0);
	else
		HAL_GPIO_WritePin(SERVO_GPIO_Port, SERVO_Pin, 1);

	/*AMP Gain settings*/
	switch (gain)
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

