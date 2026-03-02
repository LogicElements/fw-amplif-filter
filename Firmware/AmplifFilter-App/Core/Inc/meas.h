/*
 * meas.h
 *
 *  Created on: Feb 23, 2026
 *      Author: Martin Stastny
 */

#ifndef INC_MEAS_H_
#define INC_MEAS_H_
#include "main.h"

#include "arm_math.h"
#include "configuration.h"

#define BUFF_SIZE			100						// Size of buffer for average calculation
#define k_16b				(2.5/(pow(2,16) - 1))	// For calculate voltage on ADC input (External 2.5 V reference)

#define FIR_SIZE			101						// Size of digital fir filter
#define FIR_BLOCK_SIZE		1						// Size of block processed by FIR
#define STAT_BLOCK_SIZE		100						// Block size for statistic calculations

#define ADC_16b_HALF_RANGE	32768					// Half of ADC range --> ~equals to 0 V
#define ADC_16b_hysteresis	10						// Hysteresis of ADC for zero-crossing

extern uint8_t fifo_loaded;
extern float raw_value;
extern uint8_t new_raw_data;
extern uint32_t adc_value;
extern float raw_filt;

extern const float fir_coef[FIR_SIZE];

extern float buff[BUFF_SIZE];
extern float data_buffer[2000];
extern uint16_t data_buff_cnt;
extern float data_filt[2000];

extern 	float raw_average;

extern arm_fir_instance_f32 S;
extern float firState32[FIR_BLOCK_SIZE + FIR_SIZE -1];

// For Statistic functions
extern float dsp_mean;		// Mean of measured data
extern float dsp_var;		// Variance of measured data
extern float dsp_rms;		// RMS of measured data
extern float dsp_std;		// Standard deviation of measured data

Status_t measProcess(uint32_t adc_value);

void cleanBuffer(float* buff);

void insertNewVal(float* buff);

float average(float* buff);

#endif /* INC_MEAS_H_ */
