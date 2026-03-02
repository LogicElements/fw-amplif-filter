/*
 * meas.c
 *
 *  Created on: Feb 23, 2026
 *      Author: Martin Stastny
 */

#include "meas.h"

uint8_t fifo_loaded = 0;
float raw_value = 0.0f;
uint8_t new_raw_data = 0;
uint32_t adc_value = 0;
float raw_filt = 0.0;
float raw_average = 0.0;

float buff[BUFF_SIZE];

float data_buffer[2000];
uint16_t data_buff_cnt = 0;
float data_filt[2000];

arm_fir_instance_f32 S =
{
		.numTaps = 0,
		.pCoeffs = 0,
		.pState = 0

};
float firState32[FIR_BLOCK_SIZE + FIR_SIZE -1];

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

// For Statistic functions
float dsp_mean = 0.0;		// Mean of measured data
float dsp_var = 0.0;		// Variance of measured data
float dsp_rms = 0.0;		// RMS of measured data
float dsp_std = 0.0;		// Standard deviation of measured data

Status_t measProcess(uint32_t adc_value)
{
	Status_t ret = STATUS_OK;

	// Calc real voltage on AMP output
	raw_value = (((float) adc_value * k_16b) - 1.25) * 8;

	conf.meas.raw = raw_value;			// Map raw_value to internal registers
	insertNewVal(buff);					// Add raw_value to buffer
	raw_average = average(buff);		// Calc average on moving buffer
	conf.meas.average = raw_average;	// Map raw_average to internal registers

	/* FIR filter implementation */
	arm_fir_f32(&S, &raw_value, &raw_filt, FIR_BLOCK_SIZE);
	conf.dsp.fir = raw_filt;	// Map filtered value to internal register

	// Save raw and filtered value into buffers
	data_buffer[data_buff_cnt] = raw_value;
	data_filt[data_buff_cnt] = raw_filt;
	if (data_buff_cnt >= 2000)
		data_buff_cnt = 0;
	else
		data_buff_cnt++;

	/* Statistic functions */
	arm_mean_f32(buff, STAT_BLOCK_SIZE, &dsp_mean);
	arm_var_f32(buff, STAT_BLOCK_SIZE, &dsp_var);
	arm_std_f32(buff, STAT_BLOCK_SIZE, &dsp_std);
	arm_rms_f32(buff, STAT_BLOCK_SIZE, &dsp_rms);

	/* Map values to internal registers */
	conf.dsp.mean = dsp_mean;
	conf.dsp.var = dsp_var;
	conf.dsp.std = dsp_std;
	conf.dsp.rms = dsp_rms;

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
