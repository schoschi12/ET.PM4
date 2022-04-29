/** ***************************************************************************
 * @file
 * @brief Measuring voltages with the ADC(s) in different configurations
 *
 * ----------------------------------------------------------------------------
 * @author Hanspeter Hochreutener, hhrt@zhaw.ch
 * @date 17.06.2021
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdio.h>

#include "range.h"
#include "measuring.h"
#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "stm32f4xx_hal_dac.h"
#include "stm32f429i_discovery_lcd.h"
#include "speed.h"
//#include "tm_stm32f4_ili9341_ltdc.h"

/******************************************************************************
 * Defines
 *****************************************************************************/
#define RANGE_SAMPLES			256
#define SAMPLES_EXTRA			RANGE_SAMPLES / 0.8
#define B_SWEEP			260000000		///<Bandwidth of VCO frequencyy
#define LIGHTSPEED		299792458
#define T_SWEEP			2			///<Period of frquency sweep in ms!
#define TRIANGLE_FREQ	1000			///<Frequency of VCO triangle in Hz
#define SAMPLING_RATE	TRIANGLE_FREQ * 2 * SAMPLES_EXTRA	//1024000		///<Sampling rate of ADC
#define TRIANGLE_FLANK_PERIOD 1 / TRIANGLE_FREQ / 2

/******************************************************************************
 * Variables
 *****************************************************************************/

DAC_HandleTypeDef DacHandle;
DMA_HandleTypeDef hdma_dac_ch1;
//TIM_HandleTypeDef htim2;

/******************************************************************************
 * Functions
 *****************************************************************************/
void init_range(void) {
	//MEAS_timer_init(SAMPLING_RATE);
}

float range_from_frequency(double frequency_R) {
	return LIGHTSPEED * frequency_R / (2 * (B_SWEEP / (1 / (double)TRIANGLE_FREQ / 2))); //Doppler devided by frequency change per time
}

float measure_range(void) {
	float spectrum_up[RANGE_SAMPLES];
	float spectrum_down[RANGE_SAMPLES];
	MEAS_timer_init(SAMPLING_RATE);
	DAC_init();
	ADC1_IN13_ADC2_IN5_dual_init();
	tim_TIM7_TriangleWave(TRIANGLE_FREQ);
	tim_TIM7_TriangleWave_Start();
	ADC1_IN13_ADC2_IN5_dual_start();
	while (MEAS_data_ready == false)
		;
	MEAS_data_ready = false;
	complete_fft(RANGE_SAMPLES, spectrum_up, RANGE_SAMPLES * 0.1);
	complete_fft(RANGE_SAMPLES, spectrum_down,
	RANGE_SAMPLES + RANGE_SAMPLES * 0.1);
	double test = 0;
	int index_up;
	for (int i = 1; i < RANGE_SAMPLES / 2; i++) {
		if ((double) spectrum_up[i] > test) {
			test = (double) spectrum_up[i];
			index_up = i;
		}
	}
	test = 0;
	int index_down;
	for (int i = 1; i < RANGE_SAMPLES / 2; i++) {
		if ((double) spectrum_down[i] > test) {
			test = (double) spectrum_down[i];
			index_down = i;
		}
	}
	double f_1 = (double) index_up * SAMPLING_RATE / (double) RANGE_SAMPLES;
	double f_2 = (double) index_down * SAMPLING_RATE / (double) RANGE_SAMPLES;
	double frequency_R = (f_1 + f_2) / 2;
	double frequency_D = (f_1 - f_2) / 2;
	double range = range_from_frequency(frequency_R);
	double speed = (frequency_D / 158 * 1000);

	return range;
}
