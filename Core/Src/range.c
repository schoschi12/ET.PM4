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
#include <math.h>

#include "range.h"
#include "measuring.h"
#include "speed.h"
#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "stm32f4xx_hal_dac.h"
#include "stm32f429i_discovery_lcd.h"
//#include "tm_stm32f4_ili9341_ltdc.h"

/******************************************************************************
 * Defines
 *****************************************************************************/
//#define SAMPLES			256
#define B_SWEEP			260000		///<Bandwidth of VCO frequencyy
#define LIGHTSPEED		299792458
//#define T_SWEEP			1			///<Rise and fall time of triangle wave in ms!
#define SAMPLING_RATE	16000		///<Sampling rate of ADC

/******************************************************************************
 * Variables
 *****************************************************************************/

bool DAC_active = false;				///< DAC output active?
bool upcounting = true;
float fft1[ADC_NUMS];
float fft2[ADC_NUMS];
float df1;
float df2;

static uint32_t DAC_sample = 0;			///< DAC output value
static float t_sweep = 0.001;
static uint32_t state = 0;

/******************************************************************************
 * Functions
 *****************************************************************************/
void init_range(void) {
	MEAS_timer_init(SAMPLING_RATE);
}

void measure_range(void) {
	float distance;
	//float sum = 0.0f;
	//float df1;
	//float df2;
	float frequency_speed;
	float frquency_distance;

	switch (state) {
	case 0:
		ADC1_IN13_ADC2_IN5_dual_init();
		ADC1_IN13_ADC2_IN5_dual_start();
		//tim_TIM7_TriangleWave(500);
		tim_TIM7_TriangleWave_Start();
		if (getStatus() == true) { //ramp up
			state = 1;
		}
		break;
	case 1:
		//Sample frequency 16kHz, 256 Samples => 0.016s per measurement
		//Triangle rising time = 1ms
		//calculate mean value of samples during ramp up and down

		while (MEAS_data_ready == false)
			;
		MEAS_data_ready = false;

		df1 = complete_fft(ADC_NUMS, fft1);
		state = 2;
		break;
	case 2:

		if (getStatus() == false) { //ramp down
			ADC1_IN13_ADC2_IN5_dual_init();
			ADC1_IN13_ADC2_IN5_dual_start();
			while (MEAS_data_ready == false)
				;
			MEAS_data_ready = false;

			df2 = complete_fft(ADC_NUMS, fft2);
			state = 0;
			//showdata();

			frquency_distance = (df1 + df2) / 2;
			frequency_speed = abs(df1 - df2) / 2;

			distance = (float) (LIGHTSPEED * abs(frquency_distance)
					/ (2 * B_SWEEP / t_sweep));

			char text[16];
			BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
			BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
			BSP_LCD_SetFont(&Font24);

			snprintf(text, 15, "Freq_raw %4dHz", (int) frquency_distance);
			BSP_LCD_DisplayStringAt(0, 50, (uint8_t*) text, LEFT_MODE);
			snprintf(text, 15, "Distance_raw %4dm", (int) distance);
			BSP_LCD_DisplayStringAt(0, 70, (uint8_t*) text, LEFT_MODE);

			//ADC1_IN13_ADC2_IN5_dual_stop();
			//tim_TIM7_TriangleWave_Stop();
			//DAC_reset();
			//stopDAC();
		}
		break;
	default:
		state = 0;
		//ADC1_IN13_ADC2_IN5_dual_stop();
		//tim_TIM7_TriangleWave_Stop();
		//DAC_reset();
	}

	//return distance;
}

/** ***************************************************************************
 * @brief Resets the DAC
 *
 * when it is no longer used.
 *****************************************************************************/
void DAC_reset(void) {
	RCC->APB1RSTR |= RCC_APB1RSTR_DACRST;	// Reset the DAC
	RCC->APB1RSTR &= ~RCC_APB1RSTR_DACRST;	// Release reset of the DAC
}

/** ***************************************************************************
 * @brief Initialize the DAC
 *
 * The output used is DAC_OUT2 = GPIO PA5
 * @n As DAC_OUT2 = GPIO PA5 (= same GPIO as ADC12_IN5)
 * it is possible to monitor the output voltage DAC_OUT2 by the input ADC12_IN5.
 *****************************************************************************/
void DAC_init(void) {
	__HAL_RCC_DAC_CLK_ENABLE();			// Enable Clock for DAC
	DAC->CR |= DAC_CR_EN2;				// Enable DAC output 2
}

/** ***************************************************************************
 * @brief Increment the DAC value and write it to the output
 *
 *****************************************************************************/
void DAC_increment(void) {
	//DAC_sample += 68;				// Increment DAC output
	//if (DAC_sample >= (1UL << ADC_DAC_RES)) { DAC_sample = 0; }	// Go to 0
	//DAC->DHR12R2 = DAC_sample;		// Write new DAC output value

	if (DAC_sample < 4095 - DAC_STEP_SIZE && upcounting == true) {
		DAC_sample += DAC_STEP_SIZE;				// Increment DAC output
		DAC->DHR12R2 = DAC_sample;
	} else {
		upcounting = false;
	}

	if (DAC_sample != 0 && upcounting == false) {
		DAC_sample -= DAC_STEP_SIZE;				// Decrement DAC output
		DAC->DHR12R2 = DAC_sample;
	} else {
		upcounting = true;
	}
}

bool getStatus(void) {
	return upcounting;
}
