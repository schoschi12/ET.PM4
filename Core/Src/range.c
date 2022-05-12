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
#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "stm32f4xx_hal_dac.h"
#include "stm32f429i_discovery_lcd.h"
//#include "tm_stm32f4_ili9341_ltdc.h"

/******************************************************************************
 * Defines
 *****************************************************************************/
//#define SAMPLES			256

/******************************************************************************
 * Variables
 *****************************************************************************/

DAC_HandleTypeDef DacHandle;
DMA_HandleTypeDef hdma_dac_ch1;
bool DAC_active = false;				///< DAC output active?
bool upcounting = true;
static uint32_t DAC_sample = 0;
//TIM_HandleTypeDef htim2;

/******************************************************************************
 * Functions
 *****************************************************************************/

float range_from_frequency(double frequency_R) {
	double dfdt = 2 * B_SWEEP * TRIANGLE_FREQ;
	return LIGHTSPEED * frequency_R / dfdt; //Doppler devided by frequency change per time
}

float measure_range(void) {
	float spectrum_up[FFT_SIZE];
	float spectrum_down[FFT_SIZE];
	MEAS_timer_init(SAMPLING_RATE_RANGE);
	ADC1_IN13_ADC2_IN5_dual_init(ADC_NUMS_RANGE, false);
	tim_TIM7_TriangleWave(TRIANGLE_FREQ);
	tim_TIM7_TriangleWave_Start();
	ADC1_IN13_ADC2_IN5_dual_start();
	while (MEAS_data_ready == false)
		;
	MEAS_data_ready = false;
	complete_fft(FFT_SIZE, spectrum_up, ADC_NUMS_RANGE * 0.05);
	complete_fft(FFT_SIZE, spectrum_down, ADC_NUMS_RANGE * 0.55);
	double test = 0;
	int index_up = 0;
	for (int i = 1; i < FFT_SIZE / 2; i++) {
		if ((double) spectrum_up[i] > test) {
			test = (double) spectrum_up[i];
			index_up = i;
		}
	}
	test = 0;
	int index_down = 0;
	for (int i = 1; i < FFT_SIZE / 2; i++) {
		if ((double) spectrum_down[i] > test) {
			test = (double) spectrum_down[i];
			index_down = i;
		}
	}
	double f_1 = (double) index_up * 24000 / (double) 1024;
	double f_2 = (double) index_down * 24000 / (double) 1024;
	double frequency_R = (f_1 + f_2) / 2;
	double frequency_S = (f_1 - f_2) / 2;
	double range = range_from_frequency(frequency_R);
	double speed = (frequency_S / 158 * 1000);
	clear_display();
	char text[16];
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetFont(&Font24);
	snprintf(text, 15, "range %4dm", (int)range);
	BSP_LCD_DisplayStringAt(0, 30, (uint8_t*) text, LEFT_MODE);
	return range;
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



/** ***************************************************************************
 * @brief initialize timercounter 7 with variable frequency with interrupt
 *
 * Prescaler = 42e6/DAC_frequency/((4096/DAC_STEP_SIZE)*2)
 *****************************************************************************/
void tim_TIM7_TriangleWave(uint32_t DAC_frequency) {
	//Enable TIM7 timer
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
	//1 Pulse mode enable
	TIM7->CR1 &= ~(TIM_CR1_OPM);
	//Mode --> RESET
	TIM7->CR2 &= ~(TIM_CR2_MMS);

	//Prescaler
	TIM7->PSC = 42e6 / DAC_frequency / ((4096 / DAC_STEP_SIZE) * 2) - 1;
	//Period
	TIM7->ARR = 2 - 1; //42MHz

	//Clear Update Interrupt
	TIM7->SR &= ~(1UL << 0);
	//Enable update interrupt
	TIM7->DIER |= 0x01;
	//Enable NVIC Interrupt
	NVIC_EnableIRQ(TIM7_IRQn);

}
void tim_TIM7_TriangleWave_Start(void) {
	//Start perodic timer
	TIM7->CR1 |= 0x01;

	//Activate DAC
	DAC_active = true;
}
void tim_TIM7_TriangleWave_Stop(void) {
	//Stop perodic timer
	TIM7->CR1 &= ~(0x01);

	//Deactivate DAC
	DAC_active = false;
}

/** ***************************************************************************
 * @brief interrupt service routine of timercounter 7. Occurs periodically and
 * counts timer_value_ms up.
 *****************************************************************************/
void TIM7_IRQHandler(void) {
	if (TIM7->SR & 0x01) {
		TIM7->SR &= ~(1UL); //reset flag
		if (DAC_active) {
			DAC_increment();
		}

	}
}
