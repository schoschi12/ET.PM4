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
#include "stdbool.h"

#include "speed.h"
#include "measuring.h"
#include "range.h"
#include "stm32f429i_discovery_lcd.h"

/******************************************************************************
 * Defines
 *****************************************************************************/
#define SAMPLING_RATE	16000		///<Sampling rate of ADC
#define FILTER_LENGTH	9
/******************************************************************************
 * Variables
 *****************************************************************************/
double meanValue;
//uint32_t FILTER_LENGTH = 3;
double medianArr[FILTER_LENGTH];
double freqArr[FILTER_LENGTH];
double speed = 0;
double speed_shift = 0;
/******************************************************************************
 * Functions
 *****************************************************************************/
/*
 void DrawBar(uint16_t bottomX, uint16_t bottomY, uint16_t maxHeight, uint16_t maxValue, float value, uint16_t foreground, uint16_t background) {
 uint16_t height;
 height = (uint16_t)((float)value / (float)maxValue * (float)maxHeight);

 BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
 if (height == maxHeight) {
 BSP_LCD_DrawLine(bottomX, bottomY, bottomX, bottomY - height);
 } else if (height < maxHeight) {
 BSP_LCD_DrawLine(bottomX, bottomY, bottomX, bottomY - height);
 BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
 BSP_LCD_DrawLine(bottomX, bottomY - height, bottomX, bottomY - maxHeight);
 }
 }*/

void init_speed(void) {
#if !defined SIMULATION
	MEAS_timer_init(SAMPLING_RATE);
#endif
}

void swap(double *xp, double *yp) {
	double temp = *xp;
	*xp = *yp;
	*yp = temp;
}

void bubbleSort(double arr[], int n) {
	int i, j;
	for (i = 0; i < n - 1; i++)
		// Last i elements are already in place
		for (j = 0; j < n - i - 1; j++)
			if (arr[j] > arr[j + 1])
				swap(&arr[j], &arr[j + 1]);
}

float measure_speed(bool human_detection) {
	float maxValue;
	float spectrum[ADC_NUMS];
	meanValue = 0.0;
	for (int j = 0; j < FILTER_LENGTH; j++) {

		ADC1_IN13_ADC2_IN5_dual_init();
		ADC1_IN13_ADC2_IN5_dual_start();
		while (MEAS_data_ready == false)
			;
		MEAS_data_ready = false;

		maxValue = complete_fft(ADC_NUMS, spectrum);
		double test = 0;
		int index;
		for (int i = 0; i < (ADC_NUMS); i++) {
			if ((double) spectrum[i] > test) {
				test = (double) spectrum[i];
				index = i;
			}
		}

		if (human_detection) {
			//measure_human_detection(ADC_NUMS, fft1);
		} else {

			double freq = (double) index * SAMPLING_RATE / (double) ADC_NUMS;
			double freq_shift;
			if (index < ADC_NUMS / 2) {
				freq_shift = freq;
			} else {
				freq_shift = freq - 16000;
			}
			speed = freq / 158;
			speed_shift = freq_shift / 158;
			freqArr[j] = freq_shift;
			medianArr[j] = speed_shift;
			meanValue += speed_shift;
			//HAL_Delay(100);

		}
	}
	bubbleSort(medianArr, FILTER_LENGTH);
	double median;
	if (FILTER_LENGTH % 2 == 0) {
		median = (medianArr[FILTER_LENGTH / 2]
				+ medianArr[FILTER_LENGTH / 2 - 1]) / 2;
	} else {
		median = medianArr[FILTER_LENGTH / 2];
	}
	//	medianValues /= ((double) FILTER_LENGTH);
	clear_display();
	char text[16];
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetFont(&Font24);
	//		snprintf(text, 15, "F_raw %4dHz", (int) freq);
	//		BSP_LCD_DisplayStringAt(0, 50, (uint8_t*) text, LEFT_MODE);
	//		snprintf(text, 15, "S_raw %4dmm/s", (int) (speed * 1000));
	//		BSP_LCD_DisplayStringAt(0, 70, (uint8_t*) text, LEFT_MODE);
	//		snprintf(text, 15, "F_shift %4dHz", (int) freq_shift);
	//		BSP_LCD_DisplayStringAt(0, 90, (uint8_t*) text, LEFT_MODE);
	snprintf(text, 15, "Sp. %4dmm/s", (int) (median * 1000));
	BSP_LCD_DisplayStringAt(0, 30, (uint8_t*) text, LEFT_MODE);

	//medianValues = 0;
	return median;
}

void fft_showcase() {
	float maxValue;
	//uint32_t data[SAMPLES];
	float fft1[ADC_NUMS];
	//float fft2[ADC_NUMS];
	printf("test");
	//DAC_init();
	//ADC1_IN13_ADC2_IN5_dual_init();
	ADC1_IN13_ADC2_IN5_dual_init();
	ADC1_IN13_ADC2_IN5_dual_start();
	while (MEAS_data_ready == false)
		;
	/*
	 while(MEAS_data_ready == false){
	 DAC_increment();
	 printf("inc");
	 HAL_Delay(10);
	 }*/
	MEAS_data_ready = false;
	maxValue = complete_fft(ADC_NUMS, fft1);
	double test = 0;
	int index;
	for (int i = 1; i < ADC_NUMS / 2; i++) {
		if ((double) fft1[i] > test) {
			test = (double) fft1[i];
			index = i;
		}
	}
	char text[16];

	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetFont(&Font24);
	double freq = (double) index * 24000 / (double) ADC_NUMS;
	snprintf(text, 15, "Freq %4dHz", (int) freq);
	BSP_LCD_DisplayStringAt(0, 50, (uint8_t*) text, LEFT_MODE);
	snprintf(text, 15, "Speed %4dmm/s", (int) (freq / 158 * 1000));
	BSP_LCD_DisplayStringAt(0, 50, (uint8_t*) text, LEFT_MODE);
	HAL_Delay(500);

	const uint32_t Y_OFFSET = 260;
	const uint32_t X_SIZE = 240;
	/* Clear the display */
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillRect(0, 0, X_SIZE, Y_OFFSET + 1);
	for (int i = 0; i < (ADC_NUMS / 2); i++) {
		/* Draw FFT results */
		//DrawBar(30 + 2 * i, 220, 120, (uint16_t)maxValue, (float)fft2[(uint16_t)i], 0x1234, 0xFFFF);
	}
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
