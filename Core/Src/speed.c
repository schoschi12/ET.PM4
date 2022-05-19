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
#define FILTER_LENGTH	5
/******************************************************************************
 * Variables
 *****************************************************************************/
double meanValue;
//uint32_t FILTER_LENGTH = 3;
double medianArr[FILTER_LENGTH];
double freqArr[FILTER_LENGTH];
double speed = 0;
double speed_shift = 0;
double speed2 = 0;
double speed2_shift = 0;
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
	MEAS_timer_init(SAMPLING_RATE_SPEED);
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

float measure_speed(bool human_detection, bool human_set) {
	float spectrum[ADC_NUMS_SPEED];
	meanValue = 0.0;
	for (int j = 0; j < FILTER_LENGTH; j++) {

		ADC1_IN13_ADC2_IN5_dual_init(ADC_NUMS_SPEED, true);
		ADC1_IN13_ADC2_IN5_dual_start();
		while (MEAS_data_ready == false)
			;
		MEAS_data_ready = false;

		double maxValue = complete_fft(ADC_NUMS_SPEED, spectrum, 0);
		double totalAmplitude = 0;
		double cutAmplitude = 0;
		double factor = 0.3;
		double factor2 = 15;
		int index1 = 0;
		int index2 = 0;
		int totalIndex;
		int cutIndex = -1;
		int cutter = 50;
		/*
		 for (int i = 0; i < ADC_NUMS_SPEED; i++) {
		 if ((double) spectrum[i] > totalAmplitude) {
		 totalAmplitude = (double) spectrum[i];
		 index = i;
		 }
		 }*/
		if (human_set) {
			ADC1_IN13_ADC2_IN5_dual_init(ADC_NUMS_SPEED, true);
			ADC1_IN13_ADC2_IN5_dual_start();
			while (MEAS_data_ready == false)
				;
			MEAS_data_ready = false;

			maxValue = complete_fft(ADC_NUMS_SPEED, spectrum, 0);
			double test = 0;
			int index;
			for (int i = 0; i < (ADC_NUMS_SPEED); i++) {
				if ((double) spectrum[i] > test) {
					test = (double) spectrum[i];
					index = i;
				}
			}
			double freq = (double) index * SAMPLING_RATE_SPEED
					/ (double) ADC_NUMS_SPEED;
			double freq_shift;
			if (index < ADC_NUMS_SPEED / 2) {
				freq_shift = freq;
			} else {
				freq_shift = freq - 16000;
			}
			speed = freq / 158;
			speed_shift = freq_shift / 158;
			freqArr[j] = freq_shift;
			medianArr[j] = speed_shift;
			meanValue += speed_shift;
		} else {
			double spectrumAverage = 0;
			for (int i = cutter; i < (ADC_NUMS_SPEED - cutter); i++) {
				if ((double) spectrum[i] > cutAmplitude) {
					cutAmplitude = (double) spectrum[i];
					cutIndex = i;
				}
				spectrumAverage += spectrum[i]
						/ (ADC_NUMS_SPEED - (2 * cutter));
			}
			if (cutAmplitude > (maxValue * factor)) {
				index1 = cutIndex;
			}

			if (cutAmplitude > (factor2 * spectrumAverage)) {
				index2 = cutIndex;
			}
			double freq = (double) index1 * SAMPLING_RATE_SPEED
					/ (double) ADC_NUMS_SPEED;
			double freq_shift;
			double freq2 = (double) index2 * SAMPLING_RATE_SPEED
					/ (double) ADC_NUMS_SPEED;
			double freq2_shift;
			if (index1 < ADC_NUMS_SPEED / 2) {
				freq_shift = freq;
			} else {
				freq_shift = freq - 16000;
			}
			if (index2 < ADC_NUMS_SPEED / 2) {
				freq2_shift = freq2;
			} else {
				freq2_shift = freq2 - 16000;
			}
			speed = freq / 158;
			speed_shift = freq_shift / 158;
			speed2 = freq2 / 158;
			speed2_shift = freq2_shift / 158;

			freqArr[j] = freq_shift;
			medianArr[j] = speed_shift;
			meanValue += speed_shift;
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
	snprintf(text, 15, "Speed:");
	BSP_LCD_DisplayStringAt(0, 30, (uint8_t*) text, LEFT_MODE);
	snprintf(text, 15, "%.2fm/s", FILTER_LENGTH, median);
	BSP_LCD_DisplayStringAt(0, 50, (uint8_t*) text, LEFT_MODE);
	//medianValues = 0;
	return 0;	//median;
}

void fft_showcase() {
	float maxValue;
	//uint32_t data[SAMPLES];
	float fft1[ADC_NUMS_SPEED];
	//float fft2[ADC_NUMS];
	printf("test");
	//DAC_init();
	//ADC1_IN13_ADC2_IN5_dual_init();
	ADC1_IN13_ADC2_IN5_dual_init(ADC_NUMS_SPEED, true);
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
	maxValue = complete_fft(ADC_NUMS_SPEED, fft1, 0);
	double test = 0;
	int index;
	for (int i = 1; i < ADC_NUMS_SPEED / 2; i++) {
		if ((double) fft1[i] > test) {
			test = (double) fft1[i];
			index = i;
		}
	}
	char text[16];

	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetFont(&Font24);
	double freq = (double) index * 24000 / (double) ADC_NUMS_SPEED;
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
	for (int i = 0; i < (ADC_NUMS_SPEED / 2); i++) {
		/* Draw FFT results */
		//DrawBar(30 + 2 * i, 220, 120, (uint16_t)maxValue, (float)fft2[(uint16_t)i], 0x1234, 0xFFFF);
	}
}
