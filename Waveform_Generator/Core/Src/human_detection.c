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

#include "human_detection.h"
#include "stm32f429i_discovery_lcd.h"

/******************************************************************************
 * Defines
 *****************************************************************************/

/******************************************************************************
 * Variables
 *****************************************************************************/

/******************************************************************************
 * Functions
 *****************************************************************************/

int measure_human_detection(float fft1[], float fft2[], int length) {
	double magThreshold = 0.5; ///<defines the minimum factor for secondary / tertiary frequency to be considered a signal
	int freqThreshold = 20;	///<defines the maximum frequency difference between secondary / tertiary signals

	double firstTest = 0;
	double secondTest = 0;
	double thirdTest = 0;
	int firstIndex;
	int secondIndex;
	int thirdIndex;
	for (int i = 1; i < length / 2; i++) {
		if ((double) fft1[i] > firstTest) {
			thirdTest = secondTest;
			secondTest = firstTest;
			firstTest = (double) fft1[i];
			firstIndex = i;
		} else if ((double) fft1[i] > secondTest) {
			thirdTest = secondTest;
			secondTest = (double) fft1[i];
			secondIndex = i;
		} else if ((double) fft1[i] > thirdTest) {
			thirdTest = (double) fft1[i];
			thirdIndex = i;
		}
	}
	double factor1 = secondTest / firstTest;
	double factor2 = thirdTest / firstTest;
	int distance1 = abs(firstIndex - secondIndex);
	int distance2 = abs(firstIndex - thirdIndex);
	char text[16];
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetFont(&Font24);
	snprintf(text, 15, "factor1 %4d", (int) (factor1 * 100));
	BSP_LCD_DisplayStringAt(0, 50, (uint8_t*) text, LEFT_MODE);
	snprintf(text, 15, "factor2 %4d", (int) (factor1 * 100));
	BSP_LCD_DisplayStringAt(0, 70, (uint8_t*) text, LEFT_MODE);
	snprintf(text, 15, "distance1 %4d", distance1);
	BSP_LCD_DisplayStringAt(0, 90, (uint8_t*) text, LEFT_MODE);
	snprintf(text, 15, "distance2 %4d", distance2);
	BSP_LCD_DisplayStringAt(0, 110, (uint8_t*) text, LEFT_MODE);

	if (factor2 > magThreshold && distance1 < freqThreshold
			&& distance2 < freqThreshold) {
		snprintf(text, 15, "is human");
		BSP_LCD_DisplayStringAt(0, 130, (uint8_t*) text, LEFT_MODE);
		return 1;
	}
	snprintf(text, 15, "is NOT human");
	BSP_LCD_DisplayStringAt(0, 130, (uint8_t*) text, LEFT_MODE);
	return 0;
}
