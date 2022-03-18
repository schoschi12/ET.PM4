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

#include "speed.h"
#include "measuring.h"
#include "stm32f429i_discovery_lcd.h"

/******************************************************************************
 * Defines
 *****************************************************************************/
#define SAMPLES			256
#define SAMPLING_RATE	16000		///<Sampling rate of ADC

/******************************************************************************
 * Variables
 *****************************************************************************/

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

void init_speed(void){
	MEAS_timer_init(SAMPLING_RATE);
}

float measure_speed(void){
	float maxValue;
	float fft1[SAMPLES];
	float fft2[SAMPLES];
	ADC1_IN13_ADC2_IN5_dual_init();
	ADC1_IN13_ADC2_IN5_dual_start();
	while(MEAS_data_ready == false);
	MEAS_data_ready = false;
	maxValue = complete_fft(SAMPLES, fft1, fft2);
	double test = 0;
	int index;
	for (int i = 1; i < SAMPLES / 2; i++){
		if((double)fft1[i] > test){
			test = (double)fft1[i];
			index = i;
		}
	}

	char text[16];

	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetFont(&Font24);
	double freq = (double)index * SAMPLING_RATE/(double)ADC_NUMS;
	double speed = freq/158;
	snprintf(text, 15, "Freq %4dHz", (int)freq);
	BSP_LCD_DisplayStringAt(0, 50, (uint8_t *)text, LEFT_MODE);
	snprintf(text, 15, "Speed %4dmm/s", (int)(speed * 1000));
	BSP_LCD_DisplayStringAt(0, 50, (uint8_t *)text, LEFT_MODE);
	return speed;
}

void fft_showcase(){
	float maxValue;
	//uint32_t data[SAMPLES];
	float fft1[SAMPLES];
	float fft2[SAMPLES];
	printf("test");
	//DAC_init();
	ADC1_IN13_ADC2_IN5_dual_init();
	ADC1_IN13_ADC2_IN5_dual_start();
	while(MEAS_data_ready == false);
	/*
	while(MEAS_data_ready == false){
		DAC_increment();
		printf("inc");
		HAL_Delay(10);
	}*/
	MEAS_data_ready = false;
	maxValue = complete_fft(SAMPLES, fft1, fft2);
	double test = 0;
	int index;
	for (int i = 1; i < SAMPLES / 2; i++){
		if((double)fft1[i] > test){
			test = (double)fft1[i];
			index = i;
		}
	}

	char text[16];

	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	BSP_LCD_SetFont(&Font24);
	double freq = (double)index * 24000/(double)ADC_NUMS;
	snprintf(text, 15, "Freq %4dHz", (int)freq);
	BSP_LCD_DisplayStringAt(0, 50, (uint8_t *)text, LEFT_MODE);
	snprintf(text, 15, "Speed %4dmm/s", (int)(freq/158*1000));
	BSP_LCD_DisplayStringAt(0, 50, (uint8_t *)text, LEFT_MODE);
	HAL_Delay(500);

	const uint32_t Y_OFFSET = 260;
	const uint32_t X_SIZE = 240;
	/* Clear the display */
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillRect(0, 0, X_SIZE, Y_OFFSET+1);
	for (int i = 0; i < (SAMPLES / 2); i++) {
	    /* Draw FFT results */
		//DrawBar(30 + 2 * i, 220, 120, (uint16_t)maxValue, (float)fft2[(uint16_t)i], 0x1234, 0xFFFF);
	}
}
