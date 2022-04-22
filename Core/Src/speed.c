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
#include "stm32f429i_discovery_lcd.h"

/******************************************************************************
 * Defines
 *****************************************************************************/
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

void init_speed(void) {
#if !defined SIMULATION
	MEAS_timer_init(SAMPLING_RATE);
#endif
}

float measure_speed(bool human_detection) {
	float maxValue;
	float fft1[ADC_NUMS];
	//float fft2[ADC_NUMS];
#if !defined SIMULATION
	ADC1_IN13_ADC2_IN5_dual_init();
	ADC1_IN13_ADC2_IN5_dual_start();
	while (MEAS_data_ready == false)
		;
	MEAS_data_ready = false;
//#else
	//artificial_signal(200, 16000, ADC_NUMS);
#endif
	maxValue = complete_fft(ADC_NUMS, fft1);
	double test = 0;
	int index;
	for (int i = 0; i < (ADC_NUMS); i++) {
		if ((double) fft1[i] > test) {
			test = (double) fft1[i];
			index = i;
		}
	}

	if (human_detection) {
		measure_human_detection(ADC_NUMS, fft1);
	} else {
		char text[16];
		BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
		BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
		BSP_LCD_SetFont(&Font24);
		double freq = (double) index * SAMPLING_RATE / (double) ADC_NUMS;
		double freq_shift;
		if(index < ADC_NUMS / 2){
			freq_shift = freq;
		}else{
			freq_shift = freq - 16000;
		}
		double speed = freq / 158;
		double speed_shift = freq_shift / 158;
		snprintf(text, 15, "F_raw %4dHz", (int) freq);
		BSP_LCD_DisplayStringAt(0, 50, (uint8_t*) text, LEFT_MODE);
		snprintf(text, 15, "S_raw %4dmm/s", (int) (speed * 1000));
		BSP_LCD_DisplayStringAt(0, 70, (uint8_t*) text, LEFT_MODE);
		snprintf(text, 15, "F_shift %4dHz", (int) freq_shift);
		BSP_LCD_DisplayStringAt(0, 90, (uint8_t*) text, LEFT_MODE);
		snprintf(text, 15, "S_shift %4dmm/s", (int) (speed_shift * 1000));
		BSP_LCD_DisplayStringAt(0, 110, (uint8_t*) text, LEFT_MODE);
		return speed;
	}

}

void fft_showcase() {
	float maxValue;
	//uint32_t data[SAMPLES];
	float fft1[ADC_NUMS];
	float fft2[ADC_NUMS];
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
 * @brief initialize timercounter 7 to a milisecond based timer with enabled
 * interrupt
 * 84MHz --> 84MHz/((4096/DAC_STEP)*2)
 *****************************************************************************/
void tim_TIM7_periodicConfig(uint32_t DAC_frequency) {
	//Enable TIM7 timer
	RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
	//1 Pulse mode enable
	TIM7->CR1 &= ~(TIM_CR1_OPM);
	//Mode --> RESET
	TIM7->CR2 &= ~(TIM_CR2_MMS);

	//Prescaler
	TIM7->PSC = 42e6/DAC_frequency/((4096/DAC_STEP)*2) - 1;
	//Period
	TIM7->ARR = 2 - 1; //42MHz

	//Clear Update Interrupt
	TIM7->SR &= ~(1UL << 0);
	//Enable update interrupt
	TIM7->DIER |= 0x01;
	//Enable NVIC Interrupt
	NVIC_EnableIRQ(TIM7_IRQn);
	//Start perodic timer
	TIM7->CR1 |= 0x01;
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
