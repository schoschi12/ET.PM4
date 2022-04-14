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
//#include "tm_stm32f4_ili9341_ltdc.h"

/******************************************************************************
 * Defines
 *****************************************************************************/
#define SAMPLES			256
#define B_SWEEP			260000		///<Bandwidth of VCO frequencyy
#define LIGHTSPEED		299792458
#define T_SWEEP			2			///<Period of frquency sweep in ms!
#define SAMPLING_RATE	512000		///<Sampling rate of ADC

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
	MEAS_timer_init(SAMPLING_RATE);
}

void DAC_sawtooth(void) {
	/*
	 //HAL_DAC_Start_DMA();
	 HAL_DAC_Start_DMA(&hdac1, DAC_CHANNEL_2, (uint32_t*) Wave_LUT, resolution,
	 DAC_ALIGN_12B_R);
	 //HAL_TIM_Base_Start(&htim2);
	 TIM2->CR1 |= TIM_CR1_CEN;			// Enable timer
	 while (true) {
	 */

}

float measure_range(void) {
return 1.1;
}
