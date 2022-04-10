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

//DAC_HandleTypeDef DacHandle;
//DMA_HandleTypeDef hdma_dac_ch1;

int output = 0;
//TIM_HandleTypeDef htim2;

/******************************************************************************
 * Functions
 *****************************************************************************/
void init_range(void) {
	MEAS_timer_init(SAMPLING_RATE);
}

void DAC_sawtooth(void) {

	MX_DMA_Init();
	DAC_timer_init();
	DAC_init_MX();
	generate_triangle();
}
/*
 void TIM2_IRQHandler(void) {
 if (TIM2->SR & 0x01) {
 TIM2->SR &= ~(1UL); //reset flag
 if(output == 0){
 BSP_LED_On(LED4);
 output = 1;
 }else{
 BSP_LED_Off(LED4);
 output = 0;
 }
 }
 }*/

float measure_range(void) {
	return 1.1;
}
