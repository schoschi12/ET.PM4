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

/******************************************************************************
 * Defines
 *****************************************************************************/
#define SAMPLES			1024

/******************************************************************************
 * Variables
 *****************************************************************************/

/******************************************************************************
 * Functions
 *****************************************************************************/
void DrawBar(uint16_t bottomX, uint16_t bottomY, uint16_t maxHeight, uint16_t maxValue, float32_t value, uint16_t foreground, uint16_t background) {
    uint16_t height;
    height = (uint16_t)((float32_t)value / (float32_t)maxValue * (float32_t)maxHeight);
    if (height == maxHeight) {
        TM_ILI9341_DrawLine(bottomX, bottomY, bottomX, bottomY - height, foreground);
    } else if (height < maxHeight) {
        TM_ILI9341_DrawLine(bottomX, bottomY, bottomX, bottomY - height, foreground);
        TM_ILI9341_DrawLine(bottomX, bottomY - height, bottomX, bottomY - maxHeight, background);
    }
}

void fft_showcase(){

	//uint32_t data[SAMPLES];
	float fft[SAMPLES / 2];
	ADC1_IN13_ADC2_IN5_dual_init();
	ADC1_IN13_ADC2_IN5_dual_start();
	while(MEAS_data_ready == false);
	MEAS_data_readyfalse;


}
