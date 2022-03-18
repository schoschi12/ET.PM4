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
#include "stm32f429i_discovery_lcd.h"
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

/******************************************************************************
 * Functions
 *****************************************************************************/
void init_range(void){
	MEAS_timer_init(SAMPLING_RATE);
}

float measure_range(void){

}
