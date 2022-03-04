/** ***************************************************************************
 * @file
 * @brief See measuring.c
 *
 * Prefixes MEAS, ADC, DAC
 *
 *****************************************************************************/

#ifndef MEAS_H_
#define MEAS_H_


/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdbool.h>


/******************************************************************************
 * Defines
 *****************************************************************************/
extern bool MEAS_data_ready;
uint32_t MEAS_input_count;
extern bool DAC_active;


/******************************************************************************
 * Functions
 *****************************************************************************/
void MEAS_GPIO_analog_init(void);
void MEAS_timer_init(void);
void DAC_reset(void);
void DAC_init(void);
void DAC_increment(void);
void ADC_reset(void);
void ADC3_IN4_DMA_init(void);
void ADC3_IN4_DMA_start(void);
void ADC1_IN13_ADC2_IN5_dual_init(void);
void ADC1_IN13_ADC2_IN5_dual_start(void);

float complete_fft(uint32_t samples, float result1[], float result2[]);

void MEAS_show_data(void);


#endif
