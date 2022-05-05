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
//uint32_t MEAS_input_count;
extern bool DAC_active;

#define ADC_NUMS			1024 			///< Number of samples
//#define ADC_FS			24000 //600			///< Sampling freq. => 12 samples for a 50Hz period


/******************************************************************************
 * Functions
 *****************************************************************************/
void MEAS_GPIO_analog_init(void);
void MEAS_timer_init(int adc_fs);

void ADC_reset(void);
void ADC3_IN4_DMA_init(void);
void ADC3_IN4_DMA_start(void);
void ADC1_IN13_ADC2_IN5_dual_init(void);
void ADC1_IN13_ADC2_IN5_dual_start(void);
void ADC1_IN13_ADC2_IN5_dual_stop(void);
void GPIO_Amplifier_init(void);
void GPIO_set_gain(void);
void GPIO_reset_gain(void);
void GPIO_Buzzer_init(void);
void GPIO_set_Buzzer(void);
void GPIO_reset_Buzzer(void);
void GPIO_LED_init(void);

void fft_shift(float input[], float output[], int length);
float complete_fft(uint32_t samples, float output[]);

void MEAS_show_data(void);


#endif
