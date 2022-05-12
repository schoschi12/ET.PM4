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

//#define ADC_FS			24000 //600			///< Sampling freq. => 12 samples for a 50Hz period

#define SAMPLING_RATE_SPEED	16000		///<Sampling rate of ADC
#define ADC_NUMS_SPEED	1024

#define FFT_SIZE		256
#define HIGHPASS_LENGTH	5
#define B_SWEEP			260000		///<Bandwidth of VCO frequencyy
#define LIGHTSPEED		299792458
//#define T_SWEEP			2			///<Period of frquency sweep in ms!
#define TRIANGLE_FREQ	250			///<Frequency of VCO triangle in Hz
#define SAMPLING_RATE_RANGE	TRIANGLE_FREQ * 2 * FFT_SIZE * 5 / 4	//1024000		///<Sampling rate of ADC
#define TRIANGLE_FLANK_PERIOD 1 / TRIANGLE_FREQ / 2
#define ADC_NUMS_RANGE FFT_SIZE * 2 * 5 / 4

/******************************************************************************
 * Functions
 *****************************************************************************/
void MEAS_GPIO_analog_init(void);
void MEAS_timer_init(int adc_fs);

void ADC_reset(void);
void ADC3_IN4_DMA_init(void);
void ADC3_IN4_DMA_start(void);
void ADC1_IN13_ADC2_IN5_dual_init(uint32_t nums, bool is_speed);
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
float complete_fft(uint32_t samples, float output[], int offset);
void clear_display(void);

void MEAS_show_data(void);


#endif
