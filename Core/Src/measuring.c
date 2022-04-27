/** ***************************************************************************
 * @file
 * @brief Measuring voltages with the ADC(s) in different configurations
 *
 *
 * Demonstrates different ADC (Analog to Digital Converter) modes
 * ==============================================================
 *
 * - ADC in single conversion mode
 * - ADC triggered by a timer and with interrupt after end of conversion
 * - ADC combined with DMA (Direct Memory Access) to fill a buffer
 * - Dual mode = simultaneous sampling of two inputs by two ADCs
 * - Scan mode = sequential sampling of two inputs by one ADC
 * - Simple DAC output is demonstrated as well
 * - Analog mode configuration for GPIOs
 * - Display recorded data on the graphics display
 *
 * Peripherals @ref HowTo
 *
 * @image html demo_screenshot_board.jpg
 *
 *
 * HW used for the demonstrations
 * ==============================
 * A simple HW was used for testing the code.
 * It is connected to the pins marked in red in the above image.
 *
 * @image html demo_board_schematic.png
 *
 * Of course the code runs also without this HW.
 * Simply connect a signal or waveform generator to the input(s).
 *
 *
 * @anchor HowTo
 * How to Configure the Peripherals: ADC, TIMER and DMA
 * ====================================================
 *
 * All the peripherals are accessed by writing to or reading from registers.
 * From the programmer’s point of view this is done exactly as
 * writing or reading the value of a variable.
 * @n Writing to a register configures the HW of the associated peripheral
 * to do what is required.
 * @n Reading from a registers gets status and data from the HW peripheral.
 *
 * The information on which bits have to be set to get a specific behavior
 * is documented in the <b>reference manual</b> of the mikrocontroller.
 *
 *
 * ----------------------------------------------------------------------------
 * @author Hanspeter Hochreutener, hhrt@zhaw.ch
 * @date 17.06.2021
 *****************************************************************************/

/******************************************************************************
 * Includes
 *****************************************************************************/
#include <stdio.h>
#include "stm32f4xx.h"
#include "stm32f429i_discovery.h"
#include "stm32f429i_discovery_lcd.h"
#include "stm32f429i_discovery_ts.h"

#include "arm_math.h"
#include "measuring.h"

/******************************************************************************
 * Defines
 *****************************************************************************/
#define ADC_DAC_RES		12			///< Resolution
#define ADC_CLOCK		84000000	///< APB2 peripheral clock frequency
#define ADC_CLOCKS_PS	15			///< Clocks/sample: 3 hold + 12 conversion
#define TIM_CLOCK		84000000	///< APB1 timer clock frequency
#define TIM_TOP			9			///< Timer top value
#define TIM_PRESCALE	(TIM_CLOCK/ADC_FS/(TIM_TOP+1)-1) ///< Clock prescaler
#define IFFT_FLAG 0
#define BIT_REVERSE_FLAG 1

/******************************************************************************
 * Variables
 *****************************************************************************/
bool MEAS_data_ready = false;			///< New data is ready
uint32_t MEAS_input_count = 1;			///< 1 or 2 input channels?


static uint32_t ADC_sample_count = 0;	///< Index for buffer
static uint32_t ADC_samples[2 * ADC_NUMS];///< ADC values of max. 2 input channels



/******************************************************************************
 * Functions
 ******************************************************************************/

/** ***************************************************************************
 * @brief Configure GPIOs in push-pull mode
 *
 * @note configuration of both GPIO pins for the amplifier
 * - I(t) = PE03
 * - Q(t) = PE01
 *****************************************************************************/
void GPIO_Amplifier_init(void)
{
		GPIO_InitTypeDef GPIO_InitStruct = { 0 };

		/* GPIO Ports Clock Enable */
		__GPIOE_CLK_ENABLE();

		/*Configure GPIO pin : PE01 */
		GPIO_InitStruct.Pin = GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;     // digital Output
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

		/*Configure GPIO pin : PE03 */
		GPIO_InitStruct.Pin = GPIO_PIN_3;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;     // digital Output
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/** ***************************************************************************
 * @brief set GPIO Pins
 *
 * @note doubles the amplification of each channel
 * - I(t) = PE03
 * - Q(t) = PE01
 *****************************************************************************/
void GPIO_set_gain(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
}

/** ***************************************************************************
 * @brief reset GPIO Pins
 *
 * @note reset amplification to normal approx. 60dB at 1000Hz
 * - I(t) = PE03
 * - Q(t) = PE01
 *****************************************************************************/
void GPIO_reset_gain(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
}

/** ***************************************************************************
 * @brief Configure GPIOs in push-pull mode
 *
 * @note configuration of PE05 for the buzzer
 *
 *****************************************************************************/
void GPIO_Buzzer_init(void)
{
		GPIO_InitTypeDef GPIO_InitStruct = { 0 };

		/* GPIO Ports Clock Enable */
		__GPIOE_CLK_ENABLE();


		/*Configure GPIO pin : PE05 */
		GPIO_InitStruct.Pin = GPIO_PIN_5;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;     // digital Output
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

void GPIO_LED_init(void)
{
		GPIO_InitTypeDef GPIO_InitStruct  = { 0 };

		/* GPIO Ports Clock Enable */
		__GPIOF_CLK_ENABLE();


		/*Configure GPIO pin : PF06 */
		GPIO_InitStruct.Pin = GPIO_PIN_6;
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;     // digital Output
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
		HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

}
/** ***************************************************************************
 * @brief activate buzzer
 *
 * @note activate buzzer
 *
 *****************************************************************************/
void GPIO_set_Buzzer(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_SET);
}
/** ***************************************************************************
 * @brief deactivate buzzer
 *
 * @note deactivate buzzer
 *
 *****************************************************************************/
void GPIO_reset_Buzzer(void)
{
	HAL_GPIO_WritePin(GPIOE, GPIO_PIN_5, GPIO_PIN_RESET);
}

/** ***************************************************************************
 * @brief Configure GPIOs in analog mode.
 *
 * @note The input number for the ADCs is not equal to the GPIO pin number!
 * - ADC3_IN4 = GPIO PF6
 * - ADC123_IN13 = GPIO PC3
 * - ADC12_IN5 = GPIO PA5
 * - DAC_OUT2 = GPIO PA5 (= same GPIO as ADC12_IN5)
 *****************************************************************************/
void MEAS_GPIO_analog_init(void) {
	__HAL_RCC_GPIOF_CLK_ENABLE();		// Enable Clock for GPIO port F
	GPIOF->MODER |= (GPIO_MODER_MODER6_Msk);// Analog mode for PF6 = ADC3_IN4
	__HAL_RCC_GPIOC_CLK_ENABLE();		// Enable Clock for GPIO port C
	GPIOC->MODER |= (GPIO_MODER_MODER3_Msk);// Analog mode for PC3 = ADC123_IN13
	__HAL_RCC_GPIOA_CLK_ENABLE();		// Enable Clock for GPIO port A
	GPIOA->MODER |= (GPIO_MODER_MODER5_Msk);	// Analog mode for PA5 ADC12_IN5
	GPIOA->MODER |= (GPIO_MODER_MODER6_Msk);	// Analog mode for PA5 ADC12_IN5
}



/** ***************************************************************************
 * @brief Resets the ADCs and the timer
 *
 * to make sure the different demos do not interfere.
 *****************************************************************************/
void ADC_reset(void) {
	RCC->APB2RSTR |= RCC_APB2RSTR_ADCRST;	// Reset ADCs
	RCC->APB2RSTR &= ~RCC_APB2RSTR_ADCRST;	// Release reset of ADCs
	TIM2->CR1 &= ~TIM_CR1_CEN;				// Disable timer
}

/** ***************************************************************************
 * @brief Configure the timer to trigger the ADC(s)
 *
 * @note For debugging purposes the timer interrupt might be useful.
 *****************************************************************************/
void MEAS_timer_init(int adc_fs) {

	int tim_prescale = (TIM_CLOCK / adc_fs / (TIM_TOP + 1) - 1); ///< Clock prescaler
	__HAL_RCC_TIM2_CLK_ENABLE();		// Enable Clock for TIM2
	TIM2->PSC = tim_prescale;			// Prescaler for clock freq. = 1MHz
	TIM2->ARR = TIM_TOP;				// Auto reload = counter top value
	TIM2->CR2 |= TIM_CR2_MMS_1; 		// TRGO on update
	/* If timer interrupt is not needed, comment the following lines */
	TIM2->DIER |= TIM_DIER_UIE;			// Enable update interrupt
	NVIC_ClearPendingIRQ(TIM2_IRQn);	// Clear pending interrupt on line 0
	NVIC_EnableIRQ(TIM2_IRQn);			// Enable interrupt line 0 in the NVIC
}

/** ***************************************************************************
 * @brief Initialize ADCs, timer and DMA for simultaneous dual ADC acquisition
 *
 * Uses ADC1 and ADC2 in dual mode and DMA2_Stream4 Channel0
 * @n The ADC1 trigger is set to TIM2 TRGO event
 * @n ADC1 is the master and simultaneously triggers ADC2.
 * @n Both converted data from ADC1 and ADC2 are packed into a 32-bit register
 * in this way: <b> ADC_CDR[31:0] = ADC2_DR[15:0] | ADC1_DR[15:0] </b>
 * and are transfered with the DMA in one single step.
 * @n The ADC1 triggers the DMA2_Stream4 to transfer the new data directly
 * to memory without CPU intervention.
 * @n The DMA triggers the transfer complete interrupt when all data is ready.
 * @n The input used with ADC1 is ADC123_IN13 = GPIO PC3
 * @n The input used with ADC2 is ADC12_IN5 = GPIO PA5
 *****************************************************************************/

//Zu ändern: auf PC1 statt PA5 und PC3
void ADC1_IN13_ADC2_IN5_dual_init(void) {
	MEAS_input_count = 2;				// Only 1 input is converted
	__HAL_RCC_ADC1_CLK_ENABLE();		// Enable Clock for ADC1
	__HAL_RCC_ADC2_CLK_ENABLE();		// Enable Clock for ADC2
	ADC->CCR |= ADC_CCR_DMA_1;			// Enable DMA mode 2 = dual DMA
	ADC->CCR |= ADC_CCR_MULTI_1 | ADC_CCR_MULTI_2; // ADC1 and ADC2
	ADC1->CR2 |= (1UL << ADC_CR2_EXTEN_Pos);	// En. ext. trigger on rising e.
	ADC1->CR2 |= (6UL << ADC_CR2_EXTSEL_Pos);	// Timer 2 TRGO event
	ADC1->SQR3 |= (13UL << ADC_SQR3_SQ1_Pos);	// Input 13 = first conversion
	ADC2->SQR3 |= (11UL << ADC_SQR3_SQ1_Pos);	// Input 5 = first conversion
	__HAL_RCC_DMA2_CLK_ENABLE();		// Enable Clock for DMA2
	DMA2_Stream4->CR &= ~DMA_SxCR_EN;	// Disable the DMA stream 4
	while (DMA2_Stream4->CR & DMA_SxCR_EN) {
		;
	}	// Wait for DMA to finish
	DMA2->HIFCR |= DMA_HIFCR_CTCIF4;	// Clear transfer complete interrupt fl.
	DMA2_Stream4->CR |= (0UL << DMA_SxCR_CHSEL_Pos);	// Select channel 0
	DMA2_Stream4->CR |= DMA_SxCR_PL_1;		// Priority high
	DMA2_Stream4->CR |= DMA_SxCR_MSIZE_1;	// Memory data size = 32 bit
	DMA2_Stream4->CR |= DMA_SxCR_PSIZE_1;	// Peripheral data size = 32 bit
	DMA2_Stream4->CR |= DMA_SxCR_MINC;	// Increment memory address pointer
	DMA2_Stream4->CR |= DMA_SxCR_TCIE;	// Transfer complete interrupt enable
	DMA2_Stream4->NDTR = ADC_NUMS;		// Number of data items to transfer
	DMA2_Stream4->PAR = (uint32_t) &ADC->CDR;	// Peripheral register address
	DMA2_Stream4->M0AR = (uint32_t) ADC_samples;// Buffer memory loc. address
}

/** ***************************************************************************
 * @brief Start DMA, ADC and timer
 *
 *****************************************************************************/
void ADC1_IN13_ADC2_IN5_dual_start(void) {
	//DAC_init(); ///////////////////////////////
	DMA2_Stream4->CR |= DMA_SxCR_EN;	// Enable DMA
	NVIC_ClearPendingIRQ(DMA2_Stream4_IRQn);	// Clear pending DMA interrupt
	NVIC_EnableIRQ(DMA2_Stream4_IRQn);	// Enable DMA interrupt in the NVIC
	ADC1->CR2 |= ADC_CR2_ADON;			// Enable ADC1
	ADC2->CR2 |= ADC_CR2_ADON;			// Enable ADC2
	TIM2->CR1 |= TIM_CR1_CEN;			// Enable timer
	while (MEAS_data_ready == false) {
		//DAC_increment();
		//printf("inc");
		//HAL_Delay(1);
	}
}

/** ***************************************************************************
 * @brief Stop DMA, ADC and timer
 *
 *****************************************************************************/
void ADC1_IN13_ADC2_IN5_dual_stop(void) {

	DMA2_Stream4->CR &= ~(DMA_SxCR_EN);	// Disable DMA
	NVIC_ClearPendingIRQ(DMA2_Stream4_IRQn);	// Clear pending DMA interrupt
	NVIC_DisableIRQ(DMA2_Stream4_IRQn);	// Enable DMA interrupt in the NVIC
	ADC1->CR2 &= ~(ADC_CR2_ADON);			// Disable ADC1
	ADC2->CR2 &= ~(ADC_CR2_ADON);			// Disable ADC2
	TIM2->CR1 &= ~(TIM_CR1_CEN);			// Disable timer

}

/** ***************************************************************************
 * @brief Interrupt handler for the timer 2
 *
 * @note This interrupt handler was only used for debugging purposes
 * and to increment the DAC value.
 *****************************************************************************/
void TIM2_IRQHandler(void) {
	TIM2->SR &= ~TIM_SR_UIF;			// Clear pending interrupt flag
	if (DAC_active) {
		//DAC_increment();
	}
}

/** ***************************************************************************
 * @brief Interrupt handler for the ADCs
 *
 * Reads one sample from the ADC3 DataRegister and transfers it to a buffer.
 * @n Stops when ADC_NUMS samples have been read.
 *****************************************************************************/
void ADC_IRQHandler(void) {
	if (ADC3->SR & ADC_SR_EOC) {		// Check if ADC3 end of conversion
		ADC_samples[ADC_sample_count++] = ADC3->DR;	// Read input channel 1 only
		if (ADC_sample_count >= ADC_NUMS) {		// Buffer full
			TIM2->CR1 &= ~TIM_CR1_CEN;	// Disable timer
			ADC3->CR2 &= ~ADC_CR2_ADON;	// Disable ADC3
			ADC_reset();
			MEAS_data_ready = true;
		}

	}
}

/** ***************************************************************************
 * @brief Interrupt handler for DMA2 Stream1
 *
 * The samples from the ADC3 have been transfered to memory by the DMA2 Stream1
 * and are ready for processing.
 *****************************************************************************/
void DMA2_Stream1_IRQHandler(void) {
	if (DMA2->LISR & DMA_LISR_TCIF1) {	// Stream1 transfer compl. interrupt f.
		NVIC_DisableIRQ(DMA2_Stream1_IRQn);	// Disable DMA interrupt in the NVIC
		NVIC_ClearPendingIRQ(DMA2_Stream1_IRQn);// Clear pending DMA interrupt
		DMA2_Stream1->CR &= ~DMA_SxCR_EN;	// Disable the DMA
		while (DMA2_Stream1->CR & DMA_SxCR_EN) {
			;
		}	// Wait for DMA to finish
		DMA2->LIFCR |= DMA_LIFCR_CTCIF1;// Clear transfer complete interrupt fl.
		TIM2->CR1 &= ~TIM_CR1_CEN;		// Disable timer
		ADC3->CR2 &= ~ADC_CR2_ADON;		// Disable ADC3
		ADC3->CR2 &= ~ADC_CR2_DMA;		// Disable DMA mode
		ADC_reset();
		MEAS_data_ready = true;
	}
}

/** ***************************************************************************
 * @brief Interrupt handler for DMA2 Stream3
 *
 * The samples from the ADC3 have been transfered to memory by the DMA2 Stream1
 * and are ready for processing.
 *****************************************************************************/
void DMA2_Stream3_IRQHandler(void) {
	if (DMA2->LISR & DMA_LISR_TCIF3) {	// Stream3 transfer compl. interrupt f.
		NVIC_DisableIRQ(DMA2_Stream3_IRQn);	// Disable DMA interrupt in the NVIC
		NVIC_ClearPendingIRQ(DMA2_Stream3_IRQn);// Clear pending DMA interrupt
		DMA2_Stream3->CR &= ~DMA_SxCR_EN;	// Disable the DMA
		while (DMA2_Stream3->CR & DMA_SxCR_EN) {
			;
		}	// Wait for DMA to finish
		DMA2->LIFCR |= DMA_LIFCR_CTCIF3;// Clear transfer complete interrupt fl.
		TIM2->CR1 &= ~TIM_CR1_CEN;		// Disable timer
		ADC2->CR2 &= ~ADC_CR2_ADON;		// Disable ADC2
		ADC2->CR2 &= ~ADC_CR2_DMA;		// Disable DMA mode
		ADC_reset();
		MEAS_data_ready = true;
	}
}

/** ***************************************************************************
 * @brief Interrupt handler for DMA2 Stream4
 *
 * Here the interrupt handler is used together with ADC1 and ADC2
 * in dual mode where they sample simultaneously.
 * @n The samples from both ADCs packed in a 32 bit word have been transfered
 * to memory by the DMA2 and are ready for unpacking.
 * @note In dual ADC mode two values are combined (packed) in a single uint32_t
 * ADC_CDR[31:0] = ADC2_DR[15:0] | ADC1_DR[15:0]
 * and are therefore extracted before further processing.
 *****************************************************************************/
void DMA2_Stream4_IRQHandler(void) {
	if (DMA2->HISR & DMA_HISR_TCIF4) {	// Stream4 transfer compl. interrupt f.
		NVIC_DisableIRQ(DMA2_Stream4_IRQn);	// Disable DMA interrupt in the NVIC
		NVIC_ClearPendingIRQ(DMA2_Stream4_IRQn);// Clear pending DMA interrupt
		DMA2_Stream4->CR &= ~DMA_SxCR_EN;	// Disable the DMA
		while (DMA2_Stream4->CR & DMA_SxCR_EN) {
			;
		}	// Wait for DMA to finish
		DMA2->HIFCR |= DMA_HIFCR_CTCIF4;// Clear transfer complete interrupt fl.
		TIM2->CR1 &= ~TIM_CR1_CEN;		// Disable timer
		ADC1->CR2 &= ~ADC_CR2_ADON;		// Disable ADC1
		ADC2->CR2 &= ~ADC_CR2_ADON;		// Disable ADC2
		ADC->CCR &= ~ADC_CCR_DMA_1;		// Disable DMA mode
		/* Extract combined samples */
		for (int32_t i = ADC_NUMS - 1; i >= 0; i--) {
			ADC_samples[2 * i + 1] = (ADC_samples[i] >> 16);
			ADC_samples[2 * i] = (ADC_samples[i] & 0xffff);
		}
		ADC_reset();
		MEAS_data_ready = true;
	}
}

void fft_shift(float input[], float output[], int length) {
	for (int i = 0; i < length; i++) {
		if (i < (length / 2)) {
			output[i] = input[i + ((length + 1) / 2)];
		} else {
			output[i] = input[i - (length / 2)];
		}
	}
}

void filter_dc(float input[], int length) {
	float sum = 0;
	for (int i = 0; i < length; i++) {
		sum += input[i] / length;
	}
	for (int i = 0; i < length; i++) {
		input[i] -= sum;
	}
}

void artificial_signal(double freq, int sampling_rate, int samples, uint32_t ADC_samples_arti[]) {
	double real;
	double imaginary;
	//uint32_t ADC_samples_arti[2 * ADC_NUMS];
	double real_array[ADC_NUMS];
	double imaginary_array[ADC_NUMS];
	double phi = 0;
	double pi = 3.141592653589793;
	for (int i = 0; i < samples; i++) {
		real = (cos(freq * 2 * pi * i / sampling_rate)) * 0xffff;
		imaginary = (sin(freq * 2 * pi * i / sampling_rate)) * 0xffff;
		real_array[i] = real;
		imaginary_array[i] = imaginary;
		ADC_samples_arti[2 * i] = (uint32_t) real;// = ((uint16_t)real << 16) + (uint16_t)imaginary;
		ADC_samples_arti[2 * i + 1] = (uint32_t) imaginary;
		ADC_samples[2 * i] = (uint32_t) real;
		ADC_samples[2 * i + 1] = (uint32_t) imaginary;
	}
	uint16_t breaktest;
	//delay(500);
}

/**
 *
 * @param samples indicates the length of the "data" array. Has to be 64, 256 or 1024.
 * @param data contains the original data, with "samples" many samples
 * @param result will contain magnitude of frequencies. "samples" / 2 frequencies are returned.
 */
float complete_fft(uint32_t samples, float output[]) {
	//float Input1[samples];
	//float Input2[samples];
	//float middle1[samples];
	//float middle2[samples];
	uint32_t input[ADC_NUMS * 2];
	//artificial_signal(500, 16000, ADC_NUMS, input);
	float Output[samples];
	//float Output2[samples];
	//float maxValue;
	//int maxIndex;
	//arm_rfft_fast_instance_f32 S; /* ARM CFFT module */
	arm_cfft_instance_f32 complexInst; /* ARM CFFT module */
	arm_cfft_init_f32(&complexInst, samples);

	float inputComplex[samples * 2];
	for (uint16_t i = 0; i < (ADC_NUMS * 2); i++) {
		//inputComplex[i] = (float) (input[i]);
		inputComplex[i] = (float) (ADC_samples[i]);
	}

	//filter_dc(inputComplex, (samples * 2));

	arm_cfft_f32(&complexInst, inputComplex, IFFT_FLAG, BIT_REVERSE_FLAG);
	arm_cmplx_mag_f32(inputComplex, Output, samples);

	//data = ADC_samples[MEAS_input_count*0] / f;
	/*
	 for (uint16_t i = 0; i < ADC_NUMS; i++){
	 Input1[i] = (float)(ADC_samples[i*2]);
	 //Input1[i+1] = 0;
	 }
	 */
	/* Draw the  values of input channel 2 (if present) as a curve */
	/*
	 if (MEAS_input_count == 2) {
	 for (uint16_t i = 0; i < ADC_NUMS; i++){
	 Input2[i] = (float)(ADC_samples[i*2+1]);
	 //Input2[i+1] = 0;
	 }
	 }*/
	//uint32_t *ADC_samples = get_ADC_samples();
//	for(int i = 0; i < (samples * 2); i += 2){
//		/* Real part, make offset by ADC / 2 */
//		Input1[(uint16_t)i] = (float)((ADC_samples[i/2] & 0xffff0000) >> 16);
//		/* Imaginary part */
//		Input1[(uint16_t)(i + 1)] = 0;
//
//		Input2[(uint16_t)i] = (float)(ADC_samples[i/2] & 0xffff);
//		/* Imaginary part */
//		Input2[(uint16_t)(i + 1)] = 0;
//	}
	/*
	 * @n Both converted data from ADC1 and ADC2 are packed into a 32-bit register
	 * in this way: <b> ADC_CDR[31:0] = ADC2_DR[15:0] | ADC1_DR[15:0] </b>*/
	/* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
	//arm_rfft_fast_init_f32(&S, samples);
	/* Process the data through the CFFT/CIFFT module */
	//arm_rfft_fast_f32(&S, Input1, middle1, 0);
	//arm_rfft_fast_f32(&S, Input2, middle2, 0);
	/* Process the data through the Complex Magnitud-e Module for calculating the magnitude at each bin */
	//arm_cmplx_mag_f32(middle1, result1, samples);
	//arm_cmplx_mag_f32(middle2, result2, samples);
	//result1 = Output1; //Attention, the start of the vectors are the same, but the length changes! to be tested!!!
	//result2 = Output2;
	/* Calculates maxValue and returns corresponding value */
	//arm_max_f32(result2, samples, &maxValue, &maxIndex);
	//return maxValue;
	return 0;
}

/** ***************************************************************************
 * @brief Draw buffer data as curves
 *
 * and write the first two samples as numbers.
 * @n After drawing, the buffer is cleared to get ready for the next run.
 * @note Drawing outside of the display crashes the system!
 * @todo Create new .h and .c files for calculating and displaying
 * of signals and results.
 * The code of this function was put into the same file for debugging purposes
 * and should be moved to a separate file in the final version
 * because displaying is not related to measuring.
 *****************************************************************************/
void MEAS_show_data(void) {
	const uint32_t Y_OFFSET = 260;
	const uint32_t X_SIZE = 240;
	const uint32_t f = (1 << ADC_DAC_RES) / Y_OFFSET + 1;	// Scaling factor
	uint32_t data;
	uint32_t data_last;
	/* Clear the display */
	BSP_LCD_SetTextColor(LCD_COLOR_WHITE);
	BSP_LCD_FillRect(0, 0, X_SIZE, Y_OFFSET + 1);
	/* Write first 2 samples as numbers */
	BSP_LCD_SetFont(&Font24);
	BSP_LCD_SetBackColor(LCD_COLOR_WHITE);
	BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
	char text[16];
	snprintf(text, 15, "1. sample %4d", (int) (ADC_samples[0]));
	BSP_LCD_DisplayStringAt(0, 50, (uint8_t*) text, LEFT_MODE);
	snprintf(text, 15, "2. sample %4d", (int) (ADC_samples[1]));
	BSP_LCD_DisplayStringAt(0, 80, (uint8_t*) text, LEFT_MODE);
	/* Draw the  values of input channel 1 as a curve */
	BSP_LCD_SetTextColor(LCD_COLOR_BLUE);
	data = ADC_samples[MEAS_input_count * 0] / f;
	for (uint32_t i = 1; i < ADC_NUMS; i++) {
		data_last = data;
		data = (ADC_samples[MEAS_input_count * i]) / f;
		if (data > Y_OFFSET) {
			data = Y_OFFSET;
		}	// Limit value, prevent crash
		BSP_LCD_DrawLine(4 * (i - 1), Y_OFFSET - data_last, 4 * i,
				Y_OFFSET - data);
	}
	/* Draw the  values of input channel 2 (if present) as a curve */
	if (MEAS_input_count == 2) {
		BSP_LCD_SetTextColor(LCD_COLOR_RED);
		data = ADC_samples[MEAS_input_count * 0 + 1] / f;
		for (uint32_t i = 1; i < ADC_NUMS; i++) {
			data_last = data;
			data = (ADC_samples[MEAS_input_count * i + 1]) / f;
			if (data > Y_OFFSET) {
				data = Y_OFFSET;
			}	// Limit value, prevent crash
			BSP_LCD_DrawLine(4 * (i - 1), Y_OFFSET - data_last, 4 * i,
					Y_OFFSET - data);
		}
	}
	/* Clear buffer and flag */
	for (uint32_t i = 0; i < ADC_NUMS; i++) {
		ADC_samples[2 * i] = 0;
		ADC_samples[2 * i + 1] = 0;
	}
	ADC_sample_count = 0;
}

