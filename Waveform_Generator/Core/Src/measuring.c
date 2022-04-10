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
#define IFFT_FLAG		0
#define BIT_REVERSE_FLAG 	1
/******************************************************************************
 * Variables
 *****************************************************************************/
bool MEAS_data_ready = false;			///< New data is ready
uint32_t MEAS_input_count = 1;			///< 1 or 2 input channels?
bool DAC_active = false;				///< DAC output active?

static uint32_t ADC_sample_count = 0;	///< Index for buffer
static uint32_t ADC_samples[2 * ADC_NUMS];///< ADC values of max. 2 input channels
static uint32_t DAC_sample = 0;			///< DAC output value

TIM_HandleTypeDef htim6_meas;
DAC_HandleTypeDef hdac_meas;
DMA_HandleTypeDef hdma_dac2;

/******************************************************************************
 * Functions
 *******************************************************************f**********/

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
}

/** ***************************************************************************
 * @brief Resets the DAC
 *
 * when it is no longer used.
 *****************************************************************************/
void DAC_reset(void) {
	RCC->APB1RSTR |= RCC_APB1RSTR_DACRST;	// Reset the DAC
	RCC->APB1RSTR &= ~RCC_APB1RSTR_DACRST;	// Release reset of the DAC
}

/** ***************************************************************************
 * @brief Initialize the DAC
 *
 * The output used is DAC_OUT2 = GPIO PA5
 * @n As DAC_OUT2 = GPIO PA5 (= same GPIO as ADC12_IN5)
 * it is possible to monitor the output voltage DAC_OUT2 by the input ADC12_IN5.
 *****************************************************************************/
void DAC_init(void) {
	__HAL_RCC_DAC_CLK_ENABLE();			// Enable Clock for DAC
	DAC->CR |= DAC_CR_EN2;				// Enable DAC output 2
}

void DAC_init_MX(void) {
	/* USER CODE BEGIN DAC_Init 0 */

	/* USER CODE END DAC_Init 0 */

	DAC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN DAC_Init 1 */

	/* USER CODE END DAC_Init 1 */
	/** DAC Initialization
	 */
	hdac_meas.Instance = DAC;
	if (HAL_DAC_Init(&hdac_meas) != HAL_OK) {
		//Error_Handler();
	}
	/** DAC channel OUT2 config
	 */
	sConfig.DAC_Trigger = DAC_TRIGGER_T6_TRGO;
	sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
	if (HAL_DAC_ConfigChannel(&hdac_meas, &sConfig, DAC_CHANNEL_1) != HAL_OK) {
		//Error_Handler();
	}
	/** Configure Triangle wave generation on DAC OUT2
	 */
	if (HAL_DACEx_TriangleWaveGenerate(&hdac_meas, DAC_CHANNEL_1,
			DAC_TRIANGLEAMPLITUDE_4095) != HAL_OK) {
		//Error_Handler();
	}
}

/** ***************************************************************************
 * @brief Increment the DAC value and write it to the output
 *
 *****************************************************************************/
void DAC_increment(void) {
	DAC_sample += 20;				// Increment DAC output
	if (DAC_sample >= (1UL << ADC_DAC_RES)) {
		DAC_sample = 0;
	}	// Go to 0
	DAC->DHR12R2 = DAC_sample;		// Write new DAC output value
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

void DAC_timer_init(void) {

	/* USER CODE BEGIN TIM6_Init 0 */

	/* USER CODE END TIM6_Init 0 */

	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM6_Init 1 */

	/* USER CODE END TIM6_Init 1 */
	htim6_meas.Instance = TIM6;
	htim6_meas.Init.Prescaler = 6 - 1;
	htim6_meas.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim6_meas.Init.Period = 1;
	htim6_meas.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6_meas) != HAL_OK) {
		//Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6_meas, &sMasterConfig)
			!= HAL_OK) {
		//Error_Handler();
	}
	/* USER CODE BEGIN TIM6_Init 2 */

	/* USER CODE END TIM6_Init 2 */

}

/**
 * Enable DMA controller clock
 */
void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Stream6_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

void generate_triangle(void) {
	//Start Timer 6
	HAL_TIM_Base_Start(&htim6_meas);

	//Generate Triangular waveform
	HAL_DACEx_TriangleWaveGenerate(&hdac_meas, DAC_CHANNEL_1,
			DAC_TRIANGLEAMPLITUDE_2047);

	HAL_DAC_Start(&hdac_meas, DAC_CHANNEL_1);
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
	ADC2->SQR3 |= (5UL << ADC_SQR3_SQ1_Pos);	// Input 5 = first conversion
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
	DAC_init(); ///////////////////////////////
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
 * @brief Interrupt handler for the timer 2
 *
 * @note This interrupt handler was only used for debugging purposes
 * and to increment the DAC value.
 *****************************************************************************/
void TIM2_IRQHandler(void) {
	TIM2->SR &= ~TIM_SR_UIF;			// Clear pending interrupt flag
	if (DAC_active) {
		DAC_increment();
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
/**
 *
 * @param samples indicates the length of the "data" array. Has to be 64, 256 or 1024.
 * @param data contains the original data, with "samples" many samples
 * @param result will contain magnitude of frequencies. "samples" / 2 frequencies are returned.
 */
float complete_fft(uint32_t samples, float result1[], float result2[]) {
	float Input1[samples];
	float Input2[samples];
	float middle1[samples];
	float middle2[samples];
	//float Output1[samples];
	//float Output2[samples];
	float maxValue;
	uint32_t maxIndex;
	arm_rfft_fast_instance_f32 S; /* ARM CFFT module */

	//data = ADC_samples[MEAS_input_count*0] / f;
	for (uint16_t i = 0; i < ADC_NUMS; i++) {
		Input1[i] = (float) (ADC_samples[i * 2]);
		//Input1[i+1] = 0;
	}
	/* Draw the  values of input channel 2 (if present) as a curve */
	if (MEAS_input_count == 2) {
		for (uint16_t i = 0; i < ADC_NUMS; i++) {
			Input2[i] = (float) (ADC_samples[i * 2 + 1]);
			//Input2[i+1] = 0;
		}
	}
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
	arm_rfft_fast_init_f32(&S, samples);

	/* Process the data through the CFFT/CIFFT module */
	arm_rfft_fast_f32(&S, Input1, middle1, IFFT_FLAG);
	arm_rfft_fast_f32(&S, Input2, middle2, IFFT_FLAG);

	/* Process the data through the Complex Magnitud-e Module for calculating the magnitude at each bin */
	arm_cmplx_mag_f32(middle1, result1, samples);
	arm_cmplx_mag_f32(middle2, result2, samples);

	//result1 = Output1; //Attention, the start of the vectors are the same, but the length changes! to be tested!!!
	//result2 = Output2;
	/* Calculates maxValue and returns corresponding value */
	arm_max_f32(result2, samples, &maxValue, &maxIndex);
	return maxValue;

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
