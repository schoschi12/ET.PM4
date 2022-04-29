/** ***************************************************************************
 * @file
 * @brief See speed.c
 *
 *
 *****************************************************************************/



/******************************************************************************
 * Includes
 *****************************************************************************/


/******************************************************************************
 * Defines
 *****************************************************************************/
float df1;
float df2;

/******************************************************************************
 * Functions
 *****************************************************************************/
void fft_showcase();
void init_speed(void);
float measure_speed(bool human_detection);
void tim_TIM7_TriangleWave(uint32_t DAC_frequency);
void tim_TIM7_TriangleWave_Start(void);
void tim_TIM7_TriangleWave_Stop(void);
