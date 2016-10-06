//*****************************************************************************
// Compact Analyser for Power Equipment (CAPE)
// es_fft_compute.c
// Top level fft file that calls CMSIS library modules
//
// Referenced code: CMSIS library is used for FFT computations
// Link: http://www.arm.com/products/processors/cortex-m/cortex-microcontroller-software-interface-standard.php
// Doc : http://www.keil.com/pack/doc/CMSIS/General/html/index.html
//
// Author
// Meher Jain
// Vivek Sankaranarayanan
//*****************************************************************************

/******************** Header files  ******************************************/
#include <es_fft_compute.h>
#include <es_measurements.h>
#include <math.h>
#include "arm_const_structs.h"
/*****************************************************************************/

/****************** Global variables *****************************************/
//float32_t sine_test_input[FFT_LENGTH];
uint8_t fft_VI_samples_collect = 1;               /* flag to turn ON/OFF fft sample collection */
uint8_t fft_counter = 0;                          /* fft event counter */
_iq norm_Iinst_IQ_samples[FFT_LENGTH];            /* IQ instantaneous voltage samples */
_iq norm_Vinst_IQ_samples[FFT_LENGTH];            /* IQ instantaneous current samples */
float32_t fft_output_array_voltage[FFT_LENGTH];   /* fft ouptut array for voltage */
float32_t fft_output_array_current[FFT_LENGTH];   /* fft ouptut array for current */
float32_t fft_input_array[FFT_LENGTH];            /* float input array for fft algorithm */

arm_rfft_fast_instance_f32 rfft_fast_len256;      /* real fft object required tby the CMSIS library */
/*****************************************************************************/

/* desc: Call the underlying FFT drivers from CMSIS library
 * args: pointer to rfft object, pointer to input_array, pointer to output buffer
 * ret : none
 */
void fft(arm_rfft_fast_instance_f32 *S, float32_t *input_arr,
    float32_t *output_arr)
{

  /* Process the data through the CFFT/CIFFT module */
  arm_rfft_fast_f32(S, input_arr, output_arr, 0);

  /* Process the data through the Complex Magnitude Module for
   calculating the magnitude at each bin */
  arm_cmplx_mag_f32(output_arr, output_arr, FFT_LENGTH / 2);
}

/* desc: top level function that calls fft and computes THD
 * args: none
 * ret : none
 */
void fft_compute()
{
  int16_t i;
  float32_t sum_of_squares;

  /* initialisation */
  arm_rfft_fast_init_f32(&rfft_fast_len256, FFT_LENGTH);

  /* voltage array */
  for (i = 0; i < FFT_LENGTH; i++)
    fft_input_array[i] = norm_Vinst_IQ_samples[i] / 16777216.0;
//    fft_input_array[i] = sine_test_input[i];
  fft(&rfft_fast_len256, fft_input_array, fft_output_array_voltage);

  /* current array */
  for (i = 0; i < FFT_LENGTH; i++)
    fft_input_array[i] = norm_Iinst_IQ_samples[i] / 16777216.0;
  fft(&rfft_fast_len256, fft_input_array, fft_output_array_current);

  /* THD computation */
  /* THD = sqrt(V2^2 + V3^2 + V4^2 + V5^2 + ...) / V1
   * V1: Rms of fundamental component
   * V2, V3, V4.. : rms of harmonic components
   *
   * Note: Actual amplitudes are used instead of rms values. Needs to be
   * verified if this approach is correct
   */

  /* Voltage THD computation */
  /* the FFT results are scaled by FFT_LENGTH/2. Normalise them first */
  for (i = 0; i < FFT_LENGTH / 2; i++)
  {
    fft_output_array_voltage[i] = fft_output_array_voltage[i]
        / (FFT_LENGTH / 2);
  }

  /* now ingore first 2 points corresponding to dc and fundamental and do a sum of squares of the rest */
  sum_of_squares = 0;
  for (i = 2; i < FFT_LENGTH / 2; i++)
  {
    sum_of_squares +=
        (fft_output_array_voltage[i] * fft_output_array_voltage[i]);
  }

  /* now, thd = sqrt(sum_of_squares)/v1 */
  ac_metrics.Vthd = sqrt(sum_of_squares) / fft_output_array_voltage[1];

  /* Current THD computation */
  /* the FFT results are scaled by FFT_LENGTH/2. Normalise them first */
  for (i = 0; i < FFT_LENGTH / 2; i++)
  {
    fft_output_array_current[i] = fft_output_array_current[i]
        / (FFT_LENGTH / 2);
  }

  /* now ingore first 2 points corresponding to dc and fundamental and do a sum of squares of the rest */
  sum_of_squares = 0;
  for (i = 2; i < FFT_LENGTH / 2; i++)
  {
    sum_of_squares +=
        (fft_output_array_current[i] * fft_output_array_current[i]);
  }

  /* now, thd = sqrt(sum_of_squares)/v1 */
  ac_metrics.Ithd = sqrt(sum_of_squares) / fft_output_array_current[1];
}

//void generate_input()
//{
//  uint16_t i;
//
//  /* generate radians */
//  for (i = 0; i < FFT_LENGTH; i++)
//    sine_test_input[i] = i * 2 * PI / FFT_LENGTH;
//  /* generate sine */
//  for (i = 0; i < FFT_LENGTH; i++)
//    sine_test_input[i] = (sin(sine_test_input[i])
//        + 0.5 * sin(3 * sine_test_input[i]) + 0.25 * sin(5 * sine_test_input[i])
//        + 0.15 * sin(7 * sine_test_input[i]));
//}

//void fft_test()
//{
//  //arm_status status;
//  //float32_t maxValue;
//  uint16_t i = 0;
//
//  for(i = 0; i < FFT_LENGTH; i++)
//  {
//    //testOutput[i] = 0;
//  }
//
//  //status = ARM_MATH_SUCCESS;
//
////  /* Process the data through the CFFT/CIFFT module */
////  arm_rfft_fast_f32(&rfft_fast_len256, sine_test_input, testOutput, 0);
////
////  /* Process the data through the Complex Magnitude Module for
////  calculating the magnitude at each bin */
////  arm_cmplx_mag_f32(testOutput, testOutput, FFT_LENGTH/2);
//}
