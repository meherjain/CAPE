//*****************************************************************************
// Compact Analyser for Power Equipment (CAPE)
// es_fft_compute.h
// Header file for es_fft_compute.c
//
// Author
// Meher Jain
// Vivek Sankaranarayanan
//*****************************************************************************

#ifndef SRC_FFT_ES_FFT_COMPUTE_H_
#define SRC_FFT_ES_FFT_COMPUTE_H_

/******************** Header files  ******************************************/
#include "device.h"
#include "arm_math.h"
/*****************************************************************************/

/******************** Global Variables ***********************************************************/
extern uint8_t fft_VI_samples_collect;
extern uint8_t fft_counter;
extern _iq norm_Iinst_IQ_samples[FFT_LENGTH];
extern _iq norm_Vinst_IQ_samples[FFT_LENGTH];
extern float32_t fft_output_array_voltage[FFT_LENGTH];
extern float32_t fft_output_array_current[FFT_LENGTH];
extern float32_t sine_test_input[FFT_LENGTH];

/******************** constants ******************************************************************/
#define FFT_LENGTH                    256
#define FFT_EVENT_COUNTER_ELAPSED     5

/******************** Global functions ***********************************************************/
extern void fft_compute();
extern void generate_input();


#endif /* SRC_FFT_ES_FFT_COMPUTE_H_ */
