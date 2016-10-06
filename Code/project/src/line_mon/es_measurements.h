//*****************************************************************************
// Compact Analyser for Power Equipment (CAPE)
// es_measurement.h
// header file foe measurement.c
//
// Author
// Meher Jain
// Vivek Sankaranarayanan
//*****************************************************************************

#ifndef SRC_LINE_MON_ES_MEASUREMENTS_H_
#define SRC_LINE_MON_ES_MEASUREMENTS_H_

#include "device.h"
#include "arm_math.h"

/************ Structure prototypes *******************************************/
/* rms structure */
typedef struct
{
    _iq norm_acc; /**< RMS accumulator */
    _iq norm_rms; /**< normalised rms value */
    _iq norm_offset; /**< Offset for rms calibration */
    _iq gain; /**< Gain for rms calibration */
} rms_struct_t;

/* metrics structure */
typedef struct
{
    rms_struct_t Iac;
    rms_struct_t Vac;
    _iq P_apparent;
    _iq P_active;
    _iq P_inst_acc;
    _iq P_reactive;
    _iq P_PowerFactor;
    _iq Phase_shift;
    _iq frequency;
    float32_t Vthd;
    float32_t Ithd;
    _iq V_Peak;
    _iq I_Peak;
} ac_metrics_t;

/************ Global Variables    ********************************************/
extern ac_metrics_t ac_metrics;
extern uint32_t ac_raw_adc_counts[2];

/************ macros *********************************************************/
#define ADC_LEVEL_SHIFT   (0.5)
#define _Q12toIQ24(A)     ((int32_t)A<<12)
#define V_FULL_SCALE  400.0
#define I_FULL_SCALE  49.5
#define P_FULL_SCALE  V_FULL_SCALE*I_FULL_SCALE
#define IQ24toFloat   16777216.0

/************ global functions ************************************************/
extern void init_adc();

#endif /* SRC_LINE_MON_ES_MEASUREMENTS_H_ */
