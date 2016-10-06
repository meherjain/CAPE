//*****************************************************************************
// Compact Analyser for Power Equipment (CAPE)
// es_measurement.c
// modules for AC_measurement
//
// Author
// Meher Jain
// Vivek Sankaranarayanan
//*****************************************************************************

/******************** Header files  ******************************************/
#include <es_measurements.h>
/*****************************************************************************/

/****************** Global variables *****************************************/
ac_metrics_t ac_metrics;
uint32_t ac_raw_adc_counts[2];
/*****************************************************************************/

/* desc: Initialize ADC_1 for voltage/current samples
 * args: none
 * ret : none
 */
void init_adc()
{
  /* SS0 triggers the following channels
   * PE0  - AIN3 - V_ac_sense
   * PE1  - AIN2 - I_ac_sense
   */

  /* Analog pins initialisation */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
  ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_0);
  ROM_GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_1);

  /* Initialise ADC */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC1);
  /* Sequencer 0, Software trigger, highest priority */
  ROM_ADCSequenceConfigure(ADC1_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
  /* Step 0 of Sequencer 0, CH3 */
  ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 0, ADC_CTL_CH3);
  /* Step 1 of Sequencer 0, CH2, End_channel, Generate interrupt */
  ROM_ADCSequenceStepConfigure(ADC1_BASE, 0, 1,
      ADC_CTL_CH2 | ADC_CTL_IE | ADC_CTL_END);
  /* Enable sequencer */
  ROM_ADCSequenceEnable(ADC1_BASE, 0);

  /* ac_metrics initialisation */
  ac_metrics.Iac.gain = _IQ(1.0);
  ac_metrics.Iac.norm_offset = _IQ(0);
  ac_metrics.Iac.norm_acc = 0;
  ac_metrics.Iac.norm_rms = 0;
  ac_metrics.Vac.gain = _IQ(1.0);
  ac_metrics.Vac.norm_offset = _IQ(0);
  ac_metrics.Vac.norm_acc = 0;
  ac_metrics.Vac.norm_rms = 0;
  ac_metrics.P_active = 0;
  ac_metrics.P_inst_acc = 0;
  ac_metrics.P_apparent = 0;
  ac_metrics.P_reactive = 0;
  ac_metrics.P_PowerFactor = 0;
  ac_metrics.V_Peak = 0;
  ac_metrics.I_Peak = 0;
}
