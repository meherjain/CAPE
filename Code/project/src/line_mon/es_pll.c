//*****************************************************************************
// Compact Analyser for Power Equipment (CAPE)
// es_pll.c
// PLL algorithm
//
// Author
// Meher Jain
// Vivek Sankaranarayanan
//*****************************************************************************

/******************** Header files  ******************************************/
#include <es_pll.h>
/*****************************************************************************/

/****************** Global variables *****************************************/
pll_t pll_s;
/*****************************************************************************/

/* desc: Initialize ADC_1 for voltage/current samples
 * args: pointer to PLL structure
 * ret : none
 */
void pll_init(pll_t *pll_s)
{
  pll_s->cap_counts_now = 0;
  pll_s->cap_counts_prev = 0;
  pll_s->capture_detected = 0;
  pll_s->freq_in_cap_counts = 0;
  pll_s->phase_shift_index = 0;
}

/**
 * @brief PLL event
 * @param pointer to pll structure
 * @return None
 *
 * # PLL Algorithm #
 * PLL Algorithm consists of 3 main parts-
 * 1. Zero crossing (Zx) interrupt
 * 2. Frequency sync
 * 3. Phase sync
 *
 * ## Zero Crossing (Zx) interrupt ##
 * A Zero crossing detector circuit generates the reference square wave from grid that our internally generated sine wave must
 * match in frequency as well as phase. The Zx circuit works in the following way:
 * - It generates a high to low for grid's positive zero crossing (0 degrees)
 * - It generates a low to high for grid's negative zero crossing (180 degrees)
 * We have set the capture module to trigger an interrupt on falling edge thus ensuring that we get an interrupt everytime
 * grid crosses 0 degrees.
 * In the interrupt we do the following 2 things:
 * - We get the period of previous cycle in capture counts. This is used for FREQUENCY_SYNC. Counts to frequency calculation
 * is given by the following formula:
 *  \f$ Grid_Frequency = (sysclk_freq_in_Hz) / (Capture_Counts) \f$
 * - We mark the phase difference of internally generated sine wrt reference wave. This is done by capturing the value of
 * theta_index at the ZX instant. Theta_index can vary from 0 to NO_OF_MINOR_CYCLES_PER_LINE_CYCLE-1 in one line cycle. Let us assume
 * a value of 512 for NO_OF_MINOR_CYCLES_PER_LINE_CYCLE. Thus theta_index varies from 0-511 in one line cycle. The following
 * table indicates phase releationship for different values of theta_index wrt to reference square wave.
 *
 * | Theta_Index @ | Phase      |
 * | ZX interrupt  |            |
 * |:-------------:|:----------:|
 * | 0             | Inphase    |
 * | 1-255         | Phase Lead |
 * | 257-511       | Phase Lag  |
 *
 * @ref phase_shift_index stores the value of theta_index at Zero Crossing
 *
 * ## PLL Event ##
 * PLL is a scheduled event that is executed once every line cycle. As mentioned earlier it includes -
 * FREQUENCY_SYNC and PHASE_SYNC. It also includes conditions to handle a few corner cases.
 *
 * ### FREQUENCY SYNC ###
 * Frequency syncing has only one step. Based on the capture_counts of the previous line cycle (determined from ZX interrupt),
 * we calculate what must be the next value written to Timer period so that 512 interrupts can be achieved in one line cycle.
 *
 *  That is all FREQUENCY_SYNC involves. To prevent abrupt frequency changes of our internal sine, we put a limit
 *  on maximum change in tbprd allowed in one cycle.
 *
 *  ### PHASE SYNC ###
 *  Phase syncing involves matching our internal sine wave to the reference signal in such a way that our theta_index
 *  is exactly 0 at ZX interrupt. As per the earlier table, we can determine whether we are leading or lagging wrt refer-
 *  ence wave. The phase correction action behaves according to following table:
 *
 *------------------------------------------------
 * | Theta_Index @ | Phase      | Corrective    |
 * | ZX interrupt  |            | Action        |
 * |:-------------:|:----------:|:-------------:|
 * | 0             | Inphase    | No action     |
 * | 1-255         | Phase Lead | Increase tbprd|
 * | 257-511       | Phase Lag  | Reduce tbprd  |
 *------------------------------------------------
 * As seen from the above table, if we are leading wrt reference signal, we sligtly reduce our frequency by increasing
 * timer period by tbprd_stepsize. If we are lagging, then we increase our frequency by reducing timer period by tbprd_stepsize.
 *
 * tbprd_stepsize is a variable that determines that by how many counts we need to go up or down to phase sync. A higher
 * value of stepsize allows faster syncing. We employ a higher stepsize when the phase shift between the reference signal
 * is greater than 5 (out of 512) counts. The table below determines tbprd_stepsize
 *
 *------------------------------------
 * | Phase shift   | tbprd_stepsize |
 * |               |                |
 * |:-------------:|:--------------:|
 * | 0             | 0              | In phase
 * | 1-5           | 1              | Mildly out of phase
 * | >5            | 4              | Greatly out of phase
 *------------------------------------
 *
 * ### Handling Corner cases ###
 *
 * In each of the below corner cases we move from whatever the current freqeuncy is, to nominal frequency (50 or 60Hz).
 * These cases describe various failure conditions for pll.
 *
 * #### Grid Missing ####
 * When the grid is missing, no capture interrupts will be generated. In such a case we will hold the previous timer period.
 *
 * #### Grid Frequency out of range ####
 * If the grid frequency is out of range then we will hold the previous timer period.
 */
void phase_locked_loop(pll_t *pll)
{
  int32_t systick_present, systick_next, systick_final, change;
  int32_t new_capture;
  uint16_t shift_index;

  /* Both next_tprd and present_tprd start with the present Timer register value */
  systick_next = systick_present = ROM_SysTickPeriodGet();
  shift_index = pll->phase_shift_index;

  if(pll->capture_detected == true)
  {
    /* Clear flag to allow next interrupts to set it  */
    pll->capture_detected = false;

    new_capture = pll->freq_in_cap_counts;
    systick_final = new_capture/SINE_SAMPLE_SIZE;

    /**
     * FREQUENCY SYNC
     * The change in tbprd allowed is restricted to MAX_INCREMENTAL_CHANGE_IN_COUNTS to
     * ensure a smooth transition to new frequency
     */
    change = _IQsat((systick_final - systick_present),MAX_INCREMENTAL_CHANGE, -MAX_INCREMENTAL_CHANGE);
    systick_next = systick_present + change;

    /** PHASE SYNCING **/
    if(PHASE_LEAD(shift_index))
      systick_next++;
    else if(PHASE_LAG(shift_index))
      systick_next--;
  }

  if(systick_next < SYSTICK_LOW_LIMIT) // 3606
    systick_next = SYSTICK_LOW_LIMIT;
  else if(systick_next > SYSTICK_HIGH_LIMIT) // 4261
    systick_next = SYSTICK_LOW_LIMIT;
    /* update systick */
    ROM_SysTickPeriodSet(systick_next);
}



