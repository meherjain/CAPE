//*****************************************************************************
// Compact Analyser for Power Equipment (CAPE)
// es_cape.c Main project file
//
// Author
// Meher Jain
// Vivek Sankaranarayanan
//*****************************************************************************

/******************** ABOUT THIS PROJECT *************************************
 * This project was created as a part of the final project for ESD Spring 2016
 * by Meher Jain and Vivek Sankaranarayanan.
 *
 * The major fucntionality of the project:
 *  1. The project is called CAPE (Compact Analyser for Power Equipment). This
 *     basically measures the AC voltage and current and measures the harmonics
 *     in the power drawn by the load.
 *  2. It measures a bunch of AC metrics in addition to voltage and current
 *     harmonics. (Vrms, Irms, Vpeak, Ipeak etc... See measurement.c for
 *     complete list)
 *  3. The project uses a KENTEC TFT touchscreen (resistive) LCD that displays
 *     AC_metrics, Voltage/Current Fourier spectrum, Waveforms. See gui.c
 *     for more information
 *  4. The project also has an lwIP (ethernet) based datalogger. See
 *     enet_datalogger.c
 *
 *     For libraries, tool info and file naming conventions see version.h
 */

/******************** Header files  ******************************************/
/* device support */
#include <es_enet_datalogger.h>
#include <es_fft_compute.h>
#include <es_gui.h>
#include <es_measurements.h>
#include <es_pll.h>
#include "device.h"
#include "drivers/pinout.h"

/* ac measurements */
#include "drivers/touch.h"
#include "utils/uartstdio.h"
/****************************************************************************/

/****************** Global variables ****************************************/
volatile uint8_t scheduler_flag; /* Flag to schedule events */
uint32_t sys_clk; /* system clock counts */
/****************************************************************************/

/* desc: Initialises all peripherals used in the project
 * args: none
 * ret : none
 */
static void peripheral_init()
{
  /* Enable the GPIO port that is used for the on-board LED. */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

  /* Set the direction as output, and enable the GPIO pin for IO function */
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_0);
  ROM_GPIOPinTypeGPIOOutput(GPIO_PORTN_BASE, GPIO_PIN_1);

  /* Configure the device pins.*/
  PinoutSet(true, false);

  /* Configure UART.*/
  UARTStdioConfig(0, 115200, sys_clk);

  /* Uart test */
  UARTprintf("Uart functional...\n\r");

  /*
   * Systick Initialisation
   */
  ROM_SysTickPeriodSet(SYSTICK_COUNT_FOR_NOMINAL_LINE_FREQUENCY); // 3906
  ROM_SysTickIntEnable();
  ROM_SysTickEnable();

  /*
   * PWM initialisation
   * PWM0->PF1: 20KHz
   */
  /* Pin */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
  /* Select PWM functionality for the pins   */
  ROM_GPIOPinConfigure(GPIO_PF1_M0PWM1);
  /* Configure the PWM functionality for the pin */
  ROM_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1);

  /* Peripheral */
  /* PWM0 in count down mode */
  ROM_PWMGenConfigure(PWM0_BASE, PWM_GEN_0, PWM_GEN_MODE_DOWN);
  /*
   Set the PWM period to 20000Hz.  To calculate the appropriate parameter
   use the following equation: N = (1 / f) * SysClk.  Where N is the
   function parameter, f is the desired frequency, and SysClk is the
   system clock frequency.
   In this case you get: (1 / 20kHz) * 120MHz = 6000 cycles.  Note that
   the maximum period you can set is 2^16.
   */
  ROM_PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, 6000);
  /* zero duty initialy */
  ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, 0);
  /* Enable the PWM0 output signal (PF1).*/
  ROM_PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
  /* Enables the PWM generator block.*/
  ROM_PWMGenEnable(PWM0_BASE, PWM_GEN_0);

  /*
   * General purpose timer 0: Capture mode
   */
  /* T0CCP0 Initialisation (PL4-65 breadboard) */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
  ROM_GPIOPinConfigure(GPIO_PL4_T0CCP0);
  ROM_GPIOPinTypeTimer(GPIO_PORTL_BASE, GPIO_PIN_4);

  /* Timer 0 Capture Initialisation */
  ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
  TIMER0_CC_R = 0x00; /* Clocked with SYSCLK (120MHz) */
  TIMER0_CTL_R = 0; /* Timer disabled */
  TIMER0_CFG_R = TIMER_CFG_16_BIT; /* 16 bit configuration */
//  TIMER0_CFG_R = TIMER_CFG_32_BIT_TIMER; /* 32 bit configuration */
  /* Up count, Capture mode, Edge Time, Capture mode, Capture mode interrupt enable */
  TIMER0_TAMR_R =
      TIMER_TAMR_TACDIR + TIMER_TAMR_TACMR + TIMER_TAMR_TAMR_CAP/* + TIMER_TAMR_TAPWMIE*/;
  TIMER0_CTL_R = TIMER_CTL_TAEVENT_NEG; /* Capture on falling edge */
  TIMER0_IMR_R = TIMER_IMR_CAEIM; /* Capture mode event interrupt enable */
  TIMER0_ICR_R = 0xFFFFFFFF; /* Clear interrupt status */
  TIMER0_TAILR_R = 0xFFFF; /* 2^16 is the upper bound */
  TIMER0_TAPR_R = 0xFF; /* 2^8 is upper bound */
  TIMER0_CTL_R |= TIMER_CTL_TAEN; /* Enable Timer */
}

/* Application main file */
int main(void)
{
  /* For ethernet. Checking clock
   Make sure the main oscillator is enabled because this is required by
   the PHY.  The system must have a 25MHz crystal attached to the OSC
   pins. The SYSCTL_MOSC_HIGHFREQ parameter is used when the crystal
   frequency is 10MHz or higher.
   */
  SysCtlMOSCConfigSet(SYSCTL_MOSC_HIGHFREQ);

  /* Set the clocking to run directly from the crystal at 120MHz. */
  sys_clk = ROM_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
  SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
  SYSCTL_CFG_VCO_480), 120000000);

  /* enables the floating point engine for the processor */
  FPUEnable();
  /* FPU lazy stacking enables optimization when interrupt is called */
  FPULazyStackingEnable();

//  /* FFT test */
//  generate_input();

  /* Initialise all global stuff */
  peripheral_init();
  pll_init(&pll_s);
  scheduler_flag = false;
  init_adc();
  gui_init();
  enet_init();

  /* Enable Interrupts at NVIC level */
  ROM_IntEnable(INT_TIMER0A);
  /* enable interrupt processing at CPU level */
  ROM_IntMasterEnable();

  /* connect to server */
  enet_server_connect();

  fft_VI_samples_collect = 1;

  /* Enable initial trigger for touchscreen ADC */
  ROM_ADCProcessorTrigger(ADC0_BASE, 3);

  /* Begin endless loop */
  while (1)
  {
    /* Check if ADC0 conversion completed */
    if (ROM_ADCIntStatus(ADC0_BASE, 3, false))
    {
      ROM_ADCIntClear(ADC0_BASE, 3);
      /* Retrigger for next time */
      ROM_ADCProcessorTrigger(ADC0_BASE, 3);
      /* Call touch screen handler to process ADC results */
      TouchScreenHandler();
    }

    /* GrLib function that checks if any widgets have been triggered */
    WidgetMessageQueueProcess();

    /* Execute scheduled events */
    if (scheduler_flag == true)
    {
      /* reset scheduler flag in beginning itself before any event executed
       * so that any stalled event may cause future events to be lost
       */
      scheduler_flag = false;

      /* PLL EVENT */
      phase_locked_loop(&pll_s);

      /* FFT EVENT */
      if (fft_counter == FFT_EVENT_COUNTER_ELAPSED)
      {
        fft_counter = 0;
        /* disable sample collection when fft being computed for the existing samples*/
        fft_VI_samples_collect = false;
        fft_compute();
        /* renable sample collection */
        fft_VI_samples_collect = true;
      }
      else
      {
        fft_counter++;
      }

      /* DISPLAY EVENT */
      if (counter_display_event == DISPLAY_EVENT_COUNTER_ELAPSED)
      {
        counter_display_event = 0;

        if (g_iPage == PAGE_METRICS)
        {
          // Updating Vrms //
          sprintf(display_update_buffer1, "%.1f V",
              ac_metrics.Vac.norm_rms * V_FULL_SCALE / IQ24toFloat);
          CanvasTextSet(&g_sVrmsValue, (const char * )display_update_buffer1);

          // Updating Irms //
          sprintf(display_update_buffer2, "%.1f A",
              ac_metrics.Iac.norm_rms * I_FULL_SCALE / IQ24toFloat);
          CanvasTextSet(&g_sIrmsValue, (const char * )display_update_buffer2);

          // Updating Frequency //
          sprintf(display_update_buffer3, "%.1f Hz",
              120000000.0 / pll_s.freq_in_cap_counts);
          CanvasTextSet(&g_sFreqValue, (const char * )display_update_buffer3);

          // Updating Power Factor //
          sprintf(display_update_buffer4, "%.2f",
              ac_metrics.P_PowerFactor / IQ24toFloat);
          CanvasTextSet(&g_sPFValue, (const char * )display_update_buffer4);

          // Updating Apparent Power //
          sprintf(display_update_buffer5, "%.1f",
              ac_metrics.P_apparent * P_FULL_SCALE / IQ24toFloat);
          CanvasTextSet(&g_sP_apparent_val,
              (const char * )display_update_buffer5);

          // Updating active Power //
          sprintf(display_update_buffer6, "%.1f",
              ac_metrics.P_active * P_FULL_SCALE / IQ24toFloat);
          CanvasTextSet(&g_sP_active_val,
              (const char * )display_update_buffer6);

          // Updating THD Voltage //
          sprintf(display_update_buffer7, "%.1f %%", ac_metrics.Vthd);
          CanvasTextSet(&g_sTHDv_val, (const char * )display_update_buffer7);

          // Updating THD Current //
          sprintf(display_update_buffer8, "%.1f %%", ac_metrics.Ithd);
          CanvasTextSet(&g_sTHDi_val, (const char * )display_update_buffer8);

          // Updating Phase  //
          sprintf(display_update_buffer9, "%.1f dg",
              ac_metrics.Phase_shift * (180 / PI) / IQ24toFloat);
          CanvasTextSet(&g_sPhase_val, (const char * )display_update_buffer9);

          // Vpeak
          sprintf(display_update_buffer10, "%.1f V",
              ac_metrics.V_Peak * V_FULL_SCALE / IQ24toFloat);
          CanvasTextSet(&g_sVpeak_val, (const char * )display_update_buffer10);
          ac_metrics.V_Peak = 0;

          // Ipeak
          sprintf(display_update_buffer11, "%.1f A",
              ac_metrics.I_Peak * I_FULL_SCALE / IQ24toFloat);
          CanvasTextSet(&g_sIpeak_val, (const char * )display_update_buffer11);
          ac_metrics.I_Peak = 0;

          WidgetPaint(WIDGET_ROOT);  // painting the updated canvases
        }
        else if (g_iPage == PAGE_VOLTAGE_SPECTRUM)
        {
          Display_FreqSpectrum(VOLTAGE_SPECTRUM);
        }
        else if (g_iPage == PAGE_CURRENT_SPECTRUM)
        {
          Display_FreqSpectrum(CURRENT_SPECTRUM);
        }
        else if (g_iPage == PAGE_TIME_DOMAIN_SIGNAL)
        {
          Display_TimeDomain();
        }
      }
      else
      {
        counter_display_event++;
      }

      /* ENET event */
      if (nw_update_timer == ENET_EVENT_COUNTER_ELAPSED)
      {
        nw_update_timer = 0;
        enet_metrics_log();
      }
      else
        nw_update_timer++;
    }
  } /* end of while(1) */
} /* end of main */

/*
 * desc: systick interrupt handler
 *       This interrupt occurs at (line_frequency * 512 Hz)
 *       The occurrence for this interrupt is not fixed and it can vary from
 *        cycle to cycle if line frequency varies. The systick interrupt
 *        interval is modified by pll algorithm running every line cycle
 *       Major tasks in the interrupt:
 *       1. sine_index control and sine wave generation
 *       2. AC_metrics computation (Vrms, Irms, Power)
 *       3. FFT sample collection
 *       4. Waveform sample collection
 *       5. Update scheduler flag
 *       6. Upadte ethernet timers
 * args: none
 * ret : none
 */
void sys_tick_handler()
{
  volatile int32_t radians;
  volatile int32_t sin;
  volatile uint16_t pwm_counts;

//  HWREG(GPIO_PORTN_BASE + (1 << 2)) = 1;

  /* Calculate radians */
  /* radians  = (sine_index * 2 * pi) / 512 */
  radians = _IQmpy(_IQ(PI)<<1,
      _IQ21toIQ(_IQ21div(_IQ21(pll_s.sine_index),_IQ21(SINE_SAMPLE_SIZE))));
  pll_s.sine_index = (pll_s.sine_index + 1) & (SINE_SAMPLE_SIZE - 1);

  /* trigger adc on even sine index */
  if ((pll_s.sine_index & 0x01) == 0)
  {
    ROM_ADCProcessorTrigger(ADC1_BASE, 0);
  }
  else
  {
    /* read result on odd sine index */
    ROM_ADCSequenceDataGet(ADC1_BASE, 0, &ac_raw_adc_counts[0]);
    ROM_ADCIntClear(ADC1_BASE, 0);

    /* Collect VI samples for FFT if collect flag set */
    if (fft_VI_samples_collect == true)
    {
      /* Build the fft samples array for current and voltage */
      norm_Vinst_IQ_samples[pll_s.sine_index / 2] =
      _Q12toIQ24((int32_t)ac_raw_adc_counts[0]) - _IQ(ADC_LEVEL_SHIFT);
      norm_Iinst_IQ_samples[pll_s.sine_index / 2] =
      _Q12toIQ24((int32_t)ac_raw_adc_counts[1]) - _IQ(ADC_LEVEL_SHIFT);
    }

    /* collect samples for waveform. 128 samples per cycle for the display
     * Skip alternate samples
     */
    if (skip_sample == 0)
    {
      V_waveform_buffer[wave_index] =
      _Q12toIQ24((int32_t)ac_raw_adc_counts[0]) - _IQ(ADC_LEVEL_SHIFT);
      I_waveform_buffer[wave_index] =
      _Q12toIQ24((int32_t)ac_raw_adc_counts[1]) - _IQ(ADC_LEVEL_SHIFT);
      wave_index++;
      /* wrap around the collection buffers */
      if (wave_index == DISPLAY_SAMPLE_SIZE)
        wave_index = 0;
    }
    skip_sample ^= 1;

    /* Square and accumulate adc channels (after removing offset) */
    ac_metrics.Vac.norm_acc += _IQmpy(
        (_Q12toIQ24(((int32_t)ac_raw_adc_counts[0]))-_IQ(ADC_LEVEL_SHIFT)),
        (_Q12toIQ24((int32_t)ac_raw_adc_counts[0])-_IQ(ADC_LEVEL_SHIFT)));

    ac_metrics.Iac.norm_acc += _IQmpy(
        (_Q12toIQ24(((int32_t)ac_raw_adc_counts[1]))-_IQ(ADC_LEVEL_SHIFT)),
        (_Q12toIQ24((int32_t)ac_raw_adc_counts[1])-_IQ(ADC_LEVEL_SHIFT)));

    /* power calculation */
    /* accumulate instantaneous power  */
    ac_metrics.P_inst_acc += _IQmpy(
        (_Q12toIQ24(((int32_t)ac_raw_adc_counts[0]))-_IQ(ADC_LEVEL_SHIFT)),
        (_Q12toIQ24((int32_t)ac_raw_adc_counts[1])-_IQ(ADC_LEVEL_SHIFT)));

    /* Calculating Vpeak and IPeak */
    if (_IQabs(_Q12toIQ24((int32_t)ac_raw_adc_counts[0])-_IQ(ADC_LEVEL_SHIFT))
        > ac_metrics.V_Peak)
      ac_metrics.V_Peak = _IQabs(
          _Q12toIQ24((int32_t)ac_raw_adc_counts[0])-_IQ(ADC_LEVEL_SHIFT));

    if (_IQabs(_Q12toIQ24((int32_t)ac_raw_adc_counts[1])-_IQ(ADC_LEVEL_SHIFT))
        > ac_metrics.I_Peak)
      ac_metrics.I_Peak = _IQabs(
          _Q12toIQ24((int32_t)ac_raw_adc_counts[1])-_IQ(ADC_LEVEL_SHIFT));
  }

//  /* For PLL debug */
//  if (pll_s.sine_index == 0)
//  {
//    /* toggle port here */
//    HWREG(GPIO_PORTN_BASE + (1 << 2)) ^= 1;
//  }

  /* Compute Rms parameters once every cycle */
  if (pll_s.sine_index == SINE_SAMPLE_SIZE - 1)
  {
    /* voltage rms: square root of mean of accumulated value */
    ac_metrics.Vac.norm_rms = _IQsqrt(ac_metrics.Vac.norm_acc >> 8);
    ac_metrics.Vac.norm_acc = 0;

    /* current rms: square root of mean of accumulated value */
    ac_metrics.Iac.norm_rms = _IQsqrt(ac_metrics.Iac.norm_acc >> 8);
    ac_metrics.Iac.norm_acc = 0;

    /* Active power: mean of accumulated value */
    ac_metrics.P_active = ac_metrics.P_inst_acc >> 8;
    ac_metrics.P_inst_acc = 0;

    /* Apparent power: Vrms * Irms */
    ac_metrics.P_apparent = _IQmpy(ac_metrics.Vac.norm_rms,
        ac_metrics.Iac.norm_rms);

    /* Calculate reactive power: sqrt(apparent^2 - active^2) */
    ac_metrics.P_reactive =
        _IQsqrt(
            _IQmpy(ac_metrics.P_apparent,ac_metrics.P_apparent) -_IQmpy(ac_metrics.P_active,ac_metrics.P_active));

    /* Calculate Power Factor: active/apparent */
    ac_metrics.P_PowerFactor = _IQdiv(ac_metrics.P_active,
        ac_metrics.P_apparent);

    /* Calculate Phase Shift b/w sinusoid current and voltage waveform */
    ac_metrics.Phase_shift = _IQacos(
        _IQdiv(ac_metrics.P_active,ac_metrics.P_apparent));

    /* Scheduler Flag is set to true once a cycle  */
    scheduler_flag = true;

    /* Ethernet related activity required by lwip */
    lwIPTimer(SYSTICKMS);
  }

  /* Calculate sin */
  sin = _IQsin(radians);
  /* Level shift sin */
  sin += _IQ(1);
  /* Sin is now from 0 to 2. Scale to 0 to 1 */
  sin >>= 1;

  /* Sin is scaled from 0 to 1 such that:
   * sin (0) = 0.5
   * sin (pi/2) = 1
   * sine (pi) = 0.5
   */
  /* drive duty */
  pwm_counts = _IQmpy(sin, 6000);
  /* 0 duty cycle is not possible with the chip. Therefore,
   * we disable pulses at 0 duty cycle */
   */
  if (pwm_counts == 0)
    ROM_PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, false);
  else
  {
    /* renable pulses for non zero duty cyles */
    ROM_PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, pwm_counts);
    ROM_PWMOutputState(PWM0_BASE, PWM_OUT_1_BIT, true);
  }

//  HWREG(GPIO_PORTN_BASE + (1 << 2)) = 0;
}

/*
 * desc: timer interrupt handler
 *       This interrupt occurs at falling edge of the capture pin
 *       The zero crossing detector circuit is the source for the
 *        interrupt. Toggles pin whenever line voltage changes polarity
 *       Major tasks in the interrupt:
 *       1. Get frequency of the last line cycle in capture counts
 *       2. Phase_shift of generated_sine_wave with grid voltage
 * args: none
 * ret : none
 */
void timer0_isr_handler()
{
  /* Indication to pll algorithm that capture event occured */
  pll_s.capture_detected = true;
  /* raw counts. 24bit timer register */
  pll_s.cap_counts_now = (TIMER0_TAR_R & 0xFFFFFF);
  pll_s.freq_in_cap_counts = (pll_s.cap_counts_now - pll_s.cap_counts_prev)
      & 0xFFFFFF;
  pll_s.cap_counts_prev = pll_s.cap_counts_now;

  /* Phase difference is the value of sine_index at zero crossing interrupt */
  pll_s.phase_shift_index = pll_s.sine_index;

  TIMER0_ICR_R = 4;
}
