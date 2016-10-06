//*****************************************************************************
// Compact Analyser for Power Equipment (CAPE)
// es_gui.c
// Top level file that performs all the display update. This calls the underlying
// Graphics Library: http://www.ti.com/lit/ug/spmu300a/spmu300a.pdf
//
//
// Author
// Meher Jain
// Vivek Sankaranarayanan
//*****************************************************************************

/******************** Header files  ******************************************/
#include <es_gui.h>
/*****************************************************************************/

/******************** Global Variables ****************************************/
tContext sContext;
tRectangle sRect;
uint8_t g_uiSpectrum = 0;
int8_t g_iPage = 0;
bool g_Led1On = false;
bool g_Led2On = false;
extern uint32_t sys_clk;
#define VOLTAGE_YAXIS    80
#define CURRENT_YAXIS    180

_iq V_waveform_buffer[320] =
{ 0 };
_iq I_waveform_buffer[320] =
{ 0 };
uint16_t wave_index = 0;
uint8_t skip_sample = 0;

/******************** Call back function for widgets ***************************/
void OnNext(tWidget *pWidget);
void OnPrevious(tWidget *pWidget);
/*******************************************************************************/

/******************** Widgets **************************************************/
Canvas(g_sTitlePanel, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 0, 320, 24,
    CANVAS_STYLE_TEXT | CANVAS_STYLE_FILL, ClrCUYellow, 0, ClrBlack,
    &g_sFontCm20b, "AC Metrics", 0, 0);

Canvas(g_sACMetricsWidget, 0, 0, &g_sVrms, &g_sKentec320x240x16_SSD2119, 0, 0,
    1, 1, 0, 0, 0, 0, 0, 0, 0, 0);

Canvas(g_sVrms, &g_sACMetricsWidget, 0, &g_sVrmsValue,
    &g_sKentec320x240x16_SSD2119, 0, 30, 60, 28,
    CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT, 0, ClrCUYellow, ClrCUYellow,
    &g_sFontCm16b, "Vrms", 0, 0);

Canvas(g_sVrmsValue, &g_sVrms, 0, &g_sIrms, &g_sKentec320x240x16_SSD2119, 55,
    30, 60, 28, CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT, ClrCUYellow, 0, ClrBlack,
    &g_sFontCm18, "123.4 V", 0, 0);

Canvas(g_sIrms, &g_sVrmsValue, 0, &g_sIrmsValue, &g_sKentec320x240x16_SSD2119,
    0, 60, 60, 28, CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT, 0, ClrCUYellow,
    ClrCUYellow, &g_sFontCm16b, "Irms", 0, 0);

Canvas(g_sIrmsValue, &g_sIrms, 0, &g_sFreq, &g_sKentec320x240x16_SSD2119, 55,
    60, 60, 28, CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT, ClrCUYellow, 0, ClrBlack,
    &g_sFontCm18, "6.0  A", 0, 0);

Canvas(g_sFreq, &g_sIrmsValue, 0, &g_sFreqValue, &g_sKentec320x240x16_SSD2119,
    0, 90, 60, 28, CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT, 0, ClrCUYellow,
    ClrCUYellow, &g_sFontCm16b, "Freq", 0, 0);

Canvas(g_sFreqValue, &g_sFreq, 0, &g_sPF, &g_sKentec320x240x16_SSD2119, 55, 90,
    60, 28, CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT, ClrCUYellow, 0, ClrBlack,
    &g_sFontCm18, "59.9 Hz", 0, 0);

Canvas(g_sPF, &g_sFreqValue, 0, &g_sPFValue, &g_sKentec320x240x16_SSD2119, 0,
    120, 60, 28, CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT, 0, ClrCUYellow,
    ClrCUYellow, &g_sFontCm16b, "PF  ", 0, 0);

Canvas(g_sPFValue, &g_sPF, 0, &g_sP_apparent, &g_sKentec320x240x16_SSD2119, 55,
    120, 60, 28, CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT, ClrCUYellow, 0,
    ClrBlack, &g_sFontCm18, "0.99", 0, 0);

Canvas(g_sP_apparent, &g_sPFValue, 0, &g_sP_apparent_val,
    &g_sKentec320x240x16_SSD2119, 204, 30, 60, 28,
    CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT, 0, ClrCUYellow, ClrCUYellow,
    &g_sFontCm16b, "P (VA)", 0, 0);

Canvas(g_sP_apparent_val, &g_sP_apparent, 0, &g_sP_active,
    &g_sKentec320x240x16_SSD2119, 259, 30, 60, 28,
    CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT, ClrCUYellow, 0, ClrBlack,
    &g_sFontCm18, "720.5", 0, 0);

Canvas(g_sP_active, &g_sP_apparent, 0, &g_sP_active_val,
    &g_sKentec320x240x16_SSD2119, 204, 60, 60, 28,
    CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT, 0, ClrCUYellow, ClrCUYellow,
    &g_sFontCm16b, "P (W)", 0, 0);

Canvas(g_sP_active_val, &g_sP_active, 0, &g_sTHDv, &g_sKentec320x240x16_SSD2119,
    259, 60, 60, 28, CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT, ClrCUYellow, 0,
    ClrBlack, &g_sFontCm18, "740.9", 0, 0);

Canvas(g_sTHDv, &g_sP_active_val, 0, &g_sTHDv_val, &g_sKentec320x240x16_SSD2119,
    0, 150, 60, 28, CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT, 0, ClrCUYellow,
    ClrCUYellow, &g_sFontCm16b, "Thd(V)", 0, 0);

Canvas(g_sTHDv_val, &g_sTHDv, 0, &g_sTHDi, &g_sKentec320x240x16_SSD2119, 55,
    150, 60, 28, CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT, ClrCUYellow, 0,
    ClrBlack, &g_sFontCm18, "2.1 %", 0, 0);

Canvas(g_sTHDi, &g_sTHDv_val, 0, &g_sTHDi_val, &g_sKentec320x240x16_SSD2119, 0,
    180, 60, 28, CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT, 0, ClrCUYellow,
    ClrCUYellow, &g_sFontCm16b, "Thd(I)", 0, 0);

Canvas(g_sTHDi_val, &g_sTHDi, 0, &g_sPhase, &g_sKentec320x240x16_SSD2119, 55,
    180, 60, 28, CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT, ClrCUYellow, 0,
    ClrBlack, &g_sFontCm18, "7.5 %", 0, 0);

Canvas(g_sPhase, &g_sTHDv_val, 0, &g_sPhase_val, &g_sKentec320x240x16_SSD2119,
    0, 210, 60, 28, CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT, 0, ClrCUYellow,
    ClrCUYellow, &g_sFontCm16b, "Phase", 0, 0);

Canvas(g_sPhase_val, &g_sPhase, 0, &g_sVpeak, &g_sKentec320x240x16_SSD2119, 55,
    210, 60, 28, CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT, ClrCUYellow, 0,
    ClrBlack, &g_sFontCm18, "7.5 %", 0, 0);

Canvas(g_sVpeak, &g_sPhase_val, 0, &g_sVpeak_val, &g_sKentec320x240x16_SSD2119,
    204, 180, 60, 28, CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT, 0,
    ClrCUYellow, ClrCUYellow, &g_sFontCm16b, "Vpk", 0, 0);

Canvas(g_sVpeak_val, &g_sVpeak, 0, &g_sIpeak, &g_sKentec320x240x16_SSD2119, 259,
    180, 60, 28, CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT, ClrCUYellow, 0,
    ClrBlack, &g_sFontCm18, "155.5 V", 0, 0);

Canvas(g_sIpeak, &g_sVpeak_val, 0, &g_sIpeak_val, &g_sKentec320x240x16_SSD2119,
    204, 210, 60, 28, CANVAS_STYLE_TEXT | CANVAS_STYLE_TEXT_LEFT, 0,
    ClrCUYellow, ClrCUYellow, &g_sFontCm16b, "Ipk", 0, 0);

Canvas(g_sIpeak_val, &g_sIpeak, 0, 0, &g_sKentec320x240x16_SSD2119, 259, 210,
    60, 28, CANVAS_STYLE_FILL | CANVAS_STYLE_TEXT, ClrCUYellow, 0, ClrBlack,
    &g_sFontCm18, "1.5 A", 0, 0);

/* Next button */
RectangularButton(g_sNext, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 296, 0, 23,
    23, PB_STYLE_IMG, ClrCUYellow, ClrCUYellow, 0, 0, 0, 0, g_pui8Right24x23,
    g_pui8RightSmall15x14, 0, 0, OnNext);

/* Previous button */
RectangularButton(g_sPrevious, 0, 0, 0, &g_sKentec320x240x16_SSD2119, 0, 0, 23,
    23, PB_STYLE_IMG, ClrCUYellow, ClrCUYellow, 0, 0, 0, 0, g_pui8Left24x23,
    g_pui8LeftSmall15x14, 0, 0, OnPrevious);
/*******************************************************************************/

/* desc: Call back for next buttom pressed on display
 * args: pointer to widget
 * ret : none
 */
void OnNext(tWidget *pWidget)
{
  g_Led1On = !g_Led1On;
  if (g_Led1On)
  {
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0xFF);
  }
  else
  {
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_0, 0x00);
  }

  /* Page increment and wraparound */
  g_iPage++;
  if (g_iPage == LAST_PAGE + 1)
    g_iPage = 0;

  /* Setup the new page */
  display_setup_page(g_iPage);
}

/* desc: Call back for previous buttom pressed on display
 * args: pointer to widget
 * ret : none
 */
void OnPrevious(tWidget *pWidget)
{
  g_Led2On = !g_Led2On;
  if (g_Led2On)
  {
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0xFF);
  }
  else
  {
    GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, 0x00);
  }

  /* Page decrement and wraparound */
  g_iPage--;
  if (g_iPage == -1)
    g_iPage = LAST_PAGE;

  /* Setup the new page */
  display_setup_page(g_iPage);

}

/* desc: Initialize the display context. Dispaly the starting page
 * args: none
 * ret : none
 */
void gui_init()
{
  //
  // Initialize the display driver.
  //
  Kentec320x240x16_SSD2119Init();

  //
  // Initialize the graphics context.
  //
  GrContextInit(&sContext, &g_sKentec320x240x16_SSD2119);

  /* Draw logo */
  GrImageDraw(&sContext, g_pui8Image, 0, 0);
  GrFlush(&sContext);

  TouchScreenInit(sys_clk);
  TouchScreenCallbackSet(WidgetPointerMessage);

  WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sTitlePanel);
  WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sNext);
  WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sPrevious);
  WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sACMetricsWidget);
  WidgetPaint(WIDGET_ROOT);
}

/* desc: Setup the new page on a page change
 * args: page_no
 * ret : none
 */
void display_setup_page(int8_t page_no)
{
  if (page_no == PAGE_METRICS)
  {
    /* Draw logo */
    GrImageDraw(&sContext, g_pui8Image, 0, 0);
    GrFlush(&sContext);

    /* Change title text */
    CanvasTextSet(&g_sTitlePanel, "AC Metrics");

    /* Add the metrics canvas widget */
    WidgetAdd(WIDGET_ROOT, (tWidget *) &g_sACMetricsWidget);
  }
  else if ((page_no == PAGE_VOLTAGE_SPECTRUM)
      || (page_no == PAGE_CURRENT_SPECTRUM))
  {
    /* fill screen with black. No logo */
    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = 319;
    sRect.i16YMax = 239;
    GrContextForegroundSet(&sContext, ClrBlack);
    GrRectFill(&sContext, &sRect);

    /* Add the metrics canvas widget */
    WidgetRemove((tWidget *) &g_sACMetricsWidget);

    /* Draw the frequency base */
    sRect.i16XMin = 0;
    sRect.i16YMin = 229;
    sRect.i16XMax = 319;
    sRect.i16YMax = 231;
    GrContextForegroundSet(&sContext, ClrCUYellow);
    GrRectFill(&sContext, &sRect);

    /* Change title text */
    if (page_no == PAGE_VOLTAGE_SPECTRUM)
    {
      CanvasTextSet(&g_sTitlePanel, "Voltage Spectrum");
      g_uiSpectrum = VOLTAGE_SPECTRUM;
    }
    else
    {
      CanvasTextSet(&g_sTitlePanel, "Current Spectrum");
      g_uiSpectrum = CURRENT_SPECTRUM;
    }
  }

  else if ((page_no == PAGE_TIME_DOMAIN_SIGNAL))
  {
    CanvasTextSet(&g_sTitlePanel, "Time Domain Signal");

    sRect.i16XMin = 0;
    sRect.i16YMin = 0;
    sRect.i16XMax = 319;
    sRect.i16YMax = 239;
    GrContextForegroundSet(&sContext, ClrBlack);
    GrRectFill(&sContext, &sRect);

    /* Add the metrics canvas widget */
    WidgetRemove((tWidget *) &g_sACMetricsWidget);
  }

  /* Draw the widgets */
  WidgetPaint(WIDGET_ROOT);
}

/* desc: Display the frequency spectrum based on parameter
 * args: paramter (Voltage or Current)
 * ret : none
 */
void Display_FreqSpectrum(uint8_t param)
{
  /* We will use 20 bins to store 20 frequency amplitudes */
  int32_t frequency_bins[20];
  uint8_t i;

  /* Wipe the previous FFT */
  sRect.i16XMin = 0;
  sRect.i16YMin = 30;
  sRect.i16XMax = 319;
  sRect.i16YMax = 228;
  GrContextForegroundSet(&sContext, ClrBlack);

  GrRectFill(&sContext, &sRect);

  GrContextForegroundSet(&sContext, ClrCUYellow);

  if (param == VOLTAGE_SPECTRUM)
  {
    for (i = 0; i < 20; i++)
    {
      /* Scale the frequency bin values with the y axis span of the spectrum canvas */
      frequency_bins[i] = 229
          - (int32_t) (fft_output_array_voltage[i] * (240.0 - 40.0));
    }
  }
  else if (param == CURRENT_SPECTRUM)
  {
    for (i = 0; i < 20; i++)
    {
      /* Scale the frequency bin values with the y axis span of the spectrum canvas */
      frequency_bins[i] = 229
          - (int32_t) (fft_output_array_current[i] * 25 * (240.0 - 40.0));
    }
  }

  /* frequency bin now  contains y points */
  for (i = 0; i < 20; i++)
  {
    //GrLineDrawV(&sContext,i*16,229,frequency_bins[i]);
    GrLineDraw(&sContext, (i * 16 - 4), 229, i * 16, frequency_bins[i]);
    GrLineDraw(&sContext, (i * 16), frequency_bins[i], (i * 16 + 4), 229);
  }

  fft_output_array_voltage[0] = CURRENT_SPECTRUM;
}

/* desc: Display the time domain waveforms on the screen
 * args: none
 * ret : none
 */
void Display_TimeDomain(void)
{
  uint32_t XPixelCurrent = 0;
  uint32_t YPixelCurrent[DISPLAY_SAMPLE_SIZE];
  static uint32_t YPixelPrev_Volt[DISPLAY_SAMPLE_SIZE] =
  { 0 };
  static uint32_t YPixelPrev_Curr[DISPLAY_SAMPLE_SIZE] =
  { 0 };
  uint32_t DataCount = 0;

  GrContextForegroundSet(&sContext, ClrCUYellow);
  GrContextFontSet(&sContext, &g_sFontCm16b);
  GrStringDrawCentered(&sContext, "Voltage", -1, 280, 31, 0);
  GrStringDrawCentered(&sContext, "Current", -1, 280, 131, 0);

  GrContextForegroundSet(&sContext, ClrWhite);
  GrLineDrawV(&sContext, 0, 30, 239);
  GrLineDrawH(&sContext, 0, 319, VOLTAGE_YAXIS);

  DataCount = 0;
  while (DataCount < DISPLAY_SAMPLE_SIZE)
  {
//    GrContextForegroundSet(&sContext, ClrBlack);
//    GrPixelDraw(&sContext,XPixelCurrent,YPixelPrev[DataCount]);
//    if(DataCount != 0)
    YPixelCurrent[DataCount] = VOLTAGE_YAXIS
        - VOLTAGE_YAXIS * ((V_waveform_buffer[DataCount]) / 16777216.0);
    if (DataCount != 0)
    {
      GrContextForegroundSet(&sContext, ClrBlack);
      /* Clear the previous cycle's data */
      if (YPixelPrev_Volt[DataCount] != 0)
      {
        GrLineDraw(&sContext, XPixelCurrent - 1, YPixelPrev_Volt[DataCount - 1],
            XPixelCurrent, YPixelPrev_Volt[DataCount]);
      }
      GrContextForegroundSet(&sContext, ClrCUYellow);
      GrLineDraw(&sContext, XPixelCurrent - 1, YPixelCurrent[DataCount - 1],
          XPixelCurrent, YPixelCurrent[DataCount]);
    }
    DataCount++;
    XPixelCurrent++;
  }

  DataCount = 0;
  while (DataCount < DISPLAY_SAMPLE_SIZE)
  {
    YPixelPrev_Volt[DataCount] = YPixelCurrent[DataCount];
    DataCount++;
  }

  /* For current */
  GrContextForegroundSet(&sContext, ClrWhite);
  GrLineDrawH(&sContext, 0, 319, CURRENT_YAXIS);

  DataCount = 0;
  XPixelCurrent = 0;
  while (DataCount < DISPLAY_SAMPLE_SIZE)
  {
//    GrContextForegroundSet(&sContext, ClrBlack);
//    GrPixelDraw(&sContext,XPixelCurrent,YPixelPrev[DataCount]);
//    if(DataCount != 0)
    YPixelCurrent[DataCount] = CURRENT_YAXIS
        - CURRENT_YAXIS * ((I_waveform_buffer[DataCount]) * 6 / 16777216.0);
    if (DataCount != 0)
    {
      GrContextForegroundSet(&sContext, ClrBlack);
      /* Clear the previous cycle's data */
      if (YPixelPrev_Curr[DataCount] != 0)
      {
        GrLineDraw(&sContext, XPixelCurrent - 1, YPixelPrev_Curr[DataCount - 1],
            XPixelCurrent, YPixelPrev_Curr[DataCount]);
      }
      GrContextForegroundSet(&sContext, ClrCUYellow);
      GrLineDraw(&sContext, XPixelCurrent - 1, YPixelCurrent[DataCount - 1],
          XPixelCurrent, YPixelCurrent[DataCount]);
    }
    DataCount++;
    XPixelCurrent++;
  }

  DataCount = 0;
  while (DataCount < DISPLAY_SAMPLE_SIZE)
  {
    YPixelPrev_Curr[DataCount] = YPixelCurrent[DataCount];
    DataCount++;
  }
}
