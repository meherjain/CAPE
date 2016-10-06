//*****************************************************************************
//
// Kentec320x240x16_ssd2119_SPI.c - Display driver for the Kentec
//                                  BOOSTXL-K350QVG-S1 TFT display with an SSD2119
//                                  controller.  This version assumes an SPI interface
//                                  between the micro and display.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup display_api
//! @{
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_gpio.h"
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/ssi.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "driverlib/rom.h"
#include "driverlib/pin_map.h"
#include "grlib/grlib.h"
#include "Kentec320x240x16_SSD2119_SPI.h"
//*****************************************************************************
//
// This driver operates in four different screen orientations.  They are:
//
// * Portrait - The screen is taller than it is wide, and the flex connector is
//              on the left of the display.  This is selected by defining
//              PORTRAIT.
//
// * Landscape - The screen is wider than it is tall, and the flex connector is
//               on the bottom of the display.  This is selected by defining
//               LANDSCAPE.
//
// * Portrait flip - The screen is taller than it is wide, and the flex
//                   connector is on the right of the display.  This is
//                   selected by defining PORTRAIT_FLIP.
//
// * Landscape flip - The screen is wider than it is tall, and the flex
//                    connector is on the top of the display.  This is
//                    selected by defining LANDSCAPE_FLIP.
//
// These can also be imagined in terms of screen rotation; if portrait mode is
// 0 degrees of screen rotation, landscape is 90 degrees of counter-clockwise
// rotation, portrait flip is 180 degrees of rotation, and landscape flip is
// 270 degress of counter-clockwise rotation.
//
// If no screen orientation is selected, "landscape flip" mode will be used.
//
//*****************************************************************************
#if ! defined(PORTRAIT) && ! defined(PORTRAIT_FLIP) && \
    ! defined(LANDSCAPE) && ! defined(LANDSCAPE_FLIP)
#define LANDSCAPE
//#define PORTRAIT_FLIP
#endif

//*****************************************************************************
//
// Various definitions controlling coordinate space mapping and drawing
// direction in the four supported orientations.
//
//*****************************************************************************
#ifdef PORTRAIT
#define HORIZ_DIRECTION 0x28
#define VERT_DIRECTION 0x20
#define MAPPED_X(x, y) (319 - (y))
#define MAPPED_Y(x, y) (x)
#endif
#ifdef LANDSCAPE
#define HORIZ_DIRECTION 0x00
#define VERT_DIRECTION  0x08
#define MAPPED_X(x, y) (319 - (x))
#define MAPPED_Y(x, y) (239 - (y))
#endif
#ifdef PORTRAIT_FLIP
#define HORIZ_DIRECTION 0x18
#define VERT_DIRECTION 0x10
#define MAPPED_X(x, y) (y)
#define MAPPED_Y(x, y) (239 - (x))
#endif
#ifdef LANDSCAPE_FLIP
#define HORIZ_DIRECTION 0x30
#define VERT_DIRECTION  0x38
#define MAPPED_X(x, y) (x)
#define MAPPED_Y(x, y) (y)
#endif

//#define SPI3	//9-bit 3-wire SPI (SSI2) mode (SCLK, SDA, SCS)
// Default setting for the BOOSTXL-K350QVG-S1
// Need to remove the "R10" (connect the "PB7/SSI2Tx" to "PD1/AIN6_TOUCH_XP") on Launchpad
#define SPI4	//8-bit 4-wire SPI (SSI2) mode (SCLK, SDA, SCS, SDC)
// Need to remove the "R10" (connect the "PB7/SSI2Tx" to "PD1/AIN6_TOUCH_XP") on Launchpad
// Need to move the "R2" to "R3" position,
// and move "R8" to "R9" position on the BOOSTXL-K350QVG-S1

//*****************************************************************************
//
// Defines for the pins that are used to communicate with the SSD2119.
//
//*****************************************************************************
/// Commented  By Meher on 04/12/16
//#define LCD_CS_PERIPH           SYSCTL_PERIPH_GPIOA
//#define LCD_CS_BASE             GPIO_PORTA_BASE
//#define LCD_CS_PIN              GPIO_PIN_4
//
//#define LCD_DC_PERIPH           SYSCTL_PERIPH_GPIOA
//#define LCD_DC_BASE             GPIO_PORTA_BASE
//#define LCD_DC_PIN              GPIO_PIN_5
//
//#define LCD_RST_PERIPH           SYSCTL_PERIPH_GPIOD
//#define LCD_RST_BASE             GPIO_PORTD_BASE
//#define LCD_RST_PIN              GPIO_PIN_7
//
//#define LCD_LED_PERIPH           SYSCTL_PERIPH_GPIOF
//#define LCD_LED_BASE             GPIO_PORTF_BASE
//#define LCD_LED_PIN              GPIO_PIN_2

//*****************************************************************************

// Defines for the pins that are used to communicate with the SSD2119.
//
//*****************************************************************************
/// Commented  By Meher on 04/12/16
extern uint32_t sys_clk;
#define LCD_CS_PERIPH           SYSCTL_PERIPH_GPIOP
#define LCD_CS_BASE             GPIO_PORTP_BASE
#define LCD_CS_PIN              GPIO_PIN_3

#define LCD_DC_PERIPH           SYSCTL_PERIPH_GPIOP
#define LCD_DC_BASE             GPIO_PORTP_BASE
#define LCD_DC_PIN              GPIO_PIN_4

#define LCD_RST_PERIPH           SYSCTL_PERIPH_GPIOK
#define LCD_RST_BASE             GPIO_PORTK_BASE
#define LCD_RST_PIN              GPIO_PIN_6

#define LCD_LED_PERIPH           SYSCTL_PERIPH_GPIOG
#define LCD_LED_BASE             GPIO_PORTG_BASE
#define LCD_LED_PIN              GPIO_PIN_1

//*****************************************************************************
//
// The dimensions of the LCD panel.
//
//*****************************************************************************
#define LCD_VERTICAL_MAX 240
#define LCD_HORIZONTAL_MAX 320

#define SSD2119_ENTRY_MODE_REG        0x11
#define SSD2119_RAM_DATA_REG          0x22
#define SSD2119_V_RAM_POS_REG         0x44
#define SSD2119_H_RAM_START_REG       0x45
#define SSD2119_H_RAM_END_REG         0x46
#define SSD2119_X_RAM_ADDR_REG        0x4E
#define SSD2119_Y_RAM_ADDR_REG        0x4F
#define ENTRY_MODE_DEFAULT 0x6830
#define MAKE_ENTRY_MODE(x) ((ENTRY_MODE_DEFAULT & 0xFF00) | (x))

//*****************************************************************************
//
// Translates a 24-bit RGB color to a display driver-specific color.
//
// \param c is the 24-bit RGB color.  The least-significant byte is the blue
// channel, the next byte is the green channel, and the third byte is the red
// channel.
//
// This macro translates a 24-bit RGB color into a value that can be written
// into the display's frame buffer in order to reproduce that color, or the
// closest possible approximation of that color.
//
// \return Returns the display-driver specific color.
//
//*****************************************************************************
#define DPYCOLORTRANSLATE(c)    ((((c) & 0x00f80000) >> 8) |               \
                                 (((c) & 0x0000fc00) >> 5) |               \
                                 (((c) & 0x000000f8) >> 3))
//*****************************************************************************
//
// Macro used to set the LCD data bus in preparation for writing a byte to the
// device.
//
//*****************************************************************************
uint16_t pui16Data[2];

#ifdef SPI3
//*****************************************************************************
// Writes a data word to the SSD2119.  This function implements the 3-wire SPI
// interface to the LCD display.
//*****************************************************************************
static void
WriteDataSPI(unsigned short usData)
{
  pui16Data[0] = 0x100 | (usData >> 8);	// set the most significant byte of the data.
  pui16Data[1] = 0x100 | usData;// set the least significant byte of the data.

  HWREG(LCD_CS_BASE + GPIO_O_DATA + (LCD_CS_PIN << 2)) = 0;//CS="0"

  SSIDataPut(SSI2_BASE, pui16Data[0]);
  SSIDataPut(SSI2_BASE, pui16Data[1]);
  while(SSIBusy(SSI2_BASE))
  {}  // Wait until SSI0 is done transferring all the data in the transmit FIFO.

  HWREG(LCD_CS_BASE + GPIO_O_DATA + (LCD_CS_PIN << 2)) = LCD_CS_PIN;
}
//*****************************************************************************
// Writes register word to the SSD2119.  This function implements the 3-wire SPI
// interface to the LCD display.
//*****************************************************************************
static void
WriteCommandSPI(unsigned short usData)
{
  pui16Data[0] = 0;   // set the most significant byte of the data.
  pui16Data[1] = 0xFF & usData;// set the least significant byte of the data.

  HWREG(LCD_CS_BASE + GPIO_O_DATA + (LCD_CS_PIN << 2)) = 0;//CS="0"

  SSIDataPut(SSI2_BASE, pui16Data[0]);
  SSIDataPut(SSI2_BASE, pui16Data[1]);
  while(SSIBusy(SSI2_BASE))
  {}  // Wait until SSI0 is done transferring all the data in the transmit FIFO.

  HWREG(LCD_CS_BASE + GPIO_O_DATA + (LCD_CS_PIN << 2)) = LCD_CS_PIN;
}
#endif

#ifdef SPI4
static void WriteDataSPI(unsigned short usData)
{
  pui16Data[0] = (usData >> 8); // Write the most significant byte of the data to the bus.
  pui16Data[1] = usData; // Write the least significant byte of the data to the bus.

  HWREG(LCD_DC_BASE + GPIO_O_DATA + (LCD_DC_PIN << 2)) = LCD_DC_PIN;   //DC="1"
  HWREG(LCD_CS_BASE + GPIO_O_DATA + (LCD_CS_PIN << 2)) = 0;   //CS="0"

  //SSIDataPut(SSI2_BASE, pui16Data[0]);
  //SSIDataPut(SSI2_BASE, pui16Data[1]);
  //while(SSIBusy(SSI2_BASE)){ }   // Wait until SSI0 is done transferring all the data in the transmit FIFO.

  // MEHER // 04/12/16
  SSIDataPut(SSI3_BASE, pui16Data[0]);
  SSIDataPut(SSI3_BASE, pui16Data[1]);
  while (SSIBusy(SSI3_BASE))
  {
  }   // Wait until SSI0 is done transferring all the data in the transmit FIFO.

  HWREG(LCD_CS_BASE + GPIO_O_DATA + (LCD_CS_PIN << 2)) = LCD_CS_PIN;
}
//*****************************************************************************
// Writes register word to the SSD2119.  This function implements the 3-wire SPI
// interface to the LCD display.
//*****************************************************************************
static void WriteCommandSPI(unsigned short usData)
{
  pui16Data[0] = 0;   // Write the most significant byte of the data to the bus.
  pui16Data[1] = usData; // Write the least significant byte of the data to the bus.

  HWREG(LCD_DC_BASE + GPIO_O_DATA + (LCD_DC_PIN << 2)) = 0;   //DC="0"
  HWREG(LCD_CS_BASE + GPIO_O_DATA + (LCD_CS_PIN << 2)) = 0;   //CS="0"

  //SSIDataPut(SSI2_BASE, pui16Data[0]);
  //SSIDataPut(SSI2_BASE, pui16Data[1]);
  //while(SSIBusy(SSI2_BASE)){ }   // Wait until SSI0 is done transferring all the data in the transmit FIFO.

// Meher //04/12/16
  SSIDataPut(SSI3_BASE, pui16Data[0]);
  SSIDataPut(SSI3_BASE, pui16Data[1]);
  while (SSIBusy(SSI3_BASE))
  {
  }   // Wait until SSI0 is done transferring all the data in the transmit FIFO.

  HWREG(LCD_CS_BASE + GPIO_O_DATA + (LCD_CS_PIN << 2)) = LCD_CS_PIN;
}
#endif

//*****************************************************************************
// Initializes the LCD interface.
// Enable the SSI2 for LCD data/register transfer.
// Set the LCD_CS_PIN, LCD_DC_PIN, LCD_RST_PIN.
//*****************************************************************************
static void LCD_PORT_Init(void)
{
  uint32_t pui32DataRx[3];

  // The SSI2 peripheral must be enabled for use.
//  SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
// MEHER 04/12/16
// The SSI3 peripheral must be enabled for use in SCLK and SDA.
  SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI3);

  // For this example SSI2 is used with PortB[7:4].  The actual port and pins
  // used may be different on your part, consult the data sheet for more
  // information.  GPIO port B needs to be enabled so these pins can be used.
  // TODO: change this to whichever GPIO port you are using.
  //SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
// MEHER 04/12/16
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);

  // Enable the PortA[5:4] for the LCD_SCS, LCD_SDC
  // Enable the PortD[7] for LCD_RST
  // Enable the PortF[2] for LED backlight
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
  SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);

  // Change PD7 into GPIO output.
  HWREG(GPIO_PORTK_BASE + GPIO_O_LOCK) |= GPIO_LOCK_KEY;
  HWREG(GPIO_PORTK_BASE + GPIO_O_CR) |= 0x80;
  HWREG(GPIO_PORTK_BASE + GPIO_O_AFSEL) &= 0x7f;
  GPIOPinTypeGPIOOutput(LCD_RST_BASE, LCD_RST_PIN);

  GPIOPinTypeGPIOOutput(LCD_CS_BASE, LCD_CS_PIN);
  GPIOPinTypeGPIOOutput(LCD_DC_BASE, LCD_DC_PIN);

  GPIOPinTypeGPIOOutput(LCD_LED_BASE, LCD_LED_PIN);
  //GPIOPadConfigSet(LCD_LED_BASE, LCD_LED_PIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

  // Configure the pin muxing for SSI2 functions on port B4, B5, B6, and B7.
  // This step is not necessary if your part does not support pin muxing.
  // TODO: change this to select the port/pin you are using.
//    GPIOPinConfigure(GPIO_PB4_SSI2CLK);
//    GPIOPinConfigure(GPIO_PB5_SSI2FSS);
//    GPIOPinConfigure(GPIO_PB6_SSI2RX);
//    GPIOPinConfigure(GPIO_PB7_SSI2TX);

  // meher 04/12/16
  GPIOPinConfigure(GPIO_PQ0_SSI3CLK);
  //GPIOPinConfigure(GPIO_PB5_SSI2FSS);
  GPIOPinConfigure(GPIO_PQ2_SSI3XDAT0);
  //GPIOPinConfigure(GPIO_PB7_SSI2TX);

//
  // Configure the GPIO settings for the SSI pins.  This function also gives
  // control of these pins to the SSI hardware.  Consult the data sheet to
  // see which functions are allocated per pin.
  // The pins are assigned as follows:
  //      PB7 - SSI2Tx
  //      PB6 - SSI2Rx
  //      PB5 - SSI2Fss
  //      PB4 - SSI2CLK
  // TODO: change this to select the port/pin you are using.
  // GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4);
  //GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_7, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

  // PQ0 - SSI3CLK-LCD_SCL
  // PQ2 - SSI3XDAT0-LCD_SDI

  GPIOPinTypeSSI(GPIO_PORTQ_BASE, GPIO_PIN_0 | GPIO_PIN_2); // | GPIO_PIN_5 | GPIO_PIN_4);
  GPIOPadConfigSet(GPIO_PORTQ_BASE, GPIO_PIN_2, GPIO_STRENGTH_8MA,
      GPIO_PIN_TYPE_STD);

  // Configure and enable the SSI port for SPI master mode.  Use SSI0,
  // system clock supply, idle clock level low and active low clock in
  // freescale SPI mode, master mode, 1MHz SSI frequency, and 8-bit data.
  // For SPI mode, you can set the polarity of the SSI clock when the SSI
  // unit is idle.  You can also configure what clock edge you want to
  // capture data on.  Please reference the datasheet for more information on
  // the different SPI modes.
#ifdef SPI3
  SSIConfigSetExpClk(SSI3_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 20000000, 9);
#endif
#ifdef SPI4
  SSIConfigSetExpClk(SSI3_BASE, sys_clk, SSI_FRF_MOTO_MODE_0,
      SSI_MODE_MASTER, 20000000, 8);
#endif

  // Enable the SSI3 module.
  SSIEnable(SSI3_BASE);

  // Read any residual data from the SSI port.  This makes sure the receive
  // FIFOs are empty, so we don't read any unwanted junk.  This is done here
  // because the SPI SSI mode is full-duplex, which allows you to send and
  // receive at the same time.  The SSIDataGetNonBlocking function returns
  // "true" when data was returned, and "false" when no data was returned.
  // The "non-blocking" function checks if there is any data in the receive
  // FIFO and does not "hang" if there isn't.
  while (SSIDataGetNonBlocking(SSI3_BASE, &pui32DataRx[0]))
  {
  }
}

void LED_backlight_ON(void)
{
  HWREG(LCD_LED_BASE + GPIO_O_DATA + (LCD_LED_PIN << 2)) = LCD_LED_PIN;
}
void LED_backlight_OFF(void)
{
  HWREG(LCD_LED_BASE + GPIO_O_DATA + (LCD_LED_PIN << 2)) = 0;
}

//*****************************************************************************
//
//! Initializes the display driver.
//!
//! This function initializes the SSD2119 display controller on the panel,
//! preparing it to display data.
//!
//! \return None.
//
//*****************************************************************************
void Kentec320x240x16_SSD2119Init(void)
{
  unsigned int ulClockMS, ulCount;

  // Get the current processor clock frequency.
//    ulClockMS = SysCtlClockGet() / (3 * 1000);
  ulClockMS = sys_clk / (3 * 1000);

  // Initializes the LCD port
  LCD_PORT_Init();

  // Reset the LCD
  HWREG(LCD_RST_BASE + GPIO_O_DATA + (LCD_RST_PIN << 2)) = 0;  //RST="0"
  SysCtlDelay(10 * ulClockMS);
  HWREG(LCD_RST_BASE + GPIO_O_DATA + (LCD_RST_PIN << 2)) = LCD_RST_PIN; //RST="1"
  SysCtlDelay(20 * ulClockMS);

  // Enter sleep mode (if we are not already there).
  WriteCommandSPI(0x10);  //SLEEP_MODE_REG
  WriteDataSPI(0x0001);

  //
  // Set initial power parameters.
  //
  WriteCommandSPI(0x1E);    //SSD2119_PWR_CTRL_5_REG
  WriteDataSPI(0x00BA);
  WriteCommandSPI(0x28);    //SSD2119_VCOM_OTP_1_REG
  WriteDataSPI(0x0006);

  //
  // Start the oscillator.
  //
  WriteCommandSPI(0x00);
  WriteDataSPI(0x0001);

  //
  // Set pixel format and basic display orientation (scanning direction).
  //
  WriteCommandSPI(0x01);
  WriteDataSPI(0x30EF);
  WriteCommandSPI(0x02);
  WriteDataSPI(0x0600);

  //
  // Exit sleep mode.
  //
  WriteCommandSPI(0x10);    //SSD2119_SLEEP_MODE_REG
  WriteDataSPI(0x0000);

  //
  // Delay 30mS
  //
  SysCtlDelay(30 * ulClockMS);

  //
  // Configure pixel color format and MCU interface parameters.
  //
  WriteCommandSPI(SSD2119_ENTRY_MODE_REG);
  WriteDataSPI(ENTRY_MODE_DEFAULT);

  //
  // Enable the display.
  //
  WriteCommandSPI(0x07);
  WriteDataSPI(0x0033);

  //
  // Set VCIX2 voltage to 6.1V.
  //
  WriteCommandSPI(0x0C);    //SSD2119_PWR_CTRL_2_REG
  WriteDataSPI(0x0005);

  //
  // Configure gamma correction.
  //
  WriteCommandSPI(0x30);
  WriteDataSPI(0x0000);    //SSD2119_GAMMA_CTRL_1
  WriteCommandSPI(0x31);
  WriteDataSPI(0x0400);    //SSD2119_GAMMA_CTRL_2
  WriteCommandSPI(0x32);
  WriteDataSPI(0x0106);    //SSD2119_GAMMA_CTRL_3
  WriteCommandSPI(0x33);
  WriteDataSPI(0x0700);    //SSD2119_GAMMA_CTRL_4
  WriteCommandSPI(0x34);
  WriteDataSPI(0x0002);    //SSD2119_GAMMA_CTRL_5
  WriteCommandSPI(0x35);
  WriteDataSPI(0x0702);    //SSD2119_GAMMA_CTRL_6
  WriteCommandSPI(0x36);
  WriteDataSPI(0x0707);    //SSD2119_GAMMA_CTRL_7
  WriteCommandSPI(0x37);
  WriteDataSPI(0x0203);    //SSD2119_GAMMA_CTRL_8
  WriteCommandSPI(0x3A);
  WriteDataSPI(0x1400);    //SSD2119_GAMMA_CTRL_9
  WriteCommandSPI(0x3B);
  WriteDataSPI(0x0F03);    //SSD2119_GAMMA_CTRL_10

  //
  // Configure Vlcd63 and VCOMl.
  //
  WriteCommandSPI(0x0D);    //SSD2119_PWR_CTRL_3_REG
  WriteDataSPI(0x0007);
  WriteCommandSPI(0x0E);    //SSD2119_PWR_CTRL_4_REG
  WriteDataSPI(0x3100);

  //
  // Set the display size and ensure that the GRAM window is set to allow
  // access to the full display buffer.
  //
  WriteCommandSPI(SSD2119_V_RAM_POS_REG);
  WriteDataSPI((LCD_VERTICAL_MAX - 1) << 8);
  WriteCommandSPI(SSD2119_H_RAM_START_REG);
  WriteDataSPI(0x0000);
  WriteCommandSPI(SSD2119_H_RAM_END_REG);
  WriteDataSPI(LCD_HORIZONTAL_MAX - 1);
  WriteCommandSPI(SSD2119_X_RAM_ADDR_REG);
  WriteDataSPI(0x00);
  WriteCommandSPI(SSD2119_Y_RAM_ADDR_REG);
  WriteDataSPI(0x00);

  // Clear the contents of the display buffer.
  WriteCommandSPI(SSD2119_RAM_DATA_REG);
  for (ulCount = 0; ulCount < (320 * 240); ulCount++)
  {
    WriteDataSPI(0x0000);
  }
  LED_backlight_ON();
}

//*****************************************************************************
//
//! Draws a pixel on the screen.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param lX is the X coordinate of the pixel.
//! \param lY is the Y coordinate of the pixel.
//! \param ulValue is the color of the pixel.
//!
//! This function sets the given pixel to a particular color.  The coordinates
//! of the pixel are assumed to be within the extents of the display.
//!
//! \return None.
//
//*****************************************************************************
static void Kentec320x240x16_SSD2119PixelDraw(void *pvDisplayData, int lX,
    int lY, unsigned int ulValue)
{
  //
  // Set the X address of the display cursor.
  //
  WriteCommandSPI(SSD2119_X_RAM_ADDR_REG);
  WriteDataSPI(MAPPED_X(lX, lY));

  //
  // Set the Y address of the display cursor.
  //
  WriteCommandSPI(SSD2119_Y_RAM_ADDR_REG);
  WriteDataSPI(MAPPED_Y(lX, lY));

  //
  // Write the pixel value.
  //
  WriteCommandSPI(SSD2119_RAM_DATA_REG);
  WriteDataSPI(ulValue);
}

//*****************************************************************************
//
//! Draws a horizontal sequence of pixels on the screen.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param lX is the X coordinate of the first pixel.
//! \param lY is the Y coordinate of the first pixel.
//! \param lX0 is sub-pixel offset within the pixel data, which is valid for 1
//! or 4 bit per pixel formats.
//! \param lCount is the number of pixels to draw.
//! \param lBPP is the number of bits per pixel; must be 1, 4, or 8.
//! \param pucData is a pointer to the pixel data.  For 1 and 4 bit per pixel
//! formats, the most significant bit(s) represent the left-most pixel.
//! \param pucPalette is a pointer to the palette used to draw the pixels.
//!
//! This function draws a horizontal sequence of pixels on the screen, using
//! the supplied palette.  For 1 bit per pixel format, the palette contains
//! pre-translated colors; for 4 and 8 bit per pixel formats, the palette
//! contains 24-bit RGB values that must be translated before being written to
//! the display.
//!
//! \return None.
//
//*****************************************************************************
static void Kentec320x240x16_SSD2119PixelDrawMultiple(void *pvDisplayData,
    int lX, int lY, int lX0, int lCount, int lBPP, const unsigned char *pucData,
    const unsigned char *pucPalette)
{
  unsigned int ulByte;

  //
  // Set the cursor increment to left to right, followed by top to bottom.
  //
  WriteCommandSPI(SSD2119_ENTRY_MODE_REG);
  WriteDataSPI(MAKE_ENTRY_MODE(HORIZ_DIRECTION));

  //
  // Set the starting X address of the display cursor.
  //
  WriteCommandSPI(SSD2119_X_RAM_ADDR_REG);
  WriteDataSPI(MAPPED_X(lX, lY));

  //
  // Set the Y address of the display cursor.
  //
  WriteCommandSPI(SSD2119_Y_RAM_ADDR_REG);
  WriteDataSPI(MAPPED_Y(lX, lY));

  //
  // Write the data RAM write command.
  //
  WriteCommandSPI(SSD2119_RAM_DATA_REG);

  //
  // Determine how to interpret the pixel data based on the number of bits
  // per pixel.
  //
  switch (lBPP)
  {
    //
    // The pixel data is in 1 bit per pixel format.
    //
    case 1:
    {
      //
      // Loop while there are more pixels to draw.
      //
      while (lCount)
      {
        //
        // Get the next byte of image data.
        //
        ulByte = *pucData++;

        //
        // Loop through the pixels in this byte of image data.
        //
        for (; (lX0 < 8) && lCount; lX0++, lCount--)
        {
          //
          // Draw this pixel in the appropriate color.
          //
          WriteDataSPI(
              ((unsigned int *) pucPalette)[(ulByte >> (7 - lX0)) & 1]);
        }

        //
        // Start at the beginning of the next byte of image data.
        //
        lX0 = 0;
      }

      //
      // The image data has been drawn.
      //
      break;
    }

      //
      // The pixel data is in 4 bit per pixel format.
      //
    case 4:
    {
      //
      // Loop while there are more pixels to draw.  "Duff's device" is
      // used to jump into the middle of the loop if the first nibble of
      // the pixel data should not be used.  Duff's device makes use of
      // the fact that a case statement is legal anywhere within a
      // sub-block of a switch statement.  See
      // http://en.wikipedia.org/wiki/Duff's_device for detailed
      // information about Duff's device.
      //
      switch (lX0 & 1)
      {
        case 0:
          while (lCount)
          {
            //
            // Get the upper nibble of the next byte of pixel data
            // and extract the corresponding entry from the
            // palette.
            //
            ulByte = (*pucData >> 4) * 3;
            ulByte = (*(unsigned int *) (pucPalette + ulByte) & 0x00ffffff);

            //
            // Translate this palette entry and write it to the
            // screen.
            //
            WriteDataSPI(DPYCOLORTRANSLATE(ulByte));

            //
            // Decrement the count of pixels to draw.
            //
            lCount--;

            //
            // See if there is another pixel to draw.
            //
            if (lCount)
            {
              case 1:
              //
              // Get the lower nibble of the next byte of pixel
              // data and extract the corresponding entry from
              // the palette.
              //
              ulByte = (*pucData++ & 15) * 3;
              ulByte = (*(unsigned int *) (pucPalette + ulByte) & 0x00ffffff);

              //
              // Translate this palette entry and write it to the
              // screen.
              //
              WriteDataSPI(DPYCOLORTRANSLATE(ulByte));

              //
              // Decrement the count of pixels to draw.
              //
              lCount--;
            }
          }
      }

      //
      // The image data has been drawn.
      //
      break;
    }

      //
      // The pixel data is in 8 bit per pixel format.
      //
    case 8:
    {
      //
      // Loop while there are more pixels to draw.
      //
      while (lCount--)
      {
        //
        // Get the next byte of pixel data and extract the
        // corresponding entry from the palette.
        //
        ulByte = *pucData++ * 3;
        ulByte = *(unsigned int *) (pucPalette + ulByte) & 0x00ffffff;

        //
        // Translate this palette entry and write it to the screen.
        //
        WriteDataSPI(DPYCOLORTRANSLATE(ulByte));
      }

      //
      // The image data has been drawn.
      //
      break;
    }

      //
      // We are being passed data in the display's native format.  Merely
      // write it directly to the display.  This is a special case which is
      // not used by the graphics library but which is helpful to
      // applications which may want to handle, for example, JPEG images.
      //
    case 16:
    {
      unsigned short usByte;

      //
      // Loop while there are more pixels to draw.
      //
      while (lCount--)
      {
        //
        // Get the next byte of pixel data and extract the
        // corresponding entry from the palette.
        //
        usByte = *((unsigned short *) pucData);
        pucData += 2;

        //
        // Translate this palette entry and write it to the screen.
        //
        WriteDataSPI(usByte);
      }
    }
  }
}

//*****************************************************************************
//
//! Draws a horizontal line.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param lX1 is the X coordinate of the start of the line.
//! \param lX2 is the X coordinate of the end of the line.
//! \param lY is the Y coordinate of the line.
//! \param ulValue is the color of the line.
//!
//! This function draws a horizontal line on the display.  The coordinates of
//! the line are assumed to be within the extents of the display.
//!
//! \return None.
//
//*****************************************************************************
static void Kentec320x240x16_SSD2119LineDrawH(void *pvDisplayData, int lX1,
    int lX2, int lY, unsigned int ulValue)
{
  //
  // Set the cursor increment to left to right, followed by top to bottom.
  //
  WriteCommandSPI(SSD2119_ENTRY_MODE_REG);
  WriteDataSPI(MAKE_ENTRY_MODE(HORIZ_DIRECTION));

  //
  // Set the starting X address of the display cursor.
  //
  WriteCommandSPI(SSD2119_X_RAM_ADDR_REG);
  WriteDataSPI(MAPPED_X(lX1, lY));

  //
  // Set the Y address of the display cursor.
  //
  WriteCommandSPI(SSD2119_Y_RAM_ADDR_REG);
  WriteDataSPI(MAPPED_Y(lX1, lY));

  //
  // Write the data RAM write command.
  //
  WriteCommandSPI(SSD2119_RAM_DATA_REG);

  //
  // Loop through the pixels of this horizontal line.
  //
  while (lX1++ <= lX2)
  {
    //
    // Write the pixel value.
    //
    WriteDataSPI(ulValue);
  }
}

//*****************************************************************************
//
//! Draws a vertical line.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param lX is the X coordinate of the line.
//! \param lY1 is the Y coordinate of the start of the line.
//! \param lY2 is the Y coordinate of the end of the line.
//! \param ulValue is the color of the line.
//!
//! This function draws a vertical line on the display.  The coordinates of the
//! line are assumed to be within the extents of the display.
//!
//! \return None.
//
//*****************************************************************************
static void Kentec320x240x16_SSD2119LineDrawV(void *pvDisplayData, int lX,
    int lY1, int lY2, unsigned int ulValue)
{
  //
  // Set the cursor increment to top to bottom, followed by left to right.
  //
  WriteCommandSPI(SSD2119_ENTRY_MODE_REG);
  WriteDataSPI(MAKE_ENTRY_MODE(VERT_DIRECTION));

  //
  // Set the X address of the display cursor.
  //
  WriteCommandSPI(SSD2119_X_RAM_ADDR_REG);
  WriteDataSPI(MAPPED_X(lX, lY1));

  //
  // Set the starting Y address of the display cursor.
  //
  WriteCommandSPI(SSD2119_Y_RAM_ADDR_REG);
  WriteDataSPI(MAPPED_Y(lX, lY1));

  //
  // Write the data RAM write command.
  //
  WriteCommandSPI(SSD2119_RAM_DATA_REG);

  //
  // Loop through the pixels of this vertical line.
  //
  while (lY1++ <= lY2)
  {
    //
    // Write the pixel value.
    //
    WriteDataSPI(ulValue);
  }
}

//*****************************************************************************
//
//! Fills a rectangle.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param pRect is a pointer to the structure describing the rectangle.
//! \param ulValue is the color of the rectangle.
//!
//! This function fills a rectangle on the display.  The coordinates of the
//! rectangle are assumed to be within the extents of the display, and the
//! rectangle specification is fully inclusive (in other words, both sXMin and
//! sXMax are drawn, along with sYMin and sYMax).
//!
//! \return None.
//
//*****************************************************************************
static void Kentec320x240x16_SSD2119RectFill(void *pvDisplayData,
    const tRectangle *pRect, unsigned int ulValue)
{
  int lCount;

  //
  // Write the Y extents of the rectangle.
  //
  WriteCommandSPI(SSD2119_ENTRY_MODE_REG);
  WriteDataSPI(MAKE_ENTRY_MODE(HORIZ_DIRECTION));

  //
  // Write the X extents of the rectangle.
  //
  WriteCommandSPI(SSD2119_H_RAM_START_REG);
#if (defined PORTRAIT) || (defined LANDSCAPE)
  WriteDataSPI(MAPPED_X(pRect->i16XMax, pRect->i16YMax));
#else
  WriteDataSPI(MAPPED_X(pRect->i16XMin, pRect->i16YMin));
#endif

  WriteCommandSPI(SSD2119_H_RAM_END_REG);
#if (defined PORTRAIT) || (defined LANDSCAPE)
  WriteDataSPI(MAPPED_X(pRect->i16XMin, pRect->i16YMin));
#else
  WriteDataSPI(MAPPED_X(pRect->i16XMax, pRect->i16YMax));
#endif

  //
  // Write the Y extents of the rectangle
  //
  WriteCommandSPI(SSD2119_V_RAM_POS_REG);
#if (defined LANDSCAPE_FLIP) || (defined PORTRAIT)
  WriteDataSPI(MAPPED_Y(pRect->i16XMin, pRect->i16YMin) |
      (MAPPED_Y(pRect->i16XMax, pRect->i16YMax) << 8));
#else
  WriteDataSPI(
      MAPPED_Y(pRect->i16XMax, pRect->i16YMax)
          | (MAPPED_Y(pRect->i16XMin, pRect->i16YMin) << 8));
#endif

  //
  // Set the display cursor to the upper left of the rectangle (in application
  // coordinate space).
  //
  WriteCommandSPI(SSD2119_X_RAM_ADDR_REG);
  WriteDataSPI(MAPPED_X(pRect->i16XMin, pRect->i16YMin));

  WriteCommandSPI(SSD2119_Y_RAM_ADDR_REG);
  WriteDataSPI(MAPPED_Y(pRect->i16XMin, pRect->i16YMin));

  //
  // Tell the controller we are about to write data into its RAM.
  //
  WriteCommandSPI(SSD2119_RAM_DATA_REG);

  //
  // Loop through the pixels of this filled rectangle.
  //
  for (lCount = ((pRect->i16XMax - pRect->i16XMin + 1)
      * (pRect->i16YMax - pRect->i16YMin + 1)); lCount >= 0; lCount--)
  {
    //
    // Write the pixel value.
    //
    WriteDataSPI(ulValue);
  }

  //
  // Reset the X extents to the entire screen.
  //
  WriteCommandSPI(SSD2119_H_RAM_START_REG);
  WriteDataSPI(0x0000);
  WriteCommandSPI(SSD2119_H_RAM_END_REG);
  WriteDataSPI(0x013F);

  //
  // Reset the Y extent to the full screen
  //
  WriteCommandSPI(SSD2119_V_RAM_POS_REG);
  WriteDataSPI(0xEF00);
}

//*****************************************************************************
//
//! Translates a 24-bit RGB color to a display driver-specific color.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//! \param ulValue is the 24-bit RGB color.  The least-significant byte is the
//! blue channel, the next byte is the green channel, and the third byte is the
//! red channel.
//!
//! This function translates a 24-bit RGB color into a value that can be
//! written into the display's frame buffer in order to reproduce that color,
//! or the closest possible approximation of that color.
//!
//! \return Returns the display-driver specific color.
//
//*****************************************************************************
static unsigned int Kentec320x240x16_SSD2119ColorTranslate(void *pvDisplayData,
    unsigned int ulValue)
{
  //
  // Translate from a 24-bit RGB color to a 5-6-5 RGB color.
  //
  return (DPYCOLORTRANSLATE(ulValue));
}

//*****************************************************************************
//
//! Flushes any cached drawing operations.
//!
//! \param pvDisplayData is a pointer to the driver-specific data for this
//! display driver.
//!
//! This functions flushes any cached drawing operations to the display.  This
//! is useful when a local frame buffer is used for drawing operations, and the
//! flush would copy the local frame buffer to the display.  For the SSD2119
//! driver, the flush is a no operation.
//!
//! \return None.
//
//*****************************************************************************
static void Kentec320x240x16_SSD2119Flush(void *pvDisplayData)
{
  //
  // There is nothing to be done.
  //
}
//*****************************************************************************
//
//! The display structure that describes the driver for the Kentec
//! K350QVG-V2-F TFT panel with an SSD2119 controller.
//
//*****************************************************************************
const tDisplay g_sKentec320x240x16_SSD2119 =
{ sizeof(tDisplay), 0,
#if defined(PORTRAIT) || defined(PORTRAIT_FLIP)
    240,
    320,
#else
    320, 240,
#endif
    Kentec320x240x16_SSD2119PixelDraw,
    Kentec320x240x16_SSD2119PixelDrawMultiple,
    Kentec320x240x16_SSD2119LineDrawH, Kentec320x240x16_SSD2119LineDrawV,
    Kentec320x240x16_SSD2119RectFill, Kentec320x240x16_SSD2119ColorTranslate,
    Kentec320x240x16_SSD2119Flush };

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************
