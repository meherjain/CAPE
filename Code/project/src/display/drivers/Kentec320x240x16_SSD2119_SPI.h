//*****************************************************************************
//
// Kentec320x240x16_ssd2119_SPI.h - Display driver for the Kentec
//                                  BOOSTXL-K350QVG-S1 TFT display with an SSD2119
//                                  controller.  This version assumes an SPI interface
//                                  between the micro and display.
//
//*****************************************************************************
#ifndef __KENTEC320X240X16_SSD2119_SPI_H__
#define __KENTEC320X240X16_SSD2119_SPI_H__

//	LCD_SCLK: PB4/SSI2Clk
//	LCD_SDA:	PB7/SSI2Tx

//#define SPI4	//8-bit 4-wire SPI mode (SCLK, SDA, SCS, SDC)
//#define SPI3	//9-bit 3-wire SPI mode (SCLK, SDA, SCS)
extern void LED_backlight_ON(void);
extern void LED_backlight_OFF(void);
extern void Kentec320x240x16_SSD2119Init(void);
extern const tDisplay g_sKentec320x240x16_SSD2119;

#endif // __KENTEC320X240X16_SSD2119_SPI_H__

