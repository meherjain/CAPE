//*****************************************************************************
// Compact Analyser for Power Equipment (CAPE)
// es_enet_datalogger.c
// Ethernet datalogger. Uses LwIP stack
// LwIP stack: http://savannah.nongnu.org/projects/lwip/
// LwIP docs: http://lwip.wikia.com/wiki/LwIP_Application_Developers_Manual
//
// Author
// Meher Jain
// Vivek Sankaranarayanan
//*****************************************************************************

/******************** Header files  ******************************************/
#include <es_enet_datalogger.h>
#include "device.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include <string.h>
//#include "driverlib/rom_map.h"
/*****************************************************************************/

#define FAULT_SYSTICK           15          // System Tick

extern uint32_t sys_clk;
uint32_t g_ui32IPAddress;
uint8_t nw_update_timer = 0;

extern char display_update_buffer1[10];
extern char display_update_buffer2[10];
extern char display_update_buffer3[10];
extern char display_update_buffer4[10];
extern char display_update_buffer5[10];
extern char display_update_buffer6[10];
extern char display_update_buffer7[10];
extern char display_update_buffer8[10];
extern char display_update_buffer9[10];
extern char display_update_buffer10[10];
extern char display_update_buffer11[10];
volatile bool bPreviousDataTransmitted = true;
volatile bool bConnectedToServer = false;

struct tcp_pcb *pcb;
/****************************** User Callbacks ********************************/
/* desc: Callback for handling tcp connect error
 * args: Dummy argument, error
 * ret : none
 */
void OnTcpError(void *arg, err_t err)
{
  UARTprintf("Error:%d", err);
  while (1)
  {
  };
}

/* desc: On a successful transmit
 * args: Dummy argument, pointer to protocol control block, lenth of bytes transmitted
 * ret : none
 */
err_t OnSent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
  bPreviousDataTransmitted = 1;
  UARTprintf("Booyah!!\n\r");
  return ERR_OK;
}

/* desc: On a successful connect
 * args: Dummy argument, pointer to protocol control block, error code
 * ret : none
 */
err_t OnClientConnected(void *arg, struct tcp_pcb *pcb, err_t err)
{
  LWIP_UNUSED_ARG(arg);
  if (err == ERR_OK)
  {
    UARTprintf("Connection Success\n\r");
    tcp_sent(pcb, OnSent);
    bConnectedToServer = true;
  }
  return err;
}

/****************************** End of Callbacks *******************************/

/* desc: Log AC metrics over ethernet
 * args: none
 * ret : none
 */
void enet_metrics_log()
{

  char enet_update_buffer[200] =
  { 0 };

//  UARTprintf("%s\n\r",enet_update_buffer);

  if (bPreviousDataTransmitted)
  {
    sprintf(enet_update_buffer, "%s$%s$%s$%s$%s$%s$%s$%s$%s$%s$%s$\n\r",
        display_update_buffer1, display_update_buffer2, display_update_buffer3,
        display_update_buffer4, display_update_buffer5, display_update_buffer6,
        display_update_buffer7, display_update_buffer8, display_update_buffer9,
        display_update_buffer10, display_update_buffer11);
    bPreviousDataTransmitted = 0;

    // TODO: Check buffer remaining using tcp_sndbuf
    tcp_write(pcb, (const void *) enet_update_buffer,
        strlen(enet_update_buffer), 0);
    tcp_output(pcb);
  }
  else
  {
    UARTprintf("Previous data still not transmitted\n\r");
  }
}

/* desc: Attempt server connection
 * args: none
 * ret : none
 */
void enet_server_connect()
{
  ip_addr_t remote_addr;

  pcb = tcp_new();
  tcp_arg(pcb, pcb);
  tcp_err(pcb, OnTcpError);

  /* Connect */
  IP4_ADDR(&remote_addr, 192, 168, 1, 2);
  tcp_connect(pcb, &remote_addr, 5000, OnClientConnected);
  while (!bConnectedToServer)
  {
  }

  /* Send test data */
  tcp_write(pcb, "$\n\r", strlen("$\n\r"), 0);
  tcp_output(pcb);
}

/* desc: Initialise Ethernet environment
 * args: none
 * ret : none
 */
void enet_init()
{
  /* from enet_lwip prj */
  uint32_t ui32User0, ui32User1;
  uint8_t pui8MACArray[8];

  //
  // Configure the hardware MAC address for Ethernet Controller filtering of
  // incoming packets.  The MAC address will be stored in the non-volatile
  // USER0 and USER1 registers.
  //
  ROM_FlashUserGet(&ui32User0, &ui32User1);
  if ((ui32User0 == 0xffffffff) || (ui32User1 == 0xffffffff))
  {
    //
    // We should never get here.  This is an error if the MAC address has
    // not been programmed into the device.  Exit the program.
    // Let the user know there is no MAC address
    //
    UARTprintf("No MAC programmed!\n");
    while (1)
    {
    }
  }
  //
  // Convert the 24/24 split MAC address from NV ram into a 32/16 split MAC
  // address needed to program the hardware registers, then program the MAC
  // address into the Ethernet Controller registers.
  //
  pui8MACArray[0] = ((ui32User0 >> 0) & 0xff);
  pui8MACArray[1] = ((ui32User0 >> 8) & 0xff);
  pui8MACArray[2] = ((ui32User0 >> 16) & 0xff);
  pui8MACArray[3] = ((ui32User1 >> 0) & 0xff);
  pui8MACArray[4] = ((ui32User1 >> 8) & 0xff);
  pui8MACArray[5] = ((ui32User1 >> 16) & 0xff);

  //
  // Initialize the lwIP library, using DHCP.
  //
  uint32_t ipaddr;
  uint32_t netmask;
  uint32_t gateway;
  ipaddr = (192 << 24) | (168 << 16) | (1 << 8) | (3);
  netmask = (255 << 24) | (255 << 16) | (255 << 8) | (0);
  gateway = (192 << 24) | (168 << 16) | (1 << 8) | (1);
  lwIPInit(sys_clk, pui8MACArray, ipaddr, netmask, gateway,
  IPADDR_USE_STATIC);

  //
  // Set the interrupt priorities.  We set the SysTick interrupt to a higher
  // priority than the Ethernet interrupt to ensure that the file system
  // tick is processed if SysTick occurs while the Ethernet handler is being
  // processed.  This is very likely since all the TCP/IP and HTTP work is
  // done in the context of the Ethernet interrupt.
  //
  ROM_IntPrioritySet(INT_EMAC0, ETHERNET_INT_PRIORITY);
  ROM_IntPrioritySet(FAULT_SYSTICK, SYSTICK_INT_PRIORITY);
}

//*****************************************************************************
//
// Display an lwIP type IP Address.
//
//*****************************************************************************
void DisplayIPAddress(uint32_t ui32Addr)
{
  char pcBuf[16];

  //
  // Convert the IP Address into a string.
  //
  usprintf(pcBuf, "%d.%d.%d.%d", ui32Addr & 0xff, (ui32Addr >> 8) & 0xff,
      (ui32Addr >> 16) & 0xff, (ui32Addr >> 24) & 0xff);

  //
  // Display the string.
  //
  UARTprintf(pcBuf);
}

//*****************************************************************************
//
// Required by lwIP library to support any host-related timer functions.
//
//*****************************************************************************
void lwIPHostTimerHandler(void)
{
  uint32_t ui32Idx, ui32NewIPAddress;

  //
  // Get the current IP address.
  //
  ui32NewIPAddress = lwIPLocalIPAddrGet();

  //
  // See if the IP address has changed.
  //
  if (ui32NewIPAddress != g_ui32IPAddress)
  {
    //
    // See if there is an IP address assigned.
    //
    if (ui32NewIPAddress == 0xffffffff)
    {
      //
      // Indicate that there is no link.
      //
      UARTprintf("Waiting for link.\n");
    }
    else if (ui32NewIPAddress == 0)
    {
      //
      // There is no IP address, so indicate that the DHCP process is
      // running.
      //
      UARTprintf("Waiting for IP address.\n");
    }
    else
    {
      //
      // Display the new IP address.
      //
      UARTprintf("IP Address: ");
      DisplayIPAddress(ui32NewIPAddress);
      UARTprintf("\nOpen a browser and enter the IP address.\n");
    }

    //
    // Save the new IP address.
    //
    g_ui32IPAddress = ui32NewIPAddress;

    //
    // Turn GPIO off.
    //
//        MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1, ~GPIO_PIN_1);
  }

  //
  // If there is not an IP address.
  //
  if ((ui32NewIPAddress == 0) || (ui32NewIPAddress == 0xffffffff))
  {
    //
    // Loop through the LED animation.
    //

    for (ui32Idx = 1; ui32Idx < 17; ui32Idx++)
    {

      //
      // Toggle the GPIO
      //
      /*            MAP_GPIOPinWrite(GPIO_PORTN_BASE, GPIO_PIN_1,
       (MAP_GPIOPinRead(GPIO_PORTN_BASE, GPIO_PIN_1) ^
       GPIO_PIN_1));*/

      SysCtlDelay(sys_clk / (ui32Idx << 1));
    }
  }
}
