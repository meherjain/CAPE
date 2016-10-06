//*****************************************************************************
// Compact Analyser for Power Equipment (CAPE)
// es_enet_datalogger.h
// Header file for es_enet_datalogger.c
//
// Author
// Meher Jain
// Vivek Sankaranarayanan
//*****************************************************************************

#ifndef SRC_ENET_ES_ENET_DATALOGGER_H_
#define SRC_ENET_ES_ENET_DATALOGGER_H_

/******************** Header files  ******************************************/
#include "utils/lwiplib.h"
/*****************************************************************************/

//*****************************************************************************
//
// Defines for setting up the system clock.
//
//*****************************************************************************
#define SYSTICKHZ               100
#define SYSTICKMS               (1000 / SYSTICKHZ)

//*****************************************************************************
//
// Interrupt priority definitions.  The top 3 bits of these values are
// significant with lower values indicating higher priority interrupts.
//
//*****************************************************************************
#define SYSTICK_INT_PRIORITY    0x80
#define ETHERNET_INT_PRIORITY   0xC0

#define ENET_EVENT_COUNTER_ELAPSED  65
/*****************************************************************************/
extern uint8_t nw_update_timer;

/****************** Global variables *****************************************/
extern void enet_init();
extern void enet_server_connect();
extern void enet_metrics_log();

#endif /* SRC_ENET_ES_ENET_DATALOGGER_H_ */
