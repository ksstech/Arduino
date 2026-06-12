// RS485 support

#pragma once

#include <platform-ow485.h>

// format content control options
#define PO_RUNTIME                 	(1 << __COUNTER__)
#define PO_UPTIME                 	(1 << __COUNTER__)
#define PO_ADDR                   	(1 << __COUNTER__)
#define PO_ONEWIRE                	(1 << __COUNTER__)
#define PO_RLY_LED                	(1 << __COUNTER__)
#define PO_EEPROM                 	(1 << __COUNTER__)
#define PO_USERROW                	(1 << __COUNTER__)
#define PO_FIRMWARE               	(1 << __COUNTER__)
#define PO_CMDBUF                 	(1 << __COUNTER__)
#define PO_SYSSTAT                	(1 << __COUNTER__)

#define PRINT_BUFSIZE				128

void rs485Setup(void);

void serialWrite(const char *);

//void serialVPrintF(const char *, va_list);

void serialPrintF(const char *, ...);

void serialPrintOptions(uint16_t Options);

void serialPrintFOptions(uint16_t Options, const char * fmt, ...);
