// rs485Support.cpp

#include <Arduino.h>
#include <megaTinyCore.h>
#include <EEPROM.h>
#include <USERSIG.h>
#include <platform-ow485.h>

#include "OneWireTag.h"
#include "rs485Support.h"

// Global variables
uint32_t RunTime;
uint8_t UPhr, UPmin, UPsec;
uint16_t UPdays, UPmsec;

extern uint8_t DevID;
extern bool stateRelay;
extern String CmdBuf;

// External variables
extern int __heap_start, *__brkval;
extern OneWireTag owTag;
extern uint32_t countRD0, countRD1, countRD2;

char mapIndexToChar(int8_t Idx) { return ((Idx % 8) == 0) ? ' ' : ((Idx % 4) == 0) ? '|' : ((Idx % 2) == 0) ? '-' : ':'; }

auto readEEPROM  = [](int addr) -> uint8_t { return EEPROM.read(addr);  };

auto readUSERSIG = [](int addr) -> uint8_t { return USERSIG.read(addr); };

void rs485Setup(void) {
  pinMode(pin485EN, OUTPUT);
	digitalWrite(pin485EN, HIGH);
	Serial.begin(19200);
	Serial.flush();
	digitalWrite(pin485EN, LOW);
}

static void updateUpTime(void) {
  uint32_t T = RunTime = millis();
  UPmsec = T % 1000UL;
  T /= 1000;
  UPsec = T % 60UL;
  T /= 60;
  UPmin = T % 60UL;
  T /= 60;
  UPhr = T % 24UL;
  T /= 24;
  UPdays = T;
}

void serialWrite(const char * pMsg) {
  digitalWrite(pin485EN, HIGH);
  Serial.write(pMsg);
  Serial.flush();
  digitalWrite(pin485EN, LOW);
}

static void serialVPrintF(const char * fmt, va_list vaList) {
  char MsgBuf[PRINT_BUFSIZE];
  vsnprintf(MsgBuf, PRINT_BUFSIZE, fmt, vaList);
  serialWrite(MsgBuf);
}

void serialPrintF(const char * fmt, ...) {
  va_list vaList;
  va_start(vaList, fmt);
  serialVPrintF(fmt, vaList);
  va_end(vaList);
}

static void serialPrintArray(const char * Heading, uint8_t (*Func)(int), int16_t Len) {
  serialPrintF(Heading);
  for (int16_t Idx = 0; Idx < Len; ++Idx) {
    if ((Idx & 0x0F) == 0)
      serialPrintF("\n0x%02x: ", Idx);
    serialPrintF("%02X%c", Func(Idx), mapIndexToChar(Idx));
  }
  serialPrintF("\n");
}

void serialPrintOptions(uint16_t Options) {
  if (Options & PO_ONEWIRE) {
  	owTag.printRomInfo(1);
    serialPrintF("C0=%lu C1=%lu C2=%lu\n", countRD0, countRD1, countRD2);
  }
  if (Options & PO_RLY_LED)         serialPrintF("Relay=%d | LED %u / %u -> %u -> %u -> %u\n", stateRelay, pwmInfo.para[0], pwmInfo.para[1], pwmInfo.para[2], pwmInfo.para[3], pwmInfo.para[4]);
  if (Options & PO_FIRMWARE)        serialPrintF("%s %dMHz\n", (DEV_HW_INFO " / " DEV_FW_INFO " / CLK="), F_CPU/1000000);
  if (Options & PO_CMDBUF)          serialPrintF("'%s' L=%d\n", CmdBuf.c_str(), CmdBuf.length());
  if (Options & PO_SYSSTAT) {
    int Mem = (int)(&Mem) - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
    serialPrintF("Vcc=%umV  Temp=%u°C  Free=%uB\n", readSupplyVoltage(), readTemp() - 273, Mem);
  }
  if (Options & PO_EEPROM)          serialPrintArray("EEPROM", readEEPROM, 256);
  if (Options & PO_USERROW)         serialPrintArray("USERROW", readUSERSIG, 32);
  if (Options & PO_RUNTIME)         serialPrintF("%lu ", millis());
  if (Options & PO_UPTIME) {
    updateUpTime();
    serialPrintF("(%lu) %ud %02uh%02um%02u.%03u ", RunTime, UPdays, UPhr, UPmin, UPsec, UPmsec);
  }
  if (Options & PO_ADDR)            serialPrintF("%02x\n", DevID);
}

void serialPrintFOptions(uint16_t Options, const char * fmt, ...) {
	serialPrintOptions(Options);
  va_list vaList;
  va_start(vaList, fmt);
  serialVPrintF(fmt, vaList);
  va_end(vaList);
}
