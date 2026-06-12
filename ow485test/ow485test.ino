// ow485-tests.h

#include <megaTinyCore.h>
//#include <EEPROM.h>
//#include <USERSIG.h>

#define pin485RX      PIN_PB3
#define pin485TX      PIN_PB2
#define pin485REDE    PIN_PB0

void serialPrint(const char * Msg) {
    digitalWrite(pin485REDE, HIGH);
    Serial.println(Msg);
    Serial.flush();
    digitalWrite(pin485REDE, LOW);
}

void testPins(void) {
  // ATtiny3224 Physical      10        11        12        13      2         3       4         5
  // Arduino "number"         11        8         9         10      0         1       2         3
  // OW485 function           UPDI   Unused     Relay     Unused   1Wire     LED    Unused    Unused   REDE     Unused    TX        RX
  const uint8_t Pins[8] = { PIN_PA0, PIN_PA1, PIN_PA2, PIN_PA3, PIN_PA4, PIN_PA5, PIN_PA6, PIN_PA7 };
  //const uint8_t Pins[12] = { PIN_PA0, PIN_PA1, PIN_PA2, PIN_PA3, PIN_PA4, PIN_PA5, PIN_PA6, PIN_PA7, PIN_PB0, PIN_PB1, PIN_PB2, PIN_PB3 };
  uint8_t Key;
  bool bStat[8];
  char MsgBuf[64];
  serialPrint("Pin init...");
  pinMode(PIN_PA0, OUTPUT);
  pinMode(PIN_PA1, OUTPUT);
  pinMode(PIN_PA2, OUTPUT);
  pinMode(PIN_PA3, OUTPUT);
  pinMode(PIN_PA4, OUTPUT);
  pinMode(PIN_PA5, OUTPUT);
  pinMode(PIN_PA6, OUTPUT);
  pinMode(PIN_PA7, OUTPUT);
  serialPrint("Pin test starting");
  do {
    if (Serial.available()) {
      Key = (uint8_t) Serial.read();
      if (Key >= '0' && Key <= '7') {
        Key -= '0';
        bStat[Key] = bStat[Key] ? 0 : 1; 
        digitalWrite(Pins[Key], bStat[Key]);
        snprintf(MsgBuf, sizeof(MsgBuf), "Toggle A%d / P%d = %d", Key, Pins[Key], bStat[Key]);
      } else if (Key == 0x1B) {
        snprintf(MsgBuf, sizeof(MsgBuf), "Done testing...");
      } else {
        snprintf(MsgBuf, sizeof(MsgBuf), "Invalid key!");
      }
      serialPrint(MsgBuf);
    }
  } while (Key != 0x1B);
}

void setup() {
  Serial.begin(19200);
  Serial.flush();
  pinMode(pin485REDE, OUTPUT);          // Init RS485 REDE
}

void loop(void) {
  testPins();
}