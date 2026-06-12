// ow485-tests.h

void testPins(void) {
  // ATtiny3224 Physical      10      11        12        13      2         3       4         5       ??        ??      ??        ??
  // Arduino "number"         11      8         9         10      0         1       2         3
  // OW485 function           UPDI  Unused    Relay     Unused   1Wire     LED    Unused    Unused   REDE     Unused    TX        RX     
  const uint8_t Pins[8] = { PIN_PA0, PIN_PA1, PIN_PA2, PIN_PA3, PIN_PA4, PIN_PA5, PIN_PA6, PIN_PA7, PIN_PB0, PIN_PB1, PIN_PB2, PIN_PB3 };
  uint8_t Key;
  bool bStat[8];
  char MsgBuf[64];
  serialPrintOptions("Pin init...", 0);
  pinMode(PIN_PA0, OUTPUT);
  pinMode(PIN_PA1, OUTPUT);
  pinMode(PIN_PA2, OUTPUT);
  pinMode(PIN_PA3, OUTPUT);
  pinMode(PIN_PA4, OUTPUT);
  pinMode(PIN_PA5, OUTPUT);
  pinMode(PIN_PA6, OUTPUT);
  pinMode(PIN_PA7, OUTPUT);
  serialPrintOptions("Pin test starting", 0);
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
      rs485SetMode(1);
      Serial.println(MsgBuf);
      rs485SetMode(0);
    }
  } while (Key != 0x1B);
}

