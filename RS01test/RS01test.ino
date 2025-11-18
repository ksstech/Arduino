#include <WiFi.h>
#include <WiFiMulti.h>
#include <SoftwareSerial.h>
#include <Wire.h>
#include <OneWire.h>

#define AC01

#if defined(AC01)
  #include <../libraries/PrivateLibs/ac01.h>
#elif defined(RS01)
  #include <../libraries/PrivateLibs/rs01.h>
#endif

#include <../libraries/PrivateLibs/FormattedMessages.h>
#include <../libraries/PrivateLibs/EspReporting.h>

// ####################################### General macros ##########################################

#define PAUSE       500

// ######################################### RS485 support #########################################

#if defined(RS01)
SoftwareSerial Serial485(RXPin, TXPin);

void writeRS485(const char * msg) {
  digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_TX_PIN_VALUE);
  Serial485.print(msg);
  Serial485.flush();
  digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_RX_PIN_VALUE);
}

void initSerial(void) {
  Serial.begin(115200);
  pinMode(SERIAL_COMMUNICATION_CONTROL_PIN, OUTPUT);
  digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_RX_PIN_VALUE);
  Serial485.begin(57600);
}

int checkSerial(void) {
  int iRV = 0;
  if (Serial.available())           iRV = Serial.available();
  else if (Serial485.available())   iRV = Serial485.available();
  return iRV;
}

int readSerialInt(void) {
  int iRV = 0;
  if (Serial.available())           iRV = Serial.parseInt();
  else if (Serial485.available())   iRV = Serial485.parseInt();
  return iRV;
}

/**
 * @brief Output a message to both UART0 & RS485 UARTx and wait until BOTH buffer flushed
 * @param
 * @return
 */
int sysMessage(const char * msg) {
  int iRV = Serial.print(msg);
  writeRS485(msg);
  Serial.flush();
  return iRV;
}
#endif

// ######################################### LED support ###########################################

#if defined(RS01)
void initLED(void) {
  pinMode(LED0, OUTPUT);
  pinMode(LED1, OUTPUT);  
  delay(10);
  LED_ON(LED0);
  LED_ON(LED1);
}

void LED_ON(byte LED) {
  digitalWrite(LED,HIGH);
  delay(PAUSE/2);
  digitalWrite(LED,LOW);  
}
#endif

// #################################### AnalogIN support ######################################

#if defined(RS01)
const u8_t analogPin[] = { ANA_IN0, ANA_IN0, ANA_IN0, ANA_IN0 };
float analogVal[sizeof(analogPin)] = { 0 };                                 // Used to store last measured values

void initAnalogIN(void) {
  analogReadResolution(11);         // 11 bits = 0 - 2047
  analogSetAttenuation(ADC_11db);   // 0db = 1V, 2.5db = 1.5V, 6db = 2.2V, 11db = x.xV
}

void testAnalogIN(void) {
  sysMessageLn("Loop testing inputs, any key to exit.");
  while(checkSerial() == 0) {
    for (int i = 0; i < sizeof(analogPin); ++i) {
      int av = analogRead(analogPin[i]);                                    // 11bit  3.3V/2047 -> 0.001612115290669
      float vct = ( i < 2) ? (av * 0.0128) : (av * 0.00161211 * 3.81162);   // 12bit  3.3V/4095 -> 0.000805860805861
      // To minimise reporting small changes in value read we allow some fluctuation
      if ((abs(vct - analogVal[i]) / analogVal[i]) > (PERC_FLUCTUATION / 100.0))
        fmtMessageLn(" ADC %d Value = %d - %f%s", i, av, vct, (i < 2) ? "mA" : "V");
      analogVal[i] = vct;                                                   // Update last measured value
    }
  }
}
#endif

// ################################### Default Serial support ######################################

#if not defined(RS01)
void initSerial(void) { Serial.begin(115200); }

int checkSerial(void) { return Serial.available() ? Serial.available() : 0; }

int readSerialInt(void) { return Serial.available() ? Serial.parseInt() : 0; }

/**
 * @brief   Default system message API if no override specified for specific hardware
 * @param   msg - message to be displayed
 * @return  number of characters output
 */
int sysMessage(const char * msg) {
  int iRV = Serial.print(msg);
  Serial.flush();
  return iRV;
}
#endif

// ################################ [un]formatted output support ###################################

/**
 * @brief   Output message with trailing newline using default or custom sysMessage API
 * @param   msg - message to be displayed
 * @return  number of characters output including CR and/or LF
 * @note
 */
int sysMessageLn(const char * msg) {
  int iRV = sysMessage(msg);
  iRV += sysMessage(strNL);
  return iRV;
}

// ###################################### I2C BUS support ##########################################

void scanI2Cbus(void) {
  int nDevices = 0;
  for(int address = 1; address < 127; address++ ) {
    // Uses return value of the Write.endTransmisstion
    // to see if device did acknowledge the address
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) {
      fmtMessage("   I2C device found at address 0x%02X  !" strNL, address);
      nDevices++;
    } else if (error == 4) {
      fmtMessage("   Unknown error at address 0x%02X" strNL, address);
    }    
  }
  sysMessageLn((nDevices == 0) ? "   No I2C devices found" :"  done.");
}

// ##################################### DS2482 support ############################################

const u8_t OWkeys[][8] = {
  [0] = { 0x01, 0x4E, 0xBB, 0x2A, 0x04, 0x00, 0x00, 0x91 },
};

OneWire oneWire;

void testOneWire(void) {
  sysMessage("\nChecking for 1-Wire Master : ");
  if (oneWire.checkPresence()) {
    sysMessageLn("DS2482-800 present!");   
    oneWire.deviceReset();
    oneWire.setStrongPullup();

    for (int iCH = 0; iCH<8; iCH++ ) {
      fmtMessageLn("\tActive 1-Wire BUS : %d\tChecking for devices ...", iCH);
      oneWire.selectChannel(iCH);
      if (oneWire.wireReset()) {
        uint8_t currAddress[8];
        int keynum;
        while (oneWire.wireSearch(currAddress)) {
          #if defined(RS01)
            digitalWrite(LED1, HIGH);
          #endif
          // Check if tag is known, defined in table above...
          for (int key = 0; key < (sizeof(OWkeys) / sizeof(currAddress)); ++key) {
            if (memcmp(OWkeys[key], currAddress, sizeof(currAddress)) == 0)
              keynum = key;
            else
             keynum = -1;
          }
          // report tag details
          fmtMessage1Wire("\tFound ", currAddress);
          fmtMessage("  %d (%s)" strNL, keynum, (keynum > -1) ? "defined" : "unknown");
          delay(PAUSE);
          #if defined(RS01)
            digitalWrite(LED1, LOW);
          #endif
        }
        oneWire.wireResetSearch();
      } else {
        sysMessageLn("\tNo devices on 1-Wire bus");
      }
      //yield();
    }
  } else {
    sysMessageLn("No DS2482-800 present");
  }
  sysMessageLn("");
  yield();
  #if defined(RS01)
    digitalWrite(LED0, HIGH);  
    delay(2000);
    digitalWrite(LED0, LOW);   
    delay(2000);
  #endif
}

// ##################################### PCA9555 support ######################################

#define PCA9555 0x20                // 0x20 is default address
// At reset, PCA9555 ports are inputs with a high value resistor pull-ups to VDD (100k)
// If relays turning on during power up are a problem. Add a pull down resistor to each relay transistor base.
#define IN_P0 0x00                  // Read Input port0
#define IN_P1 0x01                  // Read Input port1
#define OUT_P0 0x02                 // Write Output port0
#define OUT_P1 0x03                 // Write Output port1
#define INV_P0 0x04                 // Input Port Polarity Inversion port0 if 0b11111111 is written input polarity is inverted
#define INV_P1 0x05                 // Input Polarity Inversion port1, upper 8 bits
#define CONFIG_P0 0x06              // Configuration port0 configures the direction of the I/O pins 0 is output 1 is input
#define CONFIG_P1 0x07              // Configuration port1, upper 8 bits

void initPCA9555(void) {
  writePCA9555(CONFIG_P0, 0b00000000); //defines all pins on Port0 are outputs
  writePCA9555(CONFIG_P1, 0b00000000); //defines all pins on Port1 are outputs  
  writePCA9555(OUT_P0, 0b00000000); //clears all relays
  writePCA9555(OUT_P1, 0b00000000); //clears all relays
}

void writePCA9555(int command, int value) {
  Wire.beginTransmission(PCA9555);
  Wire.write(command),Wire.write(value);
  Wire.endTransmission();
}

void testPCA9555(void) {
  u8_t Mask = 1;
  for(int i = 0; i < 8; ++i) {
    writePCA9555(OUT_P0, Mask);
    writePCA9555(OUT_P1, Mask);
    Mask <<= 1;
    fmtMessageLn("  Relay X%d - ON", i);
    delay(PAUSE);
    #if defined(RS01)
      LED_ON(LED0);
    #endif
  }
  writePCA9555(OUT_P0, 0);
  writePCA9555(OUT_P1, 0);
  sysMessageLn("  Relays - OFF");
  delay(PAUSE);  
}

// ##################################### OptoIN support #######################################

const u8_t optoInPin[] = { OPTO_IN0, OPTO_IN1, OPTO_IN2, OPTO_IN3, GPIO_IN0 };       // includes button as input

void initOptoIn(void) {
  for(int i = 0; i < sizeof(optoInPin); ++i) {
    pinMode(optoInPin[i], INPUT);
  }
}

void testOptoIN(void) {
  sysMessageLn("Loop testing inputs, any key to exit.");
  while(checkSerial() == 0) {
    for(int i = 0; i < sizeof(optoInPin); ++i) {
      if (digitalRead(optoInPin[i]) == LOW) {
        #if defined(RS01)
          digitalWrite(LED1, HIGH);
        #endif
        fmtMessageLn("   Input #%d active", i);
        delay(PAUSE);
        #if defined(RS01)
          digitalWrite(LED1, LOW);
        #endif
      }
      yield();
    }
  }
}

// ##################################### AP/Wifi support ######################################

WiFiMulti WiFiMulti;

void connectWifiAP(void) {
  int cn = 0;
  String ssid;
  sysMessageLn("WIFI Client Test" strNL "Enter data in this style : SSID,Passwd");
  while (ssid != String("exit")){    
    if (Serial.available()) {
      // First read the string until the ',' in your example
      // "ssid,passwd" this would read the "ssid" as a String
      ssid = Serial.readStringUntil(',');
      // We now have "passwd\n" left in the Serial buffer, so we read that.
      // The end of line character '\n' or '\r\n' is sent over the serial
      // terminal to signify the end of line, so we can read the
      // remaining buffer until we find that.
      String passwd = Serial.readStringUntil('\n');

      if (ssid != String("exit")) {
        fmtMessageStr("    SSID : %s" strNL, ssid);
        fmtMessageStr("  Passwd : %s" strNL, passwd);

        // We start by connecting to a WiFi network
        WiFiMulti.addAP(ssid.c_str(), passwd.c_str());
        sysMessage("Waiting for WiFi : ");
        while(WiFiMulti.run() != WL_CONNECTED) {
          sysMessage(".");
          delay(100);
          cn++;
          if (cn == 12) {
            sysMessage(strNL "  Connection ERROR! Check your setup and credentials"); 
            break;
          }            
        }
        if (cn < 12)
          fmtMessage("Connected with IP address: %s", WiFi.localIP().toString().c_str());
        sysMessage(strNL "Type 'exit' to return.");
      }
    }
  }
  sysMessageLn(strNL "WIFI test done. Exiting...");
}

void scanWifiAPs(void) {
  sysMessage("   WIFI scan start ...");
  int n = WiFi.scanNetworks();                      // return the number of networks found
  fmtMessage(" Done" strNL "   %d networks found" strNL, n);
  if (n) {
    for (int i = 0; i < n; ++i) {                   // Print SSID and RSSI for each network found
      char * pcSSID = pcStringToArray(WiFi.SSID(i));
      fmtMessageLn("   %d: %s %lddBM %s", i+1, pcSSID, WiFi.RSSI(i), (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? " " : "*");
      free(pcSSID);
    }
    connectWifiAP();
  }
}

// ################################## Standard setup & loop ###################################

void setup(void) {
  initSerial();
  initPCA9555();
  initOptoIn();
  #if defined(RS01)
    initLED();
    initAnalogIN();
  #endif
  delay(PAUSE/2);
  WiFi.mode(WIFI_STA);
  sysMessageLn(strNL strNL "Board: " HW_TYPE strNL "Firmware v" FW_VERSION);
  reportSDK();
  reportMCU();
  reportMEM();
  printMenu();
}

void loop(void) {
  int val = readSerialInt();
  if (val > 0) {
    switch(val) {
      case 1:
        sysMessageLn("->Option - I2C Scanner");
        scanI2Cbus();
        break;
      case 2:
        sysMessageLn("->Option - Relay Drivers test");
        testPCA9555();
        break;
      case 3:
        sysMessageLn("->Option - INPUT Port Test");
        testOptoIN();
        break;
      case 4:
        sysMessageLn("->Option - 1-Wire test");
        testOneWire();
        break;
      case 5:
        sysMessageLn("->Option - WIFI Scan/Client");
        scanWifiAPs();
        break;              
    #if defined(RS01)
      case 6:
        sysMessageLn("->Option - ADC channels");
        testAnalogIN();
        break;
    #endif
      default:
        sysMessageLn("Please choose a valid option");
        break;
    }
    printMenu();
  }
}

void printMenu(void) {
  sysMessageLn("");
  sysMessageLn("Available Test Options:");
  sysMessageLn("\t 1. I2C Scanner");
  sysMessageLn("\t 2. Relays Driver");
  sysMessageLn("\t 3. Input Port");
  sysMessageLn("\t 4. 1-Wire Bus MASTER");
  sysMessageLn("\t 5. WIFI Scanner/Client");
  #if defined(RS01)
    sysMessageLn("\t 6. ADC Channels");
  #endif
}

/* Application Changelog
Date:     Vers:   Comments:
2025091x  1.1     ......

20250917  1.0     Initial release built with Core v3.3.0 and SDK v5.5.1
                  Uses SoftSerial (v???) and DS2482_OneWire (v) libraries.
                  Requires "EspReporting.h" and "FormattedMessages.h" source headers
20251007  1.1     Added initial support for AC01

*/
