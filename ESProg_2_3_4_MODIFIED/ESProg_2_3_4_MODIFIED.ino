/*
  Serial Debug   -> port 23
  Firmware upload -> port 24
  ESProg Serial   -> port 25

  Commands:

  Test :    esptool.py --chip esp32 --port socket://192.168.1.234:24 --before default_reset --after hard_reset flash_id
  FW upload:  esptool.py --chip esp32 --port socket://192.168.1.234:24 write_flash  0x10000 C:\Users\TJ\Desktop\tmp\FTP_TMP\Blink.bin
  FORCE RESET : esptool.py --chip esp32 --port socket://192.168.1.234:24 flash_id

  FULL Upload including BOOT
  esptool.py --chip esp32 --port socket://192.168.1.234:24 write_flash 0xe000  C:\Users\TJ\Desktop\tmp\FTP_TMP\boot_app0.bin 0x1000 C:\Users\TJ\Desktop\tmp\FTP_TMP\bootloader_qio_80m.bin 0x10000 C:\Users\TJ\Desktop\tmp\FTP_TMP\Blink.bin 0x8000 C:\Users\TJ\Desktop\tmp\FTP_TMP\Blink.partitions.bin

  WARNING!!
  USE --no-stub option for slow unreliable connections!!
*/

#define VER0

#include <WiFi.h>

//Serial port 1
#define DTR1 23
#define RTS1 32
#define RXD1 34
#define TXD1 33

//Serial port 2
#define DTR2 25
#define RTS2 27
#define RXD2 35
#define TXD2 26

WiFiServer TargetServerCLI(23);     // TELNET port - 23 - Target CLI interface
WiFiServer TargetServerPGM(24);     // TELNET port - 24 - Target PGM interface
WiFiServer HostServerCLI(25);       // TELNET port - 25 - Host CLI interface

WiFiClient TargetClientCLI;         // Target Serial Debug
WiFiClient TargetClientPGM;         // Target Programming
WiFiClient HostClientCLI;           // ESProg local

const char* ssid = "irmacos";
const char* password = "Irm@C0$1";

void setup(void) {
  Serial.begin(115200);
  #if defined(VER0)
    Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
    Serial2.begin(256000, SERIAL_8N1, RXD1, TXD1);
  #elif defined(VER1)
    Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
  #endif

  {  // connect to wireless
    WiFi.begin(ssid, password);
    Serial.print("\nESProg v6.2b Firmware v1.0.0\nConnecting to WiFi '" + String(ssid) + String("' ..."));
  }

  int8_t i;
  for (i = 60; i != 0; i--) {
    if (WiFi.status() == WL_CONNECTED)
      break;
    delay(333);
  }

  if (i == 0) {
    Serial.println("Network connection failed!\nRestarting ESP32!");
    delay(1000);
    ESP.restart();
  }

  Serial.print("Connected.\nLocal IP address: ");
  Serial.println(WiFi.localIP());

  { //start TNET to UART servers
    TargetServerCLI.begin();
    TargetServerCLI.setNoDelay(true);

    TargetServerPGM.begin();
    TargetServerPGM.setNoDelay(true);

    HostServerCLI.begin();
    HostServerCLI.setNoDelay(true);
  #if defined(VER0)
    Serial.print("\nUsage:\nPort 23 -> Target CLI[UART0]\nPort 24 -> Target PGM[UART1].\nPort 25 -> Host CLI[UART2]\n");
  #elif defined(VER1)
    Serial.print("\nUsage:\nPort 23 -> Target CLI[UART0]\nPort 24 -> Target PGM[UART1].\nPort 25 -> Host CLI[UART1]\n");
  #endif
  }

  {
  #if defined(VER0)
    pinMode(DTR1, OUTPUT);
    digitalWrite(DTR2, HIGH);
    pinMode(RTS1, OUTPUT);
    digitalWrite(RTS2, HIGH);
  #elif defined(VER1)
    pinMode(DTR1, OUTPUT);
    digitalWrite(DTR1, HIGH);
    pinMode(RTS1, OUTPUT);
    digitalWrite(RTS1, HIGH);
  #endif
  }
}

int rst = 0;
void Reset_target(void) {
  #if defined(VER0)
    #define DTRx DTR1
    #define RTSx RTS1
  #elif defined(VER1)
    #define DTRx DTR1
    #define RTSx RTS1
  #endif
  bool dtr = (TargetClientPGM && TargetClientPGM.connected());
  digitalWrite(DTRx, !dtr);
  if (dtr == 1) {
    if (rst == 0) {
      digitalWrite(DTRx, LOW);
      delay(50);
      digitalWrite(RTSx, LOW);
      delay(50);
      rst = 1;
      Serial.println("RESET Target");
      Serial.print("Programming target ... ");
      delay(50);
      digitalWrite(RTSx, HIGH);
      delay(50);
    }
  } else {
    if (rst == 1) {
      rst = 0;
      Serial.println("Done.");
      digitalWrite(DTRx, HIGH);
      delay(50);
      digitalWrite(RTSx, LOW);
      delay(50);
      digitalWrite(RTSx, HIGH);
    }
  }
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected! Retrying ...");
    if (HostClientCLI)
      HostClientCLI.stop();
    if (TargetClientCLI)
      TargetClientCLI.stop();
    if (TargetClientPGM)
      TargetClientPGM.stop();
    return;
  }

  //Check if any client requesting access to HOST CLI
  if (HostServerCLI.hasClient()) {
    HostClientCLI = HostServerCLI.accept();
    if (HostClientCLI) {
      Serial.print("New client on port 25: ");
      Serial.println(HostClientCLI.remoteIP());
    } else {
      Serial.println("Host CLI accept failed");
    }
  } else {  //no free/disconnected spot so reject
    WiFiClient rejectClient = HostServerCLI.accept();
    rejectClient.stop();
  }

  if (HostClientCLI && HostClientCLI.connected()) {  //check clients for data
    if (HostClientCLI.available()) {
      while (HostClientCLI.available()) {
        Serial.write(HostClientCLI.read());  // Host CLI => UART0
      }
    }
  } else if (HostClientCLI) {
    HostClientCLI.stop();
  }

  // Handle TARGET CLI
  if (TargetServerCLI.hasClient()) {  //check if there are any new clients
    TargetClientCLI = TargetServerCLI.accept();
    if (TargetClientCLI) {
      Serial.print("New client on port 23: ");
      Serial.println(TargetClientCLI.remoteIP());
    } else {
      Serial.println("Target CLI accept broken");
    }
  } else {  //no free/disconnected spot so reject
    WiFiClient rejectClient = TargetServerCLI.accept();
    rejectClient.stop();
  }

  if (TargetClientCLI && TargetClientCLI.connected()) {  //check clients for data
    if (TargetClientCLI.available()) {
      while (TargetClientCLI.available()) {
        Serial1.write(TargetClientCLI.read());  // Target CLI => UART1
      }
    }
  } else if (TargetClientCLI) {
    TargetClientCLI.stop();
  }

  // Handle TARGET PGM
  if (TargetServerPGM.hasClient()) {  //check if there are any new clients
    if (TargetClientPGM && !TargetClientPGM.connected()) {
      Serial.print("Target client closed on port 25!");
      TargetClientPGM.stop();
    }
    TargetClientPGM = TargetServerPGM.accept();
    if (TargetClientPGM) {
      Serial.print("\nNew client on port 24: ");
      Serial.println(TargetClientPGM.remoteIP());
    } else {
      Serial.println("Target PGM accept broken");
    }
  } else {  //no free/disconnected spot so reject
    WiFiClient rejectClient = TargetServerPGM.accept();
    rejectClient.stop();
  }

  Reset_target();
  if (TargetClientPGM && TargetClientPGM.connected()) {  //check clients for data
    if (TargetClientPGM.available()) {
      Reset_target();
      while (TargetClientPGM.available()) {
      #if defined(VER0)
        Serial2.write(TargetClientPGM.read());  // Target PGM => UART2
      #elif defined(VER1)
        Serial1.write(TargetClientPGM.read());  // Target PGM => UART1
      #endif
      }
    }
  } else if (TargetClientPGM) {
    TargetClientPGM.stop();
    Serial.print("Target client closed on port 25.");
    #if defined(VER0)
      Serial2.flush();
    #endif
    Serial1.flush();
    Serial.flush();
  }

  // CHECK UART ports DATA
  if (Serial.available()) {  // UART0 => TCP:25 (HostServerCLI)
    size_t len = Serial.available();
    char sbuf[len];
    Serial.readBytes(sbuf, len);
    if (HostClientCLI && HostClientCLI.connected()) {
      HostClientCLI.write(sbuf, len);
      delay(1);
    }
  }

  if (Serial1.available()) {  // UART1 => TCP:23 (TargetServerCLI)
    size_t len1 = Serial1.available();
    char sbuf1[len1];
    Serial1.readBytes(sbuf1, len1);
    if (TargetClientCLI && TargetClientCLI.connected()) {
      TargetClientCLI.write(sbuf1, len1);
      delay(1);
    }
  }

#if defined(VER0)
  if (Serial2.available()) {  // UART2 => TCP:24 (TargetServerPGM)
    size_t len2 = Serial2.available();
    char sbuf2[len2];
    Serial2.readBytes(sbuf2, len2);
    if (TargetClientPGM && TargetClientPGM.connected()) {
      TargetClientPGM.write(sbuf2, len2);
      delay(1);
    }
  }
#elif defined(VER1)
  if (Serial1.available()) {  // UART1 => TCP:24 (TargetServerPGM)
    size_t len2 = Serial1.available();
    char sbuf2[len2];
    Serial1.readBytes(sbuf2, len2);
    if (TargetClientPGM && TargetClientPGM.connected()) {
      TargetClientPGM.write(sbuf2, len2);
      delay(1);
    }
  }
#endif
}
