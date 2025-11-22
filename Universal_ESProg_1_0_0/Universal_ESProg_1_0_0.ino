/* ESProg Demo software
   
    Copyright (c) 2017-2027 Next Evolution SRL. All rights reserved.
    Copyright (c) 2025 Andre M. Maree / KSS Technologies (pty) Ltd. All rights reserved

  This file is part of the ESProg demo software package.

  This fostware is distributed as e demo software for ESPProg v6.2b board,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 

  
  Serial Debug   -> port 23
  Firmware upload -> port 24
  ESProg Serial   -> port 25

  WARNING!!
  USE --no-stub option for slow unreliable connections!!

  -----------------------------------------
  C:\ESP32\esptool>esptool.py --chip esp32 --port socket://192.168.1.234:24 write_flash 0xe000  C:\Users\TJ\Desktop\tmp\FTP_TMP\boot_app0.bin 0x1000 C:\Users\TJ\Desktop\tmp\FTP_TMP\bootloader_qio_80m.bin 0x10000 C:\Users\TJ\Desktop\tmp\FTP_TMP\Blink.bin 0x8000 C:\Users\TJ\Desktop\tmp\FTP_TMP\Blink.partitions.bin
  esptool.py v3.0-dev
  Serial port socket://192.168.1.234:24
  Connecting.....
  Chip is ESP32-D0WDQ6 (revision 1)
  Features: WiFi, BT, Dual Core, Coding Scheme None
  WARNING: Detected crystal freq 18.43MHz is quite different to normalized freq 26MHz. Unsupported crystal in use?
  Crystal is 26MHz
  MAC: 30:ae:a4:32:c7:10
  Uploading stub...
  Running stub...
  Stub running...
  Configuring flash size...
  Auto-detected Flash size: 4MB
  Compressed 8192 bytes to 47...
  Wrote 8192 bytes (47 compressed) at 0x0000e000 in 0.0 seconds (effective 2048.0 kbit/s)...
  Hash of data verified.
  Compressed 17392 bytes to 11186...
  Wrote 17392 bytes (11186 compressed) at 0x00001000 in 0.5 seconds (effective 294.8 kbit/s)...
  Hash of data verified.
  Compressed 207824 bytes to 105394...
  Wrote 207824 bytes (105394 compressed) at 0x00010000 in 4.3 seconds (effective 382.3 kbit/s)...
  Hash of data verified.
  Compressed 3072 bytes to 128...
  Wrote 3072 bytes (128 compressed) at 0x00008000 in 0.0 seconds (effective 630.2 kbit/s)...
  Hash of data verified.
  
  Leaving...
  Hard resetting via RTS pin...
  
  C:\ESP32\esptool>
  -------------------------------------
*/

#include <WiFi.h>
#include <Commander.h>              // Add Commander by Bill Bigge v4.2.3
#include <../libraries/PrivateLibs/StaticBuffer.h>
#include <../libraries/PrivateLibs/FormattedMessages.h>

// UART0
#define UART0_BAUD  115200
#define RXD0  3
#define TXD0  1
#define HOST_CLI_PORT   25    // TELNET port -> UART0 (Host CLI) interface

// UART1
#define UART1_BAUD  115200
#define RXD1  34 
#define TXD1  33
#define RTS1  32
#define DTR1  23
#define TARGET_CLI_PORT 23    // TELNET port -> UART1 (Target CLI) interface

// UART2
#define UART2_BAUD  256000
#define RXD2  35 
#define TXD2  26
#define RTS2  27
#define DTR2  25
#define TARGET_PGM_PORT 24    // TELNET port -> UART2 (Target PGM) interface

#define WIFI_SSID "irmacos"
#define WIFI_PSWD "Irm@C0$1"
#define USE_UART1_2_SAME_PINS

#ifdef USE_UART1_2_SAME_PINS
  #define DTRx  DTR1
  #define RTSx  RTS1    
#else
  #define DTRx  DTR2
  #define RTSx  RTS2
#endif

Commander MyCmdr;
CharBuffer MyBuf;

WiFiServer ServerHostCLI(HOST_CLI_PORT);
WiFiClient ClientHostCLI;

WiFiServer ServerTargetCLI(TARGET_CLI_PORT);
WiFiClient ClientTargetCLI;

WiFiServer ServerTargetPGM(TARGET_PGM_PORT);
WiFiClient ClientTargetPGM;

// ###################################### Commander handlers #######################################

bool restartHandler(Commander &Cmdr) {
  Cmdr.println("... Restarting ESP32!");
  delay(1000);
  ESP.restart();
  return 0;
}

bool resetHandler(Commander &Cmdr) {
  static int rst = 0;
  bool dtr = (ClientTargetPGM && ClientTargetPGM.connected());
    digitalWrite(DTRx, !dtr);
  if (dtr == 1 ) {
    if (rst == 0) {
        digitalWrite(DTRx, LOW);
        delay(50);
        digitalWrite(RTSx, LOW);
        delay(50);
        rst = 1;
        Cmdr.println("RESET Target" strNL "Programming target ... ");
        delay(50);
        digitalWrite(RTSx, HIGH);
        delay(50);
    }
  } else if (rst == 1) {
    rst = 0;
    Cmdr.println("Done.");
    digitalWrite(DTRx, HIGH);
    delay(50);
    digitalWrite(RTSx, LOW);
    delay(50);
    digitalWrite(RTSx, HIGH);
  }
  return 0;
}

bool reportHandler(Commander &Cmdr) {
  Cmdr.println("#####################" strNL "ESProg adapter v1.0.0" strNL "#####################");
  Cmdr.println("# Prompt=" + String(Cmdr.commandPrompt()));
  Cmdr.println("# Echo=" + String(Cmdr.echo()));
  Cmdr.println("# CmndProc=" + String(Cmdr.commandProcessor()));
  Cmdr.println("# errorMessages=" + String(Cmdr.errorMessages()));
  Cmdr.println("# showHelp=" + String(Cmdr.showHelp()));
  Cmdr.println("###############################################");
  Cmdr.println("#\tUART0: " + String(Serial0.baudRate()));
  Cmdr.println("#\tUART1: " + String(Serial1.baudRate()));
  Cmdr.println("#\tUART2: " + String(Serial2.baudRate()));
  Cmdr.println("###############################################");
  Cmdr.println("# Usage" strNL "\tPort 23 -> Target CLI." strNL "\tPort 24 -> Target PGM." strNL "\tPort 25 -> Host CLI.");
  return 0;
}

bool connectHandler(Commander &Cmdr, String ssid, String pswd) {
  Cmdr.print("Connecting to WiFi with '" + String(ssid) + String("' & '") + String(pswd) + String("' ..."));
  WiFi.begin(ssid, pswd);
  int8_t i = 0;
  do {
    if (++i == 60) {
      Cmdr.println(strNL "Network connection failed!");
      restartHandler(Cmdr);
    }
    delay(333);
  } while (WiFi.status() != WL_CONNECTED);
  Cmdr.print(" Connected!" strNL "Local IP address: ");
  Cmdr.println(WiFi.localIP());
  return 0;
}

bool accessHandler(Commander &Cmdr) {
  String mySSID = "";
  String myPSWD = "";
  if ((Cmdr.countItems() == 2) && Cmdr.getString(mySSID) && Cmdr.getString(myPSWD)) {
      connectHandler(Cmdr, mySSID, myPSWD);
  } else {
    Cmdr.println("Must have <ssid> and <password> parameters");
  }
  return 0;
}

bool flashbaudHandler(Commander &Cmdr) {
  int myBaudrate = 0;
  if (Cmdr.countItems() == 1 && Cmdr.getInt(myBaudrate) && myBaudrate >= 9600 && myBaudrate <= 921600 && (myBaudrate % 100) == 0) {
    Serial2.updateBaudRate(myBaudrate);
    Cmdr.println("UART2 set to " + String(Serial2.baudRate()));
  } else {
    Cmdr.println("Invalid <baudrate>, 9600 <= baudrate <= 921600, multiple of 100");
  }
  return 0;
}

bool loglevelHandler(Commander &Cmdr) {
  int myLogLevel = 0;
  if (Cmdr.countItems() == 1 && Cmdr.getInt(myLogLevel) && myLogLevel >= 0 && myLogLevel <= 7) {
    esp_log_level_set("*", (esp_log_level_t)myLogLevel);
    Cmdr.println("Log level set to " + String(myLogLevel));
  } else {
    Cmdr.println("Invalid <loglevel>, 0 <= loglevel <= 7");
  }
  return 0;
}

const commandList_t MyCommands[] = {
  { "report", reportHandler, "Report dynamic config" },
  { "restart", restartHandler, "Restart the HOST" },
  { "reset", resetHandler, "Reset the TARGET" },
  { "access", accessHandler, "Access Wifi with ssid & password" },
  { "flashbaud", flashbaudHandler, "Set target flash baudrate" },
  { "log", loglevelHandler, "Set console log level" },
};

// ###################################### Init UART & TELNET #######################################

void initialiseHost(void) {
  Serial0.begin(UART0_BAUD, SERIAL_8N1, RXD0, TXD0);
  // start Commander
  MyCmdr.begin(&Serial0, MyCommands, sizeof(MyCommands));
  MyCmdr.echo(true);
  MyCmdr.commandPrompt(true);
  // Connect to default WiFi AccessPoint
  connectHandler(MyCmdr, WIFI_SSID, WIFI_PSWD);
  ServerHostCLI.begin();
  ServerHostCLI.setNoDelay(true);
}

void initialiseTarget(void) {
  Serial1.begin(UART1_BAUD, SERIAL_8N1, RXD1, TXD1);
  pinMode(DTR1, OUTPUT);
  pinMode(RTS1, OUTPUT);
  ServerTargetCLI.begin();
  ServerTargetCLI.setNoDelay(true);
  #ifdef USE_UART1_2_SAME_PINS
    Serial2.begin(UART2_BAUD, SERIAL_8N1, RXD1, TXD1);
  #else
    Serial2.begin(UART2_BAUD, SERIAL_8N1, RXD2, TXD2);
    pinMode(DTR2, OUTPUT);
    pinMode(RTS2, OUTPUT);
    ServerTargetPGM.begin();
    ServerTargetPGM.setNoDelay(true);
  #endif
  digitalWrite(DTRx, HIGH);
  digitalWrite(RTSx, HIGH);
}

// ######################################### Setup handler #########################################

void setup() {
  initialiseHost();                                  //  Host UART + CLI
  initialiseTarget();
  //reportHandler(MyCmdr);
}

void handleSerialToTelnet(Stream& In, WiFiClient& Out) {
  if (In.available()) {
    size_t length = In.available();
    char buffer[length];
    In.readBytes(buffer, length);
    if (Out && Out.connected()) {
      Out.write(buffer, length);
      delay(1);
    }
  }
}

// ####################################### Main loop handler #######################################

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    MyCmdr.println("WiFi not connected! Retrying ...");
    if (ClientHostCLI)
      ClientHostCLI.stop();
    if (ClientTargetCLI)
      ClientTargetCLI.stop();
    if (ClientTargetPGM)
      ClientTargetPGM.stop();  
    return;
  }
  
  if (ServerHostCLI.hasClient()) {      // handle Host CLI connection request via TCP
    ClientHostCLI = ServerHostCLI.accept();
    if (ClientHostCLI) {
    #if 0
      MyCmdr.attachInputPort(&ClientHostCLI);
      MyCmdr.attachOutputPort(&ClientHostCLI);
      MyCmdr.attachAltPort(&Serial0);
    #elif 1
      MyCmdr.attachInputPort(&ClientHostCLI);
      MyCmdr.attachAltPort(&ClientHostCLI);
    #endif
      MyCmdr.echoToAlt(true);
      MyCmdr.copyRepyAlt(true);
      MyCmdr.print("Host CLI connect: ");
      MyCmdr.print(ClientHostCLI.remoteIP());
      MyCmdr.println(":" + String(HOST_CLI_PORT));
    } else {
      MyCmdr.println("Host CLI connect: failed");
    }
  } else {
    WiFiClient rejectClient1 = ServerHostCLI.accept();
    rejectClient1.stop();
  }
  
  if (ClientHostCLI && ClientHostCLI.connected()) {
    if (ClientHostCLI.available()) {
      MyBuf.appendFrom(ClientHostCLI);
      if (MyBuf.hasLine()) {
        MyCmdr.loadString(MyBuf.get());
        MyBuf.reset();
      } else {
        MyCmdr.println(MyBuf.get());
      }
    }
  } else if (ClientHostCLI) {
    ClientHostCLI.stop();
  }

  if (ServerTargetCLI.hasClient()) {    // handle Target CLI connection request via TCP
    ClientTargetCLI = ServerTargetCLI.accept();
    if (ClientTargetCLI) {
      MyCmdr.print("Target CLI connect: ");
      MyCmdr.print(ClientTargetCLI.remoteIP());
      MyCmdr.println(":" + String(TARGET_CLI_PORT));
    } else {
      MyCmdr.println("Target CLI connect: failed");
    }
  } else {
    //no free/disconnected spot so reject
    WiFiClient rejectClient1 = ServerTargetCLI.accept();
    rejectClient1.stop();
  }

  if (ClientTargetCLI && ClientTargetCLI.connected()) { //check clients for data
    if (ClientTargetCLI.available()) {
      while (ClientTargetCLI.available())
        Serial1.write(ClientTargetCLI.read()); //get data from the telnet client and push it to the UART1
    }
  } else if (ClientTargetCLI) {
    ClientTargetCLI.stop();
  }

  if (ServerTargetPGM.hasClient()) {    // handle Target PGM connection request via TCP
    if (ClientTargetPGM && !ClientTargetPGM.connected()) {
      MyCmdr.println("Target PGM closed");
      ClientTargetPGM.stop();
    }
    ClientTargetPGM = ServerTargetPGM.accept();
    if (ClientTargetPGM) {
      MyCmdr.print("Target PGM connect: ");
      MyCmdr.print(ClientTargetPGM.remoteIP());
      MyCmdr.println(":" + String(TARGET_PGM_PORT));
    } else {
      MyCmdr.println("Target PGM connect: failed");
    }
  } else {
    //no free/disconnected spot so reject
    WiFiClient rejectClient = ServerTargetPGM.accept();
    rejectClient.stop();
  }

  resetHandler(MyCmdr);
  if (ClientTargetPGM && ClientTargetPGM.connected()) {
    if (ClientTargetPGM.available()) {
      resetHandler(MyCmdr);
      while (ClientTargetPGM.available())
        Serial2.write(ClientTargetPGM.read()); //get data from the telnet client and push it to the UART
    }
  } else if (ClientTargetPGM) {
    ClientTargetPGM.stop();
    MyCmdr.println("Target PGM closed");
    Serial2.flush();
    Serial1.flush();
    Serial.flush();
  }

  // Process command handler
  MyCmdr.update();
  handleSerialToTelnet(Serial1, ClientTargetCLI);
  handleSerialToTelnet(Serial2, ClientTargetPGM);
}

