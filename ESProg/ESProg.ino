/*
  Remote monitoring and recovery software 
  Copyright (c) 2025 Andre M. Maree / KSS Technologies (pty) Ltd. All rights reserved

  Serial Debug   -> port 23
  Firmware upload -> port 24
  ESProg Serial   -> port 25
*/

// Official libraries
#include <WiFi.h>
#include <Commander.h>              // Add Commander by Bill Bigge v4.2.3

// Select platform definition file
#include <../libraries/PrivateLibs/esprog.h>
//#include <../libraries/PrivateLibs/esp32-s3-usb-bridge.h>

// Unofficial support files
#include <../libraries/PrivateLibs/StaticBuffer.h>
#include <../libraries/PrivateLibs/FormattedMessages.h>
#include <../libraries/PrivateLibs/EspReporting.h>

#define FW_VERSION  "0.2"

#define WIFI_SSID "irmacos"
#define WIFI_PSWD "Irm@C0$1"

#define UART0_BAUD  115200
#define UART1_BAUD  115200
#define UART2_BAUD  256000

#define TARGET_CLI_PORT 23    // TELNET port -> UART1 (Target CLI) interface
#define TARGET_PGM_PORT 24    // TELNET port -> UART2 (Target PGM) interface
#define HOST_CLI_PORT   25    // TELNET port -> UART0 (Host CLI) interface

Commander MyCmdr;
CharBuffer MyBuf;
int rst = 0;
uint32_t countTXtoTarget = 0, countTXtoHost = 0, countRXFromTarget = 0, countRXfromHost = 0;

WiFiServer TargetCLIserver(TARGET_CLI_PORT);
WiFiClient TargetCLIclient;

WiFiServer TargetPGMserver(TARGET_PGM_PORT);
WiFiClient TargetPGMclient;

WiFiServer HostCLIserver(HOST_CLI_PORT);
WiFiClient HostCLIclient;

// ##################################### Local supporting APIs #####################################

bool checkWifi(Commander& Cmdr) {
  if (WiFi.status() == WL_CONNECTED)
    return true;
  Cmdr.println("WiFi not connected! Retrying ...");
  if (HostCLIclient)
    HostCLIclient.stop();
  if (TargetCLIclient)
    TargetCLIclient.stop();
  if (TargetPGMclient)
    TargetPGMclient.stop();
  return false;
}

void resetTarget(Commander& Cmdr) {
  Cmdr.println("RESET Target");
  //delay(50);
  delay(50);
  digitalWrite(RTSx, LOW);
  delay(50);
  digitalWrite(RTSx, HIGH);
}

void displayIPinfo(Commander& Cmdr, WiFiClient& Client) {
  Cmdr.print(Client.localIP());
  Cmdr.print(":");
  Cmdr.print(Client.localPort());
  Cmdr.print(" from ");
  Cmdr.print(Client.remoteIP());
  Cmdr.print(":");
  Cmdr.println(Client.remotePort());
}

// ###################################### Commander handlers #######################################

bool restartHandler(Commander& Cmdr) {
  Cmdr.println("... Restarting ESP32!");
  delay(1000);
  ESP.restart();
  return 0;
}

bool resetHandler(Commander& Cmdr) {
  bool dtr = (TargetPGMclient && TargetPGMclient.connected());
    digitalWrite(DTRx, !dtr);
  if (dtr == 1 ) {
    if (rst == 0) {
        rst = 1;
        digitalWrite(DTRx, LOW);
        resetTarget(Cmdr);
        Cmdr.println("Start target PGM...");
    }
  } else {
    if (rst == 1) {
      digitalWrite(DTRx, HIGH);
      resetTarget(Cmdr);
      rst = 0;
      Cmdr.println("Done target PGM...");
      Cmdr.print(countTXtoTarget);
      Cmdr.println(" Bytes downloaded");
      countTXtoTarget = 0;
    }
  }
  return 0;
}

bool reportHandler(Commander& Cmdr) {
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

bool connectHandler(Commander& Cmdr, String ssid, String pswd) {
  WiFi.begin(ssid, pswd);
  int8_t i = 0;
  do {
    if (++i == 60) {
      Cmdr.println(strNL "Network connection failed!");
      restartHandler(Cmdr);
    }
    delay(333);
  } while (WiFi.status() != WL_CONNECTED);
  return 0;
}

bool accessHandler(Commander& Cmdr) {
  String mySSID = "";
  String myPSWD = "";
  if ((Cmdr.countItems() == 2) && Cmdr.getString(mySSID) && Cmdr.getString(myPSWD)) {
      connectHandler(Cmdr, mySSID, myPSWD);
  } else {
    Cmdr.println("Must have <ssid> and <password> parameters");
  }
  return 0;
}

bool flashbaudHandler(Commander& Cmdr) {
  int myBaudrate = 0;
  if (Cmdr.countItems() == 1 && Cmdr.getInt(myBaudrate) && myBaudrate >= 9600 && myBaudrate <= 921600 && (myBaudrate % 100) == 0) {
    Serial2.updateBaudRate(myBaudrate);
    Cmdr.println("UART2 set to " + String(Serial2.baudRate()));
  } else {
    Cmdr.println("Invalid <baudrate>, 9600 <= baudrate <= 921600, multiple of 100");
  }
  return 0;
}

bool loglevelHandler(Commander& Cmdr) {
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

void initialiseCLI(Commander& Cmdr) {
  Cmdr.begin(&Serial0, MyCommands, sizeof(MyCommands));
  Cmdr.echo(true);
  Cmdr.commandPrompt(true);
  //reportHandler(Cmdr);
}

void initialiseHost(Commander& Cmdr) {
  Serial0.begin(UART0_BAUD, SERIAL_8N1, RXD0, TXD0);
  initialiseCLI(MyCmdr);                                // Connect to default WiFi AccessPoint
  connectHandler(MyCmdr, WIFI_SSID, WIFI_PSWD);
  HostCLIserver.begin();
  HostCLIserver.setNoDelay(true);
}

void initialiseTarget(Commander& Cmdr) {
  Serial1.begin(UART1_BAUD, SERIAL_8N1, RXD1, TXD1);
  Serial2.begin(UART2_BAUD, SERIAL_8N1, RXD1, TXD1);
  pinMode(DTRx, OUTPUT);
  pinMode(RTSx, OUTPUT);
  digitalWrite(DTRx, HIGH);
  digitalWrite(RTSx, HIGH);

  TargetCLIserver.begin();
  TargetCLIserver.setNoDelay(true);
  TargetPGMserver.begin();
  TargetPGMserver.setNoDelay(true);
}

void setup(void) {
  initialiseHost(MyCmdr);
  initialiseTarget(MyCmdr); 
  MyCmdr.println("\nESProg v6.2b Firmware v3.0.1");
  MyCmdr.print("Connected to WiFi '" + String(WIFI_SSID) + "' with local address ");
  MyCmdr.println(WiFi.localIP());
  MyCmdr.println("Usage:\nPort 23 -> Target CLI[UART1]\nPort 24 -> Target PGM[UART2].\nPort 25 -> Host CLI[UART0]");
}

// ################################### Host/Target CLI handlers ####################################

/**
 @brief
 @param
 @param
 @param
 @return
 */
int handleTCPconnection(Commander& Cmdr, WiFiServer& Server, WiFiClient& Client) {
  int iRV = 0;
  if (Server.hasClient()) {
    if (Client.connected()) {
      Cmdr.print("Disconnecting ");
      displayIPinfo(Cmdr, Client);
      Client.stop();
      iRV = 1;
    }
    Client = Server.accept();
    if (Client) {
      Cmdr.print("Client connection to ");
      displayIPinfo(Cmdr, Client);
    } else {
      Cmdr.println("Client accept failed!");
      iRV = -1;
    }
  }
  return iRV;
}

#define OPTION  1

/**
 @brief
 @param
 @param
 @param
 @return
 */
ssize_t handleTCPtoSerial(Commander& Cmdr, WiFiClient& InClient, Stream& OutStream) {
  #if (OPTION == 1)
  ssize_t sRV = 0;
  if (InClient && InClient.connected()) {
    while(InClient.available()) {
      OutStream.write(InClient.read());
      ++sRV;
    }
    Cmdr.print(sRV);
    Cmdr.print(',');
  }
  #else
  ssize_t sRV = 0;
  if (InClient && InClient.connected()) {
    sRV = InClient.available();
    if (sRV > 0) {
      char buffer[sRV];
      InClient.readBytes(buffer, sRV);
      OutStream.write(buffer, sRV);
    }
    Cmdr.print(sRV);
    Cmdr.print(',');
  }
  #endif
  return sRV;
}

/**
 @brief
 @param
 @param
 @param
 @return
 */
ssize_t handleSerialToTCP(Commander& Cmdr, Stream& InStream, WiFiClient& OutClient) {
  #if (OPTION == 1)
  ssize_t sRV = 0;
  if (InStream.available()) {
    if (OutClient && OutClient.connected()) {
      while(InStream.available()) {
        OutClient.write(InStream.read());
        ++sRV;
      }
    } else {
      Cmdr.print("Bytes dropped to ");
      displayIPinfo(Cmdr, OutClient);
    }
  }
  #else
  ssize_t sRV = InStream.available();
  if (sRV) {
    char buffer[sRV];
    InStream.readBytes(buffer, sRV);
    Cmdr.print(sRV);
    if (OutClient && OutClient.connected()) {
      OutClient.write(buffer, sRV);
      Cmdr.print(',');
      delay(1);
    } else {
      Cmdr.print("Bytes dropped to ");
      displayIPinfo(Cmdr, OutClient);
    }
  }
  #endif
  return sRV;
}

// ####################################### Main loop handler #######################################

void loop() {
  int iRV;
  if (checkWifi(MyCmdr) == false)
    return;

  if (TargetPGMserver.hasClient()) {    // handle Target PGM connection request via TCP
    if (TargetPGMclient && !TargetPGMclient.connected()) {
      MyCmdr.println("Target PGM closed");
      TargetPGMclient.stop();
    }
    TargetPGMclient = TargetPGMserver.accept();
    if (TargetPGMclient) {
      MyCmdr.print("Target PGM connect: ");
      displayIPinfo(MyCmdr, TargetPGMclient);
    } else {
      MyCmdr.println("Target PGM connect: failed");
    }
  } else {
    //no free/disconnected spot so reject
    WiFiClient rejectClient = TargetPGMserver.accept();
    rejectClient.stop();
  }

  resetHandler(MyCmdr);
  if (TargetPGMclient && TargetPGMclient.connected()) {
    if (TargetPGMclient.available()) {
      resetHandler(MyCmdr);
      while (TargetPGMclient.available())
        Serial2.write(TargetPGMclient.read()); //get data from the telnet client and push it to the UART
        ++countTXtoTarget;
    }
  } else {
    if (TargetPGMclient) {
      TargetPGMclient.stop();
      MyCmdr.println("Target PGM closed");
      Serial2.flush();
      Serial1.flush();
      Serial.flush();
    }
  }
  if (rst) {
    handleSerialToTCP(MyCmdr, Serial2, TargetPGMclient);
  } else {
    handleTCPconnection(MyCmdr, TargetCLIserver, TargetCLIclient);
    handleTCPtoSerial(MyCmdr, TargetCLIclient, Serial1);
    handleSerialToTCP(MyCmdr, Serial1, TargetCLIclient);
  }

  iRV = handleTCPconnection(MyCmdr, HostCLIserver, HostCLIclient);
  if (iRV >= 0) {
      MyCmdr.attachInputPort(&HostCLIclient);
      MyCmdr.attachOutputPort(&HostCLIclient);
      MyCmdr.attachAltPort(&Serial0);
      //MyCmdr.echoToAlt(true);
      //MyCmdr.copyRepyAlt(true);
  }
  handleTCPtoSerial(MyCmdr, HostCLIclient, Serial);
  handleSerialToTCP(MyCmdr, Serial, HostCLIclient);
  // Process command handler
  MyCmdr.update();
}

/* Changelog
Date:     Ver:  Comments:
20250530  0.1   Initial development started
20250917  0.2   Upgraded to Core 3.3.0 and IDF 5.5.1

*/