/* ESProg Demo software
   
    Copyright (c) 2017-2027 Next Evolution SRL. All rights reserved.
  This file is part of the ESProg demo software package.

  This fostware is distributed as e demo software for ESPProg v6.2b board,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 

  
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

//Serial port 1
#define RXD1 34 
#define RTS1 32
#define TXD1 33
#define DTR1 23

//Serial port 2
#define RXD2 35 
#define DTR2 25
#define TXD2 26
#define RTS2 27 

WiFiServer ServerRaw(23);  // TELNET port - 23 - Target Serial Debug interface
WiFiServer ServerProg(24); // TELNET port - 24 - Target Programming interface
WiFiServer Server(25);     // TELNET port - 25 - ESProg local Serial port 0

WiFiClient Client;        //ESProg local
WiFiClient Client1;       //Target Serial Debug
WiFiClient ClientProg;    //Target Programming

const char* ssid = "irmacos"; // Your WiFi SSID
const char* password = "Irm@C0$1"; // Your WiFi Password

int rst = 0;

void setup() {
  int8_t i;
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);    // original
//  Serial1.begin(256000, SERIAL_8N1, RXD1, TXD1);
  Serial2.begin(256000, SERIAL_8N1, RXD1, TXD1);    // original, part working
//  Serial1.begin(115200, SERIAL_8N1, RXD1, TXD1);
//  Serial2.begin(115200, SERIAL_8N1, RXD1, TXD1);  
//  Serial2.begin(256000, SERIAL_8N1, RXD2, TXD2);
  Serial.println("\nESProg adapter v2.3.4 boot - OK!");
    
  Serial.print("\nConnecting to WiFi '" + String(ssid) + String("' ..."));
  
  WiFi.begin(ssid, password);

  for (i = 60; i != 0; i--) {
    if (WiFi.status() == WL_CONNECTED) break;
    delay(333);
  }
  if (i == 0) {
    Serial.println("Network connection failed!\nRestarting ESP32!");
    delay(1000);
    ESP.restart();
  }
  
  Serial.print("Connected.\nLocal IP address: ");
  Serial.println(WiFi.localIP());
 // Serial.println("");

  //start UART and the server  

  ServerRaw.begin();
  ServerRaw.setNoDelay(true);
  Serial.print("\nUsage:\nPort 23 - debug target.\n");   
  ServerProg.begin();
  ServerProg.setNoDelay(true);
  Serial.print("Port 24 - upload target firmware.\n");   
  Server.begin();
  Server.setNoDelay(true);
  Serial.print("Port 25 - ESProg Serial.\n"); 

  
  pinMode(DTR1, OUTPUT);
  digitalWrite(DTR2, HIGH);
  pinMode(RTS1, OUTPUT);
  digitalWrite(RTS2, HIGH);  
  
}


void Reset_target()
{
   //set the DTR2 signal
  bool dtr = (ClientProg && ClientProg.connected());
//  bool dtr = 1;
  digitalWrite(DTR1, !dtr);
  //Serial.print(dtr);
  //Serial.println("- DTR!");
  if (dtr == 1 )
  {
    //Serial.println("Client Connected 1!");
    if (rst == 0){
        digitalWrite(DTR1, LOW);
        delay(50);
        digitalWrite(RTS1, LOW);
        delay(50);
        rst = 1;
        //Serial.println("\nrst = 1 Target reset");
        Serial.println("RESET Target");
        Serial.print("Programming target ... ");
       // delay(50);
       // digitalWrite(DTR2, HIGH);
        delay(50);
        digitalWrite(RTS1, HIGH);
        delay(50);
    }
  }
  else {
      if (rst == 1)
      {
        rst = 0;
        //Serial.println("\nrst = 0 Target reset");
        Serial.println("Done.");
        digitalWrite(DTR1, HIGH);
        delay(50);
        digitalWrite(RTS1, LOW);
        delay(50);
        digitalWrite(RTS1, HIGH);
      }
  } 
}

void loop() 
{
 // delay(200);
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected! Retrying ...");
    if (Client) Client.stop();
    if (Client1) Client1.stop();
    if (ClientProg) ClientProg.stop();  
    return;
  }
  
  if (Server.hasClient()) { //check if there are any new clients
  Client = Server.available();
  if (!Client) Serial.println("available broken");
  Serial.print("New client on port 25: ");
  Serial.println(Client.remoteIP());
  } else {
    //no free/disconnected spot so reject
    WiFiClient rejectClient1 = Server.available();
    rejectClient1.stop();
  }
  
  if (Client && Client.connected()) { //check clients for data
    if (Client.available()) {
      while (Client.available())
      Serial.write(Client.read()); //get data from the telnet client and push it to the UART
      }
  }
  else if (Client) Client.stop();

//DEBUG TARGET Server  
  if (ServerRaw.hasClient()) { //check if there are any new clients
  Client1 = ServerRaw.available();
  if (!Client1) Serial.println("available broken");
  Serial.print("New client on port 23: ");
  Serial.println(Client1.remoteIP());
  } else {
    //no free/disconnected spot so reject
    WiFiClient rejectClient1 = ServerRaw.available();
    rejectClient1.stop();
  }

  if (Client1 && Client1.connected()) { //check clients for data
    if (Client1.available()) {
      while (Client1.available())
      Serial1.write(Client1.read()); //get data from the telnet client and push it to the UART1
      }
  }
  else if (Client1) Client1.stop();


//programming TARGET Server
  if (ServerProg.hasClient()) { //check if there are any new clients
     if (ClientProg && !ClientProg.connected()) {
        Serial.print("Client closed on port 25!");
        ClientProg.stop();
    }
  ClientProg = ServerProg.available();
  if (!ClientProg) Serial.println("available broken");
  Serial.print("\nNew client on port 24: ");
  Serial.println(ClientProg.remoteIP());
 // Reset_target();
  } else {
    //no free/disconnected spot so reject
    WiFiClient rejectClient = ServerProg.available();
    rejectClient.stop();
  }

  Reset_target();
  if (ClientProg && ClientProg.connected()) {//check clients for data
   // Reset_target();
    if (ClientProg.available()) {
      Reset_target();
      while (ClientProg.available())
      Serial2.write(ClientProg.read()); //get data from the telnet client and push it to the UART
      }
  }
  else if (ClientProg) {
    ClientProg.stop();
      Serial.print("Client closed on port 25.");
      Serial2.flush();
      Serial1.flush();
      Serial.flush();
  }



// CHECK UART ports DATA
  
  if (Serial.available()){ //check UART for data
    size_t len = Serial.available();
    char sbuf[len];
    Serial.readBytes(sbuf, len);
    if (Client && Client.connected()){ 
      Client.write(sbuf, len);
      delay(1);
    }
  }

   if (Serial1.available()){ //check UART for data
    size_t len1 = Serial1.available();
    char sbuf1[len1];
    Serial1.readBytes(sbuf1, len1);
    if (Client1 && Client1.connected()){
      Client1.write(sbuf1, len1);
      delay(1);
    }
  }

    if (Serial2.available()){ //check UART for data
    size_t len2 = Serial2.available();
    char sbuf2[len2];
    Serial2.readBytes(sbuf2, len2);
    if (ClientProg && ClientProg.connected()){
      ClientProg.write(sbuf2, len2);
      delay(1);
    }
  }
}
