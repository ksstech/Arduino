/*
 * RS485 to OneWire - Copyright (c) 2026 Andre M. Maree / KSS Technologies (Pty) Ltd.
 * 
 * Serial Terminal Commands:
 * 
 * Device ID, followed by:
 * 
 * or - Read IButton data 
 * ow - Write new data on Ibutton device
 * rt - Relay On
 * rf - Relay Off
 * lt - LED On
 * lf - LED Off
 * si - System Info request
 * sr - reboot remotely the 1Wire RS485 Adapter
 * ss - System Status request
 * 
 * EXAMPLE: 10,rt
 * 
 * All commands should be sent as CSV ASCII text no spaces, newline terminator ONLY, NO CR.
 */

// if alias to ../libraries" is present will pick correct file up.
#include <OneWire.h>
#include <iButtonTag.h>

#define PIN_UPDI            PIN_PA0
#define PIN_RELAY           PIN_PA2       // RELAY
#define PIN_1WIRE           PIN_PA4       // iButton data
#define PIN_LED             PIN_PA5       // LED
//RS485 control
#define PIN_RS485_REDE      PIN_PB0       // RS485-REDE
#define PIN_RS485_TX        PIN_PB2
#define PIN_RS485_RX        PIN_PB3
#define RS485_SET_TX        HIGH
#define RS485_SET_RX        LOW

// Unused: PA1, PA3/EXTCLK, PA6/IN3, PA7, PB1/RESET

String command;
String num;
char id[]="12";         // set device ID - 10 is default
char buf[44];
int end;

iButtonTag ibutton( PIN_1WIRE );      // Setup iButtonTag on the selected pin
iButtonCode oldcode, currentcode;     // Variables to store ID-codes before/after

// Variable to store new ID-code, last byte will be overwritten later. Change
// the byte-values to the code you like! The last byte may not be correct - it
// has to be a checksum of the other bytes - but we'll change that later on.
iButtonCode newcode = { 0x01, 0x01, 0x02, 0x01, 0x02, 0x03, 0x01, 0x00 };

void iButton_read(void) {
  iButtonCode code;                            // Variable to store identification code 
  int8_t status = ibutton.readCode( code );   // Try to read an identification code from the probe
  tx_on();  
  Serial.printf("%s:READ command\n",id);
  switch( status ) {                          // Variable _status_ will now indicate success/failure
    case 1: // Success
      Serial.printf("%s:iButton code: ",id);
      ibutton.printCode( code );              // Variable _code_ contains the ID-code
      Serial.println();
      break;
    case 0: // No iButton
      Serial.printf("%s:No iButton detected",id);
      break;
    case -1: // Checksum invalid
    case -2: // Code all zeros
      Serial.printf("%s:iButton code invalid",id);
      break;
    default: // Unknown - Shouldn't happen
      Serial.printf("%s:Unknown status",id);
  }
  rx_on();
}

void iButton_write(void) { 
  tx_on();
  Serial.printf("%s:WRITE command\n", id);
    // Try to read old iButtonCode until a tag is present
  while(ibutton.readCode(oldcode) == 0)
    delay(250);
  Serial.printf("%s:Old iButton code: ",id);
  ibutton.printCode( oldcode );
  Serial.println();

  // Update last byte of new ID-code to correct checksum
  ibutton.updateChecksum( newcode );
  Serial.printf("%s:New iButton code: ",id);
  ibutton.printCode( newcode );
  Serial.println();

  // Try to write the new ID-code
  int8_t status = ibutton.writeCode( newcode );

  // iButton tag type TM01 (including model TM01C) is non-detectable and _can_
  // be written. To write a new code to such a tag the specific type needs to be
  // passed. Replace the line above with this one, without comment slashes:
  //
  //   int8_t status = ibutton.writeCode( newcode, IBUTTON_TM01 );

  // Evaluate success/failure based on returned status
  Serial.printf("%s:Writing procedure finished ",id );
  Serial.print(status == 1 ? "OK!" : "with error:");
  Serial.printf("(status %d)\n",status);

  // Read current iButtonCode from tag
  while(ibutton.readCode(currentcode)== 0)
    delay(250);
  Serial.printf("%s:Current iButton code: ",id);
  ibutton.printCode( currentcode );
  Serial.println();
  rx_on();
}

void rs485_print(const char * msg) {
  digitalWrite(PIN_RS485_REDE, RS485_SET_TX); // Init transmit
  delay(25);
  Serial.printf("%s:%s\n",id, msg);
  delay(50);
  digitalWrite(PIN_RS485_REDE, RS485_SET_RX);// Init receive
}

void rx_on(void) {
  delay(50);
  digitalWrite(PIN_RS485_REDE, RS485_SET_RX);// Init receive
}

void tx_on(void){
  digitalWrite(PIN_RS485_REDE, RS485_SET_TX); // Init transmit
  delay(25);
}

void BlinkLED(int tms, int dly) {
  for (int f = 1 ; f <= tms; f += 1) {
    digitalWrite(PIN_LED, HIGH);
    delay(dly);
    digitalWrite(PIN_LED, LOW);
    delay(dly);
  }    
}

void SetLED(bool stat) {
  tx_on();
  Serial.printf("%s:LED %s",id, stat ? "ON" : "OFF");
  digitalWrite(PIN_LED, stat ? HIGH : LOW);
  rx_on(); 
}

void SetRLY(bool stat) {
  tx_on(); 
  Serial.printf("%s:Relay %s",id, stat ? "ON" : "OFF");
  digitalWrite(PIN_RELAY, stat ? HIGH : LOW);
  rx_on(); 
}

void system_info(void) { }

void system_reset(void) {
   rs485_print("Reboot command"); 
  _PROTECTED_WRITE(WDT.CTRLA,WDT_PERIOD_8CLK_gc); //enable the WDT, minimum timeout
  while (1); // spin until reset
}

void system_status(void) { }

void setup() {
  Serial.begin(19200); 
  Serial.flush();
  
  pinMode(PIN_RS485_REDE, OUTPUT);   // Init RS485 REDE 
  pinMode(PIN_RELAY, OUTPUT);                               //Init Relay GPIO Pin
  digitalWrite(PIN_RELAY, LOW);
  pinMode(PIN_LED, OUTPUT);                           // Init iBUTTON reader LED Pin
  digitalWrite(PIN_LED, LOW);

  rs485_print("nEXT 1Wire RS485 Adapter v1.6.4c - Boot OK!");  
  BlinkLED(3,100);
}
 
void loop() {
  rx_on(); 
  if(Serial.available() > 0) {
    command = Serial.readStringUntil('\n');
    while((end = command.indexOf(",")) != -1) {
      num = command.substring(0,end);
      command = command.substring(end+1, command.length());
    }
    if(num.equals(id)) {
      if(command.equals("or")) {
        iButton_read();
      } else if(command.equals("ow")) {
        iButton_write();
      } else if(command.equals("rt")) {
        SetRLY(1);
      } else if(command.equals("rf")) {
        SetRLY(0);
      } else if(command.equals("lt")) {
        SetLED(1);
      } else if(command.equals("lf")) {
        SetLED(0);
      } else if(command.equals("si")) {
        system_info();
      } else if(command.equals("sr")) {
        system_reset();
      } else if(command.equals("ss")) {
        system_status();
      } else {
        rs485_print("Invalid command");
      }
    }
  }
}
