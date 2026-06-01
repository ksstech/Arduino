/*
 * NextEvolution SRL  | Jan 2026
 * Copyright (c) 2026
 * 
 * 
 * Demo Software for 1Wire-RS485 Adapter Board
 * 
 * 1Wire-RS485 Adapter direct RS485 communication 
 * 
 * Serial Terminal Commands:
 * 
 * Device ID, followed by:
 * 
 * r - Read IButton data 
 * w - Write new data on Ibutton device
 * ron - Relay ON 
 * roff - Relay OFF
 * b - reboot remotely the 1Wire RS485 Adapter
 * 
 * EXAMPLE: 10,ron
 * 
 * All commands should be sent as CSV ASCII text no spaces, newline terminator ONLY, NO CR.
 * 
 * 
 * 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:

 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 
 * 
 */


#include <OneWire.h>
//#include <../libraries/OneWire/OneWire.h>
//#include <./OneWire/OneWire.h>
#include <iButtonTag.h>

//RS485 control
#define RS485_TX_PIN_VALUE HIGH
#define RS485_RX_PIN_VALUE LOW
#define SERIAL_COMMUNICATION_CONTROL_PIN 7      // PB0 (Pin 9) - RS485-REDE - Transmission set pin

#define PIN_PROBE 0     // Connect iButton interface to pin PA4
#define LED_PIN  1      // the number of the LED pin - PA5
#define RLY  PIN_PA2    // the number of the RELAY pin - PA2

String command;
String num;
char id[]="12";         // set device ID - 10 is default
char buf[44];
int end;

iButtonTag ibutton( PIN_PROBE );      // Setup iButtonTag on the selected pin
iButtonCode oldcode, currentcode;     // Variables to store ID-codes before/after

// Variable to store new ID-code, last byte will be overwritten later. Change
// the byte-values to the code you like! The last byte may not be correct - it
// has to be a checksum of the other bytes - but we'll change that later on.
iButtonCode newcode = { 0x01, 0x01, 0x02, 0x01, 0x02, 0x03, 0x01, 0x00 };

void iButton_read()
{

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


void iButton_write()
{ 
  tx_on();
  Serial.printf("%s:WRITE command\n", id);
    // Try to read old iButtonCode until a tag is present
  while(ibutton.readCode(oldcode) == 0) delay(250);
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
  if ( status == 1 ) Serial.print( "OK!");
  else Serial.print( "with error:" ) ;
  Serial.printf("(status %d)\n",status);

  // Read current iButtonCode from tag
  while(ibutton.readCode(currentcode)== 0) delay(250);
  Serial.printf("%s:Current iButton code: ",id);
  ibutton.printCode( currentcode );
  Serial.println();
  rx_on();
}

void rs485_print(const char msg[32])
{
  digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_TX_PIN_VALUE); // Init transmit
  delay(25);   
  Serial.printf("%s:%s\n",id, msg);
  delay(50);
  digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_RX_PIN_VALUE);// Init receive
}

void rx_on()
{
  delay(50);
  digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_RX_PIN_VALUE);// Init receive
}

void tx_on(){
  digitalWrite(SERIAL_COMMUNICATION_CONTROL_PIN, RS485_TX_PIN_VALUE); // Init transmit
  delay(25);
}

void BlinkLED(int tms, int dly)
{
  for (int f = 1 ; f <= tms; f += 1) {
    digitalWrite(LED_PIN, HIGH);
    delay(dly);
    digitalWrite(LED_PIN, LOW);
    delay(dly);
  }    
}

void Relay(byte stat)
{
   tx_on(); 
   Serial.printf("%s:Relay ",id);
   if (stat == 1)
   {
     digitalWrite(LED_PIN, HIGH);
     digitalWrite(RLY, HIGH);         //RELAY ON
     Serial.println("ON");
   }else if (stat == 0)
   {
   digitalWrite(RLY, LOW);          //RELAY OFF
   Serial.println("OFF");
   digitalWrite(LED_PIN, LOW);   
   }
   rx_on(); 
}

void reset_mcu() {
   rs485_print("Reboot command"); 
  _PROTECTED_WRITE(WDT.CTRLA,WDT_PERIOD_8CLK_gc); //enable the WDT, minimum timeout
  while (1); // spin until reset
}


void setup() {
  Serial.begin(19200); 
  Serial.flush();
  
  pinMode(SERIAL_COMMUNICATION_CONTROL_PIN, OUTPUT);   // Init RS485 REDE 
  pinMode(RLY, OUTPUT);                               //Init Relay GPIO Pin
  digitalWrite(RLY, LOW);
  pinMode(LED_PIN, OUTPUT);                           // Init iBUTTON reader LED Pin
  digitalWrite(LED_PIN, LOW);

  rs485_print("nEXT 1Wire RS485 Adapter v1.6.4c - Boot OK!");
  
  BlinkLED(3,100);
}
 
void loop() {

    rx_on(); 
    if(Serial.available() > 0){
        command = Serial.readStringUntil('\n');
        while((end=command.indexOf(","))!=-1)
        { 
              num = command.substring(0,end);
              command = command.substring(end+1,command.length());
         }
    if(num.equals(id))
    {    
        if(command.equals("r")){
          iButton_read();
        }
        else if(command.equals("w")){
          iButton_write();
        }
        else if(command.equals("ron")){
          Relay(1);
        }
        else if(command.equals("roff")){
          Relay(0);
        }
        else if(command.equals("b")){
          reset_mcu();
        }
        else{
          rs485_print("Invalid command");
        }
    }
  }
}
