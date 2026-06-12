#include <Arduino.h>
#include "esp_wifi.h"
#include "esp_bt.h"
#include "Adafruit_INA3221.h"
#include "ArduinoNvs.h"
#include "driver/rtc_io.h"

/*  15/12/2025 Now all pins work reliably Version 1.0 
/*
/*
Emulate ibutton slave. interface with DS2482-800.
Wait for reset
Send presence
Wait for presence-end (line returns HIGH)
Wait for first falling edge → start of slot
Wait the sample time
Read level
Return to WAIT_FOR_START_BIT
RESET → PRESENCE
    ↓
CMD READ (0x33)
    ↓
romReadActive = true
    ↓
DS2482 issues 64 falling edges (read slots)
    ↓ (ISR)
ESP32 outputs each ROM bit at each falling edge
    ↓
64 bits sent → romReadActive = false
──────────────────────────────────────────────
LAST BIT OF 0x33 COMMAND (WRITE-1)
──────────────────────────────────────────────
t = 0 µs     DS2482 pulls LOW (~6 µs)
t = 6 µs     DS2482 releases → HIGH (for rest of 60 µs slot)

──────────────────────────────────────────────
RECOVERY / GAP
──────────────────────────────────────────────
t = 60–70 µs  line stays HIGH (tREC)

──────────────────────────────────────────────
FIRST READ SLOT
──────────────────────────────────────────────
t = 70 µs     DS2482 pulls LOW (start read slot)
t = 76 µs     DS2482 releases → HIGH
              • Slave bit window opens
              • iButton may pull LOW (bit=0) or leave HIGH (bit=1)

t = 85 µs     DS2482 samples the bit (≈15 µs sample point)

t = 130 µs    Read slot ends (60 µs total)
Slot 1:
0–8 µs    : Master pulls line low (start of slot 1)
8–65 µs   : Line high / slave may pull low for '0'
12–15 µs  : Master samples line
65 µs     : End of slot 1

Slot 2:
65–73 µs  : Master pulls line low (start of slot 2)
73–130 µs : Line high / slave may pull low for '0'
77–80 µs  : Master samples line
130 µs    : End of slot 2

*/


/* INA3221 Pins*/
#define SCL 22
#define SDA 21
#define CRI 39
#define TC 35
#define WRM 19
constexpr uint8_t INA3221Pns[5] ={SCL, SDA, CRI, TC, WRM};

/* Uart Pins*/
#define RXD 16
#define TXD 17
#define RESET 7
#define BOOT 15
constexpr uint8_t UartPins[4] = {RXD, TXD, RESET, BOOT};

/* Optocoupler Pins*/
#define OPTO1 13
#define OPTO2 27
#define OPTO3 12
#define OPTO4 2
constexpr uint8_t OptoCouplerPins[4] = {OPTO1, OPTO2, OPTO3, OPTO4};

/*OneWire Pins*/
#define BUT1 4
#define BUT2 5
#define BUT3 18 
#define BUT4 19
#define BUT5 32
#define BUT6 10 
#define BUT7 23
#define BUT8 33
constexpr uint8_t oneWirePins[8] = {BUT1, BUT2, BUT3, BUT4, BUT5, BUT6, BUT7, BUT8};
constexpr int NumPins = 8;

// Example 8-byte iButton ROM (64-bit address)
uint8_t iButtonROMs[NumPins][8];

/* IO pins 2*/
#define IO_1 0
#define IO_2 14
constexpr uint8_t InputPins[2]  = {IO_1, IO_2};

/* DAC output*/
#define DAC1 25
#define DAC2 26
constexpr uint8_t DacPins[2] = {DAC1, DAC2};

// Create an INA3221 object
Adafruit_INA3221 ina3221;


//constexpr uint8_t oneWirePins[8] = {4, 5, 18, 19, 21, 22, 23, 25};

hw_timer_t *romTimer = nullptr;
hw_timer_t *houseTimer = nullptr;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;



// 1-Wire timing constants (µs)
constexpr uint16_t T_RESET_MIN         = 450;   // 480 to 860
constexpr uint16_t T_RESET_MAX         = 960;
constexpr uint16_t T_READ_MIN          = 4; // Read pluse from master normally 6 u sec
constexpr uint16_t T_READ_START        = 11;  // Wait 8 usec then take the line down to start 
constexpr uint16_t T_READ_A_ZERO       = 5;  // Wait 5 usec before taking line low for a zero 
constexpr uint16_t T_READ_MAX          = 18;  // was 17 
constexpr uint16_t T_PRESENCE_DELAY    = 30;   //15 µs minimum  60 µs maximum
constexpr uint16_t T_PRESENCE_DURATION = 120;  // 120 µs LOW presence → VALID (60–240 µs allowed)
constexpr uint16_t T_READ_BIT_DURATION = 40; // reduced from 50 µs
constexpr uint16_t T_BIT_SAMPLE        = 15;  // typical 15 usec sample after falling edge

bool flip = true;

struct Edge {
    uint32_t time;
    bool level;  // 0=LOW, 1=HIGH
};

constexpr int ROM_BITS = 64;

struct RomBitPair {
    bool bit;
    bool complement;
};

struct OneWirePin {
    enum State { IDLE, RESET_DETECTED, COMMAND_READING, PRESENCE_PULSE,
                 READ_IDLE, READ_DETECTED, READ_BYTE} state = IDLE;
    
     // Circular buffer for edges
    static constexpr int MAX_EDGES = 128;  // safest
    volatile Edge edgeQueue[MAX_EDGES];
    volatile int head = 0;
    volatile int tail = 0;
    // Reset detection
    uint32_t resetFalling = 0;
    uint32_t resetRising  = 0;
    uint32_t presenceRising  = 0;
    // Presence pulse
    uint8_t RomArrayIndex = 0;
    // Command reading
    uint8_t command = 0;
    int bitIndex = 0;
    uint32_t lastBitTime = 0;
   //  bool readDetect = false;
    uint8_t readBit = 0;
    bool readBitReady = false;
    enum Command {NONE, 
                    READ_ROM        // (only if 1 device on bus) 0x33
                  , MATCH_ROM       // (address one specific device) 0x55
                  , SKIP_ROM        // (broadcast to all devices) 0xCC
                  , SEARCH_ROM      // (discover devices) 0xF0
                 } Cmdstate = NONE;

    bool compliment = false;   

RomBitPair romBitArray[ROM_BITS];
};

OneWirePin oneWire[NumPins];

float relayCal[NumPins]{};

// single shared array
uint8_t IntPin = 0;                // Variable to store which pin is generating the INT and to service that pin 
volatile int currentPinIndex = -1;  // Global variable to store the active pin index for the rom dump.
volatile int houseIndex = -1;       // Global varable to store pin number calling the  house timer.


bool Calibrateupdate = false;
bool optoToggle = false;
uint8_t dacCount = 0;

/*****************  Progarm start ***********************/

void setPinHigh(int index, int duration);  /// Forward decleration

char readSerialChar(void) {
   return Serial.available() ? (char)Serial.read() : '\0';
}

// CRC8 calculation (Dallas/Maxim)
uint8_t crc8(const uint8_t *data, int len) {
    uint8_t crc = 0;
    for (int i = 0; i < len; i++) {
        uint8_t inbyte = data[i];
        for (int j = 0; j < 8; j++) {
            uint8_t mix = (crc ^ inbyte) & 0x01;
            crc >>= 1;
            if (mix) crc ^= 0x8C; // polynomial x^8 + x^5 + x^4 + 1
            inbyte >>= 1;
        }
    }
    return crc;
}

// Load ROM bits for a given pin
void loadRomBits(int Pin) {
    const uint8_t *rom = iButtonROMs[Pin];
    OneWirePin &pin = oneWire[Pin];
    pin.RomArrayIndex = 0;
    // Calculate CRC for the first 7 bytes
    uint8_t crc = crc8(rom, 7);

    // Construct full ROM including CRC
    uint8_t fullRom[8];
    for (int i = 0; i < 7; i++) fullRom[i] = rom[i];
    fullRom[7] = crc;

    // Populate romBitArray (LSB first)
    int bitIndex = 0;
    for (int byteIndex = 0; byteIndex < 8; byteIndex++) {
        for (int b = 0; b < 8; b++) { // LSB first
            bool bit = (fullRom[byteIndex] >> b) & 0x01;
            pin.romBitArray[bitIndex].bit = bit;
            pin.romBitArray[bitIndex].complement = !bit;
            bitIndex++;
        }
    }
}

void generateRandomIButtonROMs() {
    randomSeed(esp_random());

    for (int i = 0; i < NumPins; i++) {
        uint8_t *rom = iButtonROMs[i];

        // 1. Family code (DS1990A)
        rom[0] = 0x01;

        // 2. 48-bit random serial
        for (int b = 1; b <= 6; b++) {
            rom[b] = random(0, 256);
        }

        // 3. CRC8 over first 7 bytes
        rom[7] = crc8(rom, 7);
    }
}

inline int IRAM_ATTR fastReadPin(int pin) {
    if (pin < 32) {
        return (GPIO.in >> pin) & 1;
    } else {
        // Pins 32-39 are in the second input register bank
        return (GPIO.in1.val >> (pin - 32)) & 1;
    }
}

// -------------------- ISR --------------------
void IRAM_ATTR recordEdge(int index, bool level) {
    auto &pin = oneWire[index];
    int nextHead = (pin.head + 1) % OneWirePin::MAX_EDGES;
    if (nextHead != pin.tail) { // prevent overflow
        pin.edgeQueue[pin.head].time  = micros();
        pin.edgeQueue[pin.head].level = level;
        pin.head = nextHead;
        IntPin = index;
    }
}

void Search_Rom(int index, int level) {
    OneWirePin &pin = oneWire[index];
    uint8_t pinNum = oneWirePins[index];
    bool bitValue = false;

    if (!pin.compliment) {
        pin.state = OneWirePin::READ_DETECTED;  
        pin.compliment = true;
        bitValue = pin.romBitArray[pin.RomArrayIndex].bit; 
    } else {                                                 
        pin.state = OneWirePin::READ_BYTE;
        pin.compliment = false;
        pin.readBitReady = false; 
        bitValue = pin.romBitArray[pin.RomArrayIndex].complement;                          
    } 
    
    if (bitValue == false) {
        if (pinNum < 32) {
            GPIO.out_w1tc = (1 << pinNum);
        } else {
            GPIO.out1_w1tc.val = (1 << (pinNum - 32));
        }
    }
    setPinHigh(index, T_READ_BIT_DURATION);                      
} 

void IRAM_ATTR oneWireISRWrapper(void* arg) {
    int index = (int)(intptr_t)arg;    // recover pin index
   
    OneWirePin &pin = oneWire[index];
 
    int level = fastReadPin(oneWirePins[index]);
 
    if (pin.state == OneWirePin::READ_DETECTED && pin.Cmdstate == OneWirePin::SEARCH_ROM && level == LOW) {
          Search_Rom(index, level);  
    } else {
    recordEdge(index, level);          // store edge in circular buffer   
    }
}

void IRAM_ATTR onROMTimer() {
    if (currentPinIndex == -1) return;
    uint8_t pin = oneWirePins[currentPinIndex];

    if (pin < 32) {
        GPIO.out_w1ts = (1 << pin);
    } else {
        // Correct register for pins 32 and 33
        GPIO.out1_w1ts.val = (1 << (pin - 32));
    }
}

void IRAM_ATTR onhouseTimer() {
    uint8_t pin = oneWirePins[houseIndex];
         // Sample the line for main loop to handle
    if (oneWire[houseIndex].state == OneWirePin::READ_BYTE || 
        oneWire[houseIndex].state == OneWirePin::COMMAND_READING) {
        
        if (pin < 32) {
            oneWire[houseIndex].readBit = (GPIO.in >> pin) & 1;
        } else {
            oneWire[houseIndex].readBit = (GPIO.in1.val >> (pin - 32)) & 1;
        }
        oneWire[houseIndex].readBitReady = true;
    }
}

void setHousetime(int index, int duration){
   houseIndex = index;
   // Activate house timer 
    timerWrite(houseTimer, 0);
    timerAlarmWrite(houseTimer, duration, false);
    timerAlarmEnable(houseTimer);
 //   Serial.printf("House timer pin %i delay %i\n",index, duration);
}

void setPinHigh(int index, int duration){
     currentPinIndex = index;
    // Activate timer ISR to set the line high if low
    timerWrite(romTimer, 0);
    timerAlarmWrite(romTimer, duration, false);
    timerAlarmEnable(romTimer); 
 //   Serial.printf("Set pin %i high %i\n",index, duration);
}

//---------------------Process command received from mater --------------
void commandReceived(int index){
   
    auto &pin = oneWire[index];

        switch (pin.command) {

                case 0x33: { // READ ROM                  
                    Serial.printf("Pin %d: READ-ROM not implemented yet\n", index, pin.command);
                    pin.Cmdstate = OneWirePin::READ_ROM;
                    pin.state = OneWirePin::IDLE;
                }
                break;

                case 0x55: { // ---------- MATCH ROM ----------
                    Serial.printf("Pin %d: MATCH-ROM not implemented yet\n", index);
                        pin.Cmdstate = OneWirePin::MATCH_ROM;
                    pin.state = OneWirePin::IDLE;
                    }
                    break;
                
                case 0xCC: { // ---------- SKIP ROM ----------
                    Serial.printf("Pin %d: MATCH-ROM not implemented yet\n", index);
                    pin.Cmdstate = OneWirePin::SKIP_ROM;
                    pin.state = OneWirePin::IDLE;
                     }
                    break;

                case 0xF0:  {// ---------- SEARCH ROM ----------
                 //  Serial.printf("Pin %d: SEARCH-ROM 0x%02X \n", index, pin.command);
                    pin.Cmdstate = OneWirePin::SEARCH_ROM;
                    pin.state = OneWirePin::READ_DETECTED;
                    pin.compliment = false;    // we going to send the first bit not the compliment
                    pin.RomArrayIndex = 0;  
                    }
                    break;

                default: {
                    Serial.printf("Pin %d: Unsupported command 0x%02X\n", index, pin.command);
                    pin.Cmdstate = OneWirePin::NONE;
                    pin.state = OneWirePin::IDLE;
                }
                break;
            }
              
}

void readRom (int index){
    auto &pin = oneWire[index]; 
}

/************ Check the byte received from master indicate if rom dump can continue *********** */
void byteReceived(int index, uint8_t level){
 
    auto &pin = oneWire[index];

       if (pin.readBit == !pin.romBitArray[pin.RomArrayIndex].bit)   
       {
        pin.state = OneWirePin::IDLE;
        pin.RomArrayIndex = 0;
        pin.Cmdstate = OneWirePin::NONE;
        pin.compliment   = false; 
        Serial.printf("ResetSearchRom I/O = %i terminated \n",oneWirePins[index]); 
        }
            
        if (pin.RomArrayIndex< 63) {
                pin.RomArrayIndex++;                 
        } else 
        {   // Finished sending full ROM   
             pin.compliment   = false; 
            pin.Cmdstate = OneWirePin::NONE;       
        }   
    pin.state = OneWirePin::READ_DETECTED;
    pin.readBitReady = false;       
}

void skipRom (int index){
 auto &pin = oneWire[index];
 pin.state = OneWirePin::IDLE;
}

void matchRom (int index){
 auto &pin = oneWire[index];
  pin.state = OneWirePin::IDLE;
}

void sendPresence (int index)
{
  auto &pin = oneWire[index];  
    pin.state = OneWirePin::PRESENCE_PULSE;
    ets_delay_us(T_PRESENCE_DELAY);     
    pin.command = 0;
    pin.bitIndex = 0;
    gpio_set_level((gpio_num_t)oneWirePins[index], LOW);   // pull low
    setPinHigh(index,T_PRESENCE_DURATION);
}

// -------------------- Process rising and falling status per pin --------------------
void processEdges(int index) {
    auto &pin = oneWire[index];

  while (pin.tail != pin.head) {
        // Copy fields manually (fix volatile issue)
        Edge e;
        e.time  = pin.edgeQueue[pin.tail].time;
        e.level = pin.edgeQueue[pin.tail].level;
        pin.tail = (pin.tail + 1) % OneWirePin::MAX_EDGES;

        switch (pin.state) {
            case OneWirePin::IDLE:
                if (!e.level) { // falling edge => possible reset
                    pin.resetFalling = e.time;
                    pin.state = OneWirePin::RESET_DETECTED;
                }
                break;
            
            case OneWirePin::RESET_DETECTED:
                if (e.level) { // rising edge => end of reset or read pulse
                 //  Serial.println(" reset");
                    pin.resetRising = e.time;
                    uint32_t width = pin.resetRising - pin.resetFalling;
                    if (width >= T_RESET_MIN && width <= T_RESET_MAX) {
                       // we have detected reset now we send presence pulse wait for 
                       // 30 usec the take line down for 120usec and release          
                         sendPresence(index);      // sending presence  pluse and get ready to read command
                      } else {
                      pin.state = OneWirePin::IDLE; // ignore false reset                     
                    }
                }
            break;
  
            case OneWirePin::PRESENCE_PULSE: 
             if (!e.level) { 
                //     Serial.println("Pres");
            } else {
                 pin.state = OneWirePin::COMMAND_READING;
            }
            break;

            case OneWirePin::COMMAND_READING: 
                    if (!e.level) 
                    { // falling edge = first bit start                     
                        pin.lastBitTime = e.time;                                                                      
                        pin.readBitReady = false;                                 
                        setHousetime(index,T_BIT_SAMPLE);
                    } else {                         
                              pin.presenceRising = e.time;
                          // we may get more than wire reset pulses this bit checks for that
                            uint32_t width =  pin.presenceRising - pin.lastBitTime ;
                           if (width >= T_RESET_MIN && width <= T_RESET_MAX) {
                                      sendPresence(index);  
                            }
                    }
            break;

           case OneWirePin::READ_BYTE:   
                if (!e.level) { // falling edge = first bit start
                         pin.readBitReady = false;                  
                         setHousetime(index,T_BIT_SAMPLE-5);
                      //   if (pin.RomArrayIndex == 63) pin.state = OneWirePin::IDLE;
                } 

            break;        
 
            case OneWirePin::READ_DETECTED:
                if (!e.level) {       

                        switch (pin.Cmdstate) {
                            case OneWirePin::SKIP_ROM:
                                skipRom (index);
                            break;
                        
                            case OneWirePin::READ_ROM:
                                readRom(index);
                            break;
                        
                            case OneWirePin::MATCH_ROM:
                                matchRom (index);

                            break;
                        
                            case OneWirePin::SEARCH_ROM:
                            break;

                            case OneWirePin::NONE:
                            {
                                Serial.printf("SearchRom for pin %i done! \n",index);
                                pin.state = OneWirePin::IDLE;
                            }
                            break;

                            default:
                                 Serial.println("Incorrect CmdState received !!");
                            break;
                        }
                   }  
               break;
            
              default:
               Serial.println("Unknown OneWirePin::?????? ");
              break;  // Ignore any other states; ROM read is ISR-only
          }
      }
}

void outDAC (uint8_t Dac, uint8_t Voltage){
      dacWrite(Dac, Voltage);
}

void printRegisterStatus() {
  //  uint32_t xtal_reg = READ_PERI_REG(RTC_IO_XTAL_32K_PAD_REG);
    Serial.printf("Current  RTCIO_XTAL_32K_PAD_REG (0x008C): 0x%08X\n", READ_PERI_REG(RTC_IO_XTAL_32K_PAD_REG));
    Serial.printf("Current  RTCIO_PAD_DAC2_REG (0x0088): 0x%08X\n", READ_PERI_REG(RTC_CNTL_DIG_ISO_REG));
    Serial.printf("Current  RTCIO_SENSOR_PADS_REG (0x007C): 0x%08X\n", READ_PERI_REG(RTC_CNTL_REG));
    Serial.printf("Current  RTC_CNTL_EXT_XTL_CONF_REG (0x005C): 0x%08X\n", READ_PERI_REG(RTC_CNTL_EXT_XTL_CONF_REG));
    Serial.printf("Current  RTCIO_PAD_DAC1_REG (0x0084): 0x%08X\n", READ_PERI_REG(RTC_CNTL_DIG_PWC_REG));

    Serial.printf("Current GPIO_PIN32_REG (0x%08X): 0x%08X\n", GPIO_PIN32_REG, READ_PERI_REG(GPIO_PIN32_REG));
    Serial.printf("Current GPIO_PIN33_REG (0x%08X): 0x%08X\n", GPIO_PIN33_REG, READ_PERI_REG(GPIO_PIN33_REG));
    Serial.printf("Current GPIO_PIN19_REG (0x%08X): 0x%08X\n", GPIO_PIN19_REG, READ_PERI_REG(GPIO_PIN19_REG));
    
    Serial.printf("GPIO 32 Output Select Register: 0x%08X\n", READ_PERI_REG(GPIO_FUNC32_OUT_SEL_CFG_REG));
    Serial.printf("GPIO 33 Output Select Register: 0x%08X\n", READ_PERI_REG(GPIO_FUNC33_OUT_SEL_CFG_REG));
    Serial.printf("GPIO 19 Output Select Register: 0x%08X\n\n", READ_PERI_REG(GPIO_FUNC19_OUT_SEL_CFG_REG));

}

void NVM_values(){
   NVS.begin();     // initialise the NVM library   
   // Recalling operational values in NVM as a blob
   // Syntax: NVS.getBlob(key, destination_pointer, size_in_bytes)
   NVS.getBlob("relay_cal", (uint8_t*)relayCal, sizeof(relayCal));   
}

void StoreInNvm() {
   Serial.println("\nStoring data");
    // 1. Open the NVS storage namespace
   NVS.begin();    
   // 2. Write the array as raw bytes to the "relay_cal" key
   // Syntax: NVS.setBlob(key, source_pointer, size_in_bytes)
   bool success = NVS.setBlob("relay_cal", (uint8_t*)relayCal, sizeof(relayCal)); 
   if (!success) {
      Serial.println("Error: Relay Calibrate array failed to save.");
   }
  
}

void display_Volt(int i){
    
    float busVoltage = ina3221.getBusVoltage(i);
    // ================================
    // 🔥 THIS IS THE KEY CHANGE
    // ================================
    float vshunt = ina3221.getShuntVoltage(i);   // relay drop in VOLTS
    float voltage_mV = vshunt * 1000.0; 
    Serial.printf(": Bus = %.4f V",busVoltage);
    Serial.printf(" | Shunt  = %.3f mV\n",voltage_mV);

}

void checkSerial(char val){

  if (val > '0') {
   
    float current = ina3221.getCurrentAmps(1) * 1000.0;
    
    switch(val){
        case '1' ... '8' :
               relayCal[val-49] = current;
               Serial.printf(" Relay port %c  I = %.3F mA",val, relayCal[val-49]);
               display_Volt(1);
               Calibrateupdate = true;
        break;
        case 't':
            if (!optoToggle) {
            digitalWrite((gpio_num_t) OPTO1, LOW);
            digitalWrite((gpio_num_t) OPTO2, LOW);
            digitalWrite((gpio_num_t) OPTO3, LOW);
            digitalWrite((gpio_num_t) OPTO4, LOW); 
            Serial.println("\nOpto couples output low !!");
            optoToggle = true;
             } else {
                 optoToggle = false;
                 digitalWrite((gpio_num_t) OPTO1, HIGH);
                 digitalWrite((gpio_num_t) OPTO2, HIGH);
                 digitalWrite((gpio_num_t) OPTO3, HIGH);
                 digitalWrite((gpio_num_t) OPTO4, HIGH);
                 Serial.println("Opto couples output high !!\n");
                 }
        break;
        case 'g':
               Serial.printf("\n GPIO8: level %d\n", digitalRead(IO_1));
               Serial.printf(" GPIO9: level %d\n", digitalRead(IO_2));
        break;
        case 'd':
            switch (dacCount) {
                case 0:
                    outDAC(DAC2,30);
                    outDAC(DAC1,230);
                    Serial.printf("\nDAC2 output %.3fV DAC1 output %.3fV\n",(30.0/255.0*3.3),(230.0/255.0*3.3));
                break;
                case 1:
                    outDAC(DAC2,90);
                    outDAC(DAC1,170);
                    Serial.printf("DAC2 output %.3fV DAC1 output %.3fV\n",(90.0/255.0*3.3),(170.0/255.0*3.3));
                break;
                case 2:
                    outDAC(DAC2,170);
                    outDAC(DAC1,90);    
                    Serial.printf("DAC2 output %.3fV DAC1 output %.3fV\n",(170.0/255.0*3.3),(90.0/255.0*3.3));        
                break;
                case 3:
                    outDAC(DAC2,230);
                    outDAC(DAC1,30);               
                     Serial.printf("DAC2 output %.3fV DAC1 output %.3fV\n",(230.0/255.0*3.3),(30.0/255.0*3.3));                   
                break;
                default:
                break;
            }

            if (dacCount < 3) dacCount++;
                else dacCount = 0;
        break;
        case 'c':
              for ( int i = 0; i < NumPins; i++){
                Serial.printf("NVM Calibrated Current Value for Relay input %i = %.3fmA\n",i,relayCal[i]);
                }
                Serial.println();
        break;
        case 'q': 
             if (Calibrateupdate) {
                 Calibrateupdate = false;
                StoreInNvm();
                for ( int i = 0; i < NumPins; i++){
                 Serial.printf("Updated Calibrated Current Value for Relay %i = %.3fmA\n",i,relayCal[i]);               
                                    }
                 }
        break;        
        default : Serial.println("Select valid option");
        break;
    }
}
}

void readINA3221(int index){
    for (uint8_t i = 0; i < 2; i++) {

    float busVoltage = ina3221.getBusVoltage(i);

    // ================================
    // 🔥 THIS IS THE KEY CHANGE
    // ================================
    float vshunt = ina3221.getShuntVoltage(i);   // relay drop in VOLTS
    float voltage_mV = vshunt * 1000.0;

    // optional: still keep current for reference only
    float current = ina3221.getCurrentAmps(i) * 1000.0;

    // OPTIONAL derived resistance (only valid if current is stable)
    float resistance = 0.0;
    if (current > 0.001) {
      resistance = vshunt / (current / 1000.0);
    }

   if(i == 0) Serial.printf("24V      ");
    else      Serial.printf("Relay %i ", index);
    Serial.printf(": Bus = %.4f V",busVoltage);
   // Serial.printf(" | Shunt  = %.3f mV",voltage_mV);
    Serial.printf(" | I = %.3F mA", current);

    if (i == 0) { 
        if (current > 43) Serial.print("  --> Moat current FAIL (> 43mA)"); 
          
    } else {
        Serial.printf(" Calibrated I = %.3f mA",relayCal[index]);
        if (current > relayCal[index]) Serial.print("  --> RELAY FAIL (contact > 2Ω)");
           else if (current < 10)  Serial.print("  --> RELAY OPEN / BAD CONTACT");       
    }
     Serial.println();
    }
}
 
void relayInput(int i){
    delay(1500);
    readINA3221(i);
}

void SETUP_GPIO_PINS() {
 //   printRegisterStatus();
    // Install ISR service (Ignore error if already installed)
    esp_err_t err = gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        Serial.printf("ISR Service Failed: %s\n", esp_err_to_name(err));
    }

    // 3. Configure each 1-Wire pin
    for (int i = 0; i < NumPins; i++) {
        gpio_num_t pin = (gpio_num_t)oneWirePins[i];
        gpio_reset_pin(pin); // Reset to default

        // Clean up pins 32/33 specifically after reset
  /*     if (pin == 32 || pin == 33) {
            CLEAR_PERI_REG_MASK(RTC_IO_XTAL_32K_PAD_REG, (pin == 32 ? RTC_IO_X32P_MUX_SEL : RTC_IO_X32N_MUX_SEL));
            uint32_t touchReg = (pin == 32) ? RTC_IO_TOUCH_PAD9_REG : RTC_IO_TOUCH_PAD8_REG;
            CLEAR_PERI_REG_MASK(touchReg, RTC_IO_TOUCH_PAD9_XPD | RTC_IO_TOUCH_PAD9_TIE_OPT);
        }
*/ 
        pinMode(pin, OUTPUT_OPEN_DRAIN);
        gpio_pullup_en(pin);
        gpio_set_level(pin, HIGH);
        gpio_set_intr_type(pin, GPIO_INTR_ANYEDGE);
        gpio_isr_handler_add(pin, oneWireISRWrapper, (void*)(intptr_t)i);
}
 //   printRegisterStatus();
}

void Setup_Timers(){
   // ROM Timer (already working)
    romTimer = timerBegin(1, 80, true);
    timerAttachInterrupt(romTimer, &onROMTimer, true);
    timerAlarmDisable(romTimer);

    // House Timer (one-shot)
    houseTimer = timerBegin(2, 80, true); // Timer 2, prescaler 80 → 1 µs tick
    timerAttachInterrupt(houseTimer, &onhouseTimer, true); // Attach once
    timerAlarmDisable(houseTimer); // Make sure it is disabled initially 
 }

void Load_rom_bits(){
    
    generateRandomIButtonROMs();

     for (int i = 0; i < NumPins; i++) { 
    loadRomBits(i); }
}

void setupINA3221() {
  
  // Initialize the INA3221
  if (!ina3221.begin(0x40, &Wire)) { // can use other I2C addresses or buses
    Serial.println("Failed to find INA3221 chip");
    while (1) delay(10);
  }

   float y = 0.0;
  // Set shunt resistances for all channels to 0.05 ohms
  for (uint8_t i = 0; i < 3; i++) {
      ina3221.setShuntResistance(i, 0.05);     
  }

  ina3221.setAveragingMode(INA3221_AVG_4_SAMPLES);
  ina3221.setBusVoltageConvTime(INA3221_CONVTIME_1MS);
/*
  ina3221.setCriticalAlertThreshold(0, 0.325);
  Serial.print("Critical threshold for channel 1: "); 
  Serial.print(ina3221.getCriticalAlertThreshold(0));
  Serial.println(" A");

  ina3221.setCriticalAlertThreshold(1, 1.0);
  Serial.print("Critical threshold for channel 2: "); 
  Serial.print(ina3221.getCriticalAlertThreshold(1));
  Serial.println(" A");

  ina3221.setCriticalAlertThreshold(2, 1.5);
  Serial.print("Critical threshold for channel 3: "); 
  Serial.print(ina3221.getCriticalAlertThreshold(2));
  Serial.println(" A");
  */
}

void setupInput(){
       // Using INPUT_PULLUP is recommended to avoid "floating" states
    pinMode(IO_1, INPUT_PULLUP); 
    pinMode(IO_2, INPUT_PULLUP); 
    pinMode((gpio_num_t)OPTO1,OUTPUT);
    pinMode((gpio_num_t)OPTO2,OUTPUT);
    pinMode((gpio_num_t)OPTO3,OUTPUT);
    pinMode((gpio_num_t)OPTO4,OUTPUT);
    digitalWrite((gpio_num_t) OPTO1, HIGH);
    digitalWrite((gpio_num_t) OPTO2, HIGH);
    digitalWrite((gpio_num_t) OPTO3, HIGH);
    digitalWrite((gpio_num_t) OPTO4, HIGH);
}

void displayMenu(){
   Serial.println("\n******  Available Menu Options ******\n");
   Serial.println("1..8: Calibrate relay input");
   Serial.println("   c: Show relay input calibrated values");
   Serial.println("   d: DAC outputs LOW MID HIGH");
   Serial.println("   g: Read GPIO input pins");
   Serial.println("   t: Toggle opto couplers output");   
   Serial.println("   q: Store calibrated values\n");
}

void Check_calibration(){
    
  char val = readSerialChar(); // Returns 0 if nothing is pressed
        
        if (val != 0) {
            // A key was actually pressed!
            checkSerial(val); 
        }
    
}


//  https://johnwargo.com/posts/2023/arduino-running-tasks-on-multiple-cores/
TaskHandle_t Task0;


void Task0code(void* pvParameters) {
    for (;;) {
    Check_calibration(); 
    vTaskDelay(1);
  }
}

void taskHandle(){
      //create a task that executes the Task0code() function, with priority 1 and executed on core 0
  xTaskCreatePinnedToCore(Task0code, "Task0", 4096, NULL, 1, &Task0, 0);
}


void setup() {
    Serial.begin(115200);
    while (!Serial);

// Stop Wi-Fi
    esp_wifi_stop();
    esp_wifi_deinit();

// Stop Bluetooth
    esp_bt_controller_disable();
    esp_bt_controller_deinit();

// Setup the INA3221 current sensor  
     setupINA3221();

// load variables stored in NVM
    NVM_values(); 
    displayMenu();
    taskHandle();

// Display menu options      
//    Check_calibration(); 

// Setup GPIO pins
    SETUP_GPIO_PINS();
 
    setupInput();
    Setup_Timers();
    Load_rom_bits();

    Serial.println("\n**** Moat Ibutton emulator test start !! ****");    
}

// -------------------- Main loop --------------------
void loop() {
    uint32_t now = micros();
    
    for (int i = 0; i < NumPins; i++) {
    
    auto &pin = oneWire[i];

        if (pin.tail != pin.head) { 
            processEdges(i);       
        }
            
            if (pin.state == OneWirePin::IDLE && flip){
            flip = false; 
            }

            if (pin.state == OneWirePin::READ_BYTE) {
                if (pin.readBitReady) {   
                 byteReceived(i, pin.readBit);        
                }
            } 

            if (pin.state == OneWirePin::COMMAND_READING) {
                if (pin.readBitReady) {                
                        bool bit = (pin.readBit != 0);
                        pin.command |= (bit << pin.bitIndex);
                        pin.bitIndex++;
                        if (pin.bitIndex >= 8) {
                            commandReceived(i);                   
                        }   
                pin.readBitReady = false;               
                }
            }

            if ((pin.state == OneWirePin::READ_DETECTED) & (pin.Cmdstate == OneWirePin::NONE)){
               Serial.printf("Ibutton  %i done \n",i);
                relayInput(i);
                pin.state = OneWirePin::IDLE;
            }  
}      
}