// rs01.h

#define HW_TYPE     "IRMACOS RS01 v1.7d"
#define FW_VERSION  "1.0"

#define RXPin        17                     // Serial485 Receive pin
#define TXPin        16                     // Serial485 Transmit pin
#define SERIAL_COMMUNICATION_CONTROL_PIN 4  // RS485 Transmission control pin
#define RS485_TX_PIN_VALUE HIGH
#define RS485_RX_PIN_VALUE LOW

#define LED0 23                     		// RED LED is on GPIO23
#define LED1 5                      		// GREEN LED in on GPIO 5

// OPTOisolated INput pins
#define OPTO_IN0		33
#define OPTO_IN1		25
#define OPTO_IN2		26
#define OPTO_IN3		27

// GPIO INput pins
#define GPIO_IN0		0						// BOOT / IO) button

// ANAlog INput pins
#define ANA_IN0		36
#define ANA_IN1		34
#define ANA_IN2		39
#define ANA_IN3		35

#define PERC_FLUCTUATION  5.0
