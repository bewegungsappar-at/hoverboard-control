#pragma once

typedef enum
{
    COMM_CHAN_UDP,
    COMM_CHAN_UART,
    COMM_CHAN_NONE
} comm_channel_t;

typedef enum
{
    SYSCONF_IN_IMU,
    SYSCONF_IN_NONE,
    SYSCONF_IN_PADDLEIMU,
    SYSCONF_IN_NUNCHUK
} sysconfig_in_t;

typedef struct
{
  comm_channel_t chan_in;
  comm_channel_t chan_out;
  char wifi_ssid[23];
  char wifi_pass[23];
  sysconfig_in_t input;
} sysconfig_t;

extern sysconfig_t sysconfig;

//////////////////////////////////////////////////////////
// macro types for different configurations and boards

#define   CFG_ODROIDGO             0
#define   CFG_TTGO_IMU_WIFI        2
#define   CFG_TTGO_PADDELEC        3
#define   CFG_TESTRUN              4
#define   CFG_PADDELEC             5
#define   CFG_PADDLE               6
#define   CFG_WIRESHARK            7
#define   CFG_PANZER              10


// Default config setting
#ifndef CONFIGURATION_SET
#   define CONFIGURATION_SET CFG_ODROIDGO
#endif
//////////////////////////////////////////////////////////


#if (CONFIGURATION_SET == CFG_ODROIDGO)
#   define DEBUG_PING
#   define ODROID_GO_HW
#   define INPUT_NUNCHUK
#   define NUNCHUK_VCCPIN 12
#   define NUNCHUK_SDAPIN 15
#   define NUNCHUK_SCLPIN 4
#   define DEBUG_CONSOLE
#endif


#if (CONFIGURATION_SET == CFG_TTGO_IMU_WIFI)
#   define TTGO
#   define DEBUG_OLED

#   define INPUT_IMU
#   define IMU_CPIN 23
#   define IMU_ZPIN 25
#endif


#if (CONFIGURATION_SET == CFG_TTGO_PADDELEC)
#   define TTGO
#   define DEBUG_OLED
//    #define INPUT_NUNCHUK
#   define NUNCHUK_VCCPIN 16
#   define IMU_CPIN 23
#   define IMU_ZPIN 25
#   define DEBUG_CONSOLE
#endif

#if (CONFIGURATION_SET == CFG_TESTRUN)
#   define INPUT_TESTRUN
#endif

#if (CONFIGURATION_SET == CFG_WIRESHARK)
#   define INPUT_NUNCHUK
#   define NUNCHUK_VCCPIN 18
#   define NUNCHUK_GNDPIN 19
//#   define DEBUG_CONSOLE
#endif

#if (CONFIGURATION_SET == CFG_PADDELEC)
#endif

#if (CONFIGURATION_SET == CFG_PADDLE)
#   define IMU_GNDPIN 14
#   define IMU_VCCPIN 32
#   define PADDELEC_STOPSWITCH_PIN1 A2
#   define PADDELEC_STOPSWITCH_PIN2 A1
#endif


#if (CONFIGURATION_SET == CFG_PANZER)
#   define SERIAL2_GNDPIN 26              // Pin used as GND - Needed for psycho
#   define INPUT_NUNCHUK
#endif



// ############################### Settings ###############################

/****************************** Debug *********************************/

//#define DEBUG_OLED
//#define TTGO                  // TODO: Rename, only used to identify other OLED display
//#define ODROID_GO_HW         // TODO: odroid Settings..

//#define DEBUG_CONSOLE         // Debug through Serial Interface
//#define DEBUG_PROTOCOL_OUTGOING_MARKUP // Write all outgoing Messages to debug com in human readable form
//#define DEBUG_PROTOCOL_MEASUREMENTS    // Write measured values to debug com

//#define DEBUG_PING                     // Send periodic Pings over protocol
//#define DEBUG_SPEED                      // Use closed loop speed control instead of pwm


/***************************** Serial ********************************/
#ifndef NUM_COM
    #define NUM_COM   3                 // total number of COM Ports
#endif

#ifndef DEBUG_COM
    #define DEBUG_COM 0                 // debug output to COM0
#endif

#ifndef MOTOR_COM
    #define MOTOR_COM 2                 // motor control output to COM2
#endif

/****  COM Port 0 ****/

#ifndef UART_BAUD0
    #define UART_BAUD0 115200           // Baudrate UART0
#endif

#ifndef SERIAL_PARAM0
    #define SERIAL_PARAM0 SERIAL_8N1    // Data/Parity/Stop UART0
#endif

#ifndef SERIAL0_RXPIN
    #define SERIAL0_RXPIN 3             // receive Pin UART0
#endif

#ifndef SERIAL0_TXPIN
    #define SERIAL0_TXPIN 1             // transmit Pin UART0
#endif

#ifndef SERIAL0_TCP_PORT
    #define SERIAL0_TCP_PORT 8880       // Wifi Port UART0
#endif

/****  COM Port 1 ****/

#ifndef UART_BAUD1
    #define UART_BAUD1 9600             // Baudrate UART1
#endif

#ifndef SERIAL_PARAM1
    #define SERIAL_PARAM1 SERIAL_8N1    // Data/Parity/Stop UART1
#endif

#ifndef SERIAL1_RXPIN
    #define SERIAL1_RXPIN 9            // receive Pin UART1
#endif

#ifndef SERIAL1_TXPIN
    #define SERIAL1_TXPIN  10            // transmit Pin UART1
#endif

#ifndef SERIAL1_TCP_PORT
    #define SERIAL1_TCP_PORT 8881       // Wifi Port UART1
#endif

/****  COM Port 2 ****/

#ifndef UART_BAUD2
    #define UART_BAUD2 115200           // Baudrate UART2
#endif

#ifndef SERIAL_PARAM2
    #define SERIAL_PARAM2 SERIAL_8N1    // Data/Parity/Stop UART2
#endif

#ifndef SERIAL2_RXPIN
    #define SERIAL2_RXPIN 16            // receive Pin UART2
#endif

#ifndef SERIAL2_TXPIN
    #define SERIAL2_TXPIN 17            // transmit Pin UART2
#endif

#ifndef SERIAL2_TCP_PORT
    #define SERIAL2_TCP_PORT 8882       // Wifi Port UART2
#endif

//#define SERIAL2_VCCPIN 5              // Pin used as VCC
//#define SERIAL2_GNDPIN 0              // Pin used as GND

#ifndef bufferSize
    #define bufferSize 1024             // Buffer size used to exchange data between COM and telnet
#endif

#ifndef MAX_NMEA_CLIENTS
    #define MAX_NMEA_CLIENTS 4
#endif

#ifndef MOTORINPUT_PERIOD
    #define MOTORINPUT_PERIOD   30  // Update Motor each xx milliseconds
#endif


/*************************** Input Method ******************************/

// Simulated Input
//#define INPUT_TESTRUN             // Cycle through some Motortestmodes (Sinus, constant speed..) Only for Test setup, can hurt you.

// IMU Based
//#define INPUT_IMU
//#define IMU_GNDPIN 14
//#define IMU_VCCPIN 32
//#define IMU_CPIN 23
//#define IMU_ZPIN 25

// Nunchuk
//#define INPUT_NUNCHUK             // look at ArduinoNunchuk.h for Nunchuk specific config options!
//#define NUNCHUK_VCCPIN 18
//#define NUNCHUK_GNDPIN 19



// Platooning
//#define INPUT_PLATOONING
// Only 2 Gametraks are possible, as the ESP32 ADC2 is not available when WIFI is used
/****  Gametrak 1 ****/
//#define       GAMETRAK1_RPIN 33  // wire length Pin Gametrak 1      -      white  - Pin 3
//#define     GAMETRAK1_PHIPIN 35  // horizontal angle Pin Gametrak 1 - blue/yellow - Pin 2
//#define   GAMETRAK1_THETAPIN 32  // vertical angle Pin Gametrak 1   - red/orange  - Pin 4
//#define   GAMETRAK1_PHI_REV 1    // Phi is inverted
//#define GAMETRAK1_THETA_REV 0    // Theta is inverted
//#define     GAMETRAK1_VCCPIN 25  // Pin used to supply Power        - black/brown - Pin 5
//#define     GAMETRAK1_GNDPIN 26  // Pin used as GND                 -       green - Pin 1
/****  Gametrak 2 ****/
//#define     GAMETRAK2_PHIPIN 34   // horizontal angle Pin
//#define       GAMETRAK2_RPIN 39   // wire length Pin
//#define   GAMETRAK2_THETAPIN 36   // vertical angle Pin - PIN13 is ADC2, not functional with wifi
//#define   GAMETRAK2_PHI_REV 1     // Phi is inverted
//#define GAMETRAK2_THETA_REV 0     // Theta is inverted