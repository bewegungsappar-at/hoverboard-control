#pragma once

typedef enum
{
    COMM_IN_UDP     = 0x00,
    COMM_IN_ESPNOW  = 0x01,
    COMM_IN_OTHER   = 0x02
} comm_input;

typedef enum
{
    COMM_OUT_UDP     = 0x00, // Send output via protocol over UDP
    COMM_OUT_ESPNOW  = 0x01,
    COMM_OUT_UART    = 0x02
} comm_output;

typedef struct
{
  comm_input input;
  comm_output output;
  char wifi_ssid[23];
  char wifi_pass[23];
} comm_settings;

extern comm_settings communicationSettings;

//////////////////////////////////////////////////////////
// macro types for different configurations and boards

#define   CFG_ODROIDGO             0
#define   CFG_ESPNOW_RELAY         1
#define   CFG_TTGO_IMU_WIFI        2
#define   CFG_TTGO_PADDELEC        3
#define   CFG_TESTRUN              4
#define   CFG_PADDELEC             5
#define   CFG_PADDLE               6
#define   CFG_WIRESHARK            7
#define   CFG_NUNCHUK_ESPNOW_RELAY 8
#define   CFG_PANZER              10


// Default config setting
#ifndef CONFIGURATION_SET
#   define CONFIGURATION_SET CFG_ODROIDGO
#endif
//////////////////////////////////////////////////////////


#if (CONFIGURATION_SET == CFG_ODROIDGO)
//#   define ESPNOW_PEERMAC 0x24,0x0A,0xC4,0xAF,0xC9,0xE0 // Woodenboard
//#   define ESPNOW_PEERMAC 0x30,0xAE,0xA4,0x26,0x26,0x14 // feather board with OLED
//#   define ESPNOW_PEERMAC 0x30,0xAE,0xA4,0xEE,0xC4,0x64 // wrover dev kit

#   define OTA_HANDLER
#   define DEBUG_PING
#   define ODROID_GO_HW
#   define INPUT_NUNCHUK
#   define NUNCHUK_VCCPIN 12
#   define NUNCHUK_SDAPIN 15
#   define NUNCHUK_SCLPIN 4
#   define DEBUG_CONSOLE
#endif


#if (CONFIGURATION_SET == CFG_ESPNOW_RELAY)
//#   define ESPNOW_PEERMAC 0x30,0xAE,0xA4,0x26,0x6A,0xD4 // feather esp remote
#   define ESPNOW_PEERMAC 0x30,0xAE,0xA4,0xE0,0x34,0x70 // odroid phail
#endif


#if (CONFIGURATION_SET == CFG_TTGO_IMU_WIFI)
#   define TTGO
#   define DEBUG_OLED

#   define INPUT_IMU
#   define IMU_CPIN 23
#   define IMU_ZPIN 25

#   define WIFI
#endif


#if (CONFIGURATION_SET == CFG_TTGO_PADDELEC)
#   define TTGO
#   define DEBUG_OLED
//    #define INPUT_NUNCHUK
#   define NUNCHUK_VCCPIN 16
#   define INPUT_PADDELECIMU
#   define IMU_CPIN 23
#   define IMU_ZPIN 25

//#   define ESPNOW_PREFIX "PADDELEC" // ESPNOW "Channel" encoded in SSID
#   define DEBUG_CONSOLE
#   define OTA_HANDLER
#endif

#if (CONFIGURATION_SET == CFG_TESTRUN)
#   define INPUT_TESTRUN
#endif

#if (CONFIGURATION_SET == CFG_WIRESHARK)
#   define OTA_HANDLER
#   define INPUT_NUNCHUK
#   define NUNCHUK_VCCPIN 18
#   define NUNCHUK_GNDPIN 19
//#   define DEBUG_CONSOLE
#endif

#if (CONFIGURATION_SET == CFG_PADDELEC)
#   define OTA_HANDLER
#endif

#if (CONFIGURATION_SET == CFG_PADDLE)
#   define OTA_HANDLER
#   define INPUT_PADDELECIMU         // look at Paddelec.h for paddelec specific config options!
#   define IMU_GNDPIN 14
#   define IMU_VCCPIN 32
#   define PADDELEC_STOPSWITCH_PIN1 A2
#   define PADDELEC_STOPSWITCH_PIN2 A1
#endif

#if (CONFIGURATION_SET == CFG_NUNCHUK_ESPNOW_RELAY)
#   define INPUT_NUNCHUK
#   define      NUNCHUK_VCCPIN 18
#   define      NUNCHUK_GNDPIN 19
//#   define ESPNOW_PEERMAC 0x30,0xAE,0xA4,0x26,0x6A,0xD4 // feather esp remote
//#   define ESPNOW_PEERMAC 0xB4,0xE6,0x2D,0xD4,0x29,0xD9 // odroid Nico
#   define ESPNOW_PEERMAC 0x30,0xAE,0xA4,0xE0,0x34,0x70 // odroid phail
#endif

#if (CONFIGURATION_SET == CFG_PANZER)
#   define OTA_HANDLER
#   define SERIAL2_GNDPIN 26              // Pin used as GND - Needed for psycho
#endif



// ############################### Settings ###############################

/****************************** Debug *********************************/

//#define DEBUG_OLED
//#define TTGO                  // TODO: Rename, only used to identify other OLED display
//#define ODROID_GO_HW         // TODO: odroid Settings..

//#define DEBUG_CONSOLE         // Debug through Serial Interface
//#define DEBUG_PLOTTER         // Plot Values through Serial Interface
                                // Get Listener: https://github.com/devinaconley/arduino-plotter/wiki/Installation-and-Quickstart
//#define DEBUG_PROTOCOL_OUTGOING_MARKUP // Write all outgoing Messages to debug com in human readable form
//#define DEBUG_PROTOCOL_MEASUREMENTS    // Write measured values to debug com
//#define debugESPNOW                    // Debug ESPnow

//#define DEBUG_PING                     // Send periodic Pings over protocol
//#define DEBUG_SPEED                      // Use closed loop speed control instead of pwm


/******************************* Wifi **********************************/

//#define WIFI
#ifdef WIFI
#include <esp_wifi.h>

#define OTA_HANDLER

//#define WIFI_TRYSTA   // Try to connect to network on boot

#define WIFI_IP         192, 168, 4, 1
#define WIFI_NETMASK    255, 255, 255, 0
#endif

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


#ifndef ESPNOW_PREFIX
  #define ESPNOW_PREFIX "ESPNOW" // ESPNOW "Channel" encoded in SSID
#endif

//  #define ESPNOW_PEERMAC 0x30,0xAE,0xA4,0x02,0x7E,0x98 // Paddelec WROOM
//  #define ESPNOW_PEERMAC 0xB4,0xE6,0x2D,0xB2,0x79,0x3D // Paddel TTGO
//  #define ESPNOW_PEERMAC 0x30,0xAE,0xA4,0x26,0x86,0x68 // Psycho Board
//  #define ESPNOW_PEERMAC 0x30,0xAE,0xA4,0x26,0x6A,0xD4 // feather board Remote
//  #define ESPNOW_PEERMAC 0x30,0xAE,0xA4,0x26,0x26,0x14 // feather board with OLED
//  #define ESPNOW_PEERMAC 0x24,0x0A,0xC4,0xAF,0xC9,0xE0 // Woodenboard
//  #define ESPNOW_PEERMAC 0xB4,0xE6,0x2D,0xD4,0x29,0xD9 // odroid Nico
//  #define ESPNOW_PEERMAC 0x30,0xAE,0xA4,0xE0,0x34,0x70 // odroid phail
//  #define ESPNOW_PEERMAC 0x30,0xAE,0xA4,0xEE,0xC4,0x64 // wrover dev kit

#ifndef MOTORINPUT_PERIOD
    #define MOTORINPUT_PERIOD   30  // Update Motor each xx milliseconds
#endif


/*************************** Input Method ******************************/

// Simulated Input
//#define INPUT_TESTRUN             // Cycle through some Motortestmodes (Sinus, constant speed..) Only for Test setup, can hurt you.

// IMU Based
//#define INPUT_IMU
//#define INPUT_PADDELECIMU         // look at Paddelec.h for paddelec specific config options!
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