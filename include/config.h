#pragma once

// ############################### Settings ###############################
// Whenever possible, settings should be managed in platformio.ini!

/****************************** System *********************************/

#define MULTITASKING

/****************************** Debug *********************************/

//#define DEBUG_OLED
//#define TTGO                  // TODO: Rename, only used to identify other OLED display

//#define DEBUG_CONSOLE         // Debug through Serial Interface
//#define DEBUG_PLOTTER         // Plot Values through Serial Interface
                                // Get Listener: https://github.com/devinaconley/arduino-plotter/wiki/Installation-and-Quickstart


/*************************** Output Method *****************************/
// Chose only one!

//#define OUTPUT_BINARY         // Binary Protocol, no checksum, easy to loose syc, very dangerous and unstable
//#define OUTPUT_BINARY_CRC     // Binary Protocol, with checksum, easy to loose syc and therefore unstable
//#define OUTPUT_PROTOCOL       // Binary Protocol, very powerful but still Alpha
//#define OUTPUT_ESPNOW         // Relay PWM and Steer through ESPnow to Slave
  //#define ESPNOW_PREFIX "CH1"   // ESPNOW "Channel" encoded in SSID
//  #define ESPNOW_PEERMAC {0x30, 0xAE, 0xA4, 0x02, 0x7E, 0x98} // Paddelec WROOM
//  #define ESPNOW_PEERMAC {0xB4, 0xE6, 0x2D, 0xB2, 0x79, 0x3D} // Paddel TTGO
//  #define ESPNOW_PEERMAC {0x30, 0xAE, 0xA4, 0x26, 0x86, 0x68} // Psycho Board
  #define ESPNOW_PEERMAC {0x30, 0xAE, 0xA4, 0x26, 0x6A, 0xD4} // Psycho Remote


/*
TTGO1 Paddel
AP Config Success. Broadcasting with AP: PADDELEC:B4:E6:2D:B2:79:3D
STA MAC: B4:E6:2D:B2:79:3D
AP MAC: B4:E6:2D:B2:79:3E

Paddelec
AP Config Success. Broadcasting with AP: PADDELEC:30:AE:A4:02:7E:98
STA MAC: 30:AE:A4:02:7E:98
AP MAC: 30:AE:A4:02:7E:99

psycho featherboard Lastenwagen
AP Config Success. Broadcasting with AP: PSYCHO:30:AE:A4:26:86:68
STA MAC: 30:AE:A4:26:86:68
AP MAC: 30:AE:A4:26:86:69

psycho feather remote
AP Config Success. Broadcasting with AP: PSYCHO:30:AE:A4:26:6A:D4
STA MAC: 30:AE:A4:26:6A:D4
AP MAC: 30:AE:A4:26:6A:D5
*/


#define MOTORINPUT_PERIOD   30  // Update Motor each xx milliseconds

/******************************* Wifi **********************************/

//#define WIFI
#ifdef WIFI
#include <esp_wifi.h>

#define OTA_HANDLER

//#define WIFI_TRYSTA   // Try to connect to network on boot

#define WIFI_SSID       "paddelec"
#define WIFI_PWD        "bewegungsappar.at"
#define WIFI_IP         192, 168, 4, 1
#define WIFI_NETMASK    255, 255, 255, 0
#endif


/***************************** Serial ********************************/

//#define NUM_COM   3                 // total number of COM Ports
//#define DEBUG_COM 0                 // debug output to COM0
//#define MOTOR_COM 2                 // motor control output to COM2
/****  COM Port 0 ****/
//#define UART_BAUD0 115200           // Baudrate UART0
#define SERIAL_PARAM0 SERIAL_8N1      // Data/Parity/Stop UART0
//#define SERIAL0_RXPIN 3             // receive Pin UART0
//#define SERIAL0_TXPIN 1             // transmit Pin UART0
#define SERIAL0_TCP_PORT 8880         // Wifi Port UART0
/****  COM Port 1 ****/
//#define UART_BAUD1 9600             // Baudrate UART1
#define SERIAL_PARAM1 SERIAL_8N1      // Data/Parity/Stop UART1
//#define SERIAL1_RXPIN 15            // receive Pin UART1
//#define SERIAL1_TXPIN  2            // transmit Pin UART1
#define SERIAL1_TCP_PORT 8881         // Wifi Port UART1
/****  COM Port 2 ****/
//#define UART_BAUD2 19200            // Baudrate UART2
#define SERIAL_PARAM2 SERIAL_8N1      // Data/Parity/Stop UART2
//#define SERIAL2_RXPIN 16            // receive Pin UART2
//#define SERIAL2_TXPIN 17            // transmit Pin UART2
#define SERIAL2_TCP_PORT 8882         // Wifi Port UART2
//#define SERIAL2_VCCPIN 5            // Pin used as VCC
//#define SERIAL2_GNDPIN 0            // Pin used as GND

#define bufferSize 1024               // Buffer size used to exchange data between COM and telnet
#define MAX_NMEA_CLIENTS 4


/*************************** Input Method ******************************/

//#define INPUT_ESPNOW
  //#define ESPNOW_PREFIX "CH1"   // ESPNOW "Channel" encoded in SSID
//#define INPUT_BLE
//#define INPUT_PADDELEC                    // look at Paddelec.h for paddelec specific config options!
//#define INPUT_PADDELECIMU
//#define INPUT_NUNCHUK                     // look at ArduinoNunchuk.h for Nunchuk specific config options!
//#define INPUT_PLATOONING
//#define INPUT_IMU

// Only 2 Gametraks are possible, as the ESP32 ADC2 is not available when WIFI is used
/****  Gametrak 1 ****/
//#define GAMETRAK1_VCCPIN    25      // Pin used to supply Power
//#define GAMETRAK1_GNDPIN    26      // Pin used as GND
//#define GAMETRAK1_RPIN      33      // wire length Pin Gametrak 1
//#define GAMETRAK1_PHIPIN    35      // horizontal angle Pin Gametrak 1
//#define GAMETRAK1_THETAPIN  32      // vertical angle Pin Gametrak 1
//#define GAMETRAK1_PHI_REV      1    // Phi is inverted
//#define GAMETRAK1_THETA_REV    0    // Theta is inverted
/****  Gametrak 2 ****/
//#define GAMETRAK2_VCCPIN    27      // Pin used to supply Power
//#define GAMETRAK2_GNDPIN    26      // Pin used as GND
//#define GAMETRAK2_RPIN      32      // wire length Pin Gametrak 2
//#define GAMETRAK2_PHIPIN    35      // horizontal angle Pin Gametrak 2
//#define GAMETRAK2_THETAPIN  33      // vertical angle Pin Gametrak 2
//#define GAMETRAK2_PHI_REV      1    // Phi is inverted
//#define GAMETRAK2_THETA_REV    1    // Theta is inverted


// ############################### Automatic Defines ###############################


#ifdef OUTPUT_BINARY_CRC
  #define OUTPUT_BINARY
#endif

#ifndef ESPNOW_PREFIX
  #define ESPNOW_PREFIX "ESPNOW"
#endif

// ############################### VALIDATE SETTINGS ###############################

#if defined(OUTPUT_PROTOCOL) && defined(OUTPUT_BINARY)
  #error OUTPUT_PROTOCOL and OUTPUT_BINARY not allowed, both on same serial.
#endif

#if !defined(OUTPUT_PROTOCOL) && !defined(OUTPUT_BINARY) && !defined(OUTPUT_ESPNOW)
  #error no Output Method defined. Nothing will be done..
#endif

#if defined(INPUT_PADDELEC) && defined(INPUT_PADDELECIMU)
  #error INPUT_PADDELEC and INPUT_PADDELECIMU cannot be used simultaneously.
#endif

#if !defined(INPUT_BLE) && \
    !defined(INPUT_ESPNOW) && \
    !defined(INPUT_IMU) && \
    !defined(INPUT_NUNCHUK) && \
    !defined(INPUT_PADDELEC) && \
    !defined(INPUT_PADDELECIMU) && \
    !defined(INPUT_PLATOONING)
  #error no Input Method defined. What should I do?
#endif

#if defined(INPUT_BLE)
  #error INPUT_BLE not yet implmented. Also set define guard here.
#endif

#if defined(WIFI) && (defined(INPUT_ESPNOW) || defined(OUTPUT_ESPNOW))
  #error ESPnow is Wifi too. Cannot coexist.
#endif

#if defined(INPUT_ESPNOW) && defined(OUTPUT_ESPNOW)
  #error ESP now has to bei EITHER Master or Slave
#endif