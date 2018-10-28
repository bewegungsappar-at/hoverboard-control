#ifndef CONFIG_H
#define CONFIG_H


#include <WiFi.h>
#include <esp_wifi.h>

#define DEBUG
extern bool debug;

//#define BLE
//#define ESPNOMW
#define OLED


//#define WIFI
#ifdef WIFI
/*********************************************************************/
/****************************** Wifi *********************************/
/*********************************************************************/
#define OTA_HANDLER 

//#define WIFI_TRYSTA   // Try to connect to network on boot

#define WIFI_SSID       "paddelec"
#define WIFI_PWD        "bewegungsappar.at"
#define WIFI_IP         192, 168, 4, 1
#define WIFI_NETMASK    255, 255, 255, 0
#endif

/*********************************************************************/
/***************************** Serial ********************************/
/*********************************************************************/
#define NUM_COM   3                 // total number of COM Ports
#define DEBUG_COM 0                 // debug output to COM0
#define MOTOR_COM 2                 // motor control output to COM2
/*************************  COM Port 0 *******************************/
#define UART_BAUD0 115200           // Baudrate UART0
#define SERIAL_PARAM0 SERIAL_8N1    // Data/Parity/Stop UART0
#define SERIAL0_RXPIN 3             // receive Pin UART0
#define SERIAL0_TXPIN 1             // transmit Pin UART0
#define SERIAL0_TCP_PORT 8880       // Wifi Port UART0
/*************************  COM Port 1 *******************************/
#define UART_BAUD1 115200           // Baudrate UART1
#define SERIAL_PARAM1 SERIAL_8N1    // Data/Parity/Stop UART1
#define SERIAL1_RXPIN 15            // receive Pin UART1
#define SERIAL1_TXPIN  2            // transmit Pin UART1
#define SERIAL1_TCP_PORT 8881       // Wifi Port UART1
/*************************  COM Port 2 *******************************/
#define UART_BAUD2 19200            // Baudrate UART2
#define SERIAL_PARAM2 SERIAL_8N1    // Data/Parity/Stop UART2
#define SERIAL2_RXPIN 16            // receive Pin UART2
#define SERIAL2_TXPIN 17            // transmit Pin UART2
#define SERIAL2_TCP_PORT 8882       // Wifi Port UART2
//#define SERIAL2_VCCPIN 5            // Pin used as VCC
//#define SERIAL2_GNDPIN 0            // Pin used as GND

#define bufferSize 1024
#define MAX_NMEA_CLIENTS 4

/*********************************************************************/
/***************************** Control *******************************/
/*********************************************************************/
//#define PADDELEC                    // look at Paddelec.h for paddelec specific config options!
//#define NUNCHUCK                    // look at ArduinoNunchuck.h for Nunchuck specific config options!
//#define PLATOONING
#define MOTORINPUT_PERIOD   20      // Update Motor Input each xx milliseconds
#define IMU

// Only 2 Gametraks are possible, as the ESP32 ADC2 is not available when WIFI is used 
/*************************  Gametrak 1 ******************************/
//#define GAMETRAK1_VCCPIN    25      // Pin used to supply Power
//#define GAMETRAK1_GNDPIN    26      // Pin used as GND
#define GAMETRAK1_RPIN      33      // wire length Pin Gametrak 1
#define GAMETRAK1_PHIPIN    35     // horizontal angle Pin Gametrak 1
#define GAMETRAK1_THETAPIN  32      // vertical angle Pin Gametrak 1
#define GAMETRAK1_PHI_REV      1    // Phi is inverted
#define GAMETRAK1_THETA_REV    0    // Theta is inverted
/*************************  Gametrak 2 ******************************
#define GAMETRAK2_VCCPIN    27      // Pin used to supply Power
#define GAMETRAK2_GNDPIN    26      // Pin used as GND
#define GAMETRAK2_RPIN      32      // wire length Pin Gametrak 2
#define GAMETRAK2_PHIPIN    35      // horizontal angle Pin Gametrak 2
#define GAMETRAK2_THETAPIN  33      // vertical angle Pin Gametrak 2
#define GAMETRAK2_PHI_REV      1    // Phi is inverted
#define GAMETRAK2_THETA_REV    1    // Theta is inverted
*/

#endif