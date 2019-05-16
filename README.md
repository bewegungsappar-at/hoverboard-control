# Cloning and build environment
This repo uses submodules, make sure to clone with `git clone --recurse-submodules ...` or initialize the submodules afterwards with `git submodule update --init --recursive`


# Paddelec control
ESP32 based control board to read Gametraks attached to a paddle. Provide input via serial to control a hoverboard with custom firmware.
See https://git.hacknology.de/projekte/bewegungsappar.at (mirrored on https://github.com/p-h-a-i-l/hoverboard-firmware-hack).
Additional feature to work as a gateway for Wii Nunchuk (I²C)

Used IDE: platform.io

# ESP32-Serial-Bridge
functionality based on https://github.com/AlphaLima/ESP32-Serial-Bridge
Transparent WiFi (TCP, UDP) to all three UART Bridge, supports both AP and STATION WiFi modes.

## Accesspoint
IPAdress: 192.168.4.1
AP SSID: paddelec
AP Password: paddelec
Used Ports:
192.168.4.1:8880  <-> COM0
192.168.4.1:8881  <-> COM1
192.168.4.1:8882  <-> COM2

## Hardware
Pinning
COM0 Rx <-> GPIO3
COM0 Tx <-> GPIO1
COM1 Rx <-> GPIO15
COM1 Tx <-> GPIO4
COM2 Rx <-> GPIO16
COM2 Tx <-> GPIO17


===============================================================

In some cases the memorylayout is to small for this scetch.
If you face this problem you can either disable Bluetooth by removing #define BLUETOOTH in config.h or change the partition size as described here: https://desire.giesecke.tk/index.php/2018/04/20/change-partition-size-arduino-ide/



=======================================

#Pinning Adarfuit Feather Huzzah32

#UART to motor control
SERIAL2_RXPIN 16            // receive Pin UART2
SERIAL2_TXPIN 17            // transmit Pin UART2

#I2C (Nunchuk)
NUNCHUK_VCCPIN    14      // Pin used to supply Power
I2C SCL            22
I2C SDA            23   (21 on NodeMCU boards)


GAMETRAK1_RPIN      39      // wire length Pin Gametrak 1
GAMETRAK1_PHIPIN    36      // horizontal angle Pin Gametrak 1
GAMETRAK1_THETAPIN  34      // vertical angle Pin Gametrak 1

GAMETRAK2_VCCPIN    27      // Pin used to supply Power