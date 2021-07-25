# Odroid Go
Needs to be fully charged to work with all computers

# Paddle
Needs to be flashed via OTA.
Example: Startup Paddelec as Access Point, start paddle and connect on PC.
IPs should be:
- 192.168.4.1: Paddelec
- 192.168.4.2: Paddle
- 192.168.4.3: PC

A charging adapter is needed to charge the paddle

# Paddelec
## HB Firmware
Use control_usart2. Need to hold down power button during flashing

## ESP32
Need additional Serial adapter, hold boot0 button during start to enter bootloader.

# Wireshark
## HB Firmware
Use control_usart3_ADC.

## ESP32
Just make sure power is not connected during flashing
