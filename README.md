# ESP32-Serial-Bridge

Transparent WiFi (TCP, UDP) to all three UART Bridge, supports both AP and STATION WiFi modes. The .ino file is the code for the ESP32. Use Arduino IDE for ESP32 to compile and upload it to the ESP32.
I made this project in order to connect Flight equipment devices devices like (Radio, Vario FLARM), to a Flight Computer (Kobo, Smartphones etc.),  but it is not limited to that. You can use it wherever you want, but on your own risk. Read license file for more details.
Accesspoint                                                    
IPAdress: 192.168.4.1                                           
AP SSID: LK8000                                                   
AP Password: Flightcomputer                                       
Used Ports:                                                                                                          
192.168.4.1:8880  <-> COM0                                     
192.168.4.1:8881  <-> COM1                                     
192.168.4.1:8882  <-> COM2                                     

===============================================================

In some cases the memorylayout is to small for this scetch.
If you face this problem you can either disable Bluetooth by removing #define BLUETOOTH in config.h or change the partition size as described here: https://desire.giesecke.tk/index.php/2018/04/20/change-partition-size-arduino-ide/


# Hardware
Pinning                                                                                     
COM0 Rx <-> GPIO3                                                                               
COM0 Tx <-> GPIO1                                                                                 
COM1 Rx <-> GPIO15                                                                               
COM1 Tx <-> GPIO4                                                                              
COM2 Rx <-> GPIO16                                                                               
COM2 Tx <-> GPIO17                                                                              
