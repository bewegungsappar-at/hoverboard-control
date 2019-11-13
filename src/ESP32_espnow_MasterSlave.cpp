/**
   ESPNOW - Basic communication - Master
   Date: 26th September 2017
   Author: Arvind Ravulavaru <https://github.com/arvindr21>
   Purpose: ESPNow Communication between a Master ESP32 and multiple ESP32 Slaves
   Description: This sketch consists of the code for the Master module.
   Resources: (A bit outdated)
   a. https://espressif.com/sites/default/files/documentation/esp-now_user_guide_en.pdf
   b. http://www.esploradores.com/practica-6-conexion-esp-now/

   << This Device Master >>

   Flow: Master
   Step 1 : ESPNow Init on Master and set it in STA mode
   Step 2 : Start scanning for Slave ESP32 (we have added a prefix of `slave` to the SSID of slave for an easy setup)
   Step 3 : Once found, add Slave as peer
   Step 4 : Register for send callback
   Step 5 : Start Transmitting data from Master to Slave(s)

   Flow: Slave
   Step 1 : ESPNow Init on Slave
   Step 2 : Update the SSID of Slave with a prefix of `slave`
   Step 3 : Set Slave in AP mode
   Step 4 : Register for receive callback and wait for data
   Step 5 : Once data arrives, print it in the serial monitor

   Note: Master and Slave have been defined to easily understand the setup.
         Based on the ESPNOW API, there is no concept of Master and Slave.
         Any devices can act as master or salve.


  // Sample Serial log with 1 master & 2 slaves
      Found 12 devices
      1: Slave:24:0A:C4:81:CF:A4 [24:0A:C4:81:CF:A5] (-44)
      3: Slave:30:AE:A4:02:6D:CC [30:AE:A4:02:6D:CD] (-55)
      2 Slave(s) found, processing..
      Processing: 24:A:C4:81:CF:A5 Status: Already Paired
      Processing: 30:AE:A4:2:6D:CD Status: Already Paired
      Sending: 9
      Send Status: Success
      Last Packet Sent to: 24:0a:c4:81:cf:a5
      Last Packet Send Status: Delivery Success
      Send Status: Success
      Last Packet Sent to: 30:ae:a4:02:6d:cd
      Last Packet Send Status: Delivery Success

*/

#include <esp_now.h>
#include <WiFi.h>
#include "config.h"
#include "main.h"
#include "input.h"
#include "serialbridge.h"
#include "protocol.h"


// Global copy of slave
esp_now_peer_info_t slaves[1] = {};
int SlaveCnt = 0;

#define debugESPNOW

#ifdef debugESPNOW
  bool debug_espnow = true;
#else
bool debug_espnow = false;
#endif

int hideAP = 0;
volatile int sendTimeout;
volatile bool sendReady = true;

#define CHANNEL_MASTER 3
#define CHANNEL_SLAVE 3
#define PRINTSCANRESULTS 0

void configDeviceAP();


// Init ESP Now with fallback
void InitESPNow() {
  if (esp_now_init() == ESP_OK) {
    if(debug_espnow) COM[DEBUG_COM]->println("ESPNow Init Success");
  }
  else {
    if(debug_espnow) COM[DEBUG_COM]->println("ESPNow Init Failed");
    // Retry InitESPNow, add a counte and then restart?
    // InitESPNow();
    // or Simply Restart
    ESP.restart();
  }
}

// Scan for slaves in AP mode
void ScanForSlave() {
  int8_t scanResults = WiFi.scanNetworks();
  //reset slaves
  memset(slaves, 0, sizeof(slaves));
  SlaveCnt = 0;
  if(debug_espnow) COM[DEBUG_COM]->println("");
  if (scanResults == 0) {
    if(debug_espnow) COM[DEBUG_COM]->println("No WiFi devices in AP Mode found");
  } else {
    if(debug_espnow) COM[DEBUG_COM]->print("Found "); if(debug_espnow) COM[DEBUG_COM]->print(scanResults); if(debug_espnow) COM[DEBUG_COM]->println(" devices ");
    for (int i = 0; i < scanResults; ++i) {
      // Print SSID and RSSI for each device found
      String SSID = WiFi.SSID(i);
      int32_t RSSI = WiFi.RSSI(i);
      String BSSIDstr = WiFi.BSSIDstr(i);

      if (PRINTSCANRESULTS) {
        if(debug_espnow) COM[DEBUG_COM]->print(i + 1); if(debug_espnow) COM[DEBUG_COM]->print(": "); if(debug_espnow) COM[DEBUG_COM]->print(SSID); if(debug_espnow) COM[DEBUG_COM]->print(" ["); if(debug_espnow) COM[DEBUG_COM]->print(BSSIDstr); if(debug_espnow) COM[DEBUG_COM]->print("]"); if(debug_espnow) COM[DEBUG_COM]->print(" ("); if(debug_espnow) COM[DEBUG_COM]->print(RSSI); if(debug_espnow) COM[DEBUG_COM]->print(")"); if(debug_espnow) COM[DEBUG_COM]->println("");
      }
      delay(10);
      // Check if the current device starts with `Slave`
      if (SSID.indexOf(ESPNOW_PREFIX) == 0) {
        // SSID of interest
        if(debug_espnow) COM[DEBUG_COM]->print(i + 1); if(debug_espnow) COM[DEBUG_COM]->print(": "); if(debug_espnow) COM[DEBUG_COM]->print(SSID); if(debug_espnow) COM[DEBUG_COM]->print(" ["); if(debug_espnow) COM[DEBUG_COM]->print(BSSIDstr); if(debug_espnow) COM[DEBUG_COM]->print("]"); if(debug_espnow) COM[DEBUG_COM]->print(" ("); if(debug_espnow) COM[DEBUG_COM]->print(RSSI); if(debug_espnow) COM[DEBUG_COM]->print(")"); if(debug_espnow) COM[DEBUG_COM]->println("");
        // Get BSSID => Mac Address of the Slave
        int mac[6];

        if ( 6 == sscanf(BSSIDstr.c_str(), "%02x:%02x:%02x:%02x:%02x:%02x",  &mac[0], &mac[1], &mac[2], &mac[3], &mac[4], &mac[5] ) ) {
          for (int ii = 0; ii < 6; ++ii ) {
            slaves[SlaveCnt].peer_addr[ii] = (uint8_t) mac[ii];
          }
        }
        slaves[SlaveCnt].channel = CHANNEL_MASTER; // pick a channel
        slaves[SlaveCnt].encrypt = 0; // no encryption
        SlaveCnt++;
      }
    }
  }

  if (SlaveCnt > 0) {
    if(debug_espnow) COM[DEBUG_COM]->print(SlaveCnt); if(debug_espnow) COM[DEBUG_COM]->println(" Slave(s) found, processing..");
  } else {
    if(debug_espnow) COM[DEBUG_COM]->println("No Slave Found, trying again.");
  }

  // clean up ram
  WiFi.scanDelete();
}

// Check if the slave is already paired with the master.
// If not, pair the slave with master
void manageSlave() {
  if (SlaveCnt > 0) {
    for (int i = 0; i < SlaveCnt; i++) {
      const esp_now_peer_info_t *peer = &slaves[i];
      const uint8_t *peer_addr = slaves[i].peer_addr;
      if(debug_espnow) COM[DEBUG_COM]->print("Processing: ");
      for (int ii = 0; ii < 6; ++ii ) {
        if(debug_espnow) COM[DEBUG_COM]->print((uint8_t) slaves[i].peer_addr[ii], HEX);
        if (ii != 5) if(debug_espnow) COM[DEBUG_COM]->print(":");
      }
      if(debug_espnow) COM[DEBUG_COM]->print(" Status: ");
      // check if the peer exists
      bool exists = esp_now_is_peer_exist(peer_addr);
      if (exists) {
        // Slave already paired.
        if(debug_espnow) COM[DEBUG_COM]->println("Already Paired");
      } else {
        // Slave not paired, attempt pair
        esp_err_t addStatus = esp_now_add_peer(peer);
        if (addStatus == ESP_OK) {
          // Pair success
          if(debug_espnow) COM[DEBUG_COM]->println("Pair success");
        } else if (addStatus == ESP_ERR_ESPNOW_NOT_INIT) {
          // How did we get so far!!
          if(debug_espnow) COM[DEBUG_COM]->println("ESPNOW Not Init");
        } else if (addStatus == ESP_ERR_ESPNOW_ARG) {
          if(debug_espnow) COM[DEBUG_COM]->println("Add Peer - Invalid Argument");
        } else if (addStatus == ESP_ERR_ESPNOW_FULL) {
          if(debug_espnow) COM[DEBUG_COM]->println("Peer list full");
        } else if (addStatus == ESP_ERR_ESPNOW_NO_MEM) {
          if(debug_espnow) COM[DEBUG_COM]->println("Out of memory");
        } else if (addStatus == ESP_ERR_ESPNOW_EXIST) {
          if(debug_espnow) COM[DEBUG_COM]->println("Peer Exists");
        } else {
          if(debug_espnow) COM[DEBUG_COM]->println("Not sure what happened");
        }
        delay(100);
      }
    }
  } else {
    // No slave found to process
    if(debug_espnow) COM[DEBUG_COM]->println("No Slave found to process");
  }
}


// send data
void sendData(const void *data, size_t n_bytes) {
  sendTimeout++;
  if(sendTimeout > 100) sendReady = true;

  if(!sendReady) return;    // Do not send new data, when no feedback was received.

  for (int i = 0; i < SlaveCnt; i++) {
    const uint8_t *peer_addr = slaves[i].peer_addr;
    if (i == 0) { // print only for first slave
      if(debug_espnow) COM[DEBUG_COM]->print("Sending: ");
      if(debug_espnow) COM[DEBUG_COM]->println((char *)data);
    }

#ifdef debugESPNOW
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
    peer_addr[0], peer_addr[1], peer_addr[2], peer_addr[3], peer_addr[4], peer_addr[5]);
    if(debug_espnow) COM[DEBUG_COM]->print("Last Packet Sent to: "); if(debug_espnow) COM[DEBUG_COM]->println(macStr);
#endif

    sendReady = false;
    esp_err_t result = esp_now_send(peer_addr, (uint8_t*)data, n_bytes);

    if(debug_espnow) COM[DEBUG_COM]->print("Send Status: ");
    if (result == ESP_OK) {
      if(debug_espnow) COM[DEBUG_COM]->println("Success");
    } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
      // How did we get so far!!
      if(debug_espnow) COM[DEBUG_COM]->println("ESPNOW not Init.");
    } else if (result == ESP_ERR_ESPNOW_ARG) {
      if(debug_espnow) COM[DEBUG_COM]->println("Invalid Argument");
    } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
      if(debug_espnow) COM[DEBUG_COM]->println("Internal Error");
    } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
      if(debug_espnow) COM[DEBUG_COM]->println("ESP_ERR_ESPNOW_NO_MEM");
    } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
      if(debug_espnow) COM[DEBUG_COM]->println("Peer not found.");
    } else {
      if(debug_espnow) COM[DEBUG_COM]->println("Not sure what happened");
    }
  }
}

// callback when data is sent from Master to Slave
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {

#ifdef debugESPNOW
  char macStr[18];
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  if(debug_espnow) COM[DEBUG_COM]->print("Last Packet Sent to: "); if(debug_espnow) COM[DEBUG_COM]->println(macStr);
  if(debug_espnow) COM[DEBUG_COM]->print("Last Packet Send Status: "); if(debug_espnow) COM[DEBUG_COM]->println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
#endif

  sendTimeout = 0;
  sendReady = true;
}

// config AP SSID
void configDeviceAP() {
  String Prefix1 = ESPNOW_PREFIX;
  String Prefix2 = ":";
  String Mac = WiFi.macAddress();
  String SSID = Prefix1 + Prefix2 + Mac;
  String Password = "123456789";
  bool result = WiFi.softAP(SSID.c_str(), Password.c_str(), CHANNEL_SLAVE, hideAP);
  if (!result) {
    if(debug_espnow) COM[DEBUG_COM]->println("AP Config failed.");
  } else {
    if(debug_espnow) COM[DEBUG_COM]->println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}

void setupEspNow() {
  #ifdef ESPNOW_PEERMAC
    memset(slaves, 0, sizeof(slaves));
    uint8_t preset_peer_addr[6] = {ESPNOW_PEERMAC};
    for(int i=0; i<6; i++) {
      slaves[0].peer_addr[i] = preset_peer_addr[i];
    }
    slaves[0].channel= CHANNEL_MASTER;
    slaves[0].encrypt= 0;
    SlaveCnt = 1;
    hideAP = 1;
  #endif

  //Set device in STA mode to begin with
  WiFi.mode(WIFI_MODE_APSTA);
  if(debug_espnow) COM[DEBUG_COM]->println("ESPNow/Multi-Slave/Master Example");
  // configure device AP mode
  configDeviceAP();
  // This is the mac address of the Master in Station Mode
  if(debug_espnow) COM[DEBUG_COM]->print("STA MAC: "); if(debug_espnow) COM[DEBUG_COM]->println(WiFi.macAddress());
  if(debug_espnow) COM[DEBUG_COM]->print("AP MAC: "); if(debug_espnow) COM[DEBUG_COM]->println(WiFi.softAPmacAddress());
  // Init ESPNow with a fallback logic
  InitESPNow();
  // Once ESPNow is successfully Init, we will register for Send CB to
  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);
  #ifdef ESPNOW_PEERMAC
      manageSlave();
  #endif

}
/*
void loopEspNow() {
  // In the loop we scan for slave
  ScanForSlave();
  // If Slave is found, it would be populate in `slave` variable
  // We will check if `slave` is defined and then we proceed further
  if (SlaveCnt > 0) { // check if slave channel is defined
    // `slave` is defined
    // Add slave as peer if it has not been added already
    manageSlave();
    // pair success or already paired
    // Send data to device
    sendData();
  } else {
    // No slave found to process
  }

  // wait for 3seconds to run the logic again
  delay(2000);
}
*/