#include "config.h"


#if (CONFIGURATION_SET == CFG_ODROIDGO)
sysconfig_t sysconfig =
{
  chan_in  : COMM_CHAN_NONE,
  chan_out : COMM_CHAN_UDP,
  { wifi_ssid : "panzer" },
  { wifi_pass : "t.j9c4hkopppxs"},
  input : SYSCONF_IN_NONE
};
#endif


#if (CONFIGURATION_SET == CFG_ESPNOW_RELAY)
sysconfig_t sysconfig =
{
  chan_in  : COMM_CHAN_ESPNOW,
  chan_out : COMM_CHAN_UART,
  { wifi_ssid : "wireshark" },
  { wifi_pass : "t.j9c4hkopppxs" },
  input : SYSCONF_IN_NONE
};
#endif


#if (CONFIGURATION_SET == CFG_TTGO_IMU_WIFI)
sysconfig_t sysconfig =
{
  chan_in  : COMM_CHAN_NONE,
  chan_out : COMM_CHAN_UART,
  { wifi_ssid : "wireshark" },
  { wifi_pass : "t.j9c4hkopppxs" },
  input : SYSCONF_IN_NONE
};
#endif


#if (CONFIGURATION_SET == CFG_TTGO_PADDELEC)
sysconfig_t sysconfig =
{
  chan_in  : COMM_CHAN_NONE,
  chan_out : COMM_CHAN_UDP,
  { wifi_ssid : "wireshark" },
  { wifi_pass : "t.j9c4hkopppxs" },
  input : SYSCONF_IN_PADDLEIMU
};
#endif

#if (CONFIGURATION_SET == CFG_TESTRUN)
sysconfig_t sysconfig =
{
  chan_in  : COMM_CHAN_NONE,
  chan_out : COMM_CHAN_UART,
  { wifi_ssid : "wireshark" },
  { wifi_pass : "t.j9c4hkopppxs" },
  input : SYSCONF_IN_NONE
};
#endif

#if (CONFIGURATION_SET == CFG_WIRESHARK)
sysconfig_t sysconfig =
{
  chan_in  : COMM_CHAN_UDP,
  chan_out : COMM_CHAN_UART,
  { wifi_ssid : "zulu" },
  { wifi_pass : "bewegungsappar.at" },
  input : SYSCONF_IN_NONE
};
#endif

#if (CONFIGURATION_SET == CFG_GAMETRAK)
sysconfig_t sysconfig =
{
  chan_in  : COMM_CHAN_NONE,
  chan_out : COMM_CHAN_UDP,
  { wifi_ssid : "zulu" },
  { wifi_pass : "bewgegungsappar.at" },
  input : SYSCONF_IN_NONE

};
#endif


#if (CONFIGURATION_SET == CFG_PADDELEC)
sysconfig_t sysconfig =
{
  chan_in  : COMM_CHAN_UDP,
  chan_out : COMM_CHAN_UART,
  { wifi_ssid : "paddelec" },
  { wifi_pass : "bewegungsappar.at" },
  input : SYSCONF_IN_NONE
};
#endif

#if (CONFIGURATION_SET == CFG_PADDLE)
sysconfig_t sysconfig =
{
  chan_in  : COMM_CHAN_UDP,
  chan_out : COMM_CHAN_UART,
  { wifi_ssid : "paddelec" },
  { wifi_pass : "bewegungsappar.at" },
  input : SYSCONF_IN_PADDLEIMU
};
#endif

#if (CONFIGURATION_SET == CFG_PAGAIE)
sysconfig_t sysconfig =
{
  chan_in  : COMM_CHAN_NONE,
  chan_out : COMM_CHAN_UDP,
  { wifi_ssid : "zulu" },
  { wifi_pass : "t.j9c4hkopppxs" },
  input : SYSCONF_IN_PADDLEIMU
};
#endif

#if (CONFIGURATION_SET == CFG_NUNCHUK_ESPNOW_RELAY)
sysconfig_t sysconfig =
{
  chan_in  : COMM_CHAN_ESPNOW,
  chan_out : COMM_CHAN_UART,
  { wifi_ssid : "wireshark" },
  { wifi_pass : "t.j9c4hkopppxs" },
  input : SYSCONF_IN_NONE
};
#endif

#if (CONFIGURATION_SET == CFG_PANZER)
sysconfig_t sysconfig =
{
  chan_in  : COMM_CHAN_UDP,
  chan_out : COMM_CHAN_UART,
  { wifi_ssid : "panzer" },
  { wifi_pass : "t.j9c4hkopppxs" },
  input : SYSCONF_IN_NONE
};
#endif