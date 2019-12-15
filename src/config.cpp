#include "config.h"


#if (CONFIGURATION_SET == CFG_ODROIDGO)
comm_settings communicationSettings =
{
  input  : COMM_IN_OTHER,
  output : COMM_OUT_UDP
};
#endif


#if (CONFIGURATION_SET == CFG_ESPNOW_RELAY)
comm_settings communicationSettings =
{
  input  : COMM_IN_ESPNOW,
  output : COMM_OUT_UART
};
#endif


#if (CONFIGURATION_SET == CFG_TTGO_IMU_WIFI)
comm_settings communicationSettings =
{
  input  : COMM_IN_OTHER,
  output : COMM_OUT_UART
};
#endif


#if (CONFIGURATION_SET == CFG_TTGO_PADDELEC)
comm_settings communicationSettings =
{
  input  : COMM_IN_OTHER,
  output : COMM_OUT_UDP
};
#endif

#if (CONFIGURATION_SET == CFG_TESTRUN)
comm_settings communicationSettings =
{
  input  : COMM_IN_OTHER,
  output : COMM_OUT_UART
};
#endif

#if (CONFIGURATION_SET == CFG_UDP_RELAY)
comm_settings communicationSettings =
{
  input  : COMM_IN_UDP,
  output : COMM_OUT_UART
};
#endif

#if (CONFIGURATION_SET == CFG_PADDELEC)
comm_settings communicationSettings =
{
  input  : COMM_IN_UDP,
  output : COMM_OUT_UART
};
#endif

#if (CONFIGURATION_SET == CFG_PADDLE)
comm_settings communicationSettings =
{
  input  : COMM_IN_OTHER,
  output : COMM_OUT_UDP
};
#endif

#if (CONFIGURATION_SET == CFG_NUNCHUK_ESPNOW_RELAY)
comm_settings communicationSettings =
{
  input  : COMM_IN_ESPNOW,
  output : COMM_OUT_UART
};
#endif

#if (CONFIGURATION_SET == CFG_NUNCHUK_REMOTE)
comm_settings communicationSettings =
{
  input  : COMM_IN_OTHER,
  output : COMM_OUT_ESPNOW
};
#endif