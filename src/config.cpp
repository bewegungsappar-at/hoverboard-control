#include "config.h"


#if (CONFIGURATION_SET == CFG_ODROIDGO)
comm_settings communicationSettings =
{
  input  : COMM_INPUT_OTHER,
  output : COMM_OUTPUT_UDP
};
#endif


#if (CONFIGURATION_SET == CFG_ESPNOW_RELAY)
comm_settings communicationSettings =
{
  input  : COMM_INPUT_ESPNOW,
  output : COMM_OUTPUT_UART
};
#endif


#if (CONFIGURATION_SET == CFG_TTGO_IMU_WIFI)
comm_settings communicationSettings =
{
  input  : COMM_INPUT_OTHER,
  output : COMM_OUTPUT_UART
};
#endif


#if (CONFIGURATION_SET == CFG_TTGO_PADDELEC)
comm_settings communicationSettings =
{
  input  : COMM_INPUT_OTHER,
  output : COMM_OUTPUT_UDP
};
#endif

#if (CONFIGURATION_SET == CFG_TESTRUN)
comm_settings communicationSettings =
{
  input  : COMM_INPUT_OTHER,
  output : COMM_OUTPUT_UART
};
#endif

#if (CONFIGURATION_SET == CFG_UDP_RELAY)
comm_settings communicationSettings =
{
  input  : COMM_INPUT_UDP,
  output : COMM_OUTPUT_UART
};
#endif


#if (CONFIGURATION_SET == CFG_NUNCHUK_ESPNOW_RELAY)
comm_settings communicationSettings =
{
  input  : COMM_INPUT_ESPNOW,
  output : COMM_OUTPUT_UART
};
#endif

#if (CONFIGURATION_SET == CFG_NUNCHUK_REMOTE)
comm_settings communicationSettings =
{
  input  : COMM_INPUT_OTHER,
  output : COMM_OUTPUT_ESPNOW
};
#endif