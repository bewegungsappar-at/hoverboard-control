#include "output.h"
#include <Arduino.h>
#include "main.h"
#include "config.h"
#include <crc.h>
#include "serialbridge.h"


#ifdef OUTPUT_PROTOCOL
  #include <protocol.h>

  size_t send_serial_data( const uint8_t *data, size_t len ) {
//    COM[DEBUG_COM]->write(data,len);
//    COM[DEBUG_COM]->println();
    return COM[MOTOR_COM]->write(data,len);
  }
#endif

#ifdef OUTPUT_ESPnowMASTER
  #include "ESPnowMaster.h"
  bool isPaired = false;

#endif


double limit(double min, double value, double max) {
  if(value<min) value = min;
  if(value>max) value = max;
  return value;
}

void setupOutput() {

    #ifdef OUTPUT_ESPnowMASTER
    setupESPnowMaster();
    #endif
}

void motorCommunication( void * pvparameters) {
#ifdef MULTITASKING
  int taskno = (int)pvparameters;
  while(1) {
#endif //MULTITASKING

#ifdef OUTPUT_ESPnowMASTER
    if(!isPaired) {
      ScanForSlave();
      // If Slave is found, it would be populated in `slave` variable
      // We will check if `slave` is defined and then we proceed further
      if (slave.channel == CHANNEL) { // check if slave channel is defined
        // `slave` is defined
        // Add slave as peer if it has not been added already
        isPaired = manageSlave();
      }
    } else {
      sendData((const void *) &motor, sizeof(motor));
    }
#endif
#ifdef OUTPUT_PROTOCOL
    PROTOCOL_MSG newMsg;
    memset((void*)&newMsg,0x00,sizeof(PROTOCOL_MSG));
    PROTOCOL_MSG *msg = &newMsg;
    PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) msg->bytes;
    PWM_STEER_CMD *writespeed = (PWM_STEER_CMD *) writevals->content;

    msg->SOM = PROTOCOL_SOM; //Start of Message;

    writevals->cmd  = PROTOCOL_CMD_WRITEVAL;  // Write value
    writevals->code = 0x07; // speed data from params array

    writespeed->base_pwm = (int16_t)motor.setpoint.pwm;
    writespeed->steer = (int16_t) motor.setpoint.steer;

    msg->len = sizeof(writevals->cmd) + sizeof(writevals->code) + sizeof(writespeed) + 1; // 1 for Checksum
    protocol_send(msg);


    while(COM[MOTOR_COM]->available())
    {
      protocol_byte( COM[MOTOR_COM]->read() );
    }

    memset((void*)&newMsg,0x00,sizeof(PROTOCOL_MSG));
    PROTOCOL_BYTES_READVALS *readvals = (PROTOCOL_BYTES_READVALS *) msg->bytes;

    msg->SOM = PROTOCOL_SOM; //Start of Message;

    readvals->cmd  = PROTOCOL_CMD_READVAL;  // Read value
    readvals->code = 0x02; // hall data from params array

    msg->len = sizeof(readvals->cmd) + sizeof(readvals->code) + 1; // 1 for Checksum

    protocol_send(msg);

    while(COM[MOTOR_COM]->available())
        {
          protocol_byte( COM[MOTOR_COM]->read() );
        }
#endif
#ifdef OUTPUT_BINARY
    /* cast & limit values to a valid range */
    int16_t steer = (int16_t) limit(-1000.0, motor.setpoint.steer, 1000.0);
    int16_t pwm   = (int16_t) limit(-1000.0, motor.setpoint.pwm,   1000.0);


    /* Send motor pwm values to motor control unit */
    COM[MOTOR_COM]->write((uint8_t *) &steer, sizeof(steer));
    COM[MOTOR_COM]->write((uint8_t *) &pwm,   sizeof(pwm));

  #ifdef OUTPUT_BINARY_CRC
    /* calc and send checksum */
    uint32_t crc = 0;
    crc32((const void *)&steer, sizeof(steer), &crc);
    crc32((const void *)&pwm,   sizeof(pwm),   &crc);

    COM[MOTOR_COM]->write((uint8_t *) &crc,   sizeof(crc));
  #endif

    /* refresh actual motor speed */
    updateSpeed();

    /* debug output */
    #ifdef WIFI
    for(byte cln = 0; cln < MAX_NMEA_CLIENTS; cln++) {
      if(TCPClient[1][cln]) {
        if(debug) TCPClient[1][cln].print(" U: ");
        if(debug) TCPClient[1][cln].printf("%8i %8i\n", pwm, steer);
      }
    }
    #endif
    if(debug) COM[DEBUG_COM]->printf("\nU: ");
//    if(debug) COM[DEBUG_COM]->printf("%6i %6i %11u ", pwm, steer, crc);
    if(debug) COM[DEBUG_COM]->printf("%6i %6i ", pwm, steer);
    if(debug) COM[DEBUG_COM]->printf("%7.2f %7.2f ", motor.measured.actualSpeed_kmh, motor.measured.actualSteer_kmh);
#endif
#ifdef MULTITASKING
    delay(MOTORINPUT_PERIOD);
  }
#endif //MULTITASKING
}

/*
* Dummy function since no speed feedback from Motor control is implemented right now.
* For now, we just use pwm, some conversion factor and low pass filter as a model.
* Values are in m/h
*/
#define SPEED_PWM_CONVERSION_FACTOR  0.2   // Assume 100% PWM = 1000 = Full Speed = 20km/h = 20000 m/h. Therefore 20000 / 1000 = 20
#define SPEED_FILTER                 0.015  // Low pass Filter Value. 1 means no filter at all, 0 no value update.
void updateSpeed() {
  #ifndef OUTPUT_ESPnowMASTER
    motor.measured.actualSpeed_kmh = motor.measured.actualSpeed_kmh * (1.0 - (SPEED_FILTER * deltaMillis)) + motor.setpoint.pwm   * (SPEED_FILTER * deltaMillis) * SPEED_PWM_CONVERSION_FACTOR;
    motor.measured.actualSteer_kmh = motor.measured.actualSteer_kmh * (1.0 - (SPEED_FILTER * deltaMillis)) + motor.setpoint.steer * (SPEED_FILTER * deltaMillis) * SPEED_PWM_CONVERSION_FACTOR;
  #endif
}