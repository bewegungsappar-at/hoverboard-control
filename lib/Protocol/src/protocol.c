/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2018 Simon Hailes <btsimonh@googlemail.com>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

//#include "stm32f1xx_hal.h"
//#include "defines.h"
//#include "config.h"
//#include "sensorcoms.h"
#include "protocol.h"
//#include "hallinterrupts.h"
//#include "softwareserial.h"
//#include "bldc.h"
#include <Arduino.h>

#include "flashcontent.h"

#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#ifdef INCLUDE_PROTOCOL

//////////////////////////////////////////////////////////////
// this is the Hall data we gather, and can be read elsewhere
// one for each wheel
typedef struct tag_HALL_DATA_STRUCT{
    long HallPosn; // 90 per revolution
    long HallSpeed; // speed part calibrated to speed demand value

    float HallPosnMultiplier; // m per hall segment

    long HallPosn_mm; // posn in mm
    long HallPosn_mm_lastread; // posn offset set via protocol in mm
    long HallSpeed_mm_per_s; // speed in m/s

    unsigned long HallTimeDiff;
    unsigned long HallSkipped;
} HALL_DATA_STRUCT;
volatile HALL_DATA_STRUCT HallData[2];

//////////////////////////////////////////////////////////
// two new protocols are created, and simultaneously active
// 1. simple ascii protocol
//  press ?<CR> for a list of commands
//  this is very suitable for development and playing
// 2. a protocol with length, checksum, ACK/NACK etc.
//  this is more suitable for machine control.
//////////////////////////////////////////////////////////
//
// ASCII protocol:
// this accepts command sup to 10 bytes long terminated with CR.
// one of these commands (I) can enable an 'immediate' mode.
// In 'immediate' mode, keypresses cause immediate action;
// for example, controlling speed, or getting real-time feedback.
//
//////////////////////////////////////////////////////////
//
// Machine protocol:
// a very simple protocol, starting 02 (SOM), with length and checksum
// examples:
// ack - 02 02 41 BD
// nack - 02 02 4E B0
// test - 02 06 54 54 65 73 74 06
// e.g. for test:
// 02 - SOM
// 06 - length = 6
// 54 - byte0 - 'cmd' 'T'
// 54 - byte1 - payload for text command - 'T'est
// 65 - byte2 - 'e'
// 73 - byte3 - 's'
// 74 - byte4 - 't'
// 06 - checksum = (00 - (06+54+54+65+73+74))&0xff = 06,
// or  can be stated as : (06+54+54+65+73+74+06)&0xff = 0
//
// if a message is received with invalid checksum, then nack will be sent.
// if a message is received complete, it will with be responded to with a 
// return message, or with the ack message
//
// for simplicities sake, we will treat the hoverboard controller as a 
// slave unit always - i.e. not ask it to send *unsolicited* messages.
// in this way, it does not need to wait for ack, etc. from the host.
// if the host gets a bad message, or no response, it can retry.
//
//////////////////////////////////////////////////////////



///////////////////////////////////////////////
// extern variables you want to read/write here
#ifdef CONTROL_SENSOR
extern SENSOR_DATA sensor_data[2];
#endif

extern uint8_t enable; // global variable for motor enable
extern volatile uint32_t timeout; // global variable for timeout
extern int dspeeds[2];
extern int pwms[2];



extern int debug_out;
extern int sensor_control;
extern int sensor_stabilise;
extern int disablepoweroff;
extern int powerofftimer;
extern uint8_t buzzerFreq;    // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern; // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...
extern int buzzerLen;
extern int enablescope; // enable scope on values

int speedB = 0;
int steerB = 0;

int control_type = 0;
char *control_types[]={
    "none",
    "Position",
    "Speed (disabled)",
    "PWM Direct"
};

POSN_DATA PosnData = {
    {0, 0},

    200, // max pwm in posn mode
    70, // min pwm in posn mode
};


SPEED_DATA SpeedData = {
    {0, 0},

    600, // max power (PWM)
    -600,  // min power 
    40 // minimum mm/s which we can ask for
};


int speed_control = 0; // incicates protocol driven

///////////////////////////////////////////////


/////////////////////////////////////////////////////////////
// specify where to send data out of with a function pointer.
#ifdef SOFTWARE_SERIAL
int (*send_serial_data)( unsigned char *data, int len ) = softwareserial_Send;
int (*send_serial_data_wait)( unsigned char *data, int len ) = softwareserial_Send_Wait;
#endif

#ifdef DEBUG_SERIAL_USART3
// need to implement a buffering function here.
// current DMA method needs attention...
int nosend( unsigned char *data, int len ){ return 0; };
int (*send_serial_data)( unsigned char *data, int len ) = nosend;
int (*send_serial_data_wait)( unsigned char *data, int len ) = nosend;
#endif
/////////////////////////////////////////////////////////////


//////////////////////////////////////////////
// variables and functions in support of parameters here
//
void protocol_send_nack();
void protocol_send_ack();
void protocol_send_test();


// e.g. to gather two separate speed variables togther,
typedef struct tag_SPEEDS{
    int speedl;
    int speedr;
} SPEEDS;
SPEEDS speedsx = {0,0};

// before read we call this...
void PreRead_getspeeds(void){
    speedsx.speedl = SpeedData.wanted_speed_mm_per_sec[0];
    speedsx.speedr = SpeedData.wanted_speed_mm_per_sec[1];
}

// after write we call this...
void PostWrite_setspeeds(void){
    SpeedData.wanted_speed_mm_per_sec[0] = speedsx.speedl;
    SpeedData.wanted_speed_mm_per_sec[1] = speedsx.speedr;
}

typedef struct tag_POSN {
    long LeftAbsolute;
    long RightAbsolute;
    long LeftOffset;
    long RightOffset;
} POSN;

POSN Position;

void PreRead_getposnupdate(){
    Position.LeftAbsolute = HallData[0].HallPosn_mm;
    Position.LeftOffset = HallData[0].HallPosn_mm - HallData[0].HallPosn_mm_lastread;
    Position.RightAbsolute = HallData[1].HallPosn_mm;
    Position.RightOffset = HallData[1].HallPosn_mm - HallData[1].HallPosn_mm_lastread;
}

void PostWrite_setposnupdate(){
    HallData[0].HallPosn_mm_lastread = Position.LeftAbsolute;
    HallData[1].HallPosn_mm_lastread = Position.RightAbsolute; 
}

typedef struct tag_POSN_INCR {
    long Left;
    long Right;
} POSN_INCR;

POSN_INCR PositionIncr;

void PostWrite_incrementposition(){
    PosnData.wanted_posn_mm[0] += PositionIncr.Left; 
    PosnData.wanted_posn_mm[1] += PositionIncr.Right; 
}


void PostWrite_writeflash(){
    if (FlashContent.magic != CURRENT_MAGIC){
//        consoleLog("incorrect magic %d, should be %d\r\nFlash not written\r\n", FlashContent.magic, CURRENT_MAGIC);
        FlashContent.magic = CURRENT_MAGIC;
        return;
    }
//    writeFlash( &FlashContent, sizeof(FlashContent) );
//    consoleLog("wrote flash\r\n");
}

void PostWrite_PID(){
//    change_PID_constants();
}

///////////////////////////////////////////////////
// structure used to gather variables we want to read/write.
#define PARAM_R     1
#define PARAM_RW    3

#pragma pack(push, 1)
typedef struct tag_PARAMSTAT {
    unsigned char code;     // code in protocol to refer to this
    char *description;          // if non-null, description
    char *uistr;          // if non-null, used in ascii protocol to adjust with f<str>num<cr>
    char ui_type;           // only UI_NONE or UI_SHORT
    void *ptr;              // pointer to value
    char len;               // length of value
    char rw;                // PARAM_R or PARAM_RW

    void (*preread)(void);                // function to call after write
    void (*postread)(void);                // function to call after write
    void (*prewrite)(void);                // function to call after write
    void (*postwrite)(void);                // function to call after write
} PARAMSTAT;
#pragma pack(pop)
///////////////////////////////////////////////////

#define UI_NONE 0
#define UI_SHORT 1

int version = 1;

// NOTE: Don't start uistr with 'a'
PARAMSTAT params[] = {
    { 0x00, NULL, NULL, UI_NONE, &version,           sizeof(version),        PARAM_R,    NULL, NULL, NULL, NULL },
#ifdef CONTROL_SENSOR
    { 0x01, NULL, NULL, UI_NONE, &sensor_data,       sizeof(sensor_data),    PARAM_R,    NULL, NULL, NULL, NULL },
#endif
#ifdef HALL_INTERRUPTS
    { 0x02, NULL, NULL, UI_NONE, (void *)&HallData,          sizeof(HallData),       PARAM_R,    NULL, NULL, NULL, NULL },
#endif
    { 0x03, NULL, NULL, UI_NONE, &SpeedData,         sizeof(SpeedData),      PARAM_RW,   PreRead_getspeeds, NULL, NULL, PostWrite_setspeeds },
    { 0x04, NULL, NULL, UI_NONE, &Position,          sizeof(Position),       PARAM_RW,   PreRead_getposnupdate, NULL, NULL, PostWrite_setposnupdate },
    { 0x05, NULL, NULL, UI_NONE, &PositionIncr,      sizeof(PositionIncr),   PARAM_RW,    NULL, NULL, NULL, PostWrite_incrementposition },
    { 0x06, NULL, NULL, UI_NONE, &PosnData,          sizeof(PosnData),       PARAM_RW,    NULL, NULL, NULL, NULL },

    { 0x80, "flash magic", "m", UI_SHORT, &FlashContent.magic, sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_writeflash },  // write this with CURRENT_MAGIC to commit to flash

    { 0x82, "posn kp x 100", "pkp", UI_SHORT, &FlashContent.PositionKpx100, sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID },
    { 0x81, "posn ki x 100", "pki", UI_SHORT, &FlashContent.PositionKix100, sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID }, // pid params for Position
    { 0x83, "posn kd x 100", "pkd", UI_SHORT, &FlashContent.PositionKdx100, sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID },
    { 0x84, "posn pwm lim", "pl", UI_SHORT, &FlashContent.PositionPWMLimit, sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID }, // e.g. 200

    { 0x86, "speed kp x 100", "skp", UI_SHORT, &FlashContent.SpeedKpx100, sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID },
    { 0x85, "speed ki x 100", "ski", UI_SHORT, &FlashContent.SpeedKix100, sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID }, // pid params for Speed
    { 0x87, "speed kd x 100", "skd", UI_SHORT, &FlashContent.SpeedKdx100, sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID },
    { 0x88, "speed pwm incr lim", "sl", UI_SHORT, &FlashContent.SpeedPWMIncrementLimit, sizeof(short), PARAM_RW, NULL, NULL, NULL, PostWrite_PID }, // e.g. 20

    { 0xA0, "hoverboard enable", "he", UI_SHORT, &FlashContent.HoverboardEnable, sizeof(short), PARAM_RW, NULL, NULL, NULL, NULL } // e.g. 20
};



///////////////////////////////////////////////////
// local functions, not really for external usage
void protocol_send_nack();
void process_message(PROTOCOL_MSG *msg);
int ascii_process_immediate(unsigned char byte);
void ascii_process_msg(char *cmd, int len);
// void ascii_byte( unsigned char byte );



///////////////////////////////////////////////////
// local variables for handling the machine protocol, 
// not really for external usage
//
typedef struct tag_PROTOCOL_STAT {
    char state;
    unsigned char CS;
    unsigned char count;
    unsigned int nonsync;
    PROTOCOL_MSG curr_msg;
} PROTOCOL_STAT;
PROTOCOL_STAT s;

#define PROTOCOL_STATE_IDLE 0
#define PROTOCOL_STATE_WAIT_LEN 1
#define PROTOCOL_STATE_WAIT_END 2
///////////////////////////////////////////////////




///////////////////////////////////////////////////
// process incomming serial a byte at a time
// and only when a complete, valid message is received,
// process it.
// msgs with invalid CS will get NACK response.
void protocol_byte( unsigned char byte ){
    switch(s.state){
        case PROTOCOL_STATE_IDLE:
            if (byte == PROTOCOL_SOM){
                s.curr_msg.SOM = byte;
                s.state = PROTOCOL_STATE_WAIT_LEN;
                s.CS = 0;
            } else {
                //////////////////////////////////////////////////////
                // if the byte was NOT SOM (02), then treat it as an 
                // ascii protocol byte.  BOTH protocol can co-exist
//                ascii_byte( byte );
                //////////////////////////////////////////////////////
            }
            break;
        case PROTOCOL_STATE_WAIT_LEN:
            s.curr_msg.len = byte;
            s.count = 0;
            s.CS += byte;
            s.state = PROTOCOL_STATE_WAIT_END;
            break;
        case PROTOCOL_STATE_WAIT_END:
            s.curr_msg.bytes[s.count++] = byte;
            s.CS += byte;
            if (s.count == s.curr_msg.len){
                if (s.CS != 0){
                    protocol_send_nack();
                    Serial.write((unsigned char *)&s.curr_msg,s.curr_msg.len);
                } else {
                    process_message(&s.curr_msg);  // this should ack or return a message
                }
                s.state = PROTOCOL_STATE_IDLE;
            }
            break;
    }
}





/////////////////////////////////////////////
// MACHINE PROTOCOL
// functions in support of the operation of the machine protocol
//
void protocol_send_nack(){
    char tmp[] = { PROTOCOL_SOM, 2, PROTOCOL_CMD_NACK, 0 };
//    protocol_send((PROTOCOL_MSG *)tmp);
    Serial.println("Will gerne NACK schicken");
}

void protocol_send_ack(){
    char tmp[] = { PROTOCOL_SOM, 2, PROTOCOL_CMD_ACK, 0 };
//    protocol_send((PROTOCOL_MSG *)tmp);
    Serial.println("Will gerne ACK schicken");
}

void protocol_send_test(){
    char tmp[] = { PROTOCOL_SOM, 6, PROTOCOL_CMD_TEST, 'T', 'e', 's', 't', 0 };
//    protocol_send((PROTOCOL_MSG *)tmp);
    Serial.println("Will gerne TEST schicken");
}




/////////////////////////////////////////////
// a complete machineprotocl message has been 
// received without error
void process_message(PROTOCOL_MSG *msg){
    PROTOCOL_BYTES *bytes = (PROTOCOL_BYTES *)msg->bytes; 
    switch (bytes->cmd){
        case PROTOCOL_CMD_READVAL:{

            PROTOCOL_BYTES_READVALS *readvals = (PROTOCOL_BYTES_READVALS *) msg->bytes;
            PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) msg->bytes;
            int i;
            for (i = 0; i < sizeof(params)/sizeof(params[0]); i++){
                if (params[i].code == readvals->code){
                    if (params[i].preread) params[i].preread();
                    // NOTE: re-uses the msg object (part of stats)
                    unsigned char *src = (unsigned char*)params[i].ptr;
                    for (int j = 0; j < params[i].len; j++){
                        writevals->content[j] = *(src++);
                    }
                    msg->len = 1+1+1+params[i].len+1;
                    // send back with 'read' command plus data like write.
                    //protocol_send(msg);
                    Serial.println("Reference 1");
                    if (params[i].postread) params[i].postread();
                    break;
                }
            }
            // nothing read
            if (i == sizeof(params)/sizeof(params[0])){
                msg->len = 1+1+1+0+1;
                // send back with 'read' command plus data like write.
                //protocol_send(msg);
                Serial.println("Reference 2");
            }
            break;
        }

        case PROTOCOL_CMD_WRITEVAL:{
            PROTOCOL_BYTES_WRITEVALS *writevals = (PROTOCOL_BYTES_WRITEVALS *) msg->bytes;
            int i;
            for (i = 0; i < sizeof(params)/sizeof(params[0]); i++){
                if (params[i].code == writevals->code){
                    if (params[i].prewrite) params[i].prewrite();
                    // NOTE: re-uses the msg object (part of stats)
                    unsigned char *dest = (unsigned char *)params[i].ptr;
                    for (int j = 0; j < params[i].len; j++){
                        *(dest++) = writevals->content[j];
                    }
                    msg->len = 1+1+0+1;
                    // send back with 'write' command with no data.
                    //protocol_send(msg);
                    Serial.println("Reference 3");
                    if (params[i].postwrite) params[i].postwrite();
                }
            }
            // nothing written
            if (i == sizeof(params)/sizeof(params[0])){
                msg->len = 1+1+1+0+1;
                // send back with 'write' command plus data like write.
                //protocol_send(msg);
                Serial.println("Reference 4");
            }
            break;
        }

        case PROTOCOL_CMD_REBOOT:
            protocol_send_ack();
//            HAL_Delay(500);
//            HAL_NVIC_SystemReset();
            break;

        case PROTOCOL_CMD_ACK:
            Serial.println("ACKed ");
            break;

        case PROTOCOL_CMD_NACK:
            Serial.print("N ");
            break;


        default:
            Serial.write(msg->bytes, msg->len);
            Serial.println("Protocol CMD Unknown");
            msg->bytes[0] = PROTOCOL_CMD_UNKNOWN;
            msg->len = 2;
//            protocol_send(msg);
            break;
    }

}
void protocol_send(PROTOCOL_MSG *msg){
    unsigned char CS = 0;
    unsigned char *src = &msg->len;
    for (int i = 0; i < msg->len; i++){
        CS -= *(src++);
    }
    msg->bytes[msg->len-1] = CS;
//    send_serial_data((unsigned char *) msg, msg->len+2);
    send_serial_data((const uint8_t *) msg, (size_t) msg->len+2);
}



#endif