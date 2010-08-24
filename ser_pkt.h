/*
 *  ser_pkt.h
 *  Created by Tim O'Brien
 *  
 *  used to encode, send, and decode serilized data
 *  created for UAVs and RC projects
 *  
 *  ideas and some code from microgear.googlecode.com
 */


/*  Packets look like this:
 *  
 *  /------------------------Header--------------------------/ /--data--/ /-------checksum-------/
 *  | Start byte0 | Start1 | packet ID | packet Type | legnth | **data** | Checksum0 | Checksum1 |
 *  
 *  the data section is variable legnth is specified (in __ bytes long).
 *  all bytes are sent as uint8_t (unsigned 8 bit ints)
 */

#ifndef SER_PKT_H
#define SER_PKT_H

#include <stdio.h>
#include <stdlib.h> // for malloc and free
#include "WProgram.h"

/*****************************************/
/**********   datatypes/macros  **********/
/*****************************************/
#define START_OF_MSG0 0x93
#define START_OF_MSG1 0xE0

// packet IDs (first byte of two)
#define BACKEND_CONTROL	0
#define USER_CONTROL	1
	#define ACC_DATA		0
	#define FULL_REMOTE		1
#define TELEM_FEEDBACK	2
	#define STATUS			0
	#define PITCH_ROLL		1
	#define MOTOR_OUTPUT	2
	#define DEBUG_OUTPUT	3
#define SETTINGS_COMM	3
	#define REQUEST_SETTING	0
	#define RECIEVE_PIDS	1
	#define SEND_PIDS		2

// packet types

typedef struct {
	double x;
	double y;
} ACCtelem;

typedef struct {
	uint16_t d0;
	uint16_t d1;
	uint16_t d2;
	uint16_t d3;
} FourU16;

/*****************************************/
/**********		declarations	**********/
/*****************************************/
void send_int16_packet(uint8_t,uint8_t, int16_t in0,int16_t in1 = 0,int16_t in2 = 0,int16_t in3 = 0);
void send_float_packet(uint8_t,uint8_t, float d0, float d1=0);
void send_byte_packet(uint8_t,uint8_t, uint8_t in0 = 0);

void send_packet (uint8_t,uint8_t,uint8_t*,uint8_t );
void console_write (uint8_t*,uint8_t);
void ugear_cksum (const uint8_t,const uint8_t,const uint8_t*,const uint8_t,uint8_t*,uint8_t* );

uint8_t* getSerialPacket();
uint8_t* getDataFromPacket (uint8_t* );

ACCtelem decode_acc_data( uint8_t *buf );
FourU16 decode_4xint16( uint8_t *buf );


#endif