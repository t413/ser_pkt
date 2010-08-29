/*
 *  ser_pkt.cpp
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

#include "WProgram.h"
#include "ser_pkt.h"

/*****************************************/
/*---------      send data      ---------*/
/*****************************************/

// Input:
//	- packet id,        - packet type,         -- 4x 16 bit int (only first one is required)
void send_int16_packet(uint8_t pktID, uint8_t pktTYPE, int16_t in0,int16_t in1,int16_t in2,int16_t in3){
	uint8_t data[10] = "";
	uint8_t * buf = data;	// copy the pointer
	
	*(int16_t *)buf = in0; buf += 2;
	*(int16_t *)buf = in1; buf += 2;
	*(int16_t *)buf = in2; buf += 2;
	*(int16_t *)buf = in3; buf += 2;
	
	send_packet(pktID, pktTYPE, data, (buf-data) );		//buf-data is the legnth of the datastream
}

// Input:
//	- packet id,      - packet type,      - 8 bit int
void send_byte_packet(uint8_t pktID, uint8_t pktTYPE, uint8_t in0){
	uint8_t data[2] = "";
	uint8_t * buf = data;	// copy the pointer
	
	*(uint8_t *)buf = in0; buf += 1;
	
	send_packet(pktID, pktTYPE, data, (buf-data) );		//buf-data is the legnth of the datastream
}

// Input:
//	- packet id,     - packet type,      - 2x float (only first one is required)
void send_float_packet(uint8_t pktID, uint8_t pktTYPE, float d0, float d1){
	uint8_t data[10] = "";
	uint8_t * buf = data;	// copy the pointer
	
	int16_t transmit_x = (int16_t)(d0 * 100);
	*(int16_t *)buf = transmit_x; buf += 2;

	int16_t transmit_y = (int16_t)(d1 * 100);
	*(int16_t *)buf = transmit_x; buf += 2;
	
	send_packet(pktID, pktTYPE, data, (buf-data) );	//buf-data is the legnth of the datastream
}


/*****************************************/
/*---------     decode data     ---------*/
/*****************************************/

// takes serialized array of data and breaks it into vars
//	- very spesific structure, must be correctly decoded
//	- Input: pointer to encoded array. the legnth is part of the datatype spec.
//	- Returns: ...
ACCtelem decode_acc_data( uint8_t * buf ) {
    ACCtelem telem;
	
	int16_t recieved_x = *(int16_t *)buf; buf += 2;
	int16_t recieved_y = *(int16_t *)buf; buf += 2;
	
	telem.x = (double) recieved_x/100.0;
	telem.y = (double) recieved_y/100.0;

	return telem;
}

FourU16 decode_4xint16( uint8_t * buf ) {
	FourU16 recieved;
	
    recieved.d0 = *(int16_t *)buf; buf += 2;
    recieved.d1 = *(int16_t *)buf; buf += 2;
    recieved.d2 = *(int16_t *)buf; buf += 2;
    recieved.d3 = *(int16_t *)buf; buf += 2;
	
	return recieved;
}



/*****************************************/
/*---------       get data      ---------*/
/*****************************************/

// honor incoming serial commands
//	- returns entire packet data
uint8_t* getSerialPacket()
{
	if (Serial.available()) 
	{
		byte index = 2;
        byte timeout = 0;
        uint8_t data[128] = "";
		bool success = 0;
		
		//Serial.println("bgn:");
		data[0] = Serial.read();
		//Serial.print(data[0],HEX);
		//Serial.print(" ");
		if (data[0] != START_OF_MSG0)
			return NULL;
		data[1] = Serial.read();
		//Serial.print(data[1],HEX);
		//Serial.print(" ");
		if (data[1] != START_OF_MSG1)
			return NULL;
		
        do {
			// break: we've (read it &&) reached the correct packet legnth
			if ((data[4] != 0)&&(index >= (5+data[4]+2) )) {
				//Serial.println("success break");
				success = 1;
				break;
			}
			//fixes problem w/ xbee starting new packet where legnth should be.
			if ((data[4] != 0)&&((data[4]==START_OF_MSG0)||(data[4]==START_OF_MSG1)||(data[4]>128) ))
				return NULL;
			// delay if there's not enough data:
			if (Serial.available() == 0) {
				delay(1);
				timeout++;
			}
			// got data, cache it:
			else {
				data[index] = Serial.read();

				//Serial.print(data[index],HEX);
				//Serial.print(" ");
								
				timeout = 0;
				index++;
			}
			
			if (timeout > 5) {
				//Serial.println("timeout break");
				return NULL;
			}
			if (index >= 128){
				//Serial.println("index break");
				return NULL;
			}
        }  while (1);
				
		if (success){
			uint8_t cksum[2];
			ugear_cksum( data[2], data[3], data+5, data[4], &cksum[0], &cksum[1] );
			if ((cksum[0] != data[5+(data[4])+0]) || (cksum[1] != data[5+(data[4])+1])) {
				//printf(" failed checksum (%X %X)\n",packet[5+(packet[4])+0],packet[5+(packet[4])+1]);
				//Serial.println("failed chksm");
				return NULL;
			}
			else {
				return data;
			}
		}
		
	}
}

// sends the given data 
//	- works for any data structure that's already serialized
//	- Input: pointer to serialized (see packetize functions) data array
//	- Returns: nothing. sends data over serial channel.
void send_packet(uint8_t pktID, uint8_t pktTYPE, uint8_t *buf, uint8_t size ) {
	uint8_t header[5];
	uint8_t cksum[2];
	
	// start of message sync bytes
	header[0] = START_OF_MSG0;
	header[1] = START_OF_MSG1;
	// packet id (1 byte)
	header[2] = pktID;
	header[3] = pktTYPE;
	// packet size (1 byte)
	header[4] = size;
	

	console_write(header , 5);

	// packet data
	console_write( buf, size );
	
	// check sum (2 bytes)
	ugear_cksum( pktID, pktTYPE, buf, size, &cksum[0], &cksum[1] );
	console_write( cksum, 2 );

}

// prints some data array to serial
//	- Input:	-some data array
//				-data array legnth
//	- Returns: actually sends serial data only.
void console_write(uint8_t * data, uint8_t legnth){
	for (uint8_t i=0; i<legnth; i++) {
		Serial.print(data[i], BYTE);
		//printf("%X ", data[i] );		
	}
}

// checksums an almost ready-to-go packet
//	- Input:	-packet type, both bytes
//				-packet payload
//				-packet payload legnth
//				-address-of the places to store the 2 checksum values
// from microgear.googlecode.com
void ugear_cksum( const uint8_t hdr1, const uint8_t hdr2,
				 const uint8_t *buf, const uint8_t size,
				 uint8_t *cksum0, uint8_t *cksum1 )
{
    uint8_t c0 = 0;
    uint8_t c1 = 0;
	
    c0 += hdr1;
    c1 += c0;
	
    c0 += hdr2;
    c1 += c0;
	
    for ( uint8_t i = 0; i < size; i++ ) {
        c0 += (uint8_t)buf[i];
        c1 += c0;
    }
	
    *cksum0 = c0;
    *cksum1 = c1;
}
