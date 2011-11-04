#include <stdio.h>
#include <string.h>
#include "uart2.h"
#include "ecanDefinitions.h"

#define TRUE 1
#define FALSE 0

void initCommunications() {
	initUart2(42); // Initialize UART2 for 57600 baud.
}

unsigned long ISO11783Decode(unsigned long id, unsigned char *src, unsigned char *dest, unsigned char *pri) {

	unsigned long pgn;

	// The source address is the lowest 8 bits
	if (src) {
	    	*src = (unsigned char)id;
	}
    
	// The priority are the highest 3 bits
	if (pri) {
		*pri = (unsigned char)((id >> 26) & 7);
	}
	
	// PDU Format byte
	unsigned char PF = (unsigned char)(id >> 8);
	
	// PDU Specific byte
	unsigned long PS = (id >> 16) & 0xFF;
	
	// Most Significant byte
	unsigned long MS = (id >> 24) & 3;

	if (PS > 239) {
		// PDU2 format, the destination is implied global and the PGN is extended.
		if (dest) {
			*dest = 0xFF;
		}
		pgn = (MS << 16) | (PS << 8) | ((unsigned long)PF);
	} else {
		// PDU1 format, the PF contains the destination address.
		if (dest) {
			*dest = PF;
		}
		pgn = (MS << 16) | (PS << 8);
	}

	return pgn;
}

void displayWindData(unsigned char data[8]) {
	
	// Wind speed. Message units are cm/s. Here we convert to m/s.
	unsigned int unpacked = data[1];
	unpacked |= ((unsigned int)data[2]) << 8;
	float airSpeed = unpacked / 100.0;
	
	// Wind direction. Message units at e-4 rads
	unpacked = data[3];
	unpacked |= ((unsigned int)data[4]) << 8;
	float direction = 0;
	if (unpacked != 0xFFFF) {
		direction = ((float)unpacked) * .0001;
	}
	
	char text[120];
	sprintf(text, "Wind data - speed: %2.1f (m/s), dir: %2.1f (rads)\n", airSpeed, direction);
	
	uart2EnqueueData((unsigned char *)text, strlen(text));
}

void displayAirData(unsigned char data[8]) {
	
	// Add air temperature data
	unsigned int unpacked = data[2];
	unpacked |= ((unsigned int)data[3]) << 8;
	float temp = unpacked / 100.0 - 273.15;
	
	// Humidity data
	unpacked = data[4];
	unpacked |= ((unsigned int)data[5]) << 8;
	float humidity = unpacked * 0.004;

	// Pressure data
	unpacked = data[6];
	unpacked |= ((unsigned int)data[7]) << 8;
	float pressure = ((float)unpacked) * 0.1;
	
	char text[120];
	sprintf(text, "Air data - temp: %2.1f (deg C), humid: %2.1f (%%), press: %2.1f (kPa)\n", temp, humidity, pressure);

	uart2EnqueueData((unsigned char *)text, strlen(text));
}

void displayWaterSpeed(unsigned char data[8]){
	
	// Add water speed. Units from the message are cm/s
	float speed = ((float)(((unsigned int)data[1]) | (((unsigned int)data[2]) << 8))) / 100.0;
	
	char text[120];
	sprintf(text, "Water speed data - speed: %2.1f (m/s)\n", speed);
	
	uart2EnqueueData((unsigned char *)text, strlen(text));
}

void displayWaterTemp(unsigned char data[8]){
	
	// Add water temp. Units from the message are cK
	unsigned int unpacked = data[1];
	unpacked |= ((unsigned int)data[2]) << 8;
	float temp = 0;
	if (unpacked != 0xFFFF) {
		temp = ((float)unpacked) / 100.0 - 273.15;
	}
	
	char text[120];
	sprintf(text, "Water temp data - temp: %2.1f (deg C)\n", temp);
	
	uart2EnqueueData((unsigned char *)text, strlen(text));
}

unsigned char displayWaterDepth(unsigned char data[8]){
	
	// Add water depth. Units from the message are cm
	unsigned int unpacked = data[1];
	unpacked |= ((unsigned int)data[2]) << 8;
	float depth = 0;
	if (unpacked != 0xFFFF) {
		depth = ((float)unpacked) / 100.0;
	}
	
	unpacked = data[5];
	unpacked |= ((unsigned int)data[6]) << 8;
	float offset = ((float)unpacked) / 100.0;

	char text[120];
	sprintf(text, "Water depth data - depth: %2.1f (m), offset: %2.1f (m)\n", depth, offset);
	
	uart2EnqueueData((unsigned char *)text, strlen(text));
}

void displayUnhandledId(unsigned long pgn, unsigned char data[8]) {
	char text[120];

	sprintf(text, "PGN %lu (%02x %02x %02x %02x %02x %02x %02x %02x) - Unprocessed\n", data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7], pgn);
	uart2EnqueueData((unsigned char *)text, strlen(text));
}

unsigned char processAllMessages() {
	unsigned char messagesLeft = 0;
	tCanMessage msg;
	unsigned long pgn;

	unsigned char messagesHandled = 0;

	do {
		int foundOne = ecan1_receive(&msg, &messagesLeft);
		if (foundOne) {
			pgn = ISO11783Decode(msg.id, 0, 0, 0);
			switch (pgn) {
			case 130306:
				displayWindData(msg.payload);
				break;
			case 130311:
				displayAirData(msg.payload);
				break;
			case 128259:
				displayWaterSpeed(msg.payload);
				break;
			case 130310:
				displayWaterTemp(msg.payload);
				break;
			case 128267:
				displayWaterDepth(msg.payload);
				break;
			default:
				//displayUnhandledId(pgn, msg.payload);
				break;
			}

			++messagesHandled;
		}
	} while (messagesLeft > 0);
	
	return messagesHandled;
}
