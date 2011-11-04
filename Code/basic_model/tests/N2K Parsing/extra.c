#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include "uart2.h"
#include "ecanDefinitions.h"
#include "nmea2000.h"

#define TRUE 1
#define FALSE 0

void initCommunications() {
	initUart2(42); // Initialize UART2 for 57600 baud.
}

void displayWindData(unsigned char data[8]) {
	
	float airSpeed, direction;
	
	unsigned char result = ParsePgn130306(data, NULL, &airSpeed, &direction);
	
	char text[120];
	sprintf(text, "Wind data - speed: %2.1f %c (m/s), dir: %2.1f %c (rads)\n", airSpeed, (result & 0x02)?'V':'I', direction, (result & 0x04)?'V':'I');
	
	uart2EnqueueData((unsigned char *)text, strlen(text));
}

void displayAirData(unsigned char data[8]) {
	
	float temp, humidity, pressure;
	ParsePgn130311(data, NULL, &temp, &humidity, &pressure);
	
	char text[120];
	sprintf(text, "Air data - temp: %2.1f (deg C), humid: %2.1f (%%), press: %2.1f (kPa)\n", temp, humidity, pressure);

	uart2EnqueueData((unsigned char *)text, strlen(text));
}

void displayWaterSpeed(unsigned char data[8]){
	
	// Add water speed. Units from the message are cm/s
	float speed;
	ParsePgn128259(data, NULL, &speed);
	
	char text[120];
	sprintf(text, "Water speed data - speed: %2.1f (m/s)\n", speed);
	
	uart2EnqueueData((unsigned char *)text, strlen(text));
}

void displayWaterTemp(unsigned char data[8]){
	
	float waterTemp;
	ParsePgn130310(data, NULL, &waterTemp, NULL, NULL);
	
	char text[120];
	sprintf(text, "Water temp data - temp: %2.1f (deg C)\n", waterTemp);
	
	uart2EnqueueData((unsigned char *)text, strlen(text));
}

unsigned char displayWaterDepth(unsigned char data[8]){
	
	float depth, offset;
	ParsePgn128267(data, NULL, &depth, &offset);

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
