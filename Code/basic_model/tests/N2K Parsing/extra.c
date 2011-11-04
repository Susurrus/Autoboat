#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdbool.h>

#include "uart2.h"
#include "ecanDefinitions.h"
#include "nmea2000.h"

struct {
	float speed;
	float direction;
	bool newData;
} windData;

struct {
	float temp;
	float pressure;
	float humidity;
	bool newData;
} airData;

struct {
	float speed;
	float temp;
	float depth;
	bool newData;
} waterData;


void initCommunications() {
	initUart2(42); // Initialize UART2 for 57600 baud.
}

void DisplayWindData(unsigned char data[8]) {
	if (windData.newData) {
		char text[120];
		sprintf(text, "Wind data - speed: %2.1f (m/s), dir: %2.1f (rads)\n", windData.speed, windData.direction);
		
		uart2EnqueueData((unsigned char *)text, strlen(text));
		windData.newData = false;
	}
}

void DisplayAirData(unsigned char data[8]) {
	if (airData.newData) {		
		char text[120];
		sprintf(text, "Air data - temp: %2.1f (deg C), humid: %2.1f (%%), press: %2.1f (kPa)\n", airData.temp, airData.humidity, airData.pressure);

		uart2EnqueueData((unsigned char *)text, strlen(text));
		airData.newData = false;
	}
}

void DisplayWaterData(unsigned char data[8]){
	if (waterData.newData) {
		char text[120];
		sprintf(text, "Water data - speed: %2.1f (m/s), temp: %2.1f (deg C), depth: %2.1f (m)\n", waterData.speed, waterData.temp, waterData.depth);
		
		uart2EnqueueData((unsigned char *)text, strlen(text));
		waterData.newData = false;
	}
}

void DisplayUnhandledId(unsigned long pgn, unsigned char data[8]) {
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
				ParsePgn130306(msg.payload, NULL, &windData.speed, &windData.direction);
				windData.newData = true;
				break;
			case 130311:
				ParsePgn130311(msg.payload, NULL, &airData.temp, &airData.humidity, &airData.pressure);
				airData.newData = true;
				break;
			case 128259:
				ParsePgn128259(msg.payload, NULL, &waterData.speed);
				waterData.newData = true;
				break;
			case 130310:
				ParsePgn130310(msg.payload, NULL, &waterData.temp, NULL, NULL);
				waterData.newData = true;
				break;
			case 128267:
				if (ParsePgn128267(msg.payload, NULL, &waterData.depth, NULL) & 0x02) {
					waterData.newData = true;
				}
				break;
			default:
				//DisplayUnhandledId(pgn, msg.payload);
				break;
			}

			++messagesHandled;
		}
	} while (messagesLeft > 0);
	
	return messagesHandled;
}
