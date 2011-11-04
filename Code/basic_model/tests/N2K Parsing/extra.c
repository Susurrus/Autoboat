#include <stdio.h>
#include <string.h>
#include <stddef.h>
#include <stdbool.h>

#include "uart2.h"
#include "ecanDefinitions.h"
#include "nmea2000.h"
#include "types.h"

struct {
	tFloatToChar speed;
	tFloatToChar direction;
	bool newData;
} windData;

struct {
	tFloatToChar temp;
	tFloatToChar pressure;
	tFloatToChar humidity;
	bool newData;
} airData;

struct {
	tFloatToChar speed;
	tFloatToChar temp;
	tFloatToChar depth;
	bool newData;
} waterData;

void GetWindData(unsigned char *data) {
	data[0] = windData.speed.chData[0];
	data[1] = windData.speed.chData[1];
	data[2] = windData.speed.chData[2];
	data[3] = windData.speed.chData[3];
	data[4] = windData.direction.chData[0];
	data[5] = windData.direction.chData[1];
	data[6] = windData.direction.chData[2];
	data[7] = windData.direction.chData[3];
	data[8] = windData.newData;
	windData.newData = false;
}

void GetAirData(unsigned char *data) {
	data[0] = airData.temp.chData[0];
	data[1] = airData.temp.chData[1];
	data[2] = airData.temp.chData[2];
	data[3] = airData.temp.chData[3];
	data[4] = airData.pressure.chData[0];
	data[5] = airData.pressure.chData[1];
	data[6] = airData.pressure.chData[2];
	data[7] = airData.pressure.chData[3];
	data[8] = airData.humidity.chData[0];
	data[9] = airData.humidity.chData[1];
	data[10] = airData.humidity.chData[2];
	data[11] = airData.humidity.chData[3];
	data[12] = airData.newData;
	airData.newData = false;
}

void GetWaterData(unsigned char *data) {
	data[0] = waterData.speed.chData[0];
	data[1] = waterData.speed.chData[1];
	data[2] = waterData.speed.chData[2];
	data[3] = waterData.speed.chData[3];
	data[4] = waterData.temp.chData[0];
	data[5] = waterData.temp.chData[1];
	data[6] = waterData.temp.chData[2];
	data[7] = waterData.temp.chData[3];
	data[8] = waterData.depth.chData[0];
	data[9] = waterData.depth.chData[1];
	data[10] = waterData.depth.chData[2];
	data[11] = waterData.depth.chData[3];
	data[12] = waterData.newData;
	waterData.newData = false;
}

void initCommunications() {
	initUart2(42); // Initialize UART2 for 57600 baud.
}

void DisplayWindData(float speed, float direction) {
	char text[120];
	sprintf(text, "Wind data - speed: %2.1f (m/s), dir: %2.1f (rads)\n", speed, direction);
	uart2EnqueueData((unsigned char *)text, strlen(text));
}

void DisplayAirData(float temp, float pressure, float humidity) {
	char text[120];
	sprintf(text, "Air data - temp: %2.1f (deg C), press: %2.1f (kPa), humid: %2.1f (%%)\n", temp, pressure, humidity);
	uart2EnqueueData((unsigned char *)text, strlen(text));
}

void DisplayWaterData(float speed, float temp, float depth){
	char text[120];
	sprintf(text, "Water data - speed: %2.1f (m/s), temp: %2.1f (deg C), depth: %2.1f (m)\n", speed, temp, depth);
	uart2EnqueueData((unsigned char *)text, strlen(text));
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
				ParsePgn130306(msg.payload, NULL, &windData.speed.flData, &windData.direction.flData);
				windData.newData = true;
				break;
			case 130311:
				ParsePgn130311(msg.payload, NULL, &airData.temp.flData, &airData.humidity.flData, &airData.pressure.flData);
				airData.newData = true;
				break;
			case 128259:
				ParsePgn128259(msg.payload, NULL, &waterData.speed.flData);
				waterData.newData = true;
				break;
			case 130310:
				ParsePgn130310(msg.payload, NULL, &waterData.temp.flData, NULL, NULL);
				waterData.newData = true;
				break;
			case 128267:
				// Only update the data in waterData if an actual depth was returned.
				if (ParsePgn128267(msg.payload, NULL, &waterData.depth.flData, NULL) & 0x02) {
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
