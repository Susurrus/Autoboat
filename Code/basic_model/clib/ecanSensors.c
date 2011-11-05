#include <stddef.h>
#include <stdbool.h>

#include "ecanDefinitions.h"
#include "nmea2000.h"
#include "types.h"

static struct {
	tFloatToChar speed;
	tFloatToChar direction;
	bool newData;
} windData;

static struct {
	tFloatToChar temp;
	tFloatToChar pressure;
	tFloatToChar humidity;
	bool newData;
} airData;

static struct {
	tFloatToChar speed;
	tFloatToChar temp;
	tFloatToChar depth;
	bool newData;
} waterData;

static struct {
	tIntToChar rpm;
	bool newData;
} throttleData;

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

void GetThrottleData(unsigned char *data) {
	data[0] = throttleData.rpm.chData[0];
	data[1] = throttleData.rpm.chData[1];
	data[2] = throttleData.newData;
	throttleData.newData = false;
}

unsigned char ProcessAllEcanMessages() {
	unsigned char messagesLeft = 0;
	tCanMessage msg;
	unsigned long pgn;

	unsigned char messagesHandled = 0;

	do {
		int foundOne = ecan1_receive(&msg, &messagesLeft);
		if (foundOne) {
			// Process throttle messages here. Anything not explicitly handled is assumed to be a NMEA2000 message.
			if (msg.id == 0x402) {
				throttleData.rpm.inData = (int)(((unsigned int)msg.payload[0]) << 8) | ((unsigned int)msg.payload[1]);
				throttleData.newData = true;
			} else {
				pgn = ISO11783Decode(msg.id, NULL, NULL, NULL);
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
				}
			}

			++messagesHandled;
		}
	} while (messagesLeft > 0);
	
	return messagesHandled;
}
