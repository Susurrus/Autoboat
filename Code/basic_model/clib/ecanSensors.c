#include "ecanSensors.h"

#include "ecanDefinitions.h"
#include "nmea2000.h"
#include "types.h"

struct WindData windDataStore;
struct AirData airDataStore;
struct WaterData waterDataStore;
struct ThrottleData throttleDataStore;

void GetWindData(unsigned char *data) {
	data[0] = windDataStore.speed.chData[0];
	data[1] = windDataStore.speed.chData[1];
	data[2] = windDataStore.speed.chData[2];
	data[3] = windDataStore.speed.chData[3];
	data[4] = windDataStore.direction.chData[0];
	data[5] = windDataStore.direction.chData[1];
	data[6] = windDataStore.direction.chData[2];
	data[7] = windDataStore.direction.chData[3];
	data[8] = windDataStore.newData;
	windDataStore.newData = false;
}

void GetAirData(unsigned char *data) {
	data[0] = airDataStore.temp.chData[0];
	data[1] = airDataStore.temp.chData[1];
	data[2] = airDataStore.temp.chData[2];
	data[3] = airDataStore.temp.chData[3];
	data[4] = airDataStore.pressure.chData[0];
	data[5] = airDataStore.pressure.chData[1];
	data[6] = airDataStore.pressure.chData[2];
	data[7] = airDataStore.pressure.chData[3];
	data[8] = airDataStore.humidity.chData[0];
	data[9] = airDataStore.humidity.chData[1];
	data[10] = airDataStore.humidity.chData[2];
	data[11] = airDataStore.humidity.chData[3];
	data[12] = airDataStore.newData;
	airDataStore.newData = false;
}

void GetWaterData(unsigned char *data) {
	data[0] = waterDataStore.speed.chData[0];
	data[1] = waterDataStore.speed.chData[1];
	data[2] = waterDataStore.speed.chData[2];
	data[3] = waterDataStore.speed.chData[3];
	data[4] = waterDataStore.temp.chData[0];
	data[5] = waterDataStore.temp.chData[1];
	data[6] = waterDataStore.temp.chData[2];
	data[7] = waterDataStore.temp.chData[3];
	data[8] = waterDataStore.depth.chData[0];
	data[9] = waterDataStore.depth.chData[1];
	data[10] = waterDataStore.depth.chData[2];
	data[11] = waterDataStore.depth.chData[3];
	data[12] = waterDataStore.newData;
	waterDataStore.newData = false;
}

void GetThrottleData(unsigned char *data) {
	data[0] = throttleDataStore.rpm.chData[0];
	data[1] = throttleDataStore.rpm.chData[1];
	data[2] = throttleDataStore.newData;
	throttleDataStore.newData = false;
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
				throttleDataStore.rpm.inData = (int)(((unsigned int)msg.payload[0]) << 8) | ((unsigned int)msg.payload[1]);
				throttleDataStore.newData = true;
			} else {
				pgn = ISO11783Decode(msg.id, NULL, NULL, NULL);
				switch (pgn) {
				case 130306:
					ParsePgn130306(msg.payload, NULL, &windDataStore.speed.flData, &windDataStore.direction.flData);
					windDataStore.newData = true;
					break;
				case 130310:
					ParsePgn130310(msg.payload, NULL, &waterDataStore.temp.flData, NULL, NULL);
					waterDataStore.newData = true;
					break;
				case 130311:
					ParsePgn130311(msg.payload, NULL, &airDataStore.temp.flData, &airDataStore.humidity.flData, &airDataStore.pressure.flData);
					airDataStore.newData = true;
					break;
				case 128259:
					ParsePgn128259(msg.payload, NULL, &waterDataStore.speed.flData);
					waterDataStore.newData = true;
					break;
				case 128267:
					// Only update the data in waterDataStore if an actual depth was returned.
					if (ParsePgn128267(msg.payload, NULL, &waterDataStore.depth.flData, NULL) & 0x02) {
						waterDataStore.newData = true;
					}
					break;
				}
			}

			++messagesHandled;
		}
	} while (messagesLeft > 0);
	
	return messagesHandled;
}
