#ifndef __ECAN_SENSORS_H__
#define __ECAN_SENSORS_H__

#include <stddef.h>
#include <stdbool.h>

#include "types.h"

struct WindData {
	tFloatToChar speed;
	tFloatToChar direction;
	bool newData;
};
extern struct WindData windDataStore;

struct AirData {
	tFloatToChar temp;
	tFloatToChar pressure;
	tFloatToChar humidity;
	bool newData;
};
extern struct AirData airDataStore;

struct WaterData {
	tFloatToChar speed;
	tFloatToChar temp;
	tFloatToChar depth;
	bool newData;
};
extern struct WaterData waterDataStore;

struct ThrottleData {
	tIntToChar rpm;
	bool newData;
};
extern struct ThrottleData throttleDataStore;

void GetWindData(unsigned char *data);

void GetAirData(unsigned char *data);

void GetWaterData(unsigned char *data);

void GetThrottleData(unsigned char *data);

unsigned char ProcessAllEcanMessages();

#endif // __ECAN_SENSORS_H__
