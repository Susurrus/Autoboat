#ifndef __ECAN_SENSORS_H__
#define __ECAN_SENSORS_H__

#include <stddef.h>
#include <stdbool.h>

#include "types.h"

struct WindData {
	tFloatToChar speed;
	tFloatToChar direction;
	bool         newData;
};
extern struct WindData windDataStore;

struct AirData {
	tFloatToChar temp;
	tFloatToChar pressure;
	tFloatToChar humidity;
	bool         newData;
};
extern struct AirData airDataStore;

struct WaterData {
	tFloatToChar speed;
	tFloatToChar temp;
	tFloatToChar depth;
	bool         newData;
};
extern struct WaterData waterDataStore;

struct ThrottleData {
	tShortToChar rpm;
	bool         newData;
};
extern struct ThrottleData throttleDataStore;

struct GpsData {
	tFloatToChar lat; // Latitude in degrees
	tFloatToChar lon; // Longitude in degrees
	tFloatToChar alt; // Altitude in meters
	tFloatToChar cog; // Course over ground in degrees eastward from north.
	tFloatToChar sog; // Speed over ground in m/s
	bool         newData; // Flag for whether this struct stores new data
};
extern struct GpsData gpsDataStore;

struct DateTimeData {
	uint16_t year;
	uint8_t	 month;
	uint8_t	 day;
	uint8_t	 hour;
	uint8_t	 min;
	uint8_t	 sec;
	bool     newData; // Flag for whether this struct stores new data
};
extern struct DateTimeData dateTimeDataStore;

void GetWindDataPacked(uint8_t *data);

void GetAirDataPacked(uint8_t *data);

void GetWaterDataPacked(uint8_t *data);

void GetThrottleDataPacked(uint8_t *data);

void GetGpsDataPacked(uint8_t* data);

void ClearGpsData(void);

uint8_t ProcessAllEcanMessages();

#endif // __ECAN_SENSORS_H__
