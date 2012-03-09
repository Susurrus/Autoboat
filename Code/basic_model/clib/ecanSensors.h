#ifndef __ECAN_SENSORS_H__
#define __ECAN_SENSORS_H__

#include <stddef.h>
#include <stdbool.h>

#include "types.h"

struct PowerData {
	tFloatToChar voltage;
	tFloatToChar current;
	tFloatToChar temperature;
	bool         newData;
};
extern struct PowerData powerDataStore;

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

// This variable is a bitfield storing which sensors are enabled and which are healthy. Enabled means that the sensor/actuator is online and communicating normally. Active is a subset of enabled when it's also broadcasting good useable data.
enum {
	SENSOR_GPS = 0x01, // GPS is enabled when any CAN messages have been received within the last second, active when enabled and the lat/lon are valid within the last second.
	SENSOR_REVO_GS = 0x02, // The Revo GS is enabled when any messages have been received within the last second, active whenever it's enabled.
	SENSOR_WSO100 = 0x04, // The WSO100 is enabled when any messages have been received within the last second, active whenever it's enabled.
	SENSOR_DST800 = 0x08, // The DST800 is enabled when any messages have been received within the last second, active whenever depth is valid within the last second (this should only be true when it's in the water)
	ACTUATOR_PROP = 0x10 // The ACS300 outputs CAN messages quite frequently. It's enabled whenever one of these messages has been received within the last second and active when it's enabled and in run mode within the last second.
};
extern uint8_t sensorsEnabled;
extern uint8_t sensorsHealthy;

/**
  * These functions are getters for the above structs for use with Matlab.
  */
void GetWindDataPacked(uint8_t *data);

void GetAirDataPacked(uint8_t *data);

void GetWaterDataPacked(uint8_t *data);

void GetThrottleDataPacked(uint8_t *data);

void GetGpsDataPacked(uint8_t* data);

/**
  * Clears the GPS data struct.
  */
void ClearGpsData(void);

/**
  * This function should be called every timestep to process any received ECAN messages.
  */
uint8_t ProcessAllEcanMessages();

#endif // __ECAN_SENSORS_H__
