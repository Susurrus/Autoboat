#ifndef _ECAN_SENSORS_H_
#define _ECAN_SENSORS_H_

#include <stddef.h>
#include <stdbool.h>

#include "types.h"

struct RudderCanData  {
	tFloatToChar Position;
	bool         NewData;
};
extern struct RudderCanData rudderCanDataStore;

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
	uint64_t usecSinceEpoch;
	bool     newData; // Flag for whether this struct stores new data
};
extern struct DateTimeData dateTimeDataStore;

typedef struct {
	bool enabled            : 1; // If the sensor is enabled, i.e. it is online and transmitting messages.
	uint8_t enabled_counter : 7; // The timeout counter for this sensor being enabled.
	bool active             : 1; // If the sensor is active, i.e. receiving valid data.
	uint8_t active_counter  : 7; // The timeout counter for this sensor being active.
} timeoutCounters;

extern struct stc {
	timeoutCounters gps; // GPS is enabled when any CAN messages have been received within the last second, active when enabled and the lat/lon are valid within the last second. This is also the only sensor that is enabled when HIL is engaged.
	timeoutCounters revo_gs; // The Revo GS is enabled when any messages have been received within the last second, active whenever it's enabled.
	timeoutCounters wso100; // The WSO100 is enabled when any messages have been received within the last second, active whenever it's enabled.
	timeoutCounters dst800; // The DST800 is enabled when any messages have been received within the last second, active whenever depth is valid within the last second (this should only be true when it's in the water)
	timeoutCounters power; // The power node is enabled when a messages has been received within the last second and active at the same time.
	timeoutCounters prop; // The ACS300 outputs CAN messages quite frequently. It's enabled whenever one of these messages has been received within the last second and active when it's enabled and in run mode within the last second.
} sensorAvailability;


/**
 * These functions are getters for the above structs for use with Matlab.
 */
void GetWindDataPacked(uint8_t *data);

void GetAirDataPacked(uint8_t *data);

void GetWaterDataPacked(uint8_t *data);

void GetThrottleDataPacked(uint8_t *data);

void GetGpsDataPacked(uint8_t *data);

void GetRudderCanDataPacked(uint8_t *data);

/**
  * Clears the GPS data struct.
  */
void ClearGpsData(void);

/**
 * This function should be called every timestep to process any received ECAN messages.
 */
uint8_t ProcessAllEcanMessages(void);

/**
 * This function updates the sensor availability. This all ends up being reflected in the
 * 'sensorAvailability' struct. It's used by both ProcessAllEcanMessages() and in commProtocol.c
 * when GPS data is received via HIL. Ideally this function should only be used where sensor data is
 * received and that should be in no more than the places where HIL data is received and the real
 * sensor data is received.
 */
void UpdateSensorsAvailability(void);

#endif // _ECAN_SENSORS_H_
