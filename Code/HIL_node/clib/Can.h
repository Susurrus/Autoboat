#ifndef CAN_H
#define CAN_H

#include <stddef.h>
#include <stdbool.h>

#include "Types.h"

struct RudderCanData  {
	tFloatToChar Position;
	bool         NewData;
};
extern struct RudderCanData rudderCanDataStore;

struct ThrottleData {
	tShortToChar rpm;
	bool         newData;
};
extern struct ThrottleData throttleDataStore;

struct GpsData {
	tLongToChar lat; // Latitude in units of 1e-7 degrees
	tLongToChar lon; // Longitude in units of 1e-7 degrees
	tLongToChar alt; // Altitude in 1e-6 meters
	tUnsignedShortToChar cog; // Course over ground in degrees eastward from north.
	tUnsignedShortToChar sog; // Speed over ground in m/s
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

/**
 * This function should be called every timestep to process any received ECAN messages.
 */
uint8_t CanReceiveMessages(void);

void CanTransmitMessages(void);

#endif // CAN_H
