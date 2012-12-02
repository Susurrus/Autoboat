#ifndef ECAN_SENSORS_H
#define ECAN_SENSORS_H

#include <stddef.h>
#include <stdbool.h>

#include "Types.h"

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

struct RevoGsData {
	tFloatToChar           heading;
	char                   magStatus;
	tFloatToChar           pitch;
	char                   pitchStatus;
	tFloatToChar           roll;
	char                   rollStatus;
	tFloatToChar           dip;
	tUnsignedShortToChar   magneticMagnitude;
};
extern struct RevoGsData revoGsDataStore;

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
	timeoutCounters rudder; // The rudder controller outputs messages quite frequently also. It's enabled whenever one of these messages has been received within the last second. It's active when it's enabled and calibrated and done calibrating.
        timeoutCounters rcNode; // The RC CAN node that provides override manual control.
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
 * Returns the availability of the RcNode. enabled is set to true if it's broadcasting status
 * messages on the CAN bus. It's active if the RC controller is on and therefore the node is
 * commanding the vessel.
 */
void GetRcNodeAvailability(bool *status);

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

#endif // ECAN_SENSORS_H
