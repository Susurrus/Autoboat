#ifndef ECAN_SENSORS_H
#define ECAN_SENSORS_H

#include <stddef.h>
#include <stdbool.h>

#include "Types.h"

struct RudderCanData  {
	float Position;
	bool         NewData;
};
extern struct RudderCanData rudderCanDataStore;

struct PowerData {
	float voltage;
	float current;
	float temperature;
	bool         newData;
};
extern struct PowerData powerDataStore;

struct WindData {
	float speed;
	float direction;
	bool         newData;
};
extern struct WindData windDataStore;

struct AirData {
	float temp;
	float pressure;
	float humidity;
	bool         newData;
};
extern struct AirData airDataStore;

struct WaterData {
	float speed;
	float temp;
	float depth;
	bool         newData;
};
extern struct WaterData waterDataStore;

struct ThrottleData {
	int16_t rpm;
	bool    newData;
};
extern struct ThrottleData throttleDataStore;

struct RevoGsData {
	float    heading; // In rads
	char     magStatus;
	float    pitch; // In rads
	char     pitchStatus;
	float    roll; // In rads
	char     rollStatus;
	float    dip; // In rads
	uint16_t magneticMagnitude;
};
extern struct RevoGsData revoGsDataStore;

/**
 * Declare bitflags for use with the GpsData struct's receivedMessages field.
 */
enum {
	GPSDATA_POSITION = 0x01,
	GPSDATA_HEADING  = 0x02,
	GPSDATA_FIX      = 0x04,
	GPSDATA_ALL      = 0x07
};

struct GpsData {
	bool newData; // Flag for whether this struct stores new data
	uint8_t mode; // The type of fix used by the GPS. @see Nmea2000.h:PGN_129539_MODE.
	uint16_t cog; // Course over ground in degrees eastward from north.
	uint16_t sog; // Speed over ground in m/s
	int16_t hdop; // Horizontal dilation of precision. Units in m.
	int16_t vdop; // Vertical dilation of precision. Units in m.
	int32_t lat; // Latitude in units of 1e-7 degrees
	int32_t lon; // Longitude in units of 1e-7 degrees
	int32_t alt; // Altitude in 1e-6 meters
};
extern struct GpsData gpsDataStore;

/**
 * This struct should not be modified by external code. It is merely used as an aggregator for
 * incoming GPS data. Once all three messages are aggregated, the data is checked for validity, and
 * then it overwrites the gpsDataStore variable if it is.
 */
struct GpsDataBundle {
	uint8_t receivedMessages; /// A bitfiled indicating which fields have been received for this timestep.
	uint8_t mode; // The type of fix used by the GPS. @see Nmea2000.h:PGN_129539_MODE.
	uint16_t cog; // Course over ground in degrees eastward from north.
	uint16_t sog; // Speed over ground in m/s
	int16_t hdop; // Horizontal dilation of precision. Units in m.
	int16_t vdop; // Vertical dilation of precision. Units in m.
	int32_t lat; // Latitude in units of 1e-7 degrees
	int32_t lon; // Longitude in units of 1e-7 degrees
	int32_t alt; // Altitude in 1e-6 meters
};
extern struct GpsDataBundle gpsNewDataStore;

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
	timeoutCounters gps; // GPS is enabled when any CAN messages have been received within the last second, active when enabled and the lat/lon are valid within the last second.
	timeoutCounters imu; // The IMU is enabled when any messages have been received within the last second, active whenever it's enabled.
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
 * 'sensorAvailability' struct. It's used by ProcessAllEcanMessages(). Ideally this function should
 * only be used where sensor data is received.
 */
void UpdateSensorsAvailability(void);

#endif // ECAN_SENSORS_H
