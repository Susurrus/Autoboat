#ifndef ECAN_SENSORS_H
#define ECAN_SENSORS_H

#include <stddef.h>
#include <stdbool.h>

#include "Types.h"
#include "Node.h"
#include "Tokimec.h"

// Store data from the Rudder Node.
struct RudderCanData  {
	float Position;
	bool  NewData;
};
extern struct RudderCanData rudderCanDataStore;

// Store data from the Power Node.
struct PowerData {
	float voltage;
	float current;
	float temperature;
	bool  newData;
};
extern struct PowerData powerDataStore;

// Store data from the wSO100 air/wind sensor
struct WindData {
	float speed;
	float direction;
	bool  newData;
};
extern struct WindData windDataStore;
struct AirData {
	float temp;
	float pressure;
	float humidity;
	bool  newData;
};
extern struct AirData airDataStore;

// Store data from the DST-800 triducer.
struct WaterData {
	float speed; // Speed through the water in m/s
	float temp;  // Water temperature in degrees Celsius
	float depth; // Water depth in m
	bool  newData;
};
extern struct WaterData waterDataStore;

// Store data from the ACS300 BLDC driver board
struct ThrottleData {
	int16_t rpm; // RPM
	bool    newData;
};
extern struct ThrottleData throttleDataStore;

// Store data from the Revolution GS IMU
struct RevoGsData {
    float heading; // In rads
    char magStatus;
    float pitch; // In rads
    char pitchStatus;
    float roll; // in rads
    char rollStatus;
    float dip; // In rads
    uint16_t magneticMagnitude;
};
extern struct RevoGsData revoGsDataStore;

// Store data from the Tokimec VSAS-2GM
extern TokimecOutput tokimecDataStore;

// Store data from the DSP-3000 z-axis gyro.
struct GyroData {
	float   zRate;
	bool    newData;
};
extern struct GyroData gyroDataStore;

// Stores all status data from a CANode.
// Invalid values for every field is the maximum positive value for that datatype.
struct NodeStatusData {
	int8_t   temp;
	uint8_t  voltage;
	uint8_t  load;
	uint16_t status;
	uint16_t errors;
};
// Holds the status data for every node, including this one.
extern struct NodeStatusData nodeStatusDataStore[NUM_NODES];

// Set the timeout period for nodes (in units of centiseconds)
#define NODE_TIMEOUT 100

/**
 * This array stores the timeout counters for each node. Once they hit `NODE_TIMEOUT`, the nodes
 * are considered offline. This transition will reset the `nodeStatusDataStore` array to its default
 * values.
 */
extern uint8_t nodeStatusTimeoutCounters[NUM_NODES];

typedef struct {
    bool newData; // True if newData has arrived and has not been processed yet
    float attitude[3]; // The attitude as Euler angles in yaw,pitch,roll (rads).
    float gyros[3]; // Rotation rate in radians/s in [x y z] format
} ImuData;

/**
 * Declare bitflags for use with the GpsData struct's newData field.
 */
enum {
        GPSDATA_NONE     = 0x00,
	GPSDATA_POSITION = 0x01, // Indicates new position data
	GPSDATA_VELOCITY = 0x02, // Indicates new sog/cog data
	GPSDATA_DOP      = 0x04  // Indicates there's new hdop/vdop/mode
};

typedef struct {
	uint8_t newData; // Bitfield with flags for whether this struct stores new data.
	uint8_t mode; // The type of fix used by the GPS. @see Nmea2000.h:PGN_129539_MODE.
	uint16_t cog; // Course over ground in .0001 radians eastward from north.
	uint16_t sog; // Speed over ground in cm/s
	int16_t hdop; // Horizontal dilation of precision. Units in m.
	int16_t vdop; // Vertical dilation of precision. Units in m.
	int32_t latitude; // Latitude in units of 1e-7 degrees
	int32_t longitude; // Longitude in units of 1e-7 degrees
	int32_t altitude; // Altitude in 1e-6 meters
	float variation; // Magnetic variation at this GPS coordinate. Units in degrees.
} GpsData;
extern GpsData gpsDataStore;

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
        timeoutCounters gyro; // The gyro is enabled and active whenever messages are received
} sensorAvailability;

// Set the timeout period for sensors (in units of the call rate of `UpdateSensorsAvailability`)
#define SENSOR_TIMEOUT 100

/**
 * Returns the water speed of the vessel in m/s. Also clears the newData member variable.
 */
float GetWaterSpeed(void);

/**
 * Returns the rotation speed of the prop in rpm. Also clears the newData member variable.
 */
float GetPropSpeed(void);

/**
 * Returns the current GPS data. Also clears the newData member variable.
 * @param[out] data A struct to copy the data into
 */
void GetGpsData(GpsData *data);

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
