#include "EcanSensors.h"

#include "Nmea2000.h"
#include "Types.h"
#include "Rudder.h"
#include "Node.h"
#include "Acs300.h"
#include "Packing.h"
#include "MavCorruptNode.h"

#include <string.h>

// Declare some macros for helping setting bit values
#define ON  1
#define OFF 0

/**
 * Check the current values of the 'state' timeout counter for the given sensor and update the sensor's
 * state accordingly. This is merely a helper macro for SENSOR_STATE_UPDATE.
 * @param sensor Should be one of the available sensors in the sensor
 * @param state Should be either active or enabled.
 */
#define SENSOR_STATE_UPDATE_STATE(sensor, state)                                                                   \
    if (sensorAvailability.sensor.state) {\
        if (sensorAvailability.sensor.state ## _counter < SENSOR_TIMEOUT) {        \
            ++sensorAvailability.sensor.state ## _counter;\
        } else {\
            sensorAvailability.sensor.state = false;                                                                   \
        }\
    } else if (!sensorAvailability.sensor.state && sensorAvailability.sensor.state ## _counter < SENSOR_TIMEOUT) { \
        sensorAvailability.sensor.state = true;                                                                    \
    }

/**
 * This macro update both the 'enabled' and 'active' state for a sensor
 */
#define SENSOR_STATE_UPDATE(sensor) \
    SENSOR_STATE_UPDATE_STATE(sensor, enabled); \
    SENSOR_STATE_UPDATE_STATE(sensor, active);

struct PowerData powerDataStore = {0};
struct WindData windDataStore = {0};
struct AirData airDataStore = {0};
struct WaterData waterDataStore = {0};
struct ThrottleData throttleDataStore = {0};
GpsData gpsDataStore = {0};
struct GpsDataBundle gpsNewDataStore = {0};
struct DateTimeData dateTimeDataStore = {// Initialize our system clock to clearly invalid values.
    UINT16_MAX,
    UINT8_MAX,
    UINT8_MAX,
    UINT8_MAX,
    UINT8_MAX,
    UINT8_MAX,
    UINT64_MAX,
    false
};
struct RevoGsData revoGsDataStore = {0};
TokimecOutput tokimecDataStore = {};
struct NodeStatusData nodeStatusDataStore[NUM_NODES] = {
    {INT8_MAX, UINT8_MAX, UINT8_MAX, UINT16_MAX, UINT16_MAX},
    {INT8_MAX, UINT8_MAX, UINT8_MAX, UINT16_MAX, UINT16_MAX},
    {INT8_MAX, UINT8_MAX, UINT8_MAX, UINT16_MAX, UINT16_MAX},
    {INT8_MAX, UINT8_MAX, UINT8_MAX, UINT16_MAX, UINT16_MAX},
    {INT8_MAX, UINT8_MAX, UINT8_MAX, UINT16_MAX, UINT16_MAX},
    {INT8_MAX, UINT8_MAX, UINT8_MAX, UINT16_MAX, UINT16_MAX},
    {INT8_MAX, UINT8_MAX, UINT8_MAX, UINT16_MAX, UINT16_MAX}
};
uint8_t nodeStatusTimeoutCounters[NUM_NODES] = {
    NODE_TIMEOUT,
    NODE_TIMEOUT,
    NODE_TIMEOUT,
    NODE_TIMEOUT,
    NODE_TIMEOUT,
    NODE_TIMEOUT,
    NODE_TIMEOUT
};
struct GyroData gyroDataStore = {0};

// At startup assume all sensors are disconnected.
struct stc sensorAvailability = {
    {1, SENSOR_TIMEOUT, 1, SENSOR_TIMEOUT},
    {1, SENSOR_TIMEOUT, 1, SENSOR_TIMEOUT},
    {1, SENSOR_TIMEOUT, 1, SENSOR_TIMEOUT},
    {1, SENSOR_TIMEOUT, 1, SENSOR_TIMEOUT},
    {1, SENSOR_TIMEOUT, 1, SENSOR_TIMEOUT},
    {1, SENSOR_TIMEOUT, 1, SENSOR_TIMEOUT},
    {1, SENSOR_TIMEOUT, 1, SENSOR_TIMEOUT},
    {1, SENSOR_TIMEOUT, 1, SENSOR_TIMEOUT},
    {1, SENSOR_TIMEOUT, 1, SENSOR_TIMEOUT}
};

