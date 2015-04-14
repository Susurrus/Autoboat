#include "EcanSensors.h"

#include "Ecan1.h"
#include "Nmea2000.h"
#include "Types.h"
#include "Rudder.h"
#include "CanMessages.h"
#include "Node.h"
#include "Acs300.h"
#include "Packing.h"
#include "PrimaryNode.h"

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
    } else if (sensorAvailability.sensor.state ## _counter < SENSOR_TIMEOUT) { \
        sensorAvailability.sensor.state = true;                                                                    \
    }

/**
 * This macro update both the 'enabled' and 'active' state for a sensor
 */
#define SENSOR_STATE_UPDATE(sensor)                 \
    do {                                            \
        SENSOR_STATE_UPDATE_STATE(sensor, enabled); \
        SENSOR_STATE_UPDATE_STATE(sensor, active);  \
    } while (0)

#define SENSOR_STATE_CLEAR_ACTIVE_COUNTER(sensor)               \
    do {                                                        \
        sensorAvailability.sensor.active_counter = 0;           \
        sensorAvailability.sensor.last_active = nodeSystemTime; \
    } while (0)

#define SENSOR_STATE_CLEAR_ENABLED_COUNTER(sensor)     \
    do {                                               \
        sensorAvailability.sensor.enabled_counter = 0; \
    } while (0)

struct PowerData powerDataStore = {0};
SolarData solarDataStore = {UINT16_MAX, UINT16_MAX};
struct WindData windDataStore = {0};
struct AirData airDataStore = {0};
struct WaterData waterDataStore = {0};
struct ThrottleData throttleDataStore = {0};
GpsData gpsDataStore = {
    0,
    0,
    0,
    0,
    UINT16_MAX,
    UINT16_MAX,
    0,
    0,
    0,
    0,
    0
};
struct DateTimeData dateTimeDataStore = {// Initialize our system clock to clearly invalid values.
    0,
    0,
    0,
    0,
    0,
    0,
    0,
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
    {1, SENSOR_TIMEOUT, 1, SENSOR_TIMEOUT, 0},
    {1, SENSOR_TIMEOUT, 1, SENSOR_TIMEOUT, 0},
    {1, SENSOR_TIMEOUT, 1, SENSOR_TIMEOUT, 0},
    {1, SENSOR_TIMEOUT, 1, SENSOR_TIMEOUT, 0},
    {1, SENSOR_TIMEOUT, 1, SENSOR_TIMEOUT, 0},
    {1, SENSOR_TIMEOUT, 1, SENSOR_TIMEOUT, 0},
    {1, SENSOR_TIMEOUT, 1, SENSOR_TIMEOUT, 0},
    {1, SENSOR_TIMEOUT, 1, SENSOR_TIMEOUT, 0},
    {1, SENSOR_TIMEOUT, 1, SENSOR_TIMEOUT, 0}
};

uint8_t dcSourceStatusBytes[PGN_SIZE_DC_SOURCE_STATUS];
Nmea2000FastPacket dsSourceStatusPacket = {0, 0, 0, 0, dcSourceStatusBytes, sizeof(dcSourceStatusBytes)};
uint8_t gnssPositionDataBytes[PGN_SIZE_GNSS_POSITION_DATA];
Nmea2000FastPacket gnssPositionDataPacket = {0, 0, 0, 0, gnssPositionDataBytes, sizeof(gnssPositionDataBytes)};

float GetWaterSpeed(void)
{
    waterDataStore.newData = false;
    return waterDataStore.speed;
}

float GetPropSpeed(void)
{
    throttleDataStore.newData = false;
    return throttleDataStore.rpm;
}

void GetGpsData(GpsData *data)
{
    *data = gpsDataStore;
    gpsDataStore.newData = GPSDATA_NONE;
}

void ClearGpsData(void)
{
    gpsDataStore.latitude = 0.0;
    gpsDataStore.longitude = 0.0;
    gpsDataStore.altitude = 0.0;
    gpsDataStore.cog = 0;
    gpsDataStore.sog = 0;
    gpsDataStore.newData = 0;
}

uint8_t ProcessAllEcanMessages(void)
{
    uint8_t messagesLeft = 0;
    CanMessage msg;
    uint32_t pgn;

    uint8_t messagesHandled = 0;

    do {
        int foundOne = Ecan1Receive(&msg, &messagesLeft);
        if (foundOne) {
            // Process non-NMEA2000 messages here. They're distinguished by having standard frames.
            if (msg.frame_type == CAN_FRAME_STD) {
                if (msg.id == ACS300_CAN_ID_HRTBT) { // From the ACS300
                    SENSOR_STATE_CLEAR_ENABLED_COUNTER(prop);
                    if ((msg.payload[6] & 0x40) == 0) { // Checks the status bit to determine if the ACS300 is enabled.
                        SENSOR_STATE_CLEAR_ACTIVE_COUNTER(prop);
                    }
                    Acs300DecodeHeartbeat(msg.payload, (uint16_t*)&throttleDataStore.rpm, NULL, NULL, NULL);
                    throttleDataStore.newData = true;
                } else if (msg.id == ACS300_CAN_ID_WR_PARAM) {
                    // Track the current velocity from the secondary controller.
                    uint16_t address;

                    union {
                        uint16_t param_u16;
                        int16_t param_i16;
                    } value;
                    Acs300DecodeWriteParam(msg.payload, &address, &value.param_u16);
                    if (address == ACS300_PARAM_CC) {
                        currentCommands.secondaryManualThrottleCommand = value.param_i16;
                    }
                } else if (msg.id == CAN_MSG_ID_STATUS) {
                    uint8_t node, cpuLoad, voltage;
                    int8_t temp;
                    uint16_t status, errors;
                    CanMessageDecodeStatus(&msg, &node, &cpuLoad, &temp, &voltage, &status, &errors);

                    // If we've found a valid node, store the data for it.
                    if (node > 0 && node <= NUM_NODES) {
                        // Update all of the data broadcast by this node.
                        nodeStatusDataStore[node - 1].load = cpuLoad;
                        nodeStatusDataStore[node - 1].temp = temp;
                        nodeStatusDataStore[node - 1].voltage = voltage;
                        nodeStatusDataStore[node - 1].status = status;
                        nodeStatusDataStore[node - 1].errors = errors;

                        // And reset the timeout counter for this node.
                        nodeStatusTimeoutCounters[node - 1] = 0;

                        // And add some extra logic for specific nodes and tracking their
                        // availability.
                        switch (node) {
                            case CAN_NODE_RC:
                                SENSOR_STATE_CLEAR_ENABLED_COUNTER(rcNode);
                                // Only if the RC transmitter is connected and in override mode
                                // should the RC node be considered active.
                                if (status & 0x01) {
                                    SENSOR_STATE_CLEAR_ACTIVE_COUNTER(rcNode);
                                }
                            break;
                            case CAN_NODE_RUDDER_CONTROLLER:
                                SENSOR_STATE_CLEAR_ENABLED_COUNTER(rudder);
                                // As long as the sensor is done calibrating and hasn't errored out,
                                // it's active too.
                                if ((status & 0x01) && !(status & 0x02) && !errors) {
                                    SENSOR_STATE_CLEAR_ACTIVE_COUNTER(rudder);
                                }
                            break;
                        }
                    }
                } else if (msg.id == CAN_MSG_ID_RUDDER_DETAILS) {
                    SENSOR_STATE_CLEAR_ENABLED_COUNTER(rudder);
                    CanMessageDecodeRudderDetails(&msg,
                            &rudderSensorData.RudderPotValue,
                            &rudderSensorData.RudderPotLimitStarboard,
                            &rudderSensorData.RudderPotLimitPort,
                            &rudderSensorData.LimitHitPort,
                            &rudderSensorData.LimitHitStarboard,
                            &rudderSensorData.Enabled,
                            &rudderSensorData.Calibrated,
                            &rudderSensorData.Calibrating);
                    if (rudderSensorData.Enabled &&
                            rudderSensorData.Calibrated &&
                            !rudderSensorData.Calibrating) {
                        SENSOR_STATE_CLEAR_ACTIVE_COUNTER(rudder);
                    }
                } else if (msg.id == CAN_MSG_ID_IMU_DATA) {
                    SENSOR_STATE_CLEAR_ENABLED_COUNTER(imu);
                    SENSOR_STATE_CLEAR_ACTIVE_COUNTER(imu);
                    CanMessageDecodeImuData(&msg,
                            &tokimecDataStore.yaw,
                            &tokimecDataStore.pitch,
                            &tokimecDataStore.roll);
                } else if (msg.id == CAN_MSG_ID_GYRO_DATA) {
                    SENSOR_STATE_CLEAR_ENABLED_COUNTER(gyro);
                    SENSOR_STATE_CLEAR_ACTIVE_COUNTER(gyro);
                    CanMessageDecodeGyroData(&msg, &gyroDataStore.zRate);
                } else if (msg.id == CAN_MSG_ID_ANG_VEL_DATA) {
                    SENSOR_STATE_CLEAR_ENABLED_COUNTER(imu);
                    SENSOR_STATE_CLEAR_ACTIVE_COUNTER(imu);
                    CanMessageDecodeAngularVelocityData(&msg,
                            &tokimecDataStore.x_angle_vel,
                            &tokimecDataStore.y_angle_vel,
                            &tokimecDataStore.z_angle_vel);
                } else if (msg.id == CAN_MSG_ID_ACCEL_DATA) {
                    SENSOR_STATE_CLEAR_ENABLED_COUNTER(imu);
                    SENSOR_STATE_CLEAR_ACTIVE_COUNTER(imu);
                    CanMessageDecodeAccelerationData(&msg,
                            &tokimecDataStore.x_accel,
                            &tokimecDataStore.y_accel,
                            &tokimecDataStore.z_accel);
                } else if (msg.id == CAN_MSG_ID_GPS_POS_DATA) {
                    SENSOR_STATE_CLEAR_ENABLED_COUNTER(imu);
                    SENSOR_STATE_CLEAR_ACTIVE_COUNTER(imu);
                    CanMessageDecodeGpsPosData(&msg,
                            &tokimecDataStore.latitude,
                            &tokimecDataStore.longitude);
                } else if (msg.id == CAN_MSG_ID_GPS_EST_POS_DATA) {
                    SENSOR_STATE_CLEAR_ENABLED_COUNTER(imu);
                    SENSOR_STATE_CLEAR_ACTIVE_COUNTER(imu);
                    CanMessageDecodeGpsPosData(&msg,
                            &tokimecDataStore.est_latitude,
                            &tokimecDataStore.est_longitude);
                } else if (msg.id == CAN_MSG_ID_GPS_VEL_DATA) {
                    SENSOR_STATE_CLEAR_ENABLED_COUNTER(imu);
                    SENSOR_STATE_CLEAR_ACTIVE_COUNTER(imu);
                    CanMessageDecodeGpsVelData(&msg,
                            &tokimecDataStore.gpsDirection,
                            &tokimecDataStore.gpsSpeed,
                            &tokimecDataStore.magneticBearing,
                            &tokimecDataStore.status);
                }
            } else {
                pgn = Iso11783Decode(msg.id, NULL, NULL, NULL);
                switch (pgn) {
                case PGN_ID_SYSTEM_TIME:
                { // From GPS
                    SENSOR_STATE_CLEAR_ENABLED_COUNTER(gps);
                    uint8_t rv = ParsePgn126992(msg.payload, NULL, NULL, &dateTimeDataStore.year, &dateTimeDataStore.month, &dateTimeDataStore.day, &dateTimeDataStore.hour, &dateTimeDataStore.min, &dateTimeDataStore.sec, &dateTimeDataStore.usecSinceEpoch);
                    // Check if all 6 parts of the datetime were successfully decoded before triggering an update
                    if ((rv & 0xFC) == 0xFC) {
                        SENSOR_STATE_CLEAR_ACTIVE_COUNTER(gps);
                        dateTimeDataStore.newData = true;
                    }
                }
                break;
                case PGN_ID_RUDDER:
                {
                    // Overloaded message that can either be commands from the RC node or the rudder
                    // angle from the rudder node. Since the Parse* function only stores valid data,
                    // we can just pass in both variables to be written to.
                    uint8_t rv = ParsePgn127245(msg.payload, NULL, NULL,
                                                &currentCommands.secondaryManualRudderCommand,
                                                &rudderSensorData.RudderAngle);
                    // If a valid rudder angle was received, the rudder node is enabled.
                    if ((rv & 0x08)) {
                        SENSOR_STATE_CLEAR_ENABLED_COUNTER(rudder);
                    }
                }
                break;
                case PGN_ID_BATTERY_STATUS:
                { // From the Power Node
                    SENSOR_STATE_CLEAR_ENABLED_COUNTER(power);
                    uint8_t rv = ParsePgn127508(msg.payload, NULL, NULL, &powerDataStore.voltage, &powerDataStore.current, &powerDataStore.temperature);
                    if ((rv & 0x0C) == 0xC) {
                        SENSOR_STATE_CLEAR_ACTIVE_COUNTER(power);
                        powerDataStore.newData = true;
                    }
                }
                break;
                case PGN_ID_SPEED: // From the DST800
                    SENSOR_STATE_CLEAR_ENABLED_COUNTER(dst800);
                    if (ParsePgn128259(msg.payload, NULL, &waterDataStore.speed)) {
                        SENSOR_STATE_CLEAR_ACTIVE_COUNTER(dst800);
                        waterDataStore.newData = true;
                    }
                    break;
                case PGN_ID_WATER_DEPTH:
                { // From the DST800
                    SENSOR_STATE_CLEAR_ENABLED_COUNTER(dst800);
                    // Only update the data in waterDataStore if an actual depth was returned.
                    uint8_t rv = ParsePgn128267(msg.payload, NULL, &waterDataStore.depth, NULL);
                    if ((rv & 0x02) == 0x02) {
                        SENSOR_STATE_CLEAR_ACTIVE_COUNTER(dst800);
                        waterDataStore.newData = true;
                    }
                }
                break;
                case PGN_ID_POSITION_RAP_UPD:
                { // From the GPS200
                    // Keep the GPS enabled
                    SENSOR_STATE_CLEAR_ENABLED_COUNTER(gps);

                    // Decode the position
                    int32_t lat, lon;
                    uint8_t rv = ParsePgn129025(msg.payload, &lat, &lon);

                    // Only do something if both latitude and longitude were parsed successfully and
                    // the last fix update we got says that the data is good.
                    // Additionally jumps to 0,0 are ignored. I've seen this happen a few times.
                    // Note that only unique position readings are allowed. This check is due to the
                    // GPS200 unit being used outputting data at 5Hz, but only internally updating
                    // at 4Hz. To prevent backtracking, ignoring duplicate positions is done.
                    if ((rv & 0x03) == 0x03 &&
                        (gpsDataStore.mode == PGN129539_MODE_2D || gpsDataStore.mode == PGN129539_MODE_3D) &&
                        (lat != gpsDataStore.latitude && lon != gpsDataStore.longitude) &&
                        (lat != 0 && lon != 0)) {
                        // Mark that we found new position data
                        gpsDataStore.newData |= GPSDATA_POSITION;

                        // Since we've received good data, keep the GPS active
                        SENSOR_STATE_CLEAR_ACTIVE_COUNTER(gps);

                        // Finally copy the new data into the GPS struct
                        gpsDataStore.latitude = lat;
                        gpsDataStore.longitude = lon;
                    }
                }
                break;
                case PGN_ID_COG_SOG_RAP_UPD:
                { // From the GPS200
                    SENSOR_STATE_CLEAR_ENABLED_COUNTER(gps);
                    uint16_t cog, sog;
                    uint8_t rv = ParsePgn129026(msg.payload, NULL, NULL, &cog, &sog);

                    // Only update if both course-over-ground and speed-over-ground were parsed
                    // and the last reported GPS mode indicates a proper fix.
                    if ((rv & 0x0C) == 0x0C &&
                        (gpsDataStore.mode == PGN129539_MODE_2D || gpsDataStore.mode == PGN129539_MODE_3D)) {
                        // Mark that we found new velocity data
                        gpsDataStore.newData |= GPSDATA_VELOCITY;

                        // Since we've received good data, keep the GPS active
                        SENSOR_STATE_CLEAR_ACTIVE_COUNTER(gps);

                        // Finally copy the new data into the GPS struct
                        gpsDataStore.cog = cog;
                        gpsDataStore.sog = sog;
                    }
                }
                break;
                case PGN_ID_GNSS_DOPS:
                { // From the GPS200
                    SENSOR_STATE_CLEAR_ENABLED_COUNTER(gps);
                    uint8_t rv = ParsePgn129539(msg.payload, NULL, NULL, &gpsDataStore.mode, &gpsDataStore.hdop, &gpsDataStore.vdop, NULL);

                    // If there was valid data in the mode and hdop/vdop fields,
                    if ((rv & 0x1C) == 0x1C) {
                        // Mark that we found new DoP data
                        gpsDataStore.newData |= GPSDATA_DOP;

                        // Since we've received good data, keep the GPS active
                        SENSOR_STATE_CLEAR_ACTIVE_COUNTER(gps);
                    }
                }
                break;
                case PGN_ID_WIND_DATA: // From the WSO100
                    SENSOR_STATE_CLEAR_ENABLED_COUNTER(wso100);
                    if (ParsePgn130306(msg.payload, NULL, &windDataStore.speed, &windDataStore.direction)) {
                        SENSOR_STATE_CLEAR_ACTIVE_COUNTER(wso100);
                        windDataStore.newData = true;
                    }
                    break;
                case PGN_ID_ENV_PARAMETERS: // From the DST800
                    SENSOR_STATE_CLEAR_ENABLED_COUNTER(dst800);
                    if (ParsePgn130310(msg.payload, NULL, &waterDataStore.temp, NULL, NULL)) {
                        // The DST800 is only considered active when a water depth is received
                        waterDataStore.newData = true;
                    }
                    break;
                case PGN_ID_ENV_PARAMETERS2: // From the WSO100
                    SENSOR_STATE_CLEAR_ENABLED_COUNTER(wso100);
                    if (ParsePgn130311(msg.payload, NULL, NULL, NULL, &airDataStore.temp, &airDataStore.humidity, &airDataStore.pressure)) {
                        SENSOR_STATE_CLEAR_ACTIVE_COUNTER(wso100);
                        airDataStore.newData = true;
                    }
                    break;
                case PGN_ID_DC_SOURCE_STATUS:
                    if (Nmea2000FastPacketExtract(msg.validBytes, msg.payload, &dsSourceStatusPacket)) {
                        Pgn127173Data data;
                        ParsePgn127173(dsSourceStatusPacket.messageBytes, &data);
                        if (data.dcSourceId == DC_SOURCE_SOLAR_ARRAY_1) {
                            if (data.current >= 0) {
                                solarDataStore.current = data.current;
                            } else {
                                solarDataStore.current = 0;
                            }
                            if (data.voltage >= 0) {
                                solarDataStore.voltage = data.voltage;
                            } else {
                                solarDataStore.voltage = 0;
                            }
                        }
                    }
                break;
                case PGN_ID_GNSS_POSITION_DATA:
                    if (Nmea2000FastPacketExtract(msg.validBytes, msg.payload, &gnssPositionDataPacket)) {
                        Pgn129029Data data;
                        ParsePgn129029(gnssPositionDataPacket.messageBytes, &data);
                        gpsDataStore.altitude = data.altitude; // Units are the same, just precision differs.
                        gpsDataStore.satellites = data.satellites;
                    }
                break;
                }
            }

            ++messagesHandled;
        }
    } while (messagesLeft > 0);

    return messagesHandled;
}

/**
 * This function should be called at a constant rate (same units as SENSOR_TIMEOUT) and updates the
 * availability of any sensors and onboard nodes. This function is separated from the
 * `ProcessAllEcanMessages()` function because that function should be called as fast as possible,
 * while this one should be called at the base tick rate of the system.
 */
void UpdateSensorsAvailability(void)
{
    // Now if any nodes have timed out, reset their struct data since any data we have for them is
    // now invalid. Otherwise, keep incrementing their timeout counters. These are reset in
    // `ProcessAllEcanMessages()`.
    int i;
    for (i = 0; i < NUM_NODES; ++i) {
        // Be sure to not do this for the current node, as it won't ever receive CAN messages from
        // itself.
        if (i != nodeId - 1) {
            if (nodeStatusTimeoutCounters[i] < NODE_TIMEOUT) {
                ++nodeStatusTimeoutCounters[i];
            } else {
                nodeStatusDataStore[i].errors = UINT16_MAX;
                nodeStatusDataStore[i].load = UINT8_MAX;
                nodeStatusDataStore[i].status = UINT16_MAX;
                nodeStatusDataStore[i].temp = INT8_MAX;
                nodeStatusDataStore[i].voltage = UINT8_MAX;
            }
        }
    }

    // Now update the '.enabled' or '.active' status for every sensor. We keep timeout counters that
    // timeout a sensor after SENSOR_TIMEOUT amount of time, which depends on how often this function
    // is called.
    SENSOR_STATE_UPDATE(gps);
    SENSOR_STATE_UPDATE(imu);
    SENSOR_STATE_UPDATE(wso100);
    SENSOR_STATE_UPDATE(dst800);
    SENSOR_STATE_UPDATE(power);
    SENSOR_STATE_UPDATE(prop);
    SENSOR_STATE_UPDATE(rudder);
    SENSOR_STATE_UPDATE(rcNode);
    SENSOR_STATE_UPDATE(gyro);
    SENSOR_STATE_UPDATE(gps);
}
