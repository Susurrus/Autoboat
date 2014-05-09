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

float GetWaterSpeed(void)
{
    waterDataStore.newData = false;
    return waterDataStore.speed;
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

    // Here we increment the timeout counters for each sensor/actuator we're tracking the status of. This function is assumed to be called at 100Hz and as such the timeout value is 100.
    SENSOR_TIMEOUT_COUNTER_INCREMENT(gps);
    SENSOR_TIMEOUT_COUNTER_INCREMENT(imu);
    SENSOR_TIMEOUT_COUNTER_INCREMENT(wso100);
    SENSOR_TIMEOUT_COUNTER_INCREMENT(dst800);
    SENSOR_TIMEOUT_COUNTER_INCREMENT(power);
    SENSOR_TIMEOUT_COUNTER_INCREMENT(prop);
    SENSOR_TIMEOUT_COUNTER_INCREMENT(rudder);
    SENSOR_TIMEOUT_COUNTER_INCREMENT(rcNode);
    SENSOR_TIMEOUT_COUNTER_INCREMENT(gyro);

    // Increment all of the node timeout counters if they haven't timed-out yet.
    int i;
    for (i = 0; i < NUM_NODES; ++i) {
        // Be sure to not do this for the current node, as it won't ever receive CAN messages from
        // itself.
        if (i != nodeId - 1) { // Subtract 1 to account for 0-indexing of arrays.
            if (nodeStatusTimeoutCounters[i] < NODE_TIMEOUT) {
                ++nodeStatusTimeoutCounters[i];
            }
        }
    }

    do {
        int foundOne = Ecan1Receive(&msg, &messagesLeft);
        if (foundOne) {
            // Process non-NMEA2000 messages here. They're distinguished by having standard frames.
            if (msg.frame_type == CAN_FRAME_STD) {
                if (msg.id == ACS300_CAN_ID_HRTBT) { // From the ACS300
                    sensorAvailability.prop.enabled_counter = 0;
                    if ((msg.payload[6] & 0x40) == 0) { // Checks the status bit to determine if the ACS300 is enabled.
                        sensorAvailability.prop.active_counter = 0;
                    }
                    Acs300DecodeHeartbeat(msg.payload, (uint16_t*) & throttleDataStore.rpm, NULL, NULL, NULL);
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

                        // And add some extra logic for integrating the RC node statusAvailability
                        // logic.
                        if (node == CAN_NODE_RC) {
                            sensorAvailability.rcNode.enabled_counter = 0;
                            // Only if the RC transmitter is connected and in override mode should the RC node be considered
                            // active.
                            if (status & 0x01) {
                                sensorAvailability.rcNode.active_counter = 0;
                            }
                        }
                    }
                } else if (msg.id == CAN_MSG_ID_RUDDER_DETAILS) {
                    sensorAvailability.rudder.enabled_counter = 0;
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
                        sensorAvailability.rudder.active_counter = 0;
                    }
                    // Track transitions in rudder calibrating state.
                    if (nodeErrors & PRIMARY_NODE_RESET_CALIBRATING) {
                        if (!rudderSensorData.Calibrating) {
                            nodeErrors &= ~PRIMARY_NODE_RESET_CALIBRATING;
                        }
                    } else {
                        if (rudderSensorData.Calibrating) {
                            nodeErrors |= PRIMARY_NODE_RESET_CALIBRATING;
                        }
                    }
                    // Track transitions in rudder calibrated state.
                    if (nodeErrors & PRIMARY_NODE_RESET_UNCALIBRATED) {
                        if (rudderSensorData.Calibrated) {
                            nodeErrors &= ~PRIMARY_NODE_RESET_UNCALIBRATED;
                        }
                    } else {
                        if (!rudderSensorData.Calibrated) {
                            nodeErrors |= PRIMARY_NODE_RESET_UNCALIBRATED;
                        }
                    }
                } else if (msg.id == CAN_MSG_ID_IMU_DATA) {
                    sensorAvailability.imu.enabled_counter = 0;
                    sensorAvailability.imu.active_counter = 0;
                    CanMessageDecodeImuData(&msg,
                            &tokimecDataStore.yaw,
                            &tokimecDataStore.pitch,
                            &tokimecDataStore.roll);
                } else if (msg.id == CAN_MSG_ID_GYRO_DATA) {
                    sensorAvailability.gyro.enabled_counter = 0;
                    sensorAvailability.gyro.active_counter = 0;
                    CanMessageDecodeGyroData(&msg, &gyroDataStore.zRate);
                } else if (msg.id == CAN_MSG_ID_ANG_VEL_DATA) {
                    sensorAvailability.imu.enabled_counter = 0;
                    sensorAvailability.imu.active_counter = 0;
                    CanMessageDecodeAngularVelocityData(&msg,
                            &tokimecDataStore.x_angle_vel,
                            &tokimecDataStore.y_angle_vel,
                            &tokimecDataStore.z_angle_vel);
                } else if (msg.id == CAN_MSG_ID_ACCEL_DATA) {
                    sensorAvailability.imu.enabled_counter = 0;
                    sensorAvailability.imu.active_counter = 0;
                    CanMessageDecodeAccelerationData(&msg,
                            &tokimecDataStore.x_accel,
                            &tokimecDataStore.y_accel,
                            &tokimecDataStore.z_accel);
                } else if (msg.id == CAN_MSG_ID_GPS_POS_DATA) {
                    sensorAvailability.imu.enabled_counter = 0;
                    sensorAvailability.imu.active_counter = 0;
                    CanMessageDecodeGpsPosData(&msg,
                            &tokimecDataStore.latitude,
                            &tokimecDataStore.longitude);
                } else if (msg.id == CAN_MSG_ID_GPS_EST_POS_DATA) {
                    sensorAvailability.imu.enabled_counter = 0;
                    sensorAvailability.imu.active_counter = 0;
                    CanMessageDecodeGpsPosData(&msg,
                            &tokimecDataStore.est_latitude,
                            &tokimecDataStore.est_longitude);
                } else if (msg.id == CAN_MSG_ID_GPS_VEL_DATA) {
                    sensorAvailability.imu.enabled_counter = 0;
                    sensorAvailability.imu.active_counter = 0;
                    CanMessageDecodeGpsVelData(&msg,
                            &tokimecDataStore.gpsDirection,
                            &tokimecDataStore.gpsSpeed,
                            &tokimecDataStore.magneticBearing,
                            &tokimecDataStore.status);
                }
            } else {
                pgn = Iso11783Decode(msg.id, NULL, NULL, NULL);
                switch (pgn) {
                case PGN_SYSTEM_TIME:
                { // From GPS
                    sensorAvailability.gps.enabled_counter = 0;
                    uint8_t rv = ParsePgn126992(msg.payload, NULL, NULL, &dateTimeDataStore.year, &dateTimeDataStore.month, &dateTimeDataStore.day, &dateTimeDataStore.hour, &dateTimeDataStore.min, &dateTimeDataStore.sec, &dateTimeDataStore.usecSinceEpoch);
                    // Check if all 6 parts of the datetime were successfully decoded before triggering an update
                    if ((rv & 0xFC) == 0xFC) {
                        sensorAvailability.gps.active_counter = 0;
                        dateTimeDataStore.newData = true;
                    }
                }
                break;
                case PGN_RUDDER:
                { // From the Rudder Controller
                    // Track rudder messages that are either rudder commands OR actual rudder angles. Since the Parse* function only
                    // stores valid data, we can just pass in both variables to be written to.
                    ParsePgn127245(msg.payload, NULL, NULL, &currentCommands.secondaryManualRudderCommand, &rudderSensorData.RudderAngle);
                }
                break;
                case PGN_BATTERY_STATUS:
                { // From the Power Node
                    sensorAvailability.power.enabled_counter = 0;
                    uint8_t rv = ParsePgn127508(msg.payload, NULL, NULL, &powerDataStore.voltage, &powerDataStore.current, &powerDataStore.temperature);
                    if ((rv & 0x0C) == 0xC) {
                        sensorAvailability.power.active_counter = 0;
                        powerDataStore.newData = true;
                    }
                }
                break;
                case PGN_SPEED: // From the DST800
                    sensorAvailability.dst800.enabled_counter = 0;
                    if (ParsePgn128259(msg.payload, NULL, &waterDataStore.speed)) {
                        sensorAvailability.dst800.active_counter = 0;
                        waterDataStore.newData = true;
                    }
                    break;
                case PGN_WATER_DEPTH:
                { // From the DST800
                    sensorAvailability.dst800.enabled_counter = 0;
                    // Only update the data in waterDataStore if an actual depth was returned.
                    uint8_t rv = ParsePgn128267(msg.payload, NULL, &waterDataStore.depth, NULL);
                    if ((rv & 0x02) == 0x02) {
                        sensorAvailability.dst800.active_counter = 0;
                        waterDataStore.newData = true;
                    }
                }
                break;
                case PGN_POSITION_RAP_UPD:
                { // From the GPS200
                    sensorAvailability.gps.enabled_counter = 0;
                    uint8_t rv = ParsePgn129025(msg.payload, &gpsNewDataStore.lat, &gpsNewDataStore.lon);

                    // Only do something if both latitude and longitude were parsed successfully.
                    if ((rv & 0x03) == 0x03) {
                        // If there was already position data in there, assume this is the start of a new clustering and clear out old data.
                        if (gpsNewDataStore.receivedMessages & GPSDATA_POSITION) {
                            gpsNewDataStore.receivedMessages = GPSDATA_POSITION;
                        }                            // Otherwise we...
                        else {
                            // Record that a position message was received.
                            gpsNewDataStore.receivedMessages |= GPSDATA_POSITION;

                            // And if it was the last message in this bundle, check that it's valid,
                            // and copy it over to the reading struct setting it as new data.
                            if (gpsNewDataStore.receivedMessages == GPSDATA_ALL) {
                                // Validity is checked by verifying that this had a valid 2D/3D fix,
                                // and lat/lon is not 0,
                                if ((gpsNewDataStore.mode == PGN_129539_MODE_2D || gpsNewDataStore.mode == PGN_129539_MODE_3D) &&
                                        (gpsNewDataStore.lat != 0 || gpsNewDataStore.lon != 0)) {

                                    // Copy the entire struct over and then overwrite the newData
                                    // field with a valid "true" value.
                                    memcpy(&gpsDataStore, &gpsNewDataStore, sizeof (gpsNewDataStore));
                                    gpsDataStore.newData = true;

                                    // Also set our GPS as receiving good data.
                                    sensorAvailability.gps.active_counter = 0;
                                }

                                // Regardless of whether this data was useful, we clear for next bundle.
                                gpsNewDataStore.receivedMessages = GPSDATA_NONE;
                            }
                        }
                    }
                }
                break;
                case PGN_COG_SOG_RAP_UPD:
                { // From the GPS200
                    sensorAvailability.gps.enabled_counter = 0;
                    uint8_t rv = ParsePgn129026(msg.payload, NULL, NULL, &gpsNewDataStore.cog, &gpsNewDataStore.sog);

                    // Only update if both course-over-ground and speed-over-ground were parsed successfully.
                    if ((rv & 0x0C) == 0x0C) {
                        // If there was already heading data in there, assume this is the start of a new clustering and clear out old data.
                        if (gpsNewDataStore.receivedMessages & GPSDATA_HEADING) {
                            gpsNewDataStore.receivedMessages = GPSDATA_HEADING;
                        }                            // Otherwise we...
                        else {
                            // Record that a position message was received.
                            gpsNewDataStore.receivedMessages |= GPSDATA_HEADING;

                            // And if it was the last message in this bundle, check that it's valid,
                            // and copy it over to the reading struct setting it as new data.
                            if (gpsNewDataStore.receivedMessages == GPSDATA_ALL) {
                                // Validity is checked by verifying that this had a valid 2D/3D fix,
                                // and lat/lon is not 0,
                                if ((gpsNewDataStore.mode == PGN_129539_MODE_2D || gpsNewDataStore.mode == PGN_129539_MODE_3D) &&
                                        (gpsNewDataStore.lat != 0 || gpsNewDataStore.lon != 0)) {

                                    // Copy the entire struct over and then overwrite the newData
                                    // field with a valid "true" value.
                                    memcpy(&gpsDataStore, &gpsNewDataStore, sizeof (gpsNewDataStore));
                                    gpsDataStore.newData = true;

                                    // Also set our GPS as receiving good data.
                                    sensorAvailability.gps.active_counter = 0;
                                }

                                // Regardless of whether this data was useful, we clear for next bundle.
                                gpsNewDataStore.receivedMessages = GPSDATA_NONE;
                            }
                        }
                    }
                }
                break;
                case PGN_GNSS_DOPS:
                { // From the GPS200
                    sensorAvailability.gps.enabled_counter = 0;
                    uint8_t rv = ParsePgn129539(msg.payload, NULL, NULL, &gpsNewDataStore.mode, &gpsNewDataStore.hdop, &gpsNewDataStore.vdop, NULL);

                    if ((rv & 0x1C) == 0x1C) {
                        // If there was already heading data in there, assume this is the start of a new clustering and clear out old data.
                        if (gpsNewDataStore.receivedMessages & GPSDATA_FIX) {
                            gpsNewDataStore.receivedMessages = GPSDATA_FIX;
                        }                            // Otherwise we...
                        else {
                            // Record that a position message was received.
                            gpsNewDataStore.receivedMessages |= GPSDATA_FIX;

                            // And if it was the last message in this bundle, check that it's valid,
                            // and copy it over to the reading struct setting it as new data.
                            if (gpsNewDataStore.receivedMessages == GPSDATA_ALL) {
                                // Validity is checked by verifying that this had a valid 2D/3D fix,
                                // and lat/lon is not 0,
                                if ((gpsNewDataStore.mode == PGN_129539_MODE_2D || gpsNewDataStore.mode == PGN_129539_MODE_3D) &&
                                        (gpsNewDataStore.lat != 0 || gpsNewDataStore.lon != 0)) {

                                    // Copy the entire struct over and then overwrite the newData
                                    // field with a valid "true" value.
                                    memcpy(&gpsDataStore, &gpsNewDataStore, sizeof (gpsNewDataStore));
                                    gpsDataStore.newData = true;
                                    // Also set our GPS as receiving good data.
                                    sensorAvailability.gps.active_counter = 0;
                                }

                                // Regardless of whether this data was useful, we clear for next bundle.
                                gpsNewDataStore.receivedMessages = GPSDATA_NONE;
                            }
                        }
                    }
                }
                break;
                case PGN_MAG_VARIATION: // From the GPS200
                    ParsePgn127258(msg.payload, NULL, NULL, NULL, &gpsDataStore.variation);
                    break;
                case PGN_WIND_DATA: // From the WSO100
                    sensorAvailability.wso100.enabled_counter = 0;
                    if (ParsePgn130306(msg.payload, NULL, &windDataStore.speed, &windDataStore.direction)) {
                        sensorAvailability.wso100.active_counter = 0;
                        windDataStore.newData = true;
                    }
                    break;
                case PGN_ENV_PARAMETERS: // From the DST800
                    sensorAvailability.dst800.enabled_counter = 0;
                    if (ParsePgn130310(msg.payload, NULL, &waterDataStore.temp, NULL, NULL)) {
                        // The DST800 is only considered active when a water depth is received
                        waterDataStore.newData = true;
                    }
                    break;
                case PGN_ENV_PARAMETERS2: // From the WSO100
                    sensorAvailability.wso100.enabled_counter = 0;
                    if (ParsePgn130311(msg.payload, NULL, NULL, NULL, &airDataStore.temp, &airDataStore.humidity, &airDataStore.pressure)) {
                        sensorAvailability.wso100.active_counter = 0;
                        airDataStore.newData = true;
                    }
                    break;
                }
            }

            ++messagesHandled;
        }
    } while (messagesLeft > 0);

    // Now if any nodes have timed out, reset their struct data. This code doesn't do anything but
    // modify the NodeStatusData struct. All node-disconnection issues that affect this system state
    // is handled in `UpdateSensorsAvailability()`.
    for (i = 0; i < NUM_NODES; ++i) {
        // Be sure to not do this for the current node, as it won't ever receive CAN messages from
        // itself.
        if (i != nodeId - 1) {
            if (nodeStatusTimeoutCounters[i] >= NODE_TIMEOUT) {
                nodeStatusDataStore[i].errors = UINT16_MAX;
                nodeStatusDataStore[i].load = UINT8_MAX;
                nodeStatusDataStore[i].status = UINT16_MAX;
                nodeStatusDataStore[i].temp = INT8_MAX;
                nodeStatusDataStore[i].voltage = UINT8_MAX;
            }
        }
    }

    // Check for any errors on the ECAN peripheral:
    uint8_t errors[2];
    Ecan1GetErrorStatus(errors);
    if (nodeStatus & PRIMARY_NODE_STATUS_ECAN_TX_ERR) {
        if (!errors[0]) {
            nodeStatus &= ~PRIMARY_NODE_STATUS_ECAN_TX_ERR;
        }
    } else {
        if (errors[0]) {
            nodeStatus |= PRIMARY_NODE_STATUS_ECAN_TX_ERR;
        }
    }
    if (nodeStatus & PRIMARY_NODE_STATUS_ECAN_RX_ERR) {
        if (!errors[1]) {
            nodeStatus &= ~PRIMARY_NODE_STATUS_ECAN_RX_ERR;
        }
    } else {
        if (errors[1]) {
            nodeStatus |= PRIMARY_NODE_STATUS_ECAN_RX_ERR;
        }
    }

    UpdateSensorsAvailability();

    return messagesHandled;
}

void UpdateSensorsAvailability(void)
{
    // Turn on the GPS indicator LED depending on the GPS status.
    if (sensorAvailability.gps.enabled && sensorAvailability.gps.enabled_counter >= SENSOR_TIMEOUT) {
        sensorAvailability.gps.enabled = false;
        _LATB15 = OFF;
    } else if (!sensorAvailability.gps.enabled && sensorAvailability.gps.enabled_counter == 0) {
        sensorAvailability.gps.enabled = true;
        _LATB15 = ON;
    }
    if (sensorAvailability.gps.active && sensorAvailability.gps.active_counter >= SENSOR_TIMEOUT) {
        sensorAvailability.gps.active = false;
        nodeStatus |= PRIMARY_NODE_STATUS_GPS_DISCON;
    } else if (!sensorAvailability.gps.active && sensorAvailability.gps.active_counter == 0) {
        sensorAvailability.gps.active = true;
        nodeStatus &= ~PRIMARY_NODE_STATUS_GPS_DISCON;
    }
    if (sensorAvailability.imu.enabled && sensorAvailability.imu.enabled_counter >= SENSOR_TIMEOUT) {
        sensorAvailability.imu.enabled = false;
    } else if (!sensorAvailability.imu.enabled && sensorAvailability.imu.enabled_counter == 0) {
        sensorAvailability.imu.enabled = true;
    }
    if (sensorAvailability.imu.active && sensorAvailability.imu.active_counter >= SENSOR_TIMEOUT) {
        sensorAvailability.imu.active = false;
    } else if (!sensorAvailability.imu.active && sensorAvailability.imu.active_counter == 0) {
        sensorAvailability.imu.active = true;
    }
    if (sensorAvailability.wso100.enabled && sensorAvailability.wso100.enabled_counter >= SENSOR_TIMEOUT) {
        sensorAvailability.wso100.enabled = false;
    } else if (!sensorAvailability.wso100.enabled && sensorAvailability.wso100.enabled_counter == 0) {
        sensorAvailability.wso100.enabled = true;
    }
    if (sensorAvailability.wso100.active && sensorAvailability.wso100.active_counter >= SENSOR_TIMEOUT) {
        sensorAvailability.wso100.active = false;
    } else if (!sensorAvailability.wso100.active && sensorAvailability.wso100.active_counter == 0) {
        sensorAvailability.wso100.active = true;
    }
    if (sensorAvailability.dst800.enabled && sensorAvailability.dst800.enabled_counter >= SENSOR_TIMEOUT) {
        sensorAvailability.dst800.enabled = false;
    } else if (!sensorAvailability.dst800.enabled && sensorAvailability.dst800.enabled_counter == 0) {
        sensorAvailability.dst800.enabled = true;
    }
    if (sensorAvailability.dst800.active && sensorAvailability.dst800.active_counter >= SENSOR_TIMEOUT) {
        sensorAvailability.dst800.active = false;
    } else if (!sensorAvailability.dst800.active && sensorAvailability.dst800.active_counter == 0) {
        sensorAvailability.dst800.active = true;
    }
    if (sensorAvailability.power.enabled && sensorAvailability.power.enabled_counter >= SENSOR_TIMEOUT) {
        sensorAvailability.power.enabled = false;
    } else if (!sensorAvailability.power.enabled && sensorAvailability.power.enabled_counter == 0) {
        sensorAvailability.power.enabled = true;
    }
    if (sensorAvailability.power.active && sensorAvailability.power.active_counter >= SENSOR_TIMEOUT) {
        sensorAvailability.power.active = false;
    } else if (!sensorAvailability.power.active && sensorAvailability.power.active_counter == 0) {
        sensorAvailability.power.active = true;
    }
    // Track the ACS300 board. If it's not transmitting, assume we're in e-stop as that's the only
    // way to tell.
    if (sensorAvailability.prop.enabled && sensorAvailability.prop.enabled_counter >= SENSOR_TIMEOUT) {
        sensorAvailability.prop.enabled = false;
        nodeErrors |= PRIMARY_NODE_RESET_ESTOP;
    } else if (!sensorAvailability.prop.enabled && sensorAvailability.prop.enabled_counter == 0) {
        sensorAvailability.prop.enabled = true;
        nodeErrors &= ~PRIMARY_NODE_RESET_ESTOP;
    }
    if (sensorAvailability.prop.active && sensorAvailability.prop.active_counter >= SENSOR_TIMEOUT) {
        sensorAvailability.prop.active = false;
    } else if (!sensorAvailability.prop.active && sensorAvailability.prop.active_counter == 0) {
        sensorAvailability.prop.active = true;
    }
    // And if the rudder node disconnects, set the uncalibrated reset line. There's no need to peform
    // the inverse check when it becomes active again, because that will be done when the CAN message
    // is received.
    if (sensorAvailability.rudder.enabled && sensorAvailability.rudder.enabled_counter >= SENSOR_TIMEOUT) {
        sensorAvailability.rudder.enabled = false;
        nodeErrors |= PRIMARY_NODE_RESET_UNCALIBRATED;
    } else if (!sensorAvailability.rudder.enabled && sensorAvailability.rudder.enabled_counter == 0) {
        sensorAvailability.rudder.enabled = true;
    }
    if (sensorAvailability.rudder.active && sensorAvailability.rudder.active_counter >= SENSOR_TIMEOUT) {
        sensorAvailability.rudder.active = false;
    } else if (!sensorAvailability.rudder.active && sensorAvailability.rudder.active_counter == 0) {
        sensorAvailability.rudder.active = true;
    }
    /// RC Node:
    // The RC node is considered enabled if it's broadcasting on the CAN bus. If the RC node ever
    // becomes disabled, then we stay in reset. This means the RC node needs to be on and transmitting
    // CAN messages properly to the primary node for the primary node to not be in reset.
    if (sensorAvailability.rcNode.enabled && sensorAvailability.rcNode.enabled_counter >= SENSOR_TIMEOUT) {
        sensorAvailability.rcNode.enabled = false;
        nodeErrors |= PRIMARY_NODE_RESET_MANUAL_OVERRIDE;
    } else if (!sensorAvailability.rcNode.enabled && sensorAvailability.rcNode.enabled_counter == 0) {
        sensorAvailability.rcNode.enabled = true;
        if (!sensorAvailability.rcNode.active) {
            nodeErrors &= ~PRIMARY_NODE_RESET_MANUAL_OVERRIDE;
        }
    }
    // If the RC node stops being active, yet is still enabled, then we aren't in an error state. Otherwise
    // if the RC node is active, we are.
    if (sensorAvailability.rcNode.active && sensorAvailability.rcNode.active_counter >= SENSOR_TIMEOUT) {
        sensorAvailability.rcNode.active = false;
        if (sensorAvailability.rcNode.enabled) {
            nodeErrors &= ~PRIMARY_NODE_RESET_MANUAL_OVERRIDE;
        }
    } else if (!sensorAvailability.rcNode.active && sensorAvailability.rcNode.active_counter == 0) {
        sensorAvailability.rcNode.active = true;
        nodeErrors |= PRIMARY_NODE_RESET_MANUAL_OVERRIDE;
    }
}
