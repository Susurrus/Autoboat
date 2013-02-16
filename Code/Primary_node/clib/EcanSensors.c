#include "EcanSensors.h"
#include "primary_node.h"

#include "Ecan1.h"
#include "Nmea2000.h"
#include "Types.h"
#include "Rudder.h"
#include "CanMessages.h"
#include "Node.h"
#include "Acs300.h"
#include "Packing.h"

struct PowerData powerDataStore = {};
struct WindData windDataStore = {};
struct AirData airDataStore = {};
struct WaterData waterDataStore = {};
struct ThrottleData throttleDataStore = {};
struct GpsData gpsDataStore = {};
struct GpsDataBundle gpsNewDataStore = {};
struct DateTimeData dateTimeDataStore = {};
struct RevoGsData revoGsDataStore = {};

struct stc sensorAvailability = {};

void GetWindDataPacked(uint8_t *data)
{
	LEPackReal32(&data[0], windDataStore.speed);
	LEPackReal32(&data[4], windDataStore.direction);
	data[8] = windDataStore.newData;
	windDataStore.newData = false;
}

void GetAirDataPacked(uint8_t *data)
{
	LEPackReal32(&data[0], airDataStore.temp);
	LEPackReal32(&data[4], airDataStore.pressure);
	LEPackReal32(&data[8], airDataStore.humidity);
	data[12] = airDataStore.newData;
	airDataStore.newData = false;
}

void GetWaterDataPacked(uint8_t *data)
{
	LEPackReal32(&data[0], waterDataStore.speed);
	LEPackReal32(&data[4], waterDataStore.temp);
	LEPackReal32(&data[8], waterDataStore.depth);
	data[12] = waterDataStore.newData;
	waterDataStore.newData = false;
}

void GetThrottleDataPacked(uint8_t *data)
{
	LEPackInt16(&data[0], throttleDataStore.rpm);
	data[2] = throttleDataStore.newData;
	throttleDataStore.newData = false;
}

void GetGpsDataPacked(uint8_t *data)
{
	LEPackInt32(&data[0], gpsDataStore.lat);
	LEPackInt32(&data[4], gpsDataStore.lon);
	LEPackInt32(&data[8], gpsDataStore.alt);
	LEPackUint16(&data[12], gpsDataStore.cog);
	LEPackUint16(&data[14], gpsDataStore.sog);

	data[16] = gpsDataStore.newData;

	// Mark this data as old now
	gpsDataStore.newData = 0;
}

void ClearGpsData(void)
{
	gpsDataStore.lat = 0.0;
	gpsDataStore.lon = 0.0;
	gpsDataStore.alt = 0.0;
	gpsDataStore.cog = 0;
	gpsDataStore.sog = 0;
	gpsDataStore.newData = 0;
}

uint8_t GetGpsEnabled(void)
{
    return sensorAvailability.gps.enabled;
}

uint8_t GetPropActive(void)
{
    return sensorAvailability.prop.enabled;
}

/**
 * A MATLAB-helper function for returning the RC Node status.
 * @param enabled A 2-element array for returning the enabled/active status of the RcNode.
 */
void GetRcNodeAvailability(bool *status)
{
	status[0] = sensorAvailability.rcNode.enabled;
	status[1] = sensorAvailability.rcNode.active;
}

uint8_t ProcessAllEcanMessages(void)
{
	uint8_t messagesLeft = 0;
	CanMessage msg;
	uint32_t pgn;

	uint8_t messagesHandled = 0;

	// Here we increment the timeout counters for each sensor/actuator we're tracking the status of. This function is assumed to be called at 100Hz and as such the timeout value is 100.
	if (sensorAvailability.gps.enabled_counter < 100) {
		++sensorAvailability.gps.enabled_counter;
	}
	if (sensorAvailability.gps.active_counter < 100) {
		++sensorAvailability.gps.active_counter;
	}
	if (sensorAvailability.imu.enabled_counter < 100) {
		++sensorAvailability.imu.enabled_counter;
	}
	if (sensorAvailability.imu.active_counter < 100) {
		++sensorAvailability.imu.active_counter;
	}
	if (sensorAvailability.wso100.enabled_counter < 100) {
		++sensorAvailability.wso100.enabled_counter;
	}
	if (sensorAvailability.wso100.active_counter < 100) {
		++sensorAvailability.wso100.active_counter;
	}
	if (sensorAvailability.dst800.enabled_counter < 100) {
		++sensorAvailability.dst800.enabled_counter;
	}
	if (sensorAvailability.dst800.active_counter < 100) {
		++sensorAvailability.dst800.active_counter;
	}
	if (sensorAvailability.power.enabled_counter < 100) {
		++sensorAvailability.power.enabled_counter;
	}
	if (sensorAvailability.power.active_counter < 100) {
		++sensorAvailability.power.active_counter;
	}
	if (sensorAvailability.prop.enabled_counter < 100) {
		++sensorAvailability.prop.enabled_counter;
	}
	if (sensorAvailability.prop.active_counter < 100) {
		++sensorAvailability.prop.active_counter;
	}
	if (sensorAvailability.rudder.enabled_counter < 100) {
		++sensorAvailability.rudder.enabled_counter;
	}
	if (sensorAvailability.rudder.active_counter < 100) {
		++sensorAvailability.rudder.active_counter;
	}
	if (sensorAvailability.rcNode.enabled_counter < 100) {
		++sensorAvailability.rcNode.enabled_counter;
	}
	if (sensorAvailability.rcNode.active_counter < 100) {
		++sensorAvailability.rcNode.active_counter;
	}

	do {
		int foundOne = Ecan1Receive(&msg, &messagesLeft);
		if (foundOne) {
			// Process throttle messages here. Anything not explicitly handled is assumed to be a NMEA2000 message.
			if (msg.id == ACS300_CAN_ID_HRTBT) { // From the ACS300
				sensorAvailability.prop.enabled_counter = 0;
				if ((msg.payload[6] & 0x40) == 0) { // Checks the status bit to determine if the ACS300 is enabled.
					sensorAvailability.prop.active_counter = 0;
				}
				Acs300DecodeHeartbeat(msg.payload, (uint16_t*)&throttleDataStore.rpm, NULL, NULL, NULL);
				throttleDataStore.newData = true;
			} else if (msg.id == CAN_MSG_ID_STATUS) {
				uint8_t node;
				uint16_t status;
				CanMessageDecodeStatus(&msg, &node, NULL, NULL, NULL, &status, NULL);
				if (node == CAN_NODE_RC) {
					sensorAvailability.rcNode.enabled_counter = 0;
					// Only if the RC transmitter is connected should the RC node be considered
					// active.
					if (status & 0x01) {
						sensorAvailability.rcNode.active_counter = 0;
					}
				}
			} else if (msg.id == CAN_MSG_ID_RUDDER_DETAILS) {
				sensorAvailability.rudder.enabled_counter = 0;
				CanMessageDecodeRudderDetails(&msg,
											  &rudderSensorData.RudderPotValue,
											  &rudderSensorData.RudderPotLimitStarboard,
											  &rudderSensorData.RudderPotLimitPort,
											  &rudderSensorData.LimitHitStarboard,
											  &rudderSensorData.LimitHitPort,
											  &rudderSensorData.Enabled,
											  &rudderSensorData.Calibrated,
											  &rudderSensorData.Calibrating);
				if (rudderSensorData.Enabled &&
					rudderSensorData.Calibrated &&
					!rudderSensorData.Calibrating) {
					sensorAvailability.rudder.active_counter = 0;
				}
			} else if (msg.id == CAN_MSG_ID_IMU_DATA) {
				sensorAvailability.imu.enabled_counter = 0;
				sensorAvailability.imu.active_counter = 0;
				CanMessageDecodeImuData(&msg,
										&revoGsDataStore.heading,
                                        &revoGsDataStore.pitch,
                                        &revoGsDataStore.roll);
			} else {
				pgn = Iso11783Decode(msg.id, NULL, NULL, NULL);
				switch (pgn) {
				case PGN_SYSTEM_TIME: { // From GPS
					sensorAvailability.gps.enabled_counter = 0;
					uint8_t rv = ParsePgn126992(msg.payload, NULL, NULL, &dateTimeDataStore.year, &dateTimeDataStore.month, &dateTimeDataStore.day, &dateTimeDataStore.hour, &dateTimeDataStore.min, &dateTimeDataStore.sec, &dateTimeDataStore.usecSinceEpoch);
					// Check if all 6 parts of the datetime were successfully decoded before triggering an update
					if ((rv & 0xFC) == 0xFC) {
						sensorAvailability.gps.active_counter = 0;
						dateTimeDataStore.newData = true;
					}
				} break;
				case PGN_RUDDER: { // From the Rudder Controller
					if (ParsePgn127245(msg.payload, NULL, NULL, NULL, &rudderSensorData.RudderAngle) == 0x10){
						// No action necessary.
					}
				} break;
				case PGN_BATTERY_STATUS: { // From the Power Node
					sensorAvailability.power.enabled_counter = 0;
					uint8_t rv = ParsePgn127508(msg.payload, NULL, NULL, &powerDataStore.voltage, &powerDataStore.current, &powerDataStore.temperature);
					if ((rv & 0x0C) == 0xC) {
						sensorAvailability.power.active_counter = 0;
						powerDataStore.newData = true;
					}
				} break;
				case PGN_SPEED: // From the WSO100
					sensorAvailability.wso100.enabled_counter = 0;
					if (ParsePgn128259(msg.payload, NULL, &waterDataStore.speed)) {
						sensorAvailability.wso100.active_counter = 0;
						waterDataStore.newData = true;
					}
				break;
				case PGN_WATER_DEPTH: { // From the DST800
					sensorAvailability.dst800.enabled_counter = 0;
					// Only update the data in waterDataStore if an actual depth was returned.
					uint8_t rv = ParsePgn128267(msg.payload, NULL, &waterDataStore.depth, NULL);
					if ((rv & 0x02) == 0x02) {
						sensorAvailability.dst800.active_counter = 0;
						waterDataStore.newData = true;
					}
				} break;
				case PGN_POSITION_RAP_UPD: { // From the GPS200
					sensorAvailability.gps.enabled_counter = 0;
					uint8_t rv = ParsePgn129025(msg.payload, &gpsNewDataStore.lat, &gpsNewDataStore.lon);
					// Only do something if both latitude and longitude were parsed successfully.
					if ((rv & 0x03) == 0x03) {
						// If there was already position data in there, assume this is the start of a new clustering and clear out old data.
						if (gpsNewDataStore.receivedMessages & GPSDATA_POSITION) {
							gpsNewDataStore.receivedMessages = GPSDATA_POSITION;
						}
						// Otherwise we...
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
									memcpy(&gpsDataStore, &gpsNewDataStore, sizeof(gpsNewDataStore));
									gpsDataStore.newData = true;

									// Also set our GPS as receiving good data.
									sensorAvailability.gps.active_counter = 0;
								}

								// Regardless of whether this data was useful, we clear for next bundle.
								gpsNewDataStore.receivedMessages = 0;
							}
						}
					}
				} break;
				case PGN_COG_SOG_RAP_UPD: { // From the GPS200
					sensorAvailability.gps.enabled_counter = 0;
					uint8_t rv = ParsePgn129026(msg.payload, NULL, NULL, &gpsNewDataStore.cog, &gpsNewDataStore.sog);
					// Only update if both course-over-ground and speed-over-ground were parsed successfully.
					if ((rv & 0x0C) == 0x0C) {
						// If there was already heading data in there, assume this is the start of a new clustering and clear out old data.
						if (gpsNewDataStore.receivedMessages & GPSDATA_HEADING) {
							gpsNewDataStore.receivedMessages = GPSDATA_HEADING;
						}
						// Otherwise we...
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
									memcpy(&gpsDataStore, &gpsNewDataStore, sizeof(gpsNewDataStore));
									gpsDataStore.newData = true;

									// Also set our GPS as receiving good data.
									sensorAvailability.gps.active_counter = 0;
								}

								// Regardless of whether this data was useful, we clear for next bundle.
								gpsNewDataStore.receivedMessages = 0;
							}
						}
					}
				} break;
				case PGN_GNSS_DOPS: { // From the GPS200
					sensorAvailability.gps.enabled_counter = 0;
					uint8_t rv = ParsePgn129539(msg.payload, NULL, NULL, &gpsNewDataStore.mode, &gpsNewDataStore.hdop, &gpsNewDataStore.vdop, NULL);
					if ((rv & 0x1C) == 0x1C) {
						// If there was already heading data in there, assume this is the start of a new clustering and clear out old data.
						if (gpsNewDataStore.receivedMessages & GPSDATA_FIX) {
							gpsNewDataStore.receivedMessages = GPSDATA_FIX;
						}
						// Otherwise we...
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
									memcpy(&gpsDataStore, &gpsNewDataStore, sizeof(gpsNewDataStore));
									gpsDataStore.newData = true;
									// Also set our GPS as receiving good data.
									sensorAvailability.gps.active_counter = 0;
								}
								
								// Regardless of whether this data was useful, we clear for next bundle.
								gpsNewDataStore.receivedMessages = 0;
							}
						}
					}
				} break;
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
			LATBbits.LATB8 = 0;
			LATBbits.LATB9 = 0;
		}
	} while (messagesLeft > 0);

	UpdateSensorsAvailability();

	return messagesHandled;
}

void UpdateSensorsAvailability(void)
{
	// Now disable or enable sensors as needed.
	if (sensorAvailability.gps.enabled && sensorAvailability.gps.enabled_counter >= 100) {
		sensorAvailability.gps.enabled = false;
	} else if (!sensorAvailability.gps.enabled && sensorAvailability.gps.enabled_counter == 0) {
		sensorAvailability.gps.enabled = true;
	}
	if (sensorAvailability.gps.active && sensorAvailability.gps.active_counter >= 100) {
		sensorAvailability.gps.active = false;
	} else if (!sensorAvailability.gps.active && sensorAvailability.gps.active_counter == 0) {
		sensorAvailability.gps.active = true;
	}
	if (sensorAvailability.imu.enabled && sensorAvailability.imu.enabled_counter >= 100) {
		sensorAvailability.imu.enabled = false;
	} else if (!sensorAvailability.imu.enabled && sensorAvailability.imu.enabled_counter == 0) {
		sensorAvailability.imu.enabled = true;
	}
	if (sensorAvailability.imu.active && sensorAvailability.imu.active_counter >= 100) {
		sensorAvailability.imu.active = false;
	} else if (!sensorAvailability.imu.active && sensorAvailability.imu.active_counter == 0) {
		sensorAvailability.imu.active = true;
	}
	if (sensorAvailability.wso100.enabled && sensorAvailability.wso100.enabled_counter >= 100) {
		sensorAvailability.wso100.enabled = false;
	} else if (!sensorAvailability.wso100.enabled && sensorAvailability.wso100.enabled_counter == 0) {
		sensorAvailability.wso100.enabled = true;
	}
	if (sensorAvailability.wso100.active && sensorAvailability.wso100.active_counter >= 100) {
		sensorAvailability.wso100.active = false;
	} else if (!sensorAvailability.wso100.active && sensorAvailability.wso100.active_counter == 0) {
		sensorAvailability.wso100.active = true;
	}
	if (sensorAvailability.dst800.enabled && sensorAvailability.dst800.enabled_counter >= 100) {
		sensorAvailability.dst800.enabled = false;
	} else if (!sensorAvailability.dst800.enabled && sensorAvailability.dst800.enabled_counter == 0) {
		sensorAvailability.dst800.enabled = true;
	}
	if (sensorAvailability.dst800.active && sensorAvailability.dst800.active_counter >= 100) {
		sensorAvailability.dst800.active = false;
	} else if (!sensorAvailability.dst800.active && sensorAvailability.dst800.active_counter == 0) {
		sensorAvailability.dst800.active = true;
	}
	if (sensorAvailability.power.enabled && sensorAvailability.power.enabled_counter >= 100) {
		sensorAvailability.power.enabled = false;
	} else if (!sensorAvailability.power.enabled && sensorAvailability.power.enabled_counter == 0) {
		sensorAvailability.power.enabled = true;
	}
	if (sensorAvailability.power.active && sensorAvailability.power.active_counter >= 100) {
		sensorAvailability.power.active = false;
	} else if (!sensorAvailability.power.active && sensorAvailability.power.active_counter == 0) {
		sensorAvailability.power.active = true;
	}
	if (sensorAvailability.prop.enabled && sensorAvailability.prop.enabled_counter >= 100) {
		sensorAvailability.prop.enabled = false;
	} else if (!sensorAvailability.prop.enabled && sensorAvailability.prop.enabled_counter == 0) {
		sensorAvailability.prop.enabled = true;
	}
	if (sensorAvailability.prop.active && sensorAvailability.prop.active_counter >= 100) {
		sensorAvailability.prop.active = false;
	} else if (!sensorAvailability.prop.active && sensorAvailability.prop.active_counter == 0) {
		sensorAvailability.prop.active = true;
	}
	if (sensorAvailability.rudder.enabled && sensorAvailability.rudder.enabled_counter >= 100) {
		sensorAvailability.rudder.enabled = false;
	} else if (!sensorAvailability.rudder.enabled && sensorAvailability.rudder.enabled_counter == 0) {
		sensorAvailability.rudder.enabled = true;
	}
	if (sensorAvailability.rudder.active && sensorAvailability.rudder.active_counter >= 100) {
		sensorAvailability.rudder.active = false;
	} else if (!sensorAvailability.rudder.active && sensorAvailability.rudder.active_counter == 0) {
		sensorAvailability.rudder.active = true;
	}
	if (sensorAvailability.rcNode.enabled && sensorAvailability.rcNode.enabled_counter >= 100) {
		sensorAvailability.rcNode.enabled = false;
	} else if (!sensorAvailability.rcNode.enabled && sensorAvailability.rcNode.enabled_counter == 0) {
		sensorAvailability.rcNode.enabled = true;
	}
	if (sensorAvailability.rcNode.active && sensorAvailability.rcNode.active_counter >= 100) {
		sensorAvailability.rcNode.active = false;
	} else if (!sensorAvailability.rcNode.active && sensorAvailability.rcNode.active_counter == 0) {
		sensorAvailability.rcNode.active = true;
	}
}
