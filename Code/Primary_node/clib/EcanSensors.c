#include "EcanSensors.h"
#include "primary_node.h"

#include "ecanDefinitions.h"
#include "ecanFunctions.h"
#include "Nmea2000.h"
#include "Types.h"
#include "Rudder.h"
#include "CanMessages.h"
#include "Node.h"
#include "Acs300.h"

struct PowerData powerDataStore = {};
struct WindData windDataStore = {};
struct AirData airDataStore = {};
struct WaterData waterDataStore = {};
struct ThrottleData throttleDataStore = {};
struct GpsData gpsDataStore = {};
struct DateTimeData dateTimeDataStore = {};
struct RevoGsData revoGsDataStore = {};

struct stc sensorAvailability = {};

void GetWindDataPacked(uint8_t *data)
{
	data[0] = windDataStore.speed.chData[0];
	data[1] = windDataStore.speed.chData[1];
	data[2] = windDataStore.speed.chData[2];
	data[3] = windDataStore.speed.chData[3];
	data[4] = windDataStore.direction.chData[0];
	data[5] = windDataStore.direction.chData[1];
	data[6] = windDataStore.direction.chData[2];
	data[7] = windDataStore.direction.chData[3];
	data[8] = windDataStore.newData;
	windDataStore.newData = false;
}

void GetAirDataPacked(uint8_t *data)
{
	data[0] = airDataStore.temp.chData[0];
	data[1] = airDataStore.temp.chData[1];
	data[2] = airDataStore.temp.chData[2];
	data[3] = airDataStore.temp.chData[3];
	data[4] = airDataStore.pressure.chData[0];
	data[5] = airDataStore.pressure.chData[1];
	data[6] = airDataStore.pressure.chData[2];
	data[7] = airDataStore.pressure.chData[3];
	data[8] = airDataStore.humidity.chData[0];
	data[9] = airDataStore.humidity.chData[1];
	data[10] = airDataStore.humidity.chData[2];
	data[11] = airDataStore.humidity.chData[3];
	data[12] = airDataStore.newData;
	airDataStore.newData = false;
}

void GetWaterDataPacked(uint8_t *data)
{
	data[0] = waterDataStore.speed.chData[0];
	data[1] = waterDataStore.speed.chData[1];
	data[2] = waterDataStore.speed.chData[2];
	data[3] = waterDataStore.speed.chData[3];
	data[4] = waterDataStore.temp.chData[0];
	data[5] = waterDataStore.temp.chData[1];
	data[6] = waterDataStore.temp.chData[2];
	data[7] = waterDataStore.temp.chData[3];
	data[8] = waterDataStore.depth.chData[0];
	data[9] = waterDataStore.depth.chData[1];
	data[10] = waterDataStore.depth.chData[2];
	data[11] = waterDataStore.depth.chData[3];
	data[12] = waterDataStore.newData;
	waterDataStore.newData = false;
}

void GetThrottleDataPacked(uint8_t *data)
{
	data[0] = throttleDataStore.rpm.chData[0];
	data[1] = throttleDataStore.rpm.chData[1];
	data[2] = throttleDataStore.newData;
	throttleDataStore.newData = false;
}

void GetGpsDataPacked(uint8_t *data)
{
	data[0] = gpsDataStore.lat.chData[0];
	data[1] = gpsDataStore.lat.chData[1];
	data[2] = gpsDataStore.lat.chData[2];
	data[3] = gpsDataStore.lat.chData[3];
	data[4] = gpsDataStore.lon.chData[0];
	data[5] = gpsDataStore.lon.chData[1];
	data[6] = gpsDataStore.lon.chData[2];
	data[7] = gpsDataStore.lon.chData[3];
	data[8] = gpsDataStore.alt.chData[0];
	data[9] = gpsDataStore.alt.chData[1];
	data[10] = gpsDataStore.alt.chData[2];
	data[11] = gpsDataStore.alt.chData[3];

	data[12] = gpsDataStore.cog.chData[0];
	data[13] = gpsDataStore.cog.chData[1];
	data[14] = gpsDataStore.sog.chData[0];
	data[15] = gpsDataStore.sog.chData[1];

	data[16] = gpsDataStore.newData;

	// Mark this data as old now
	gpsDataStore.newData = 0;
}

void ClearGpsData(void)
{
	gpsDataStore.lat.lData = 0.0;
	gpsDataStore.lon.lData = 0.0;
	gpsDataStore.alt.lData = 0.0;
	gpsDataStore.cog.usData = 0;
	gpsDataStore.sog.usData = 0;
	gpsDataStore.newData = 0;
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
	tCanMessage msg;
	uint32_t pgn;

	uint8_t messagesHandled = 0;

	// Here we increment the timeout counters for each sensor/actuator we're tracking the status of. This function is assumed to be called at 100Hz and as such the timeout value is 100.
	if (sensorAvailability.gps.enabled_counter < 100) {
		++sensorAvailability.gps.enabled_counter;
	}
	if (sensorAvailability.gps.active_counter < 100) {
		++sensorAvailability.gps.active_counter;
	}
	if (sensorAvailability.revo_gs.enabled_counter < 100) {
		++sensorAvailability.revo_gs.enabled_counter;
	}
	if (sensorAvailability.revo_gs.active_counter < 100) {
		++sensorAvailability.revo_gs.active_counter;
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
		int foundOne = ecan1_receive(&msg, &messagesLeft);
		if (foundOne) {
			// Process throttle messages here. Anything not explicitly handled is assumed to be a NMEA2000 message.
			if (msg.id == ACS300_CAN_ID_HRTBT) { // From the ACS300
				sensorAvailability.prop.enabled_counter = 0;
				if ((msg.payload[6] & 0x40) == 0) { // Checks the status bit to determine if the ACS300 is enabled.
					sensorAvailability.prop.active_counter = 0;
				}
				throttleDataStore.rpm.chData[0] = msg.payload[1];
				throttleDataStore.rpm.chData[1] = msg.payload[0];
				throttleDataStore.newData = true;
			} else if (msg.id == CAN_MSG_ID_STATUS) {
				uint8_t node;
				uint16_t status, errors;
				CanMessageDecodeStatus(&msg, &node, &status, &errors);
				if (node == CAN_NODE_RC) {
					sensorAvailability.rcNode.enabled_counter = 0;
					if (status & 0x01) {
						sensorAvailability.rcNode.active_counter = 0;
					}
				}
			} else if (msg.id == CAN_MSG_ID_RUDDER_DETAILS) {
				sensorAvailability.rudder.enabled_counter = 0;
				CanMessageDecodeRudderDetails(&msg,
											  &rudderSensorData.RudderPotValue.usData,
											  &rudderSensorData.RudderPotLimitStarboard.usData,
											  &rudderSensorData.RudderPotLimitPort.usData,
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
					if (ParsePgn127245(msg.payload, NULL, NULL, NULL, NULL, &rudderSensorData.RudderAngle.flData) == 0x10){
						// No action necessary.
					}
				} break;
				case PGN_BATTERY_STATUS: { // From the Power Node
					sensorAvailability.power.enabled_counter = 0;
					uint8_t rv = ParsePgn127508(msg.payload, NULL, NULL, &powerDataStore.voltage.flData, &powerDataStore.current.flData, &powerDataStore.temperature.flData);
					if ((rv & 0x0C) == 0xC) {
						sensorAvailability.power.active_counter = 0;
						powerDataStore.newData = true;
					}
				} break;
				case PGN_SPEED: // From the WSO100
					sensorAvailability.wso100.enabled_counter = 0;
					if (ParsePgn128259(msg.payload, NULL, &waterDataStore.speed.flData)) {
						sensorAvailability.wso100.active_counter = 0;
						waterDataStore.newData = true;
					}
				break;
				case PGN_WATER_DEPTH: { // From the DST800
					sensorAvailability.dst800.enabled_counter = 0;
					// Only update the data in waterDataStore if an actual depth was returned.
					uint8_t rv = ParsePgn128267(msg.payload, NULL, &waterDataStore.depth.flData, NULL);
					if ((rv & 0x02) == 0x02) {
						sensorAvailability.dst800.active_counter = 0;
						waterDataStore.newData = true;
					}
				} break;
				case PGN_POSITION_RAP_UPD: { // From the GPS100
					// Only record the live GPS data if we aren't in HIL mode.
					if ((systemStatus.status & (1 << 1)) == 0) {
						sensorAvailability.gps.enabled_counter = 0;
						uint8_t rv = ParsePgn129025(msg.payload, &gpsDataStore.lat.lData, &gpsDataStore.lon.lData);
						// Only update if both latitude and longitude were parsed successfully.
						if ((rv & 0x03) == 0x03) {
							sensorAvailability.gps.active_counter = 0;
							gpsDataStore.newData = true;
						}
					}
				} break;
				case PGN_COG_SOG_RAP_UPD: { // From the GPS100
					// Only record the live GPS data if we aren't in HIL mode.
					if ((systemStatus.status & (1 << 1)) == 0) {
						sensorAvailability.gps.enabled_counter = 0;
						uint8_t rv = ParsePgn129026(msg.payload, NULL, NULL, &gpsDataStore.cog.usData, &gpsDataStore.sog.usData);
						// Only update if both course-over-ground and speed-over-ground were parsed successfully.
						if ((rv & 0x0C) == 0x0C) {
							sensorAvailability.gps.active_counter = 0;
							gpsDataStore.newData = true;
						}
					}
				} break;
				case PGN_WIND_DATA: // From the WSO100
					sensorAvailability.wso100.enabled_counter = 0;
					if (ParsePgn130306(msg.payload, NULL, &windDataStore.speed.flData, &windDataStore.direction.flData)) {
						sensorAvailability.wso100.active_counter = 0;
						windDataStore.newData = true;
					}
				break;
				case PGN_ENV_PARAMETERS: // From the DST800
					sensorAvailability.dst800.enabled_counter = 0;
					if (ParsePgn130310(msg.payload, NULL, &waterDataStore.temp.flData, NULL, NULL)) {
						// The DST800 is only considered active when a water depth is received
						waterDataStore.newData = true;
					}
				break;
				case PGN_ENV_PARAMETERS2: // From the WSO100
					sensorAvailability.wso100.enabled_counter = 0;
					if (ParsePgn130311(msg.payload, NULL, NULL, NULL, &airDataStore.temp.flData, &airDataStore.humidity.flData, &airDataStore.pressure.flData)) {
						sensorAvailability.wso100.active_counter = 0;
						airDataStore.newData = true;
					}
				break;
				}
			}

			++messagesHandled;
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
	if (sensorAvailability.revo_gs.enabled && sensorAvailability.revo_gs.enabled_counter >= 100) {
		sensorAvailability.revo_gs.enabled = false;
	} else if (!sensorAvailability.revo_gs.enabled && sensorAvailability.revo_gs.enabled_counter == 0) {
		sensorAvailability.revo_gs.enabled = true;
	}
	if (sensorAvailability.revo_gs.active && sensorAvailability.revo_gs.active_counter >= 100) {
		sensorAvailability.revo_gs.active = false;
	} else if (!sensorAvailability.revo_gs.active && sensorAvailability.revo_gs.active_counter == 0) {
		sensorAvailability.revo_gs.active = true;
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
