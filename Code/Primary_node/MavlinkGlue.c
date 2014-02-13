/**
 * This file contains all of the MAVLink interfacing necessary by Sealion.
 * It relies heavily on the MavlinkMessageScheduler for scheduling transmission
 * of MAVLink messages such as to not overload the interface.
 *
 * The main functions are at the bottom: MavLinkTransmit() and MavLinkReceive()
 * handle the dispatching of messages (and sending of non-FSM reliant ones) and
 * the reception of messages and forwarding of reception events to the relevent
 * FSMs. The two state machine functions (MavLinkEvaluateMissionState and
 * MavLinkEvaluateParameterState) both contain all of the state logic for the
 * MAVLink mission and parameter protocols. As the specifications for those two
 * protocols are not fully defined they have been tested with QGroundControl to
 * work correctly.
 *
 * This code was written to be as generic as possible. If you remove all of the
 * custom messages and switch the transmission from uart1EnqueueData() it should
 * be almost exclusively relient on modules like MissionManager and the scheduler.
 */
// C standard library includes
#include <stdio.h>

// User code includes
#include "Uart1.h"
#include "MessageScheduler.h"
#include "EcanSensors.h"
#include "Rudder.h"
#include "MavlinkGlue.h"
#include "MissionManager.h"
#include "Node.h"
#include "PrimaryNode.h"
#include "Parameters.h"
#include "DataStore.h"
#include "ParametersHelper.h"

// MATLAB-generated code is included here, really only required for the declaration of the
// InternalVariables struct.
#include "controller.h"

// Declare our internal variable data store for some miscellaneous data output over MAVLink.
InternalVariables controllerVars;

/**
 * This function converts latitude/longitude/altitude into a north/east/down local tangent plane. The
 * code I use by default is auto-generated C code from a Simulink block in the autonomous controller.
 * @param[in] lat/lon/alt in units of 1e7deg/1e7deg/1e3m.
 * @param[out] Output in a north/east/down coordinate frame in units of meters.
 */
extern void lla2ltp(const int32_t[3], float[3]);

// Set up some state machine variables for the parameter protocol
enum PARAM_STATE {
	PARAM_STATE_INACTIVE = 0,

	PARAM_STATE_SINGLETON_SEND_VALUE,

	PARAM_STATE_STREAM_SEND_VALUE,
	PARAM_STATE_STREAM_DELAY
};

// Set up state machine variables for the mission protocol
enum MISSION_STATE {
	MISSION_STATE_INACTIVE = 0,

	// States handling transmitting the mission count. It needs to be retransmit twice.
	MISSION_STATE_SEND_MISSION_COUNT,
	MISSION_STATE_MISSION_COUNT_TIMEOUT,
	MISSION_STATE_SEND_MISSION_COUNT2,
	MISSION_STATE_MISSION_COUNT_TIMEOUT2,
	MISSION_STATE_SEND_MISSION_COUNT3,
	MISSION_STATE_MISSION_COUNT_TIMEOUT3,

	// States handling transmitting a mission item. It needs to be retransmit twice.
	MISSION_STATE_SEND_MISSION_ITEM,
	MISSION_STATE_MISSION_ITEM_TIMEOUT,
	MISSION_STATE_SEND_MISSION_ITEM2,
	MISSION_STATE_MISSION_ITEM_TIMEOUT2,
	MISSION_STATE_SEND_MISSION_ITEM3,
	MISSION_STATE_MISSION_ITEM_TIMEOUT3,

	// States handling transmission of the current mission
	MISSION_STATE_SEND_CURRENT,
	MISSION_STATE_CURRENT_TIMEOUT,
	MISSION_STATE_SEND_CURRENT2,
	MISSION_STATE_CURRENT_TIMEOUT2,

	// States handling sending a mission request
	MISSION_STATE_SEND_MISSION_REQUEST,
	MISSION_STATE_MISSION_REQUEST_TIMEOUT,
	MISSION_STATE_SEND_MISSION_REQUEST2,
	MISSION_STATE_MISSION_REQUEST_TIMEOUT2,
	MISSION_STATE_SEND_MISSION_REQUEST3,
	MISSION_STATE_MISSION_REQUEST_TIMEOUT3
};

// These flags are for use with the SYS_STATUS MAVLink message as a mapping from the Autoboat's
// sensors to the sensors/controllers available in SYS_STATUS.
enum ONBOARD_SENSORS {
	ONBOARD_SENSORS_IMU = (1 << 0) | (1 << 1) | (1 << 2),
	ONBOARD_SENSORS_WSO100 = 1 << 3,
	ONBOARD_SENSORS_GPS = 1 << 5,
	ONBOARD_CONTROL_YAW_POS = 1 << 12,
	ONBOARD_CONTROL_XY_POS = 1 << 14,
	ONBOARD_CONTROL_MOTOR = 1 << 15
};

// Store a module-wide variable for common MAVLink system variables.
static mavlink_system_t mavlink_system = {
	20, // Arbitrarily chosen MAV number
	MAV_COMP_ID_ALL,
	MAV_TYPE_SURFACE_BOAT,
	MAV_STATE_UNINIT,
	MAV_MODE_PREFLIGHT | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED, // The vehicle is booting up and have manual control enabled.
	0 // Unused and unsure of expected usage
};

/** manual control parameters **/
// Specify the button that must be pressed for throttle values to be recorded.
#define TRIGGER_ENABLE_BUTTON 0x0010
// Specify the button that activates rudder calibration.
#define RUDDER_CAL_BUTTON     0x0008
// Autonomous/manual switching is done through QGC

// Latch onto the first groundstation unit and only receive and transmit to it.
static uint8_t groundStationSystemId = 0;
static uint8_t groundStationComponentId = 0;

// Declare a character buffer here to prevent continual allocation/deallocation of MAVLink buffers.
static uint8_t buf[MAVLINK_MAX_PACKET_LEN];
static uint16_t len;

// This is a variable declared in Simulink that contains the GPS origin used for global/local
// coordinate conversions. It's organized as latitude (1e7 degrees), longitude (1e7 degrees), and
// altitude (1e6 meters). 
extern int32_t gpsOrigin[3];

// Track how well MAVLink decoding is going:
// WARN: Possible overflow over long-enough duration
uint16_t mavLinkMessagesReceived = 0;
uint16_t mavLinkMessagesFailedParsing = 0;

// Define a timeout (in units of main timesteps of MavlinkReceive()) for transmitting
// MAVLink messages as part of the PARAMETER and MISSION protocols. Messages will be retransmit
// twice before it's considered hopeless.
#define MAVLINK_RESEND_TIMEOUT 300

// Specify how long between transmitting parameters in a parameter transmission stream.
#define INTRA_PARAM_DELAY 5

// Track manual control data transmit via MAVLink
struct {
	int16_t Rudder;
	int16_t Throttle;
	uint16_t Buttons; 
} mavlinkManualControlData;

// Set up the message scheduler for MAVLink transmission
#define MAVLINK_MSGS_SIZE 16
uint8_t ids[MAVLINK_MSGS_SIZE] = {
	MAVLINK_MSG_ID_HEARTBEAT,
	MAVLINK_MSG_ID_SYS_STATUS,
	MAVLINK_MSG_ID_SYSTEM_TIME,
	MAVLINK_MSG_ID_LOCAL_POSITION_NED,
	MAVLINK_MSG_ID_ATTITUDE,
	MAVLINK_MSG_ID_GPS_RAW_INT,
	MAVLINK_MSG_ID_WSO100,
	MAVLINK_MSG_ID_BASIC_STATE,
	MAVLINK_MSG_ID_RUDDER_RAW,
	MAVLINK_MSG_ID_DST800,
	MAVLINK_MSG_ID_REVO_GS,
	MAVLINK_MSG_ID_MAIN_POWER,
	MAVLINK_MSG_ID_GPS200,
	MAVLINK_MSG_ID_NODE_STATUS,
	MAVLINK_MSG_ID_WAYPOINT_STATUS,
	MAVLINK_MSG_ID_DSP3000
};
uint16_t tsteps[MAVLINK_MSGS_SIZE][2][8] = {};
uint8_t  mSizes[MAVLINK_MSGS_SIZE];
MessageSchedule mavlinkSchedule = {
	MAVLINK_MSGS_SIZE,
	ids,
	mSizes,
	0,
	tsteps
};

/**
 * Initialize MAVLink transmission. This just sets up the MAVLink scheduler with the basic
 * repeatedly-transmit messages.
 */
void MavLinkInit(void)
{
	// First initialize the MessageSchedule struct with the proper sizes.
	const uint8_t const mavMessageSizes[] = MAVLINK_MESSAGE_LENGTHS;
	int i;
	for (i = 0; i < MAVLINK_MSGS_SIZE; ++i) {
		mavlinkSchedule.MessageSizes[i] = mavMessageSizes[ids[i]];
	}

	//const uint8_t const periodicities[MAVLINK_MSGS_SIZE] = {2, 2, 1, 10, 10, 5, 2, 10, 1, 5, 2, 5, 1, 1, 1, 20};
	const uint8_t const periodicities[MAVLINK_MSGS_SIZE] = {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 20};
	for (i = 0; i < sizeof(periodicities); ++i) {
		if (!AddMessageRepeating(&mavlinkSchedule, ids[i], periodicities[i])) {
			FATAL_ERROR();
		}
	}
}

/**
 * This function creates a MAVLink heartbeat message with some basic parameters and
 * caches that message (along with its size) in the module-level variables declared
 * above. This buffer should be transmit at 1Hz back to the groundstation.
 */
void MavLinkSendHeartbeat(void)
{
	mavlink_message_t msg;

	// Update MAVLink state and run mode based on the system state.

	// If the vehicle is in ESTOP switch into emergency mode.
	if (nodeErrors & PRIMARY_NODE_RESET_ESTOP) {
		mavlink_system.state = MAV_STATE_EMERGENCY;
		mavlink_system.mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
	}
	// If the startup reset line is triggered, indicate we're booting up. This is the only unarmed state
	// although that's not technically true with this controller.
	else if (nodeErrors & PRIMARY_NODE_RESET_STARTUP) {
		mavlink_system.state = MAV_STATE_BOOT;
		mavlink_system.mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
	}
	// Otherwise if we're undergoing calibration indicate that
	else if (nodeErrors & PRIMARY_NODE_RESET_CALIBRATING) {
		mavlink_system.state = MAV_STATE_CALIBRATING;
		mavlink_system.mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}
	// Otherwise if there're any other errors we're in standby
	else if (nodeErrors) {
		mavlink_system.state = MAV_STATE_STANDBY;
		mavlink_system.mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}
	// Finally we're active if there're no errors. Also indicate within the mode that we're armed.
	else {
		mavlink_system.state = MAV_STATE_ACTIVE;
		mavlink_system.mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}

	/// Then we update the system mode using MAV_MODE_FLAGs
	// Set manual/autonomous mode. Note that they're not mutually exclusive within the MAVLink protocol,
	// though I treat them as such for my autopilot.
	if (nodeStatus & PRIMARY_NODE_STATUS_AUTOMODE) {
		mavlink_system.mode |= (MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED);
		mavlink_system.mode &= ~MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
	} else {
		mavlink_system.mode &= ~(MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED);
		mavlink_system.mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
	}

	// Pack the message
	mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, mavlink_system.type, MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY, mavlink_system.mode, 0, mavlink_system.state);

	// Copy the message to the send buffer
	len = mavlink_msg_to_send_buffer(buf, &msg);
	if (!Uart1WriteData(buf, (uint8_t)len)) {
		FATAL_ERROR();
	}
}

/**
 * This function transmits the system time. Looks like it's necessary for QGC to
 * record timestamps on data reliably. For some reason it doesn't just use the local
 * time of message reception. Hopefully this fixes that.
 */
void MavLinkSendSystemTime(void)
{
	mavlink_message_t msg;

	// Pack the message
	mavlink_msg_system_time_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
								 dateTimeDataStore.usecSinceEpoch, nodeSystemTime*10);

	// Copy the message to the send buffer
	len = mavlink_msg_to_send_buffer(buf, &msg);
	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * This function transmits a MAVLink SYS_STATUS message. It relies on various external information such as sensor/actuator status
 * from ecanSensors.h, the controllerVars struct exported by Simulink, and the drop rate calculated within ecanSensors.c.
 */
void MavLinkSendStatus(void)
{
	mavlink_message_t msg;

	// Declare that we have onboard sensors: 3D gyro, 3D accelerometer, 3D magnetometer, absolute pressure, GPS
	// And that we have the following controllers: yaw position, x/y position control, motor outputs/control.
	uint32_t systemsPresent = ONBOARD_SENSORS_IMU |
	                          ONBOARD_SENSORS_WSO100  |
	                          ONBOARD_SENSORS_GPS     |
	                          ONBOARD_CONTROL_YAW_POS |
	                          ONBOARD_CONTROL_XY_POS  |
	                          ONBOARD_CONTROL_MOTOR;

	uint32_t systemsEnabled = ONBOARD_CONTROL_YAW_POS;
	systemsEnabled |= sensorAvailability.gps.enabled?ONBOARD_SENSORS_GPS:0;
	systemsEnabled |= sensorAvailability.imu.enabled?ONBOARD_SENSORS_IMU:0;
	systemsEnabled |= sensorAvailability.wso100.enabled?ONBOARD_SENSORS_WSO100:0;
	// The DST800 doesn't map into this bitfield.
	// The power node doesn't map into this bitfield.
	systemsEnabled |= sensorAvailability.prop.enabled?(ONBOARD_CONTROL_XY_POS|ONBOARD_CONTROL_MOTOR):0;

	uint32_t systemsActive = ONBOARD_CONTROL_YAW_POS;
	systemsActive |= sensorAvailability.gps.active?ONBOARD_SENSORS_GPS:0;
	systemsActive |= sensorAvailability.imu.active?ONBOARD_SENSORS_IMU:0;
	systemsActive |= sensorAvailability.wso100.active?ONBOARD_SENSORS_WSO100:0;
	// The DST800 doesn't map into this bitfield.
	// The power node doesn't map into this bitfield.
	systemsActive |= sensorAvailability.prop.active?(ONBOARD_CONTROL_XY_POS|ONBOARD_CONTROL_MOTOR):0;

	// Grab the globally-declared battery sensor data and map into the values necessary for transmission.
	uint16_t voltage = (uint16_t)(GetPowerRailVoltage() * 1000);
	int16_t amperage = (int16_t)(GetPowerRailCurrent() * 100);

	// Calculate the drop rate
	uint16_t dropRate = 0;
	if (mavLinkMessagesFailedParsing) {
		dropRate = (uint16_t)(((float)mavLinkMessagesFailedParsing) * 10000.0f / ((float)(mavLinkMessagesReceived + mavLinkMessagesFailedParsing)));
	}

	mavlink_msg_sys_status_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
		systemsPresent, systemsEnabled, systemsActive,
		(uint16_t)(nodeCpuLoad)*10,
		voltage, amperage, -1,
		dropRate, mavLinkMessagesFailedParsing, 0, 0, 0, 0);
	len = mavlink_msg_to_send_buffer(buf, &msg);

	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * Pull the raw GPS sensor data from the gpsDataStore struct within the GPS module and
 * transmit it via MAVLink over UART1.
 */
void MavLinkSendRawGps(void)
{
	mavlink_message_t msg;

	// We need to made the mode received from NMEA2000 messages to NMEA0183 fix type.
	// NMEA2000    | NMEA0183 | Meaning
	// 0,3,4,5,6,7 |   0      | invalid/no fix
	//    2        |   3      | 3D fix
	//    1        |   2      | 2D fix
	uint8_t mavlinkGpsMode = gpsDataStore.mode == 2?3:(gpsDataStore.mode == 1?2:0);

	mavlink_msg_gps_raw_int_pack(mavlink_system.sysid, mavlink_system.compid, &msg, ((uint64_t)nodeSystemTime)*10000,
		mavlinkGpsMode, gpsDataStore.latitude, gpsDataStore.longitude, gpsDataStore.altitude,
		gpsDataStore.hdop, gpsDataStore.vdop,
		gpsDataStore.sog, (uint16_t)(((float)gpsDataStore.cog) * 180 / M_PI / 100),
		0xFF);

	len = mavlink_msg_to_send_buffer(buf, &msg);

	Uart1WriteData(buf, (uint8_t)len);
}

/**
  * Transmit the main battery state as obtained from the power node via the CAN bus.
  */
void MavLinkSendMainPower(void)
{
	mavlink_message_t msg;

	mavlink_msg_main_power_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
		(uint16_t)(powerDataStore.voltage * 100.0f),(uint16_t)(powerDataStore.current * 10.0f));

	len = mavlink_msg_to_send_buffer(buf, &msg);

	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * Transmits the custom BASIC_STATE message. This just transmits a bunch of random variables
 * that are good to know but arbitrarily grouped.
 */
void MavLinkSendBasicState(void)
{
	mavlink_message_t msg;

	mavlink_msg_basic_state_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
		currentCommands.autonomousRudderCommand, currentCommands.primaryManualRudderCommand, currentCommands.secondaryManualRudderCommand, rudderSensorData.RudderAngle,
		currentCommands.autonomousThrottleCommand, currentCommands.primaryManualThrottleCommand, currentCommands.secondaryManualThrottleCommand, 0,
		controllerVars.Acmd,
		controllerVars.L2Vector[0], controllerVars.L2Vector[1]
	);

	len = mavlink_msg_to_send_buffer(buf, &msg);

	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * Transmits the z-axis rotation rate from the DSP3000. Note that this is in the body frame. Data is
 * in rads/s and clockwise positive.
 */
void MavLinkSendDsp3000(void)
{
	mavlink_message_t msg;

	mavlink_msg_dsp3000_pack(mavlink_system.sysid, mavlink_system.compid, &msg, gyroDataStore.zRate);

	len = mavlink_msg_to_send_buffer(buf, &msg);

	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * Transmits the vehicle attitude pulled from the Revolution GS 3-axis compass. This functions e
 * xpects nodeSystemTime to be in centiseconds which are then converted to ms for transmission.
 * Yaw should be in radians where positive is eastward from north.
 */
void MavLinkSendAttitude(void)
{
	mavlink_message_t msg;

	mavlink_msg_attitude_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
	                          nodeSystemTime*10,
							  revoGsDataStore.roll, revoGsDataStore.pitch, controllerVars.Heading,
							  0.0, 0.0, 0.0);

	len = mavlink_msg_to_send_buffer(buf, &msg);

	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * This function takes in the local position and local velocity (as 3-tuples) from
 * Matlab as real32s and ships them off over a MAVLink MAVLINK_MSG_ID_LOCAL_POSITION_NED
 * message.
 * It also expects a systemStatus struct with a uint32_t time element that holds the
 * current system time in centiseconds.
 */
void MavLinkSendLocalPosition(void)
{
	mavlink_message_t msg;

	mavlink_msg_local_position_ned_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
	                                    nodeSystemTime*10,
	                                    controllerVars.LocalPosition[0], controllerVars.LocalPosition[1], controllerVars.LocalPosition[2],
	                                    controllerVars.Velocity[0], controllerVars.Velocity[1], controllerVars.Velocity[2]);

	len = mavlink_msg_to_send_buffer(buf, &msg);

	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * Transmits the current GPS position of the origin of the local coordinate frame that the North-East-Down
 * coordinates are all relative too. They should be in units of 1e-7 degrees.
 */
void MavLinkSendGpsGlobalOrigin(void)
{
	mavlink_message_t msg;

	mavlink_msg_gps_global_origin_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
	                                   gpsOrigin[0], gpsOrigin[1], gpsOrigin[2]);

	len = mavlink_msg_to_send_buffer(buf, &msg);

	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * Transmit the current mission index via UART1. GetCurrentMission returns a -1 if there're no missions,
 * so we check and only transmit valid current missions.
 */
void MavLinkSendCurrentMission(void)
{
	int8_t currentMission;

	GetCurrentMission(&currentMission);

	if (currentMission != -1) {
		mavlink_message_t msg;
		mavlink_msg_mission_current_pack(mavlink_system.sysid, mavlink_system.compid, &msg, (uint16_t)currentMission);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		Uart1WriteData(buf, (uint8_t)len);
	}
}

/**
 * Transmit a mission acknowledgement message. The type of message is the sole argument to this
 * function (see enum MAV_MISSIONRESULT).
 */
void MavLinkSendMissionAck(uint8_t type)
{
	mavlink_message_t msg;
	mavlink_msg_mission_ack_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
	                             groundStationSystemId, groundStationComponentId, type);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * Transmit a command acknowledgement message. The type of message is the sole argument to this
 * function (see enum MAV_RESULT).
 */
void MavLinkSendCommandAck(uint8_t command, uint8_t result)
{
	mavlink_message_t msg;
	mavlink_msg_command_ack_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
	                             command, result);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendMissionCount(void)
{
	uint8_t missionCount;
	mavlink_message_t msg;
	GetMissionCount(&missionCount);
	mavlink_msg_mission_count_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
	                               groundStationSystemId, groundStationComponentId, missionCount);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * Only broadcast GLOBAL reference frame mission items. While missions are converted to LOCAL_NED
 * when stored onboard the controller, the global coordinates are preserved in the otherCoordinates
 * member.
 * @param currentMissionIndex The 0-based index of the current mission.
 */
void MavLinkSendMissionItem(uint8_t currentMissionIndex)
{
	Mission m;
	uint8_t result;
	GetMission(currentMissionIndex, &m, &result);
	if (result) {
		mavlink_message_t msg;
		int8_t missionManagerCurrentIndex;
		GetCurrentMission(&missionManagerCurrentIndex);
		mavlink_msg_mission_item_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
		                              groundStationSystemId, groundStationComponentId, currentMissionIndex,
		                              MAV_FRAME_GLOBAL, m.action, (currentMissionIndex == (uint8_t)missionManagerCurrentIndex),
		                              m.autocontinue, m.parameters[0], m.parameters[1], m.parameters[2], m.parameters[3],
		                              m.otherCoordinates[0], m.otherCoordinates[1], m.otherCoordinates[2]);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		Uart1WriteData(buf, (uint8_t)len);
	}
}

void MavLinkSendMissionRequest(uint8_t currentMissionIndex)
{
	mavlink_message_t msg;
	mavlink_msg_mission_request_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
	                                 groundStationSystemId, groundStationComponentId, currentMissionIndex);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * The following functions are helper functions for reading the various parameters aboard the boat.
 * @param id The ID of this parameter.
 */
void _transmitParameter(uint16_t id)
{
	if (id < PARAMETERS_TOTAL) {
		// Then use the helper functions from Parameters.h to get the current value. If there was an
		// error, just return having done nothing.
		float param_value = 0.0;
		ParameterGetValueById(id, &param_value);

		// Finally encode the message and transmit.
		mavlink_message_t msg;
		mavlink_msg_param_value_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
			onboardParameters[id].name, param_value, onboardParameters[id].dataType,
			PARAMETERS_TOTAL, id);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		Uart1WriteData(buf, (uint8_t)len);
	}
}

/** Custom Sealion Messages **/

void MavLinkSendRudderRaw(void)
{
	mavlink_message_t msg;

	mavlink_msg_rudder_raw_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                                rudderSensorData.RudderPotValue, rudderSensorData.LimitHitPort, 0, rudderSensorData.LimitHitStarboard,
                                rudderSensorData.RudderPotLimitPort, rudderSensorData.RudderPotLimitStarboard);

	len = mavlink_msg_to_send_buffer(buf, &msg);

	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendWindAirData(void)
{
	mavlink_message_t msg;
	mavlink_msg_wso100_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
		windDataStore.speed, windDataStore.direction,
		airDataStore.temp, airDataStore.pressure, airDataStore.humidity);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendDst800Data(void)
{
	mavlink_message_t msg;
	mavlink_msg_dst800_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
	                        waterDataStore.speed, waterDataStore.temp, waterDataStore.depth);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendRevoGsData(void)
{
	mavlink_message_t msg;
	mavlink_msg_revo_gs_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
		revoGsDataStore.heading, revoGsDataStore.magStatus,
		revoGsDataStore.pitch, revoGsDataStore.pitchStatus,
		revoGsDataStore.roll, revoGsDataStore.rollStatus,
		revoGsDataStore.dip, revoGsDataStore.magneticMagnitude);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendGps200Data(void)
{
	mavlink_message_t msg;
	mavlink_msg_gps200_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
	                        gpsDataStore.variation);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendNodeStatusData(void)
{
	mavlink_message_t msg;
	mavlink_msg_node_status_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
	                             nodeStatusDataStore[CAN_NODE_HIL - 1].status,
								 nodeStatusDataStore[CAN_NODE_HIL - 1].errors,
								 nodeStatusDataStore[CAN_NODE_HIL - 1].temp,
								 nodeStatusDataStore[CAN_NODE_HIL - 1].load,
								 nodeStatusDataStore[CAN_NODE_HIL - 1].voltage,

	                             nodeStatusDataStore[CAN_NODE_IMU_SENSOR - 1].status,
								 nodeStatusDataStore[CAN_NODE_IMU_SENSOR - 1].errors,
								 nodeStatusDataStore[CAN_NODE_IMU_SENSOR - 1].temp,
								 nodeStatusDataStore[CAN_NODE_IMU_SENSOR - 1].load,
								 nodeStatusDataStore[CAN_NODE_IMU_SENSOR - 1].voltage,

	                             nodeStatusDataStore[CAN_NODE_POWER_SENSOR - 1].status,
								 nodeStatusDataStore[CAN_NODE_POWER_SENSOR - 1].errors,
								 nodeStatusDataStore[CAN_NODE_POWER_SENSOR - 1].temp,
								 nodeStatusDataStore[CAN_NODE_POWER_SENSOR - 1].load,
								 nodeStatusDataStore[CAN_NODE_POWER_SENSOR - 1].voltage,

	                             nodeStatus,
								 nodeErrors,
								 nodeTemp,
								 nodeCpuLoad,
								 nodeVoltage,

	                             nodeStatusDataStore[CAN_NODE_RC - 1].status,
								 nodeStatusDataStore[CAN_NODE_RC - 1].errors,
								 nodeStatusDataStore[CAN_NODE_RC - 1].temp,
								 nodeStatusDataStore[CAN_NODE_RC - 1].load,
								 nodeStatusDataStore[CAN_NODE_RC - 1].voltage,

	                             nodeStatusDataStore[CAN_NODE_RUDDER_CONTROLLER - 1].status,
								 nodeStatusDataStore[CAN_NODE_RUDDER_CONTROLLER - 1].errors,
								 nodeStatusDataStore[CAN_NODE_RUDDER_CONTROLLER - 1].temp,
								 nodeStatusDataStore[CAN_NODE_RUDDER_CONTROLLER - 1].load,
								 nodeStatusDataStore[CAN_NODE_RUDDER_CONTROLLER - 1].voltage);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendWaypointStatusData(void)
{
	mavlink_message_t msg;
	int8_t missionIndex;
	Mission cMission = {}, nMission;
	boolean_T wasFound; // Using stupid Simulink datatypes to avoid errors
	GetCurrentMission(&missionIndex);
	GetMission(missionIndex, &nMission, &wasFound);
	if (!wasFound) {
		return; // Error out if we couldn't successfully fetch a mission.
	}
	// Fetch either the first waypoint or the starting point depending on what the next waypoint is.
	if (missionIndex > 0) {
		GetMission(missionIndex - 1, &cMission, &wasFound);
	} else {
		GetStartingPoint(&cMission);
	}
	mavlink_msg_waypoint_status_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
	                                 cMission.otherCoordinates[0], cMission.otherCoordinates[1], cMission.coordinates[0], cMission.coordinates[1],
									 nMission.otherCoordinates[0], nMission.otherCoordinates[1], nMission.coordinates[0], nMission.coordinates[1]);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkReceiveCommandLong(const mavlink_command_long_t *msg)
{
	if (msg->target_system == mavlink_system.sysid) {
		switch (msg->command) {
			case MAV_CMD_PREFLIGHT_STORAGE:
				{
					uint8_t result = MAV_RESULT_FAILED;
					if (msg->param1) {
						if (DataStoreSaveParameters()) {
							result = MAV_RESULT_ACCEPTED;
						}
					} else {
						if (DataStoreLoadParameters()) {
							result = MAV_RESULT_ACCEPTED;
						}
					}
					MavLinkSendCommandAck(msg->command, result);
				}
				break;
			default:
				MavLinkSendCommandAck(msg->command, MAV_RESULT_UNSUPPORTED);
				break;
		}
	}
}

void MavLinkReceiveManualControl(const mavlink_manual_control_t *msg)
{
    static uint16_t lastButtons = 0;
	if (msg->target == mavlink_system.sysid) {
		// Record the rudder angle
		if (msg->r != INT16_MAX) {
			mavlinkManualControlData.Rudder = msg->r;
		}

		// If the trigger has been pulled was part of this data packet, update the throttle value.
		if ((msg->buttons & TRIGGER_ENABLE_BUTTON) != 0 && msg->z != INT16_MAX) {
			mavlinkManualControlData.Throttle = msg->z;
		}

		// Record the buttons that are pressed
		mavlinkManualControlData.Buttons = msg->buttons;

        // If the rudder calibration button has been pressed, send that command.
        if (!(lastButtons & RUDDER_CAL_BUTTON) && (msg->buttons & RUDDER_CAL_BUTTON)) {
            RudderStartCalibration();
        }

		// Keep track of what buttons are currently pressed so that up- and down-events can be
		// tracked.
        lastButtons = msg->buttons;
	}
}

void MavLinkReceiveSetMode(const mavlink_set_mode_t *msg)
{
    if (msg->target_system == mavlink_system.sysid) {
        // Set autonomous mode. Also latch the current vehicle location when this switch occurs. This
		// will make the vehicle follow a line from this location to the next waypoint, which is expected
		// behavior.
        if ((msg->base_mode & MAV_MODE_FLAG_AUTO_ENABLED) &&
            !(msg->base_mode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)) {
			SetStartingPointToCurrentLocation();
            nodeStatus |= PRIMARY_NODE_STATUS_AUTOMODE;
        }
        // Or set manual mode
        else if (!(msg->base_mode & MAV_MODE_FLAG_AUTO_ENABLED) &&
            (msg->base_mode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)) {
            nodeStatus &= ~PRIMARY_NODE_STATUS_AUTOMODE;
        }
    }
}

/**
 * Returns the throttle and rudder manual control commands received over MAVLink.
 * @param rc The rudder command (floating point radians)
 * @param tc The throttle command (16-bit integer, -1000=full reverse, 1000=full forward)
 */
void GetMavLinkManualControl(float *rc, int16_t *tc)
{
	if (rc) {
		*rc = mavlinkManualControlData.Rudder;
	}
	if (tc) {
		*tc = mavlinkManualControlData.Throttle;
	}
}

/** Core MAVLink functions handling transmission and state machines **/

/**
 *
 * @param event An event from PARAM_EVENT.
 * @param data A pointer to data, its meaning depends on the current state of the parameter protocol.
 */
void MavLinkEvaluateParameterState(enum PARAM_EVENT event, const void *data)
{
	// Track the parameter protocol state
	static uint8_t state = PARAM_STATE_INACTIVE;

	// Keep a record of the current parameter being used
	static uint16_t currentParameter;

	// Used for the delaying parameter transmission
	static uint8_t delayCountdown = 0;

	// Store the state to change into
	uint8_t nextState = state;

	// First check the parameter protocol state
	switch (state) {
		case PARAM_STATE_INACTIVE:
			if (event == PARAM_EVENT_REQUEST_LIST_RECEIVED) {
				currentParameter = 0;
				nextState = PARAM_STATE_STREAM_SEND_VALUE;
			} else if (event == PARAM_EVENT_SET_RECEIVED) {
				mavlink_param_set_t x = *(mavlink_param_set_t*)data;
				currentParameter = ParameterSetValueByName(x.param_id, &x.param_value);
				// If there was an error, just reset.
				if (currentParameter == UINT16_MAX) {
					currentParameter = 0;
				}
				nextState = PARAM_STATE_SINGLETON_SEND_VALUE;
			} else if (event == PARAM_EVENT_REQUEST_READ_RECEIVED) {
				currentParameter = *(uint16_t*)data;
				nextState = PARAM_STATE_SINGLETON_SEND_VALUE;
			}
		break;

		case PARAM_STATE_SINGLETON_SEND_VALUE: {
			if (event == PARAM_EVENT_NONE) {
				_transmitParameter(currentParameter);
				nextState = PARAM_STATE_INACTIVE;
			}
		} break;

		case PARAM_STATE_STREAM_SEND_VALUE: {
			if (event == PARAM_EVENT_NONE) {
				_transmitParameter(currentParameter);

				// And increment the current parameter index for the next iteration and
				// we finish if we've hit the limit of parameters.
				if (++currentParameter == PARAMETERS_TOTAL) {
					nextState = PARAM_STATE_INACTIVE;
				} else {
					nextState = PARAM_STATE_STREAM_DELAY;
				}
			}
		} break;

		// Add a delay of INTRA_PARAM_DELAY timesteps before attempting to schedule another one
		case PARAM_STATE_STREAM_DELAY: {
			if (event == PARAM_EVENT_ENTER_STATE) {
					delayCountdown = INTRA_PARAM_DELAY;
			} else if (event == PARAM_EVENT_NONE) {
				if (delayCountdown-- == 0) {
					nextState = PARAM_STATE_STREAM_SEND_VALUE;
				}
			}
		} break;

		default: break;
	}

	// Here is when we actually transition between states, calling init/exit code as necessary
	if (nextState != state) {
		MavLinkEvaluateParameterState(PARAM_EVENT_EXIT_STATE, NULL);
		state = nextState;
		MavLinkEvaluateParameterState(PARAM_EVENT_ENTER_STATE, NULL);
	}
}

/**
 * Set the starting point for the mission manager to the boat's current location.
 */
void SetStartingPointToCurrentLocation(void)
{
	// Update the starting point for the track to be the current vehicle position.
	// We tack on GPS coordinates if we have some.
	Mission newStartPoint = {};
	newStartPoint.coordinates[0] = controllerVars.LocalPosition[0];
	newStartPoint.coordinates[1] = controllerVars.LocalPosition[1];
	newStartPoint.coordinates[2] = controllerVars.LocalPosition[2];
	if (gpsDataStore.mode == 1 || gpsDataStore.mode == 2) {
		newStartPoint.otherCoordinates[0] = gpsDataStore.latitude;
		newStartPoint.otherCoordinates[1] = gpsDataStore.longitude;
		newStartPoint.otherCoordinates[2] = gpsDataStore.altitude;
	}
	SetStartingPoint(&newStartPoint);
}

/**
 * This function implements the mission protocol state machine for the MAVLink protocol.
 * events can be passed as the first argument, or NO_EVENT if desired. data is a pointer
 * to data if there is any to be passed to the state logic. data is not guaranteed to persist
 * beyond the single call to this function.
 */
void MavLinkEvaluateMissionState(enum MISSION_EVENT event, const void *data)
{
	// Internal counter variable for use with the COUNTDOWN state
	static uint16_t counter = 0;

	// Keep track of the expected length of the incoming mission list
	static uint16_t mavlinkNewMissionListSize;

	// Track a mission index for some multi-state loops.
	static uint8_t currentMissionIndex;

	// Track the state
	static uint8_t state = MISSION_STATE_INACTIVE;

	// Keep track of the next state to transition into
	uint8_t nextState = state;

	// Then check the mission protocol state
	switch (state) {
		case MISSION_STATE_INACTIVE:
			// If a REQUEST_LIST is received, reset the current mission and move into the receive
			// missions mode.
			if (event == MISSION_EVENT_REQUEST_LIST_RECEIVED) {
				currentMissionIndex = 0;
				nextState = MISSION_STATE_SEND_MISSION_COUNT;
			}
			// Otherwise if a mission count was received, prepare to receive new missions.
			else if (event == MISSION_EVENT_COUNT_RECEIVED) {
				// Don't allow for writing of new missions if we're in autonomous mode.
				if (nodeStatus & PRIMARY_NODE_STATUS_AUTOMODE) {
					MavLinkSendMissionAck(MAV_MISSION_ERROR);
					nextState = MISSION_STATE_INACTIVE;
				}

				uint8_t newListSize = *(uint8_t *)data;

				// If we received a 0-length mission list, just respond with a MISSION_ACK error.
				if (newListSize == 0) {
					MavLinkSendMissionAck(MAV_MISSION_ERROR);
					nextState = MISSION_STATE_INACTIVE;
				}
				// If there isn't enough room, respond with a MISSION_ACK error.
				else if (newListSize > mList.maxSize) { // mList is exported by MATLAB code.
					MavLinkSendMissionAck(MAV_MISSION_NO_SPACE);
					nextState = MISSION_STATE_INACTIVE;
				}
				// Otherwise we're set to start retrieving a new mission list so we request the first mission.
				else {
					// Update the size of the mission list to the new list size.
					mavlinkNewMissionListSize = newListSize;

					// Clear all the old waypoints.
					ClearMissionList();

					// Update the starting point to the vehicle's current location
					SetStartingPointToCurrentLocation();

					// And wait for info on the first mission.
					currentMissionIndex = 0;

					// And finally trigger the proper response.
					nextState = MISSION_STATE_SEND_MISSION_REQUEST;
				}
			} else if (event == MISSION_EVENT_CLEAR_ALL_RECEIVED) {
				// If we're in autonomous mode, don't allow for clearing the mission list
				if (nodeStatus & PRIMARY_NODE_STATUS_AUTOMODE) {
					MavLinkSendMissionAck(MAV_MISSION_ERROR);
					nextState = MISSION_STATE_INACTIVE;
				}
				// But if we're in manual mode, go ahead and clear everything.
				else {
					// Clear the old list
					ClearMissionList();

					// Update the starting point to the vehicle's current location
					SetStartingPointToCurrentLocation();

					// And then send our acknowledgement.
					MavLinkSendMissionAck(MAV_MISSION_ACCEPTED);
					nextState = MISSION_STATE_INACTIVE;
				}
			} else if (event == MISSION_EVENT_SET_CURRENT_RECEIVED) {
				SetCurrentMission(*(uint8_t*)data);
				MavLinkSendCurrentMission();
				nextState = MISSION_STATE_INACTIVE;
			}
		break;

		case MISSION_STATE_SEND_MISSION_COUNT:
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionCount();
				nextState = MISSION_STATE_MISSION_COUNT_TIMEOUT;
			}
		break;

		case MISSION_STATE_MISSION_COUNT_TIMEOUT:
			if (event == MISSION_EVENT_ENTER_STATE) {
				counter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				if (counter++ > MAVLINK_RESEND_TIMEOUT) {
					nextState = MISSION_STATE_SEND_MISSION_COUNT2;
				}
			} else if (event == MISSION_EVENT_REQUEST_LIST_RECEIVED) {
				nextState = MISSION_STATE_SEND_MISSION_COUNT;
			} else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
				// If the current mission is requested, send it.
				if (data && *(uint8_t *)data == currentMissionIndex) {
					MavLinkSendMissionItem(currentMissionIndex);
					nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT;
				} else {
					MavLinkSendMissionAck(MAV_MISSION_INVALID_SEQUENCE);
					nextState = MISSION_STATE_INACTIVE;
				}
			}
		break;

		case MISSION_STATE_SEND_MISSION_COUNT2:
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionCount();
				nextState = MISSION_STATE_MISSION_COUNT_TIMEOUT2;
			}
		break;

		case MISSION_STATE_MISSION_COUNT_TIMEOUT2:
			if (event == MISSION_EVENT_ENTER_STATE) {
				counter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				if (counter++ > MAVLINK_RESEND_TIMEOUT) {
					nextState = MISSION_STATE_SEND_MISSION_COUNT3;
				}
			} else if (event == MISSION_EVENT_REQUEST_LIST_RECEIVED) {
				nextState = MISSION_STATE_SEND_MISSION_COUNT;
			} else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
				// If the current mission is requested, send it.
				if (data && *(uint8_t *)data == currentMissionIndex) {
					MavLinkSendMissionItem(currentMissionIndex);
					nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT;
				} else {
					MavLinkSendMissionAck(MAV_MISSION_INVALID_SEQUENCE);
					nextState = MISSION_STATE_INACTIVE;
				}
			}
		break;

		case MISSION_STATE_SEND_MISSION_COUNT3:
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionCount();
				nextState = MISSION_STATE_MISSION_COUNT_TIMEOUT3;
			}
		break;

		case MISSION_STATE_MISSION_COUNT_TIMEOUT3:
			if (event == MISSION_EVENT_ENTER_STATE) {
				counter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				if (counter++ > MAVLINK_RESEND_TIMEOUT) {
					nextState = MISSION_STATE_INACTIVE;
				}
			} else if (event == MISSION_EVENT_REQUEST_LIST_RECEIVED) {
				nextState = MISSION_STATE_SEND_MISSION_COUNT;
			} else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
				// If the current mission is requested, send it.
				if (data && *(uint8_t *)data == currentMissionIndex) {
					MavLinkSendMissionItem(currentMissionIndex);
					nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT;
				} else {
					MavLinkSendMissionAck(MAV_MISSION_INVALID_SEQUENCE);
					nextState = MISSION_STATE_INACTIVE;
				}
			}
		break;

		case MISSION_STATE_SEND_MISSION_ITEM: {
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionItem(currentMissionIndex);
				nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT2;
			}
		} break;

		case MISSION_STATE_MISSION_ITEM_TIMEOUT:
			if (event == MISSION_EVENT_ENTER_STATE) {
				counter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				// Keep track of how long it's taking for a request to be received so we can timeout
				// if necessary.
				if (counter++ > MAVLINK_RESEND_TIMEOUT) {
					nextState = MISSION_STATE_SEND_MISSION_ITEM2;
				}
			} else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
				// If the mission that was already sent was requested again, retransmit it.
				if (*(uint8_t *)data == currentMissionIndex) {
					nextState = MISSION_STATE_SEND_MISSION_ITEM;
				}
				// Otherwise if the next mission was requested, move on to sending that one.
				else if (*(uint8_t *)data == currentMissionIndex + 1) {
					++currentMissionIndex;
					nextState = MISSION_STATE_SEND_MISSION_ITEM;
				} else {
					MavLinkSendMissionAck(MAV_MISSION_INVALID_SEQUENCE);
					nextState = MISSION_STATE_INACTIVE;
				}
			} else if (event == MISSION_EVENT_ACK_RECEIVED) {
				nextState = MISSION_STATE_INACTIVE;
			}
		break;

		case MISSION_STATE_SEND_MISSION_ITEM2: {
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionItem(currentMissionIndex);
				nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT2;
			}
		} break;

		case MISSION_STATE_MISSION_ITEM_TIMEOUT2:
			if (event == MISSION_EVENT_ENTER_STATE) {
				counter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				// Keep track of how long it's taking for a request to be received so we can timeout
				// if necessary.
				if (counter++ > MAVLINK_RESEND_TIMEOUT) {
					nextState = MISSION_STATE_SEND_MISSION_ITEM3;
				}
			} else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
				// If the mission that was already sent was requested again, retransmit it.
				if (*(uint8_t *)data == currentMissionIndex) {
					nextState = MISSION_STATE_SEND_MISSION_ITEM;
				}
				// Otherwise if the next mission was requested, move on to sending that one.
				else if (*(uint8_t *)data == currentMissionIndex + 1) {
					++currentMissionIndex;
					nextState = MISSION_STATE_SEND_MISSION_ITEM;
				} else {
					MavLinkSendMissionAck(MAV_MISSION_INVALID_SEQUENCE);
					nextState = MISSION_STATE_INACTIVE;
				}
			} else if (event == MISSION_EVENT_ACK_RECEIVED) {
				nextState = MISSION_STATE_INACTIVE;
			}
		break;

		case MISSION_STATE_SEND_MISSION_ITEM3: {
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionItem(currentMissionIndex);
				nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT3;
			}
		} break;

		case MISSION_STATE_MISSION_ITEM_TIMEOUT3:
			if (event == MISSION_EVENT_ENTER_STATE) {
				counter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				// Keep track of how long it's taking for a request to be received so we can timeout
				// if necessary.
				if (counter++ > MAVLINK_RESEND_TIMEOUT) {
					nextState = MISSION_STATE_INACTIVE;
				}
			} else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
				// If the mission that was already sent was requested again, retransmit it.
				if (*(uint8_t *)data == currentMissionIndex) {
					nextState = MISSION_STATE_SEND_MISSION_ITEM;
				}
				// Otherwise if the next mission was requested, move on to sending that one.
				else if (*(uint8_t *)data == currentMissionIndex + 1) {
					++currentMissionIndex;
					nextState = MISSION_STATE_SEND_MISSION_ITEM;
				} else {
					MavLinkSendMissionAck(MAV_MISSION_INVALID_SEQUENCE);
					nextState = MISSION_STATE_INACTIVE;
				}
			} else if (event == MISSION_EVENT_ACK_RECEIVED) {
				nextState = MISSION_STATE_INACTIVE;
			}
		break;

		case MISSION_STATE_SEND_MISSION_REQUEST: {
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionRequest(currentMissionIndex);
				nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT;
			}
		} break;

		// Implement the countdown timer for receiving a mission item
		case MISSION_STATE_MISSION_REQUEST_TIMEOUT:
			if (event == MISSION_EVENT_ENTER_STATE) {
				counter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				// Keep track of how long it's taking for a request to be received so we can timeout
				// if necessary.
				if (counter++ > MAVLINK_RESEND_TIMEOUT) {
					nextState = MISSION_STATE_SEND_MISSION_REQUEST2;
				}
			}
			// If an ACK was received, we just stop. One shouldn't have been received, so just stop
			// for now.
			else if (event == MISSION_EVENT_ACK_RECEIVED) {
				nextState = MISSION_STATE_INACTIVE;
			}
			//
			else if (event == MISSION_EVENT_ITEM_RECEIVED) {
				const mavlink_mission_item_t *incomingMission = (mavlink_mission_item_t *)data;

				// Make sure that they're coming in in the right order, and if they don't return an error in
				// the acknowledgment response.
				if (currentMissionIndex == incomingMission->seq) {
					int missionAddStatus = MavLinkAppendMission(incomingMission);
					if (missionAddStatus != -1) {
						// If this is going to be the new current mission, then we should set it as such.
						if (incomingMission->current) {
							SetCurrentMission(incomingMission->seq);
						}

						// If this was the last mission we were expecting, respond with an ACK
						// confirming that we've successfully received the entire mission list.
						if (currentMissionIndex == mavlinkNewMissionListSize - 1) {
							MavLinkSendMissionAck(MAV_MISSION_ACCEPTED);
							nextState = MISSION_STATE_INACTIVE;
						}
						// Otherwise we just increment and request the next mission.
						else {
							++currentMissionIndex;
							MavLinkSendMissionRequest(currentMissionIndex);
							nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT;
						}
					}
					// If we've run out of space before the last message, respond saying so.
					else {
						MavLinkSendMissionAck(MAV_MISSION_NO_SPACE);
						nextState = MISSION_STATE_INACTIVE;
					}
				}
			}
		break;

		case MISSION_STATE_SEND_MISSION_REQUEST2: {
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionRequest(currentMissionIndex);
				nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT2;
			}
		} break;

		// Implement the countdown timer for receiving a mission item
		case MISSION_STATE_MISSION_REQUEST_TIMEOUT2:
			if (event == MISSION_EVENT_ENTER_STATE) {
				counter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				// Keep track of how long it's taking for a request to be received so we can timeout
				// if necessary.
				if (counter++ > MAVLINK_RESEND_TIMEOUT) {
					nextState = MISSION_STATE_SEND_MISSION_REQUEST3;
				}
			}
			// If an ACK was received, we just stop. One shouldn't have been received, so just stop
			// for now.
			else if (event == MISSION_EVENT_ACK_RECEIVED) {
				nextState = MISSION_STATE_INACTIVE;
			}
			//
			else if (event == MISSION_EVENT_ITEM_RECEIVED) {
				const mavlink_mission_item_t *incomingMission = (mavlink_mission_item_t *)data;

				// Make sure that they're coming in in the right order, and if they don't return an error in
				// the acknowledgment response.
				if (currentMissionIndex == incomingMission->seq) {
					int missionAddStatus = MavLinkAppendMission(incomingMission);
					if (missionAddStatus != -1) {
						// If this is going to be the new current mission, then we should set it as such.
						if (incomingMission->current) {
							SetCurrentMission(incomingMission->seq);
						}

						// If this was the last mission we were expecting, respond with an ACK
						// confirming that we've successfully received the entire mission list.
						if (currentMissionIndex == mavlinkNewMissionListSize - 1) {
							MavLinkSendMissionAck(MAV_MISSION_ACCEPTED);
							nextState = MISSION_STATE_INACTIVE;
						}
						// Otherwise we just increment and request the next mission.
						else {
							++currentMissionIndex;
							MavLinkSendMissionRequest(currentMissionIndex);
							nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT;
						}
					}
					// If we've run out of space before the last message, respond saying so.
					else {
						MavLinkSendMissionAck(MAV_MISSION_NO_SPACE);
						nextState = MISSION_STATE_INACTIVE;
					}
				}
			}
		break;

		case MISSION_STATE_SEND_MISSION_REQUEST3: {
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionRequest(currentMissionIndex);
				nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT3;
			}
		} break;

		// Implement the countdown timer for receiving a mission item
		case MISSION_STATE_MISSION_REQUEST_TIMEOUT3:
			if (event == MISSION_EVENT_ENTER_STATE) {
				counter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				// Keep track of how long it's taking for a request to be received so we can timeout
				// if necessary.
				if (counter++ > MAVLINK_RESEND_TIMEOUT) {
					nextState = MISSION_STATE_INACTIVE;
				}
			}
			// If an ACK was received, we just stop. One shouldn't have been received, so just stop
			// for now.
			else if (event == MISSION_EVENT_ACK_RECEIVED) {
				nextState = MISSION_STATE_INACTIVE;
			}
			//
			else if (event == MISSION_EVENT_ITEM_RECEIVED) {
				const mavlink_mission_item_t *incomingMission = (mavlink_mission_item_t *)data;

				// Make sure that they're coming in in the right order, and if they don't return an error in
				// the acknowledgment response.
				if (currentMissionIndex == incomingMission->seq) {
					int missionAddStatus = MavLinkAppendMission(incomingMission);
					if (missionAddStatus != -1) {
						// If this is going to be the new current mission, then we should set it as such.
						if (incomingMission->current) {
							SetCurrentMission(incomingMission->seq);
						}

						// If this was the last mission we were expecting, respond with an ACK
						// confirming that we've successfully received the entire mission list.
						if (currentMissionIndex == mavlinkNewMissionListSize - 1) {
							MavLinkSendMissionAck(MAV_MISSION_ACCEPTED);
							nextState = MISSION_STATE_INACTIVE;
						}
						// Otherwise we just increment and request the next mission.
						else {
							++currentMissionIndex;
							MavLinkSendMissionRequest(currentMissionIndex);
							nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT;
						}
					}
					// If we've run out of space before the last message, respond saying so.
					else {
						MavLinkSendMissionAck(MAV_MISSION_NO_SPACE);
						nextState = MISSION_STATE_INACTIVE;
					}
				}
			}
		break;
	}

	// Here is when we actually transition between states, calling init/exit code as necessary
	if (nextState != state) {
		MavLinkEvaluateMissionState(MISSION_EVENT_EXIT_STATE, NULL);
		state = nextState;
		MavLinkEvaluateMissionState(MISSION_EVENT_ENTER_STATE, NULL);
	}
}

int MavLinkAppendMission(const mavlink_mission_item_t *mission)
{
	// We first copy the incoming mission data into our version of a Mission struct,
	// which does not have some fields that are unnecessary.
	Mission m = {
		{
			mission->x,
			mission->y,
			mission->z
		},
		{
			0.0,
			0.0,
			0.0
		},
		mission->frame,
		mission->command,
		{
			mission->param1,
			mission->param2,
			mission->param3,
			mission->param4
		},
		mission->autocontinue
	};

	// Attempt to record this mission to the list, recording the result, which will be 0 for failure.
	// We also map all incoming Global Lat/Long/Alt messages to North-East-Down here.
	// These can be created in QGroundControl by just double-clicking on the Map. While the NED coordinates
	// are stored, the global coordinates are as well so that they can be transmit as global coordinates
	// to QGC, which doesn't display local waypoints on the primary map.
	if (m.refFrame == MAV_FRAME_GLOBAL || m.refFrame == MAV_FRAME_GLOBAL_RELATIVE_ALT) {
		m.refFrame = MAV_FRAME_LOCAL_NED;
		// Preserve the global coordinates in the "otherCoordinates" members.
		m.otherCoordinates[0] = m.coordinates[0];
		m.otherCoordinates[1] = m.coordinates[1];
		m.otherCoordinates[2] = m.coordinates[2];
		const int32_t x[3] = {
			(int32_t)(m.coordinates[0] * 1e7), // Stored in 1e-7 degrees
			(int32_t)(m.coordinates[1] * 1e7), // Stored in 1e-7 degres
			(int32_t)(m.coordinates[2] * 1e3)  // Stored in 1e-3 meters
		};
		lla2ltp(x, m.coordinates);
	}

	int8_t missionAddStatus;
	AppendMission(&m, &missionAddStatus);
	return missionAddStatus;
}

/**
* @brief Receive communication packets and handle them. Should be called at the system sample rate.
*
* This function decodes packets on the protocol level and also handles
* their value by calling the appropriate functions.
*/
void MavLinkReceive(void)
{
	mavlink_message_t msg = {0};
	mavlink_status_t status = {0};

	// Track whether we actually handled any data in this function call.
	// Used for updating the number of MAVLink messages handled
	bool processedData = false;

	// Track if a mission message was processed in this call. This is used to determine if a
	// NONE_EVENT should be sent to the mission manager. The manager needs to be called every
	// timestep such that its internal state machine works properly.
	bool processedMissionMessage = false;

	// Track if a parameter message was processed in this call. This is used to determine if a
	// NONE_EVENT should be sent to the parameter manager. The manager needs to be called every
	// timestep such that its internal state machine works properly.
	bool processedParameterMessage = false;

    uint8_t c;
	while (Uart1ReadByte(&c)) {
		processedData = true;
		// Parse another byte and if there's a message found process it.
		if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

			// Latch the groundstation system and component ID if we haven't yet. We exclude the
			// combination of systemid:3/compid:D, because that's the combo used by the 3DR radios.
			if (!groundStationSystemId && !groundStationComponentId &&
			    (msg.sysid != '3' && msg.compid != 'D')) {
				groundStationSystemId = msg.sysid;
				groundStationComponentId = msg.compid;
			}

			switch(msg.msgid) {

				// Check for commands like write data to EEPROM
				case MAVLINK_MSG_ID_COMMAND_LONG: {
					mavlink_command_long_t mavCommand;
					mavlink_msg_command_long_decode(&msg, &mavCommand);
					MavLinkReceiveCommandLong(&mavCommand);
				} break;

				case MAVLINK_MSG_ID_SET_MODE: {
						mavlink_set_mode_t modeMessage;
						mavlink_msg_set_mode_decode(&msg, &modeMessage);
						MavLinkReceiveSetMode(&modeMessage);
				} break;

				// Check for manual commands via Joystick from QGC.
				case MAVLINK_MSG_ID_MANUAL_CONTROL: {
					mavlink_manual_control_t manualControl;
					mavlink_msg_manual_control_decode(&msg, &manualControl);
					MavLinkReceiveManualControl(&manualControl);
				} break;

				// If we are not doing any mission protocol operations, record the size of the incoming mission
				// list and transition into the write missions state machine loop.
				case MAVLINK_MSG_ID_MISSION_COUNT: {
					uint8_t mavlinkNewMissionListSize = mavlink_msg_mission_count_get_count(&msg);
					MavLinkEvaluateMissionState(MISSION_EVENT_COUNT_RECEIVED, &mavlinkNewMissionListSize);
					processedMissionMessage = true;
				} break;

				// Handle receiving a mission.
				case MAVLINK_MSG_ID_MISSION_ITEM: {
					mavlink_mission_item_t currentMission;
					mavlink_msg_mission_item_decode(&msg, &currentMission);
					MavLinkEvaluateMissionState(MISSION_EVENT_ITEM_RECEIVED, &currentMission);
					processedMissionMessage = true;
				} break;

				// Responding to a mission request entails moving into the first active state and scheduling a MISSION_COUNT message.
				// Will also schedule a transmission of a GPS_ORIGIN message. This is used for translating global to local coordinates
				// in QGC.
				case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
					MavLinkSendGpsGlobalOrigin();
					MavLinkEvaluateMissionState(MISSION_EVENT_REQUEST_LIST_RECEIVED, NULL);
					processedMissionMessage = true;
				} break;

				// When a mission request message is received, respond with that mission information from the MissionManager
				case MAVLINK_MSG_ID_MISSION_REQUEST: {
					uint8_t receivedMissionIndex = mavlink_msg_mission_request_get_seq(&msg);
					MavLinkEvaluateMissionState(MISSION_EVENT_REQUEST_RECEIVED, &receivedMissionIndex);
					processedMissionMessage = true;
				} break;

				// Allow for clearing waypoints. Here we respond simply with an ACK message if we successfully
				// cleared the mission list.
				case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
					MavLinkEvaluateMissionState(MISSION_EVENT_CLEAR_ALL_RECEIVED, NULL);
					processedMissionMessage = true;
				break;

				// Allow for the groundstation to set the current mission. This requires a WAYPOINT_CURRENT response message agreeing with the received current message index.
				case MAVLINK_MSG_ID_MISSION_SET_CURRENT: {
					uint8_t newCurrentMission = mavlink_msg_mission_set_current_get_seq(&msg);
					MavLinkEvaluateMissionState(MISSION_EVENT_SET_CURRENT_RECEIVED, &newCurrentMission);
					processedMissionMessage = true;
				} break;

				case MAVLINK_MSG_ID_MISSION_ACK: {
					uint8_t type = mavlink_msg_mission_ack_get_type(&msg);
					MavLinkEvaluateMissionState(MISSION_EVENT_ACK_RECEIVED, &type);
					processedMissionMessage = true;
				} break;

				// If they're requesting a list of all parameters, call a separate function that'll track the state and transmit the necessary messages.
				// This reason that this is an external function is so that it can be run separately at 20Hz.
				case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
					MavLinkEvaluateParameterState(PARAM_EVENT_REQUEST_LIST_RECEIVED, NULL);
					processedParameterMessage = true;
				} break;

				// If a request comes for a single parameter then set that to be the current parameter and move into the proper state.
				case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
					uint16_t currentParameter = mavlink_msg_param_request_read_get_param_index(&msg);
					MavLinkEvaluateParameterState(PARAM_EVENT_REQUEST_READ_RECEIVED, &currentParameter);
					processedParameterMessage = true;
				} break;

				case MAVLINK_MSG_ID_PARAM_SET: {
					mavlink_param_set_t p;
					mavlink_msg_param_set_decode(&msg, &p);
					MavLinkEvaluateParameterState(PARAM_EVENT_SET_RECEIVED, &p);
					processedParameterMessage = true;
				} break;
			}
		}
	}

	// Now if no mission messages were received, trigger the Mission Manager anyways with a NONE
	// event.
	if (!processedMissionMessage) {
		MavLinkEvaluateMissionState(MISSION_EVENT_NONE, NULL);
	}

	// Now if no parameter messages were received, trigger the Parameter Manager anyways with a NONE
	// event.
	if (!processedParameterMessage) {
		MavLinkEvaluateParameterState(PARAM_EVENT_NONE, NULL);
	}

	// Update the number of messages received, both successful and not. Note that the 'status' variable
	// will be updated on every call to *_parse_char(), so this will always be a valid value.
	if (processedData) {
		mavLinkMessagesReceived += status.packet_rx_success_count;
		mavLinkMessagesFailedParsing += status.packet_rx_drop_count;
	}
}

/**
 * This function handles transmission of MavLink messages taking into account transmission
 * speed, message size, and desired transmission rate.
 */
void MavLinkTransmit(void)
{
	// And now transmit all messages for this timestep
	uint8_t msgs[MAVLINK_MSGS_SIZE];
	uint8_t count = GetMessagesForTimestep(&mavlinkSchedule, msgs);
	int i;
	for (i = 0; i < count; ++i) {

		switch (msgs[i]) {

			/** Common Messages **/

			case MAVLINK_MSG_ID_HEARTBEAT: {
				MavLinkSendHeartbeat();
			} break;

			case MAVLINK_MSG_ID_SYSTEM_TIME: {
				MavLinkSendSystemTime();
			} break;

			case MAVLINK_MSG_ID_SYS_STATUS: {
				MavLinkSendStatus();
			} break;

			case MAVLINK_MSG_ID_ATTITUDE: {
				MavLinkSendAttitude();
			} break;

			case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
				MavLinkSendLocalPosition();
			} break;

			case MAVLINK_MSG_ID_GPS_RAW_INT: {
				MavLinkSendRawGps();
			} break;

			/** SeaSlug Messages **/

			case MAVLINK_MSG_ID_NODE_STATUS: {
				MavLinkSendNodeStatusData();
			} break;

			case MAVLINK_MSG_ID_WAYPOINT_STATUS: {
				MavLinkSendWaypointStatusData();
			} break;

			case MAVLINK_MSG_ID_WSO100: {
				MavLinkSendWindAirData();
			} break;

			case MAVLINK_MSG_ID_BASIC_STATE:
				MavLinkSendBasicState();
			break;

			case MAVLINK_MSG_ID_RUDDER_RAW:
				MavLinkSendRudderRaw();
			break;

			case MAVLINK_MSG_ID_DST800:
				MavLinkSendDst800Data();
			break;

			case MAVLINK_MSG_ID_REVO_GS:
				MavLinkSendRevoGsData();
			break;

			case MAVLINK_MSG_ID_GPS200:
				MavLinkSendGps200Data();
			break;

			case MAVLINK_MSG_ID_DSP3000:
				MavLinkSendDsp3000();
			break;

			case MAVLINK_MSG_ID_MAIN_POWER:
				MavLinkSendMainPower();
			break;

			default: {

			} break;
		}
	}
}
