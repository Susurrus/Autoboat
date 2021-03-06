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
#include "MavlinkGlue.h"
#include "MavCorruptNode.h"

uint32_t nodeSystemTime = 0;

mavlink_mission_item_t mList[16];
uint8_t currentMission = 0;

/**
 * This function converts latitude/longitude/altitude into a north/east/down local tangent plane. The
 * code I use by default is auto-generated C code from a Simulink block in the autonomous controller.
 * @param[in] lat/lon/alt in units of 1e7deg/1e7deg/1e3m.
 * @param[out] Output in a north/east/down coordinate frame in units of meters.
 */
extern void lla2ltp(const int32_t x[3], float y[3]);

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

// Store a module-wide variable for common MAVLink system variables.
static mavlink_system_t mavlink_system = {
	20, // Arbitrarily chosen MAV number
	MAV_COMP_ID_ALL
};

/** manual control parameters **/
// Specify the button that must be pressed for throttle values to be recorded.
// On the Logitech F710 we use, it's the Button, which in D-mode is button 4.
#define TRIGGER_ENABLE_BUTTON 1 << 4
// Specify the button that activates rudder calibration.
// On the Logitech F710 we use, it's the Button, which in D-mode is button 8.
#define RUDDER_CAL_BUTTON     1 << 8
// Autonomous/manual switching is done through QGC

// Latch onto the first groundstation unit and only receive and transmit to it.
static uint8_t groundStationSystemId = 0;
static uint8_t groundStationComponentId = 0;

// Declare a character buffer here to prevent continual allocation/deallocation of MAVLink buffers.
static uint8_t buf[MAVLINK_MAX_PACKET_LEN];
static uint16_t len;

// Internal counter variable for use with the COUNTDOWN state
static uint8_t missionTimeoutCounter = 0;

// Define a timeout (in units of main timesteps of MavlinkReceive()) for transmitting
// MAVLink messages as part of the MISSION protocol. Messages will be retransmit
// twice before giving up.
#define MISSION_RESEND_TIMEOUT 100

// This is a variable declared in Simulink that contains the GPS origin used for global/local
// coordinate conversions. It's organized as latitude (1e7 degrees), longitude (1e7 degrees), and
// altitude (1e6 meters). 
int32_t gpsOrigin[3];

// Track how well MAVLink decoding is going:
// WARN: Possible overflow over long-enough duration
uint16_t mavLinkMessagesReceived = 0;
uint16_t mavLinkMessagesFailedParsing = 0;


// Set up the message scheduler for MAVLink transmission
#define MAVLINK_MSGS_SIZE 17
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
	MAVLINK_MSG_ID_DSP3000,
	MAVLINK_MSG_ID_TOKIMEC
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

	//const uint8_t const periodicities[MAVLINK_MSGS_SIZE] = {2, 2, 1, 10, 10, 5, 2, 10, 1, 5, 2, 5, 1, 1, 1, 20, 20};
	// We only report things that the GUI needs at 1Hz because it only updates that fast.
	// REVO_GS - Not currently connected, so no need to report it often.
	// WSO100 is just an environmental sensor, no need for quick updates.
	const uint8_t const periodicities[MAVLINK_MSGS_SIZE] = {2, 4, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	for (i = 0; i < sizeof(periodicities); ++i) {
		if (periodicities[i] != 0 && !AddMessageRepeating(&mavlinkSchedule, ids[i], periodicities[i])) {
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

	// Pack the message
	mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, MAV_TYPE_SURFACE_BOAT, MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY, MAV_STATE_UNINIT, 0, MAV_MODE_PREFLIGHT | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED);

	// Copy the message to the send buffer
	len = mavlink_msg_to_send_buffer(buf, &msg);
	if (!Uart1WriteData(buf, (uint8_t)len)) {
//		FATAL_ERROR();
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
								 0, nodeSystemTime*10);

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
	uint32_t systemsPresent = 0;

	uint32_t systemsEnabled = 0;

	uint32_t systemsActive = 0;

	// Grab the globally-declared battery sensor data and map into the values necessary for transmission.
	uint16_t voltage = (uint16_t)(0 * 1000);
	int16_t amperage = (int16_t)(0 * 100);

	// Calculate the drop rate
	uint16_t dropRate = 0;
	if (mavLinkMessagesFailedParsing) {
		dropRate = (uint16_t)(((float)mavLinkMessagesFailedParsing) * 10000.0f / ((float)(mavLinkMessagesReceived + mavLinkMessagesFailedParsing)));
	}

	mavlink_msg_sys_status_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
		systemsPresent, systemsEnabled, systemsActive,
		0,
		voltage, amperage, -1,
		dropRate, mavLinkMessagesFailedParsing, 0, 0, 0, 0);
	len = mavlink_msg_to_send_buffer(buf, &msg);

	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendStatusText(enum MAV_SEVERITY severity, const char *text)
{
	mavlink_message_t msg;
	char msgText[MAVLINK_MSG_ID_STATUSTEXT_LEN] = {};
	strncpy(msgText, text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
	mavlink_msg_statustext_pack(mavlink_system.sysid, mavlink_system.compid, &msg, severity, msgText);

	len = mavlink_msg_to_send_buffer(buf, &msg);

	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * Transmits the z-axis rotation rate from the DSP3000. Note that this is in the body frame. Data is
 * in rads/s and clockwise positive.
 */
void MavLinkSendTokimec(void)
{
	mavlink_message_t msg;

	mavlink_msg_tokimec_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
	                         0, 0, 0,
	                         0, 0, 0,
	                         0, 0, 0,
							 0,
							 0, 0,
							 0, 0,
							 0, 0,
							 0);

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
	uint8_t mavlinkGpsMode = 0;

	mavlink_msg_gps_raw_int_pack(mavlink_system.sysid, mavlink_system.compid, &msg, ((uint64_t)nodeSystemTime)*10000,
		mavlinkGpsMode, 0, 0, 0,
		0, 0,
		0, (uint16_t)(((float)0) * 180 / M_PI / 100),
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
		(uint16_t)(0 * 100.0f),(uint16_t)(0 * 10.0f));

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
		0, 0, 0, 0,
		0, 0, 0, 0,
		0,
		0, 0
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

	mavlink_msg_dsp3000_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 0);

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

	// The roll as reported from the Tokimec is opposite from what the ATTITUDE message expects
	// (at least according to QGC).
	float roll = (float)0 / 8192.0;
	float pitch = (float)0 / 8192.0;
	mavlink_msg_attitude_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
	                          nodeSystemTime*10,
							  -roll, pitch, 0,
							  0.0, 0.0, 0.0);

	len = mavlink_msg_to_send_buffer(buf, &msg);

	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * This function outputs a  MAVLink MAVLINK_MSG_ID_LOCAL_POSITION_NED
 * message using the filtered data generated by the controller.
 */
void MavLinkSendLocalPosition(void)
{
	mavlink_message_t msg;

	mavlink_msg_local_position_ned_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
	                                    nodeSystemTime*10,
	                                    0, 0, 0,
	                                    0, 0, 0);

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
	mavlink_message_t msg;
	mavlink_msg_mission_count_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
	                               groundStationSystemId, groundStationComponentId, 16);
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
	uint8_t result = true;
        mavlink_mission_item_t m;
	if (result) {
		mavlink_message_t msg;
                m = mList[currentMissionIndex];


                mavlink_msg_mission_item_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                                              groundStationSystemId, groundStationComponentId, currentMissionIndex,
                                              m.frame, m.command, false,
                                              m.autocontinue, m.param1, m.param2, m.param3, m.param4,
                                              m.x, m.y, m.z);
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

/** Custom Sealion Messages **/

void MavLinkSendRudderRaw(void)
{
	mavlink_message_t msg;

	mavlink_msg_rudder_raw_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                                0, 0, 0, 0,
                                0, 0);

	len = mavlink_msg_to_send_buffer(buf, &msg);

	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendWindAirData(void)
{
	mavlink_message_t msg;
	mavlink_msg_wso100_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
		0, 0,
		0, 0, 0);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendDst800Data(void)
{
	mavlink_message_t msg;
	mavlink_msg_dst800_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
	                        0, 0, 0);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendRevoGsData(void)
{
	mavlink_message_t msg;
	mavlink_msg_revo_gs_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
		0, 0,
		0, 0,
		0, 0,
		0, 0);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendGps200Data(void)
{
	mavlink_message_t msg;
	mavlink_msg_gps200_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
	                        0);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendNodeStatusData(void)
{
	mavlink_message_t msg;
	mavlink_msg_node_status_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
	                             0,0,0,0,0,
	                             0,0,0,0,0,
	                             0,0,0,0,0,
	                             0,0,0,0,0,
	                             0,0,0,0,0,
	                             0,0,0,0,0);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendWaypointStatusData(void)
{
	mavlink_message_t msg;
	mavlink_msg_waypoint_status_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
	                                 0, 0, 0, 0,
					0, 0, 0, 0);
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
//						if (DataStoreSaveParameters()) {
							result = MAV_RESULT_ACCEPTED;
//						}
					} else {
//						if (DataStoreLoadParameters()) {
							result = MAV_RESULT_ACCEPTED;
//						}
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

        // If the rudder calibration button has been pressed, send that command.
        if (!(lastButtons & RUDDER_CAL_BUTTON) && (msg->buttons & RUDDER_CAL_BUTTON)) {
//            RudderStartCalibration();
        }

		// Keep track of what buttons are currently pressed so that up- and down-events can be
		// tracked.
        lastButtons = msg->buttons;
	}
}

void MavLinkReceiveSetMode(const mavlink_set_mode_t *msg)
{
}

/**
 * Returns the throttle and rudder manual control commands received over MAVLink.
 * @param rc The rudder command (floating point radians)
 * @param tc The throttle command (16-bit integer, -1000=full reverse, 1000=full forward)
 */
void GetMavLinkManualControl(float *rc, int16_t *tc)
{
}

/**
 * Set the starting point for the mission manager to the boat's current location.
 */
void SetStartingPointToCurrentLocation(void)
{
	// Update the starting point for the track to be the current vehicle position.
	// We tack on GPS coordinates if we have some.
}

/**
 * This function implements the mission protocol state machine for the MAVLink protocol.
 * events can be passed as the first argument, or NO_EVENT if desired. data is a pointer
 * to data if there is any to be passed to the state logic. data is not guaranteed to persist
 * beyond the single call to this function.
 */
void MavLinkEvaluateMissionState(enum MISSION_EVENT event, const void *data)
{
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
				if (0) {
					MavLinkSendMissionAck(MAV_MISSION_ERROR);
                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, "MISSION_ACK(ERROR) sent");
					nextState = MISSION_STATE_INACTIVE;
				}

				uint8_t newListSize = *(uint8_t *)data;

				// If we received a 0-length mission list, just respond with a MISSION_ACK error.
				if (newListSize == 0) {
					MavLinkSendMissionAck(MAV_MISSION_ERROR);
                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, "MISSION_ACK(ERROR) sent");
					nextState = MISSION_STATE_INACTIVE;
				}
				// If there isn't enough room, respond with a MISSION_ACK error.
				else if (newListSize > 16) { // mList is exported by MATLAB code.
					MavLinkSendMissionAck(MAV_MISSION_NO_SPACE);
                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, "MISSION_ACK(NO_SPACE) sent");
					nextState = MISSION_STATE_INACTIVE;
				}
				// Otherwise we're set to start retrieving a new mission list so we request the first mission.
				else {
					// Update the size of the mission list to the new list size.
					mavlinkNewMissionListSize = newListSize;

					// Update the starting point to the vehicle's current location
					SetStartingPointToCurrentLocation();

					// And wait for info on the first mission.
					currentMissionIndex = 0;

					// And finally trigger the proper response.
					nextState = MISSION_STATE_SEND_MISSION_REQUEST;
				}
			} else if (event == MISSION_EVENT_CLEAR_ALL_RECEIVED) {
				// If we're in autonomous mode, don't allow for clearing the mission list
				if (0) {
					MavLinkSendMissionAck(MAV_MISSION_ERROR);
                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, "MISSION_ACK(ERROR) sent");
					nextState = MISSION_STATE_INACTIVE;
				}
				// But if we're in manual mode, go ahead and clear everything.
				else {

					// Update the starting point to the vehicle's current location
					SetStartingPointToCurrentLocation();

					// And then send our acknowledgement.
					MavLinkSendMissionAck(MAV_MISSION_ACCEPTED);
                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, "MISSION_ACK(ACCEPTED) sent");
					nextState = MISSION_STATE_INACTIVE;
				}
			} else if (event == MISSION_EVENT_SET_CURRENT_RECEIVED) {
				MavLinkSendCurrentMission();
				nextState = MISSION_STATE_INACTIVE;
			}
		break;

		case MISSION_STATE_SEND_MISSION_COUNT:
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionCount();
                                char x[50];
                                snprintf(x, sizeof(x), "MISSION_COUNT(%d) sent", 16);
                                MavLinkSendStatusText(MAV_SEVERITY_DEBUG, x);
				nextState = MISSION_STATE_MISSION_COUNT_TIMEOUT;
			}
		break;

		case MISSION_STATE_MISSION_COUNT_TIMEOUT:
			if (event == MISSION_EVENT_ENTER_STATE) {
				missionTimeoutCounter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				if (missionTimeoutCounter > MISSION_RESEND_TIMEOUT) {
					nextState = MISSION_STATE_SEND_MISSION_COUNT2;
				}
			} else if (event == MISSION_EVENT_REQUEST_LIST_RECEIVED) {
				nextState = MISSION_STATE_SEND_MISSION_COUNT;
			} else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
				// If the current mission is requested, send it.
				if (data && *(uint8_t *)data == currentMissionIndex) {
					MavLinkSendMissionItem(currentMissionIndex);
                                        char x[50];
                                        snprintf(x, sizeof(x), "MISSION_ITEM(%d) sent", currentMissionIndex);
                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, x);
					nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT;
				} else {
					MavLinkSendMissionAck(MAV_MISSION_INVALID_SEQUENCE);
                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, "MISSION_ACK(INVALID_SEQUENCE) sent");
					nextState = MISSION_STATE_INACTIVE;
				}
			}
		break;

		case MISSION_STATE_SEND_MISSION_COUNT2:
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionCount();
                                char x[50];
                                snprintf(x, sizeof(x), "MISSION_COUNT(%d) sent", 16);
				nextState = MISSION_STATE_MISSION_COUNT_TIMEOUT2;
			}
		break;

		case MISSION_STATE_MISSION_COUNT_TIMEOUT2:
			if (event == MISSION_EVENT_ENTER_STATE) {
				missionTimeoutCounter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				if (missionTimeoutCounter > MISSION_RESEND_TIMEOUT) {
					nextState = MISSION_STATE_SEND_MISSION_COUNT3;
				}
			} else if (event == MISSION_EVENT_REQUEST_LIST_RECEIVED) {
				nextState = MISSION_STATE_SEND_MISSION_COUNT;
			} else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
				// If the current mission is requested, send it.
				if (data && *(uint8_t *)data == currentMissionIndex) {
					MavLinkSendMissionItem(currentMissionIndex);
                                        char x[50];
                                        snprintf(x, sizeof(x), "MISSION_ITEM(%d) sent", currentMissionIndex);
                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, x);
					nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT;
				} else {
					MavLinkSendMissionAck(MAV_MISSION_INVALID_SEQUENCE);
                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, "MISSION_ACK(INVALID_SEQUENCE) sent");
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
				missionTimeoutCounter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				if (missionTimeoutCounter > MISSION_RESEND_TIMEOUT) {
					nextState = MISSION_STATE_INACTIVE;
				}
			} else if (event == MISSION_EVENT_REQUEST_LIST_RECEIVED) {
				nextState = MISSION_STATE_SEND_MISSION_COUNT;
			} else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
				// If the current mission is requested, send it.
				if (data && *(uint8_t *)data == currentMissionIndex) {
					MavLinkSendMissionItem(currentMissionIndex);
                                        char x[50];
                                        snprintf(x, sizeof(x), "MISSION_ITEM(%d) sent", currentMissionIndex);
                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, x);
					nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT;
				} else {
					MavLinkSendMissionAck(MAV_MISSION_INVALID_SEQUENCE);
                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, "MISSION_ACK(INVALID_SEQUENCE) sent");
					nextState = MISSION_STATE_INACTIVE;
				}
			}
		break;

		case MISSION_STATE_SEND_MISSION_ITEM: {
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionItem(currentMissionIndex);
                                char x[50];
                                snprintf(x, sizeof(x), "MISSION_ITEM(%d) sent", currentMissionIndex);
                                MavLinkSendStatusText(MAV_SEVERITY_DEBUG, x);
				nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT2;
			}
		} break;

		case MISSION_STATE_MISSION_ITEM_TIMEOUT:
			if (event == MISSION_EVENT_ENTER_STATE) {
				missionTimeoutCounter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				// Keep track of how long it's taking for a request to be received so we can timeout
				// if necessary.
				if (missionTimeoutCounter > MISSION_RESEND_TIMEOUT) {
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
                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, "MISSION_ACK(INVALID_SEQUENCE) sent");
					nextState = MISSION_STATE_INACTIVE;
				}
			} else if (event == MISSION_EVENT_ACK_RECEIVED) {
				nextState = MISSION_STATE_INACTIVE;
			}
		break;

		case MISSION_STATE_SEND_MISSION_ITEM2: {
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionItem(currentMissionIndex);
                                char x[50];
                                snprintf(x, sizeof(x), "MISSION_ITEM(%d) sent", currentMissionIndex);
                                MavLinkSendStatusText(MAV_SEVERITY_DEBUG, x);
				nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT2;
			}
		} break;

		case MISSION_STATE_MISSION_ITEM_TIMEOUT2:
			if (event == MISSION_EVENT_ENTER_STATE) {
				missionTimeoutCounter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				// Keep track of how long it's taking for a request to be received so we can timeout
				// if necessary.
				if (missionTimeoutCounter > MISSION_RESEND_TIMEOUT) {
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
                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, "MISSION_ACK(INVALID_SEQUENCE) sent");
					nextState = MISSION_STATE_INACTIVE;
				}
			} else if (event == MISSION_EVENT_ACK_RECEIVED) {
				nextState = MISSION_STATE_INACTIVE;
			}
		break;

		case MISSION_STATE_SEND_MISSION_ITEM3: {
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionItem(currentMissionIndex);
                                char x[50];
                                snprintf(x, sizeof(x), "MISSION_ITEM(%d) sent", currentMissionIndex);
                                MavLinkSendStatusText(MAV_SEVERITY_DEBUG, x);
				nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT3;
			}
		} break;

		case MISSION_STATE_MISSION_ITEM_TIMEOUT3:
			if (event == MISSION_EVENT_ENTER_STATE) {
				missionTimeoutCounter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				// Keep track of how long it's taking for a request to be received so we can timeout
				// if necessary.
				if (missionTimeoutCounter > MISSION_RESEND_TIMEOUT) {
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
                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, "MISSION_ACK(INVALID_SEQUENCE) sent");
					nextState = MISSION_STATE_INACTIVE;
				}
			} else if (event == MISSION_EVENT_ACK_RECEIVED) {
				nextState = MISSION_STATE_INACTIVE;
			}
		break;

		case MISSION_STATE_SEND_MISSION_REQUEST: {
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionRequest(currentMissionIndex);
                                char x[50];
                                snprintf(x, sizeof(x), "MISSION_REQUEST(%d) sent", currentMissionIndex);
                                MavLinkSendStatusText(MAV_SEVERITY_DEBUG, x);
				nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT;
			}
		} break;

		// Implement the countdown timer for receiving a mission item
		case MISSION_STATE_MISSION_REQUEST_TIMEOUT:
			if (event == MISSION_EVENT_ENTER_STATE) {
				missionTimeoutCounter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				// Keep track of how long it's taking for a request to be received so we can timeout
				// if necessary.
				if (missionTimeoutCounter > MISSION_RESEND_TIMEOUT) {
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

						// If this was the last mission we were expecting, respond with an ACK
						// confirming that we've successfully received the entire mission list.
						if (currentMissionIndex == mavlinkNewMissionListSize - 1) {
							MavLinkSendMissionAck(MAV_MISSION_ACCEPTED);
                                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, "MISSION_ACK(ACCEPTED) sent");
							nextState = MISSION_STATE_INACTIVE;
						}
						// Otherwise we just increment and request the next mission.
						else {
							++currentMissionIndex;
							MavLinkSendMissionRequest(currentMissionIndex);
                                                        char x[50];
                                                        snprintf(x, sizeof(x), "MISSION_REQUEST(%d) sent", currentMissionIndex);
                                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, x);
							nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT;
						}
					}
					// If we've run out of space before the last message, respond saying so.
					else {
						MavLinkSendMissionAck(MAV_MISSION_NO_SPACE);
                                                MavLinkSendStatusText(MAV_SEVERITY_DEBUG, "MISSION_ACK(NO_SPACE) sent");
						nextState = MISSION_STATE_INACTIVE;
					}
				}
			}
		break;

		case MISSION_STATE_SEND_MISSION_REQUEST2: {
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionRequest(currentMissionIndex);
                                char x[50];
                                snprintf(x, sizeof(x), "MISSION_REQUEST(%d) sent", currentMissionIndex);
                                MavLinkSendStatusText(MAV_SEVERITY_DEBUG, x);
				nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT2;
			}
		} break;

		// Implement the countdown timer for receiving a mission item
		case MISSION_STATE_MISSION_REQUEST_TIMEOUT2:
			if (event == MISSION_EVENT_ENTER_STATE) {
				missionTimeoutCounter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				// Keep track of how long it's taking for a request to be received so we can timeout
				// if necessary.
				if (missionTimeoutCounter > MISSION_RESEND_TIMEOUT) {
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

						// If this was the last mission we were expecting, respond with an ACK
						// confirming that we've successfully received the entire mission list.
						if (currentMissionIndex == mavlinkNewMissionListSize - 1) {
							MavLinkSendMissionAck(MAV_MISSION_ACCEPTED);
                                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, "MISSION_ACK(ACCEPTED) sent");
							nextState = MISSION_STATE_INACTIVE;
						}
						// Otherwise we just increment and request the next mission.
						else {
							++currentMissionIndex;
							MavLinkSendMissionRequest(currentMissionIndex);
                                                        char x[50];
                                                        snprintf(x, sizeof(x), "MISSION_REQUEST(%d) sent", currentMissionIndex);
                                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, x);
							nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT;
						}
					}
					// If we've run out of space before the last message, respond saying so.
					else {
						MavLinkSendMissionAck(MAV_MISSION_NO_SPACE);
                                                MavLinkSendStatusText(MAV_SEVERITY_DEBUG, "MISSION_ACK(NO_SPACE) sent");
						nextState = MISSION_STATE_INACTIVE;
					}
				}
			}
		break;

		case MISSION_STATE_SEND_MISSION_REQUEST3: {
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionRequest(currentMissionIndex);
                                char x[50];
                                snprintf(x, sizeof(x), "MISSION_REQUEST(%d) sent", currentMissionIndex);
                                MavLinkSendStatusText(MAV_SEVERITY_DEBUG, x);
				nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT3;
			}
		} break;

		// Implement the countdown timer for receiving a mission item
		case MISSION_STATE_MISSION_REQUEST_TIMEOUT3:
			if (event == MISSION_EVENT_ENTER_STATE) {
				missionTimeoutCounter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				// Keep track of how long it's taking for a request to be received so we can timeout
				// if necessary.
				if (missionTimeoutCounter > MISSION_RESEND_TIMEOUT) {
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

						// If this was the last mission we were expecting, respond with an ACK
						// confirming that we've successfully received the entire mission list.
						if (currentMissionIndex == mavlinkNewMissionListSize - 1) {
							MavLinkSendMissionAck(MAV_MISSION_ACCEPTED);
                                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, "MISSION_ACK(ACCEPTED) sent");
							nextState = MISSION_STATE_INACTIVE;
						}
						// Otherwise we just increment and request the next mission.
						else {
							++currentMissionIndex;
							MavLinkSendMissionRequest(currentMissionIndex);
                                                        char x[50];
                                                        snprintf(x, sizeof(x), "MISSION_REQUEST(%d) sent", currentMissionIndex);
                                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, x);
							nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT;
						}
					}
					// If we've run out of space before the last message, respond saying so.
					else {
						MavLinkSendMissionAck(MAV_MISSION_NO_SPACE);
                                                MavLinkSendStatusText(MAV_SEVERITY_DEBUG, "MISSION_ACK(NO_SPACE) sent");
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

void IncrementMissionCounter(void)
{
    if (missionTimeoutCounter < UINT8_MAX) {
        ++missionTimeoutCounter;
    }
}

int MavLinkAppendMission(const mavlink_mission_item_t *m)
{
	return true;
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
                                        char x[50];
                                        snprintf(x, sizeof(x), "MAVLINK_REQUEST(%d) received", currentMission.seq);
                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, x);
					MavLinkEvaluateMissionState(MISSION_EVENT_ITEM_RECEIVED, &currentMission);
					processedMissionMessage = true;
				} break;

				// Responding to a mission request entails moving into the first active state and scheduling a MISSION_COUNT message.
				// Will also schedule a transmission of a GPS_ORIGIN message. This is used for translating global to local coordinates
				// in QGC.
				case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
					MavLinkSendGpsGlobalOrigin();
                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, "MAVLINK_REQUEST_LIST received");
					MavLinkEvaluateMissionState(MISSION_EVENT_REQUEST_LIST_RECEIVED, NULL);
					processedMissionMessage = true;
				} break;

				// When a mission request message is received, respond with that mission information from the MissionManager
				case MAVLINK_MSG_ID_MISSION_REQUEST: {
					uint8_t receivedMissionIndex = mavlink_msg_mission_request_get_seq(&msg);
                                        char x[50];
                                        snprintf(x, sizeof(x), "MAVLINK_REQUEST(%d) received", receivedMissionIndex);
                                        MavLinkSendStatusText(MAV_SEVERITY_DEBUG, x);
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
			}
		}
	}

	// Now if no mission messages were received, trigger the Mission Manager anyways with a NONE
	// event.
	if (!processedMissionMessage) {
		MavLinkEvaluateMissionState(MISSION_EVENT_NONE, NULL);
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

			case MAVLINK_MSG_ID_TOKIMEC:
				MavLinkSendTokimec();
			break;

			case MAVLINK_MSG_ID_MAIN_POWER:
				MavLinkSendMainPower();
			break;

			default: {

			} break;
		}
	}
}
