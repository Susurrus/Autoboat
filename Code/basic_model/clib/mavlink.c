#include "uart1.h"
#include "uart2.h"
#include "gps.h"
#include "MavlinkMessageScheduler.h"
#include "ecanSensors.h"

/* Include generated header files */
#include "MissionManager.h"
#include "code_gen.h"

#include <stdint.h>
#include <stdio.h>
#include "mavlink.h"

// Store a module-wide variable for common MAVLink system variables.
static mavlink_system_t mavlink_system = {
	20, // Arbitrarily chosen MAV number
	MAV_COMP_ID_SYSTEM_CONTROL,
	MAV_TYPE_SURFACE_BOAT,
	MAV_STATE_UNINIT,
	MAV_MODE_PREFLIGHT,
	0 // Unused and unsure of expected usage
};

// Latch onto the first groundstation unit and only receive and transmit to it.
static uint8_t groundStationSystemId = 0;
static uint8_t groundStationComponentId = 0;

// Store the MAVLink communication status. This struct is used by various MAVLink functions
// to track the MAVLink decoding state among some other parameters.
static mavlink_status_t status;

// Globally declare here how many parameters we have. Should be dealt with better later.
static uint16_t parameterCount = 4;

// Set up some state machine variables for the parameter protocol
enum {
	PARAM_STATE_INACTIVE = 0,
	PARAM_STATE_SINGLETON_TRANSMIT_START,
	PARAM_STATE_SINGLETON_TRANSMIT_WAITING,
	PARAM_STATE_STREAM_TRANSMIT_START,
	PARAM_STATE_STREAM_TRANSMIT_PARAM,
	PARAM_STATE_STREAM_TRANSMIT_WAITING,
	PARAM_STATE_STREAM_TRANSMIT_DELAY
};
static uint8_t currentParameter;
static uint8_t delayCountdown = 0;
static uint8_t parameterProtocolState = PARAM_STATE_INACTIVE;

// Set up state machine variables for the mission protocol
enum {
	MISSION_STATE_INACTIVE = 0,
	
	// States handling reading of our mission list
	MISSION_STATE_SEND_MISSION_COUNT,
	MISSION_STATE_COUNTDOWN,
	MISSION_STATE_SEND_MISSION_ITEM,
	
	// States handling current mission setting
	MISSION_STATE_SEND_CURRENT,
	
	// States handling writing to our mission list
	MISSION_STATE_SEND_REQUEST,
	MISSION_STATE_ACK_ERROR,
	MISSION_STATE_ACK_NO_SPACE,
	MISSION_STATE_ACK_INVALID_SEQUENCE,
	MISSION_STATE_ACK_ACCEPTED
};
// Set up the events necessary for the mission protocol state machine
enum {
	MISSION_EVENT_NO_EVENT = 0,
	MISSION_EVENT_STATE_ENTRY,
	MISSION_EVENT_STATE_EXIT,
	
	// Message reception events
	MISSION_EVENT_COUNT_RECEIVED,
	MISSION_EVENT_ACK_RECEIVED,
	MISSION_EVENT_REQUEST_RECEIVED,
	MISSION_EVENT_REQUEST_LIST_RECEIVED,
	MISSION_EVENT_CLEAR_ALL_RECEIVED,
	MISSION_EVENT_SET_CURRENT_RECEIVED,
	MISSION_EVENT_ITEM_RECEIVED,
	
	// Message transmission events
	MISSION_EVENT_COUNT_DISPATCHED,
	MISSION_EVENT_ACK_DISPATCHED,
	MISSION_EVENT_REQUEST_DISPATCHED,
	MISSION_EVENT_CURRENT_DISPATCHED,
	MISSION_EVENT_ITEM_DISPATCHED
};

// Declare a character buffer here to prevent continual allocation/deallocation of MAVLink buffers.
static uint8_t buf[MAVLINK_MAX_PACKET_LEN];
static uint16_t len;

/**
 * Initialize MAVLink transmission. This just sets up the MAVLink scheduler with the basic
 * repeatedly-transmit messages.
 */
void MavLinkInit(void)
{
	AddMessage(MAVLINK_MSG_ID_HEARTBEAT, 1);
	AddMessage(MAVLINK_MSG_ID_SYS_STATUS, 1);
	AddMessage(MAVLINK_MSG_ID_GPS_RAW_INT, 1);
	AddMessage(MAVLINK_MSG_ID_STATUS_AND_ERRORS, 4);
	AddMessage(MAVLINK_MSG_ID_WSO100, 2);
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
	mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, mavlink_system.type, MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY, mavlink_system.mode, 0, mavlink_system.state);
	 
	// Copy the message to the send buffer
	len = mavlink_msg_to_send_buffer(buf, &msg);
	uart1EnqueueData(buf, (uint8_t)len);
}

/**
 * This function transmits a MAVLink status message.
 * It takes in a single argument for the computation load on the system. It should be between 0 and 100 and be in % of usage of the mainloop time.
 * TODO: Add info on the GPS sensor.
 */
void MavLinkSendStatus(void)
{
	mavlink_message_t msg;

	// Declare that we have onboard sensors: 3D gyro, 3D accelerometer, 3D magnetometer, GPS
	// And that we have the following controllers: yaw position, x/y position control, motor outputs/control.
	// We reuse this variable for both the enabled and health status parameters as these are all assumed on and good.
	// TODO: Use actual sensor health status for that portion of the status message.
	uint32_t systemsPresent = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 5) |
	                          (1 << 12) | (1 << 14) | (1 << 15);
	
	mavlink_msg_sys_status_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 
	                            systemsPresent, systemsPresent, systemsPresent, 
								((uint16_t)systemStatus.cpu_load)*10, 14000, 20000, 75, 20, 0, 0, 0, 0, 0);
	
	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	uart1EnqueueData(buf, (uint8_t)len);
}

/**
 * Pull the raw GPS sensor data from the gpsSensorData struct within the GPS module and
 * transmit it via MAVLink over UART1. This function should only be called when the GPS
 * data has been updated.
 */
void MavLinkSendRawGps(void)
{
	mavlink_message_t msg;
	
	tGpsData gpsSensorData;
	GetGpsData(&gpsSensorData);
 
	mavlink_msg_gps_raw_int_pack(mavlink_system.sysid, mavlink_system.compid, &msg, ((uint64_t)systemStatus.time)*10000,
	                             gpsSensorData.fix, (int32_t)(gpsSensorData.lat.flData*1e7), (int32_t)(gpsSensorData.lon.flData*1e7), (int32_t)(gpsSensorData.alt.flData*1e7),
								 (uint16_t)gpsSensorData.hdop.flData*100, 0xFFFF,
								 (uint16_t)gpsSensorData.sog.flData*100, (uint16_t)gpsSensorData.cog.flData * 100,
								 gpsSensorData.sats);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	uart1EnqueueData(buf, (uint8_t)len);
}

/**
 * Transmits the vehicle attitude. Right now just the yaw value.
 * Expects systemStatus.time to be in centiseconds which are then converted
 * to ms for transmission.
 * Yaw should be in radians where positive is eastward from north.
 */
void MavLinkSendAttitude(float yaw)
{
	mavlink_message_t msg;

	mavlink_msg_attitude_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                              systemStatus.time*10, 0.0, 0.0, yaw, 0.0, 0.0, 0.0);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	uart1EnqueueData(buf, (uint8_t)len);
}

/**
 * Transmits some information necessary for the HUD. Specifically unique to this packet
 * is airspeed, throttle, and climb rate of which I only care about the throttle value.
 */
void MavLinkSendVfrHud(float groundSpeed, int16_t heading, uint16_t throttle)
{
	mavlink_message_t msg;

	mavlink_msg_vfr_hud_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                              0.0, groundSpeed, heading, throttle, 0.0, 0.0);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	uart1EnqueueData(buf, (uint8_t)len);
}

/**
 * This function takes in the local position and local velocity (as 3-tuples) from
 * Matlab as real32s and ships them off over a MAVLink MAVLINK_MSG_ID_LOCAL_POSITION_NED
 * message.
 * It also expects a systemStatus struct with a uint32_t time element that holds the
 * current system time in centiseconds.
 */
void MavLinkSendLocalPosition(float *data)
{
	mavlink_message_t msg;

	mavlink_msg_local_position_ned_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                                            systemStatus.time*10, data[0], data[1], data[2], data[3], data[4], data[5]);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	uart1EnqueueData(buf, (uint8_t)len);
}

/**
 * Transmits the current GPS position of the origin of the local coordinate frame that the North-East-Down
 * coordinates are all relative too. They should be in units of 1e-7 degrees.
 */
void MavLinkSendGpsGlobalOrigin(int32_t latitude, int32_t longitude, int32_t altitude)
{
	mavlink_message_t msg;

	mavlink_msg_gps_global_origin_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
	                                   latitude, longitude, altitude);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	uart1EnqueueData(buf, (uint8_t)len);
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
		mavlink_msg_mission_current_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &msg, (uint16_t)currentMission);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		uart1EnqueueData(buf, (uint8_t)len);
	}
}

/**
 * Transmit a mission acknowledgement message. The type of message is the sole argument to this 
 * function (see enum MAV_MISSIONRESULT).
 */
void MavLinkSendMissionAck(uint8_t type)
{
	mavlink_message_t msg;
	mavlink_msg_mission_ack_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &msg, 
	                             groundStationSystemId, groundStationComponentId, type);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	uart1EnqueueData(buf, (uint8_t)len);
}

void MavLinkSendMissionCount(void)
{
	uint8_t missionCount;
	mavlink_message_t msg;
	GetMissionCount(&missionCount);
	mavlink_msg_mission_count_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &msg,
	                               groundStationSystemId, groundStationComponentId, missionCount);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	uart1EnqueueData(buf, (uint8_t)len);
}

void MavLinkSendMissionItem(uint8_t currentMissionIndex)
{
	Mission m;
	uint8_t result;
	GetMission(currentMissionIndex, &m, &result);
	if (result) {
		mavlink_message_t msg;
		int8_t missionManagerCurrentIndex;
		GetCurrentMission(&missionManagerCurrentIndex);
		mavlink_msg_mission_item_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &msg,
		                              groundStationSystemId, groundStationComponentId, currentMissionIndex,
		                              m.refFrame, m.action, (currentMissionIndex == (uint8_t)missionManagerCurrentIndex),
		                              m.autocontinue, m.parameters[0], m.parameters[1], m.parameters[2], m.parameters[3],
		                              m.coordinates[0], m.coordinates[1], m.coordinates[2]);
		len = mavlink_msg_to_send_buffer(buf, &msg);
		uart1EnqueueData(buf, (uint8_t)len);
	}
}

void MavLinkSendMissionRequest(uint8_t currentMissionIndex)
{
	mavlink_message_t msg;
	mavlink_msg_mission_request_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &msg,
	                                 groundStationSystemId, groundStationComponentId, currentMissionIndex);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	uart1EnqueueData(buf, (uint8_t)len);
}

/**
 * The following functions are helper functions for reading the various parameters aboard the boat.
 */
void _transmitParameter0(void)
{
	mavlink_message_t msg;
	mavlink_param_union_t x;
	x.param_uint32 = (systemStatus.status & (1 << 0))?1:0;
	mavlink_msg_param_value_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
	                             "MODE_AUTO", x.param_float, MAV_VAR_UINT32, parameterCount, 0);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	uart1EnqueueData(buf, (uint8_t)len);
}
void _transmitParameter1(void)
{
	mavlink_message_t msg;
	mavlink_param_union_t x;
	x.param_uint32 = (systemStatus.status & (1 << 1))?1:0;
	mavlink_msg_param_value_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
								 "MODE_HIL", x.param_float, MAV_VAR_UINT32, parameterCount, 1);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	uart1EnqueueData(buf, (uint8_t)len);
}
void _transmitParameter2(void)
{
	mavlink_message_t msg;
	mavlink_param_union_t x;
	x.param_uint32 = (systemStatus.status & (1 << 2))?1:0;
	mavlink_msg_param_value_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
								 "MODE_HILSENSE", x.param_float, MAV_VAR_UINT32, parameterCount, 2);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	uart1EnqueueData(buf, (uint8_t)len);
}
void _transmitParameter3(void)
{
	mavlink_message_t msg;
	mavlink_param_union_t x;
	x.param_uint32 = (systemStatus.status & (1 << 3))?1:0;
	mavlink_msg_param_value_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
								 "MODE_RCDISCON", x.param_float, MAV_VAR_UINT32, parameterCount, 3);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	uart1EnqueueData(buf, (uint8_t)len);
}

/** Custom Sealion Messages **/

void MavLinkSendRudderRaw(uint16_t position, uint8_t port_limit, uint8_t starboard_limit)
{
//	mavlink_message_t msg;
//
//	mavlink_msg_rudder_raw_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
//                                position, port_limit, 0, starboard_limit);
//
//	len = mavlink_msg_to_send_buffer(buf, &msg);
//	
//	uart1EnqueueData(buf, (uint8_t)len);
}

void MavLinkSendStatusAndErrors(void)
{
	mavlink_message_t msg;
	mavlink_msg_status_and_errors_pack(mavlink_system.sysid, mavlink_system.compid, &msg, systemStatus.status, systemStatus.reset);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	uart1EnqueueData(buf, (uint8_t)len);

	// And finally update the MAVLink state based on the system state.
	
	// If the startup reset line is triggered, indicate we're booting up. This is the only unarmed state
	// although that's not technically true with this controller.
	if (systemStatus.reset & (1 << 0)) {
		mavlink_system.state = MAV_STATE_BOOT;
		mavlink_system.mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
	// Otherwise if we're undergoing calibration indicate that
	} else if (systemStatus.reset & (1 << 5)) {
		mavlink_system.state = MAV_STATE_CALIBRATING;
		mavlink_system.mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	// Otherwise if there're any other errors we're in standby
	} else if (systemStatus.reset > 0) {
		mavlink_system.state = MAV_STATE_STANDBY;
		mavlink_system.mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	// Finally we're active if there're no errors. Also indicate within the mode that we're armed.
	} else {
		mavlink_system.state = MAV_STATE_ACTIVE;
		mavlink_system.mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}
	
	/// Then we update the system mode using MAV_MODE_FLAGs
	// Set manual/autonomous mode. Note that they're not mutually exclusive within the MAVLink protocol,
	// though I treat them as such for my autopilot.
	if (systemStatus.status & (1 << 0)) {
		mavlink_system.mode |= (MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED);
		mavlink_system.mode &= ~MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
	} else {
		mavlink_system.mode &= ~(MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED);
		mavlink_system.mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
	}	
	// Set HIL status
	if (systemStatus.status & (1 << 1)) {
		mavlink_system.mode |= MAV_MODE_FLAG_HIL_ENABLED;
	} else {
		mavlink_system.mode &= ~MAV_MODE_FLAG_HIL_ENABLED;
	}
}

void MavLinkSendWindAirData(void)
{
	mavlink_message_t msg;
	mavlink_msg_wso100_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 
	                        windData.speed.flData, windData.direction.flData, airData.temp.flData, airData.pressure.flData, airData.humidity.flData);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	uart1EnqueueData(buf, (uint8_t)len);
}

/** Core receive->state machine->transmit MAVLink functions **/

void MavLinkEvaluateParameterState(uint8_t event, void *data)
{
	
	// First check the parameter protocol state
	switch (parameterProtocolState) {
		// Initialize variables for starting a a single parameter transmission
		case PARAM_STATE_SINGLETON_TRANSMIT_START: {
			AddTransientMessage(MAVLINK_MSG_ID_PARAM_VALUE);
			parameterProtocolState = PARAM_STATE_SINGLETON_TRANSMIT_WAITING;
		} break;
		
		case PARAM_STATE_SINGLETON_TRANSMIT_WAITING: {
			// A do-nothing placeholder state
		} break;
		
		// Initialize variables for starting a parameter transmission stream
		case PARAM_STATE_STREAM_TRANSMIT_START: {
			currentParameter = 0;
			parameterProtocolState = PARAM_STATE_STREAM_TRANSMIT_PARAM;
		} break;

		// Now transmit the current parameter
		case PARAM_STATE_STREAM_TRANSMIT_PARAM: {
			if (currentParameter < parameterCount) {
				AddTransientMessage(MAVLINK_MSG_ID_PARAM_VALUE);
				parameterProtocolState = PARAM_STATE_STREAM_TRANSMIT_WAITING;
			} else {
				parameterProtocolState = PARAM_STATE_INACTIVE;
			}
		} break;
		
		case PARAM_STATE_STREAM_TRANSMIT_WAITING: {
			// We do nothing in this state. It's just a placeholder until
			// we are switched into the _DELAY state.
		} break;
		
		// Add a delay of 10 timesteps before attempting to schedule another one
		case PARAM_STATE_STREAM_TRANSMIT_DELAY: {
			if (delayCountdown++ == 9) {
				delayCountdown = 0;
				parameterProtocolState = PARAM_STATE_STREAM_TRANSMIT_PARAM;
			}
		} break;
		
		default: break;
	}
}

/**
 * This function implements the mission protocol state machine for the MAVLink protocol.
 * events can be passed as the first argument, or NO_EVENT if desired. data is a pointer
 * to data if there is any to be passed to the state logic. data is not guaranteed to persist
 * beyond the single call to this function.
 */
void MavLinkEvaluateMissionState(uint8_t event, void *data)
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
			if (event == MISSION_EVENT_REQUEST_LIST_RECEIVED) {
				AddTransientMessage(MAVLINK_MSG_ID_MISSION_COUNT);
				currentMissionIndex = 0;
				nextState = MISSION_STATE_SEND_MISSION_COUNT;
			} else if (event == MISSION_EVENT_COUNT_RECEIVED) {
				// Don't allow for writing of new missions if we're in autonomous mode.
				if ((systemStatus.status & 0x0001) > 0) {
					AddTransientMessage(MAVLINK_MSG_ID_MISSION_ACK);
					nextState = MISSION_STATE_ACK_ERROR;
				} else {
					
					uint8_t newListSize = *(uint8_t *)data;
					
					// Only respond with a request if there are missions to request.
					// If we received a 0-length mission list, just respond with a MISSION_ACK error
					if (newListSize == 0) {
						AddTransientMessage(MAVLINK_MSG_ID_MISSION_ACK);
						nextState = MISSION_STATE_ACK_ERROR;
					} else if (newListSize > mList.maxSize) {
						AddTransientMessage(MAVLINK_MSG_ID_MISSION_ACK);
						nextState = MISSION_STATE_ACK_NO_SPACE;
					}
					// Otherwise we're set to start retrieving a new mission list so we request the first mission.
					else {
						mavlinkNewMissionListSize = newListSize;
						ClearMissionList();
						currentMissionIndex = 0;
						AddTransientMessage(MAVLINK_MSG_ID_MISSION_REQUEST);
						nextState = MISSION_STATE_SEND_REQUEST;
					}
				}
			} else if (event == MISSION_EVENT_CLEAR_ALL_RECEIVED) {
				// If we're in autonomous mode, don't allow for clearing the mission list
				if ((systemStatus.status & 0x0001) > 0) {
					AddTransientMessage(MAVLINK_MSG_ID_MISSION_ACK);
					nextState = MISSION_STATE_ACK_ERROR;
				}
				// But if we're in manual mode, go ahead and clear everything.
				else {
					ClearMissionList();
					AddTransientMessage(MAVLINK_MSG_ID_MISSION_ACK);
					nextState = MISSION_STATE_ACK_ACCEPTED;
				}
			} else if (event == MISSION_EVENT_SET_CURRENT_RECEIVED) {
				SetCurrentMission(*(uint8_t*)data);
				AddTransientMessage(MAVLINK_MSG_ID_MISSION_CURRENT);
				nextState = MISSION_STATE_SEND_CURRENT;
			}
		break;
		
		case MISSION_STATE_SEND_MISSION_COUNT:
			if (event == MISSION_EVENT_COUNT_DISPATCHED) {
				nextState = MISSION_STATE_COUNTDOWN;
			}
		break;

		case MISSION_STATE_COUNTDOWN:
			if (event == MISSION_EVENT_STATE_ENTRY) {
				counter = 0;
			} else if (event == MISSION_EVENT_NO_EVENT) {
				if (counter++ > 400) {
					uart2EnqueueData("timeout\r\n", 9); 
					nextState = MISSION_STATE_INACTIVE;
				}
			} else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
				if (*(uint8_t *)data == currentMissionIndex) {
					AddTransientMessage(MAVLINK_MSG_ID_MISSION_ITEM);
					nextState = MISSION_STATE_SEND_MISSION_ITEM;
				}
			} else if (event == MISSION_EVENT_ACK_RECEIVED) {
				nextState = MISSION_STATE_INACTIVE;
			} else if (event == MISSION_EVENT_ITEM_RECEIVED) {
				
				mavlink_mission_item_t incomingMission = *(mavlink_mission_item_t *)data;
			
				// Make sure that they're coming in in the right order, and if they don't return an error in
				// the acknowledgment response.
				if (currentMissionIndex == incomingMission.seq) {
					Mission m = {
						incomingMission.x, incomingMission.y, incomingMission.z,
						incomingMission.frame, incomingMission.command,
						incomingMission.param1, incomingMission.param2, incomingMission.param3, incomingMission.param4, 
						incomingMission.autocontinue
					};
					
					// Attempt to record this mission to the list, recording the result, which will be 0 for failure.
					int8_t missionAddStatus;
					AppendMission(&m, &missionAddStatus);
					
					if (missionAddStatus != -1) {
						// If this is going to be the new current mission, then we should set it as such.
						if (incomingMission.current) {
							SetCurrentMission(incomingMission.seq);
						}
						
						// If this was the last mission we were expecting, respond with an ACK confirming that we've successfully
						// received the entire mission list. Otherwise we just increment and request the next mission.
						if (currentMissionIndex == mavlinkNewMissionListSize - 1) {
							AddTransientMessage(MAVLINK_MSG_ID_MISSION_ACK);
							nextState = MISSION_STATE_ACK_ACCEPTED;
						} else {
							++currentMissionIndex;
							AddTransientMessage(MAVLINK_MSG_ID_MISSION_REQUEST);
							nextState = MISSION_STATE_SEND_REQUEST;
						}
					}
					// If we've run out of space before the last message, respond saying so.
					else {
						AddTransientMessage(MAVLINK_MSG_ID_MISSION_ACK);
						nextState = MISSION_STATE_ACK_NO_SPACE;
					}
				}
			} 
		break;
		
		case MISSION_STATE_SEND_MISSION_ITEM:
			if (event == MISSION_EVENT_ITEM_DISPATCHED) {
				MavLinkSendMissionItem(currentMissionIndex);
				++currentMissionIndex;
				nextState = MISSION_STATE_COUNTDOWN;
			}
		break;
		
		case MISSION_STATE_SEND_CURRENT:
			if (event == MISSION_EVENT_CURRENT_DISPATCHED) {
				nextState = MISSION_STATE_INACTIVE;
			}
		break;

		case MISSION_STATE_ACK_NO_SPACE:
			if (event == MISSION_EVENT_ACK_DISPATCHED) {
				MavLinkSendMissionAck(MAV_MISSION_NO_SPACE);
				nextState = MISSION_STATE_INACTIVE;
			}
		break;

		case MISSION_STATE_ACK_ERROR:
			if (event == MISSION_EVENT_ACK_DISPATCHED) {
				MavLinkSendMissionAck(MAV_MISSION_ERROR);
				nextState = MISSION_STATE_INACTIVE;
			}
		break;

		case MISSION_STATE_ACK_ACCEPTED:
			if (event == MISSION_EVENT_ACK_DISPATCHED) {
				MavLinkSendMissionAck(MAV_MISSION_ACCEPTED);
				nextState = MISSION_STATE_INACTIVE;
			}
		break;

		case MISSION_STATE_ACK_INVALID_SEQUENCE:
			if (event == MISSION_EVENT_ACK_DISPATCHED) {
				MavLinkSendMissionAck(MAV_MISSION_INVALID_SEQUENCE);
				nextState = MISSION_STATE_INACTIVE;
			}
		break;
		
		case MISSION_STATE_SEND_REQUEST: {
			if (event == MISSION_EVENT_REQUEST_DISPATCHED) {
				MavLinkSendMissionRequest(currentMissionIndex);
				nextState = MISSION_STATE_COUNTDOWN;
			}
		} break;
	}
	
	// Here is when we actually transition between states, calling init/exit code as necessary
	if (nextState != state) {
		// Spit out some debugging info
		// TODO: Remove
		char x[5];
		sprintf(x, "%d\r\n", nextState);
		uart2EnqueueData((uint8_t*)x, (uint8_t)strlen(x)); 
	
		MavLinkEvaluateMissionState(MISSION_EVENT_STATE_EXIT, NULL);
		state = nextState;
		MavLinkEvaluateMissionState(MISSION_EVENT_STATE_ENTRY, NULL);
	}
}

/**
* @brief Receive communication packets and handle them. Should be called at the system sample rate.
*
* This function decodes packets on the protocol level and also handles
* their value by calling the appropriate functions.
*/
void MavLinkReceive(void)
{
	mavlink_message_t msg;
 
	while(GetLength(&uart1RxBuffer) > 0) {
		uint8_t c;
		Read(&uart1RxBuffer, &c);
		// Parse another byte and if there's a message found process it.
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			
			if (msg.msgid == MAVLINK_MSG_ID_MISSION_REQUEST) {
				char x[5];
				int y = mavlink_msg_mission_request_get_seq(&msg);
				sprintf(x, "Rx:%d id=%d\r\n", msg.msgid, y);
				uart2EnqueueData((uint8_t*)x, (uint8_t)strlen(x)); 
			} else if (msg.msgid == MAVLINK_MSG_ID_MISSION_ITEM) {
				char x[5];
				int y = mavlink_msg_mission_item_get_seq(&msg);
				sprintf(x, "Rx:%d id=%d\r\n", msg.msgid, y);
				uart2EnqueueData((uint8_t*)x, (uint8_t)strlen(x)); 
			} else {
				char x[5];
				sprintf(x, "Rx:%d\r\n", msg.msgid);
				uart2EnqueueData((uint8_t*)x, (uint8_t)strlen(x)); 
			}
		
			// Latch the groundstation system and component ID if we haven't yet.
			if (!groundStationSystemId && !groundStationComponentId) {
				groundStationSystemId = msg.sysid;
				groundStationComponentId = msg.compid;
			}
		
			// Handle message
			mavlink_message_t out_msg;
 
			switch(msg.msgid) {

				// If we are not doing any mission protocol operations, record the size of the incoming mission
				// list and transition into the write missions state machine loop.
				case MAVLINK_MSG_ID_MISSION_COUNT: {
					uint8_t mavlinkNewMissionListSize = mavlink_msg_mission_count_get_count(&msg);
					MavLinkEvaluateMissionState(MISSION_EVENT_COUNT_RECEIVED, &mavlinkNewMissionListSize);
				} break;

				// Handle receiving a mission.
				case MAVLINK_MSG_ID_MISSION_ITEM: {
					mavlink_mission_item_t currentMission;
					mavlink_msg_mission_item_decode(&msg, &currentMission);
					MavLinkEvaluateMissionState(MISSION_EVENT_ITEM_RECEIVED, &currentMission);
				} break;

				// Responding to a mission request entails moving into the first active state and scheduling a MISSION_COUNT message
				case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
					MavLinkEvaluateMissionState(MISSION_EVENT_REQUEST_LIST_RECEIVED, NULL);
				break;

				// When a mission request message is received, respond with that mission information from the MissionManager
				case MAVLINK_MSG_ID_MISSION_REQUEST: {
					uint8_t receivedMissionIndex = mavlink_msg_mission_request_get_seq(&msg);
					MavLinkEvaluateMissionState(MISSION_EVENT_REQUEST_RECEIVED, &receivedMissionIndex);
				} break;

				// Allow for clearing waypoints. Here we respond simply with an ACK message if we successfully
				// cleared the mission list.
				case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
					MavLinkEvaluateMissionState(MISSION_EVENT_CLEAR_ALL_RECEIVED, NULL);
				break;

				// Allow for the groundstation to set the current mission. This requires a WAYPOINT_CURRENT response message agreeing with the received current message index.
				case MAVLINK_MSG_ID_MISSION_SET_CURRENT: {
					uint8_t newCurrentMission = mavlink_msg_mission_set_current_get_seq(&msg);
					MavLinkEvaluateMissionState(MISSION_EVENT_SET_CURRENT_RECEIVED, &newCurrentMission);					
				} break;

				case MAVLINK_MSG_ID_MISSION_ACK: {
					MavLinkEvaluateMissionState(MISSION_EVENT_ACK_RECEIVED, NULL);
				} break;
					
				// If they're requesting a list of all parameters, call a separate function that'll track the state and transmit the necessary messages.
				// This reason that this is an external function is so that it can be run separately at 20Hz.
				case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
					if (parameterProtocolState == PARAM_STATE_INACTIVE) {
						parameterProtocolState = PARAM_STATE_STREAM_TRANSMIT_START;
					}
				} break;
				
				// If a request comes for a single parameter then set that to be the current parameter and move into the proper state.
				case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
					if (parameterProtocolState == PARAM_STATE_INACTIVE) {
						currentParameter = (uint8_t)mavlink_msg_param_request_read_get_param_index(&msg);
						parameterProtocolState = PARAM_STATE_SINGLETON_TRANSMIT_START;
					}
				} break;
				
				// Handle receiving a parameter setting message. Here we check the string of the .param_id element of the message as the param_index value	
				// isn't transmit for some reason. This should be abstracted and be more automatic, but for now everything's carefully hardcoded.
				// TODO: Move all of this parameter processing into the MavlinkTransmit state machines
				case MAVLINK_MSG_ID_PARAM_SET: {
					if (parameterProtocolState == PARAM_STATE_INACTIVE) {
						mavlink_param_set_t x;
						mavlink_msg_param_set_decode(&msg, &x);
						mavlink_param_union_t data;
						data.param_float = x.param_value;
						if (strcmp(x.param_id, "MODE_AUTO") == 0) {
							if (data.param_uint32) {
								systemStatus.status |= (1 << 0);
							} else {
								systemStatus.status &= ~(1 << 0);
							}
							currentParameter = 0;
							_transmitParameter0();
						} else if (strcmp(x.param_id, "MODE_HIL") == 0) {
							if (data.param_uint32) {
								systemStatus.status |= (1 << 1);
							} else {
								systemStatus.status &= ~(1 << 1);
							}
							currentParameter = 1;
							_transmitParameter1();
						} else if (strcmp(x.param_id, "MODE_HILSENSE") == 0) {
							if (data.param_uint32) {
								systemStatus.status |= (1 << 2);
							} else {
								systemStatus.status &= ~(1 << 2);
							}
							currentParameter = 2;
							_transmitParameter2();
						} else if (strcmp(x.param_id, "MODE_RCDISCON") == 0) {
							if (data.param_uint32) {
								systemStatus.status |= (1 << 3);
							} else {
								systemStatus.status &= ~(1 << 3);
							}
							currentParameter = 3;
							_transmitParameter3();
						}
						
						// Trigger a response parameter value message
						parameterProtocolState = PARAM_STATE_SINGLETON_TRANSMIT_START;
					}
				} break;
				
				default: {
					c = '\0';
				} break;
			}
		}
	}
 
	status.packet_rx_drop_count;
}
/**
 * This function handles transmission of MavLink messages taking into account transmission
 * speed, message size, and desired transmission rate.
 */
void MavLinkTransmit(void)
{
	
	// And now transmit all messages for this timestep
	SListItem *messagesToSend = IncrementTimestep();
	SListItem *j;
	for (j = messagesToSend; j; j = j->sibling) {
	
		if (j->_transient) {
			char x[5];
			sprintf(x, "Tx:%d\r\n", j->id);
			uart2EnqueueData((uint8_t*)x, (uint8_t)strlen(x));
		}
			
		switch(j->id) {
			case MAVLINK_MSG_ID_HEARTBEAT: {
				MavLinkSendHeartbeat();
			} break;
			
			case MAVLINK_MSG_ID_SYS_STATUS: {
				MavLinkSendStatus();
			} break;
			
			case MAVLINK_MSG_ID_GPS_RAW_INT: {
				MavLinkSendRawGps();
			} break;
			
			case MAVLINK_MSG_ID_STATUS_AND_ERRORS: {
				MavLinkSendStatusAndErrors();
			} break;
			
			case MAVLINK_MSG_ID_WSO100: {
				MavLinkSendWindAirData();
			} break;
			
			case MAVLINK_MSG_ID_PARAM_VALUE: {
				switch (currentParameter) {
					case 0:
						_transmitParameter0();
					break;
					case 1:
						_transmitParameter1();
					break;
					case 2:
						_transmitParameter2();
					break;
					case 3:
						_transmitParameter3();
					break;
					default:
					break;
				}
				if (parameterProtocolState == PARAM_STATE_STREAM_TRANSMIT_WAITING) {
					++currentParameter;
					parameterProtocolState = PARAM_STATE_STREAM_TRANSMIT_DELAY;
				} else if (parameterProtocolState == PARAM_STATE_SINGLETON_TRANSMIT_WAITING) {
					parameterProtocolState = PARAM_STATE_INACTIVE;
				}
			} break;

			case MAVLINK_MSG_ID_MISSION_COUNT: {
				MavLinkSendMissionCount();
				MavLinkEvaluateMissionState(MISSION_EVENT_COUNT_DISPATCHED, NULL);
			} break;

			case MAVLINK_MSG_ID_MISSION_ITEM:
				MavLinkEvaluateMissionState(MISSION_EVENT_ITEM_DISPATCHED, NULL);
			break;
			
			case MAVLINK_MSG_ID_MISSION_REQUEST: {
				MavLinkEvaluateMissionState(MISSION_EVENT_REQUEST_DISPATCHED, NULL);
			} break;
			
			case MAVLINK_MSG_ID_MISSION_CURRENT: {
				MavLinkSendCurrentMission();
				MavLinkEvaluateMissionState(MISSION_EVENT_CURRENT_DISPATCHED, NULL);
			} break;
			
			case MAVLINK_MSG_ID_MISSION_ACK: {
				MavLinkEvaluateMissionState(MISSION_EVENT_ACK_DISPATCHED, NULL);
			} break;
			
			default: {
			
			} break;
		}
	}
}
