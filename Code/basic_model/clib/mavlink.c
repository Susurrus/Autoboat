#include "uart1.h"
#include "uart2.h"
#include "gps.h"

/* Include generated header file */
#include "MissionManager.h"

#include <stdint.h>
#include <common/mavlink.h>
//#include <sealion/mavlink.h>

// Store a module-wide variable for common MAVLink system variables.
static mavlink_system_t mavlink_system = {
	20, // Arbitrarily chosen MAV number
	MAV_COMP_ID_SYSTEM_CONTROL,
	MAV_TYPE_SURFACE_BOAT,
	MAV_STATE_UNINIT,
	MAV_MODE_PREFLIGHT,
	0 // Unused and unsure of expected usage
};

// Store the MAVLink communication status
static mavlink_status_t status;

// Declare a character buffer here to prevent continual allocation/deallocation of MAVLink buffers.
static uint8_t buf[MAVLINK_MAX_PACKET_LEN];
static uint16_t len;

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
void MavLinkSendStatus(uint8_t load)
{
	mavlink_message_t msg;

	// Declare that we have onboard sensors: 3D gyro, 3D accelerometer, 3D magnetometer, GPS
	// And that we have the following controllers: yaw position, x/y position control, motor outputs/control.
	// We reuse this variable for both the enabled and health status parameters as these are all assumed on and good.
	// TODO: Use actual sensor health status for that portion of the status message.
	uint32_t systemsPresent = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 5) |
	                          (1 << 12) | (1 << 14) | (1 << 15);
	
	mavlink_msg_sys_status_pack(mavlink_system.sysid, mavlink_system.compid, &msg, systemsPresent, systemsPresent, systemsPresent, ((uint16_t)load)*10, 14000, 20000, 75, 20, 0, 0, 0, 0, 0);
	
	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	uart1EnqueueData(buf, (uint8_t)len);
}

/**
 * Pull the raw GPS sensor data from the gpsSensorData struct within the GPS module and
 * transmit it via MAVLink over UART1. This function should only be called when the GPS
 * data has been updated.
 */
void MavLinkSendRawGps(uint32_t systemTime)
{
	mavlink_message_t msg;
	
	tGpsData gpsSensorData;
	GetGpsData(&gpsSensorData);
 
	mavlink_msg_gps_raw_int_pack(mavlink_system.sysid, mavlink_system.compid, &msg, ((uint64_t)systemTime)*10000,
								 gpsSensorData.fix, (int32_t)(gpsSensorData.lat.flData*1e7), (int32_t)(gpsSensorData.lon.flData*1e7), (int32_t)(gpsSensorData.alt.flData*1e7),
								 (uint16_t)gpsSensorData.hdop.flData*100, 0xFFFF,
								 (uint16_t)gpsSensorData.sog.flData*100, (uint16_t)gpsSensorData.cog.flData * 100,
								 gpsSensorData.sats);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	uart1EnqueueData(buf, (uint8_t)len);
}

/**
 * Transmits the vehicle attitude. Right now just the yaw value.
 * Expects systemTime to be in centiseconds which are then converted
 * to ms for transmission.
 * Yaw should be in radians where positive is eastward from north.
 */
void MavLinkSendAttitude(uint32_t systemTime, float yaw)
{
	mavlink_message_t msg;

	mavlink_msg_attitude_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                              systemTime*10, 0.0, 0.0, yaw, 0.0, 0.0, 0.0);

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
 * This function takes in the system time, local position, and local velocity from
 * Matlab, unpacks it, and ships it off over a MAVLink MAVLINK_MSG_ID_LOCAL_POSITION_NED
 * message.
 */
void MavLinkSendLocalPosition(uint8_t *data)
{
	mavlink_message_t msg;
	tUnsignedLongToChar systemTime;
	tFloatToChar localPosX, localPosY, localVelX, localVelY;
	
	systemTime.chData[0] = data[0];
	systemTime.chData[1] = data[1];
	systemTime.chData[2] = data[2];
	systemTime.chData[3] = data[3];
	
	localPosX.chData[0] = data[4];
	localPosX.chData[1] = data[5];
	localPosX.chData[2] = data[6];
	localPosX.chData[3] = data[7];
	
	localPosY.chData[0] = data[8];
	localPosY.chData[1] = data[9];
	localPosY.chData[2] = data[10];
	localPosY.chData[3] = data[11];
	
	localVelX.chData[0] = data[12];
	localVelX.chData[1] = data[13];
	localVelX.chData[2] = data[14];
	localVelX.chData[3] = data[15];
	
	localVelY.chData[0] = data[16];
	localVelY.chData[1] = data[17];
	localVelY.chData[2] = data[18];
	localVelY.chData[3] = data[19];

	mavlink_msg_local_position_ned_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                                        systemTime.ulData, localPosX.flData, localPosY.flData, 0.0, localVelX.flData, localVelY.flData, 0.0);

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

void MavLinkUpdateAndSendStatusErrors(uint16_t status, uint16_t errors)
{
	/// First we update the system state
	
	// If the startup reset line is triggered, indicate we're booting up. This is the only unarmed state
	// although that's not technically true with this controller.
	if (errors & (1 << 0)) {
		mavlink_system.state = MAV_STATE_BOOT;
		mavlink_system.mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
	// Otherwise if we're undergoing calibration indicate that
	} else if (errors & (1 << 5)) {
		mavlink_system.state = MAV_STATE_CALIBRATING;
		mavlink_system.mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	// Otherwise if there're any other errors we're in standby
	} else if (errors > 0) {
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
	if (status & (1 << 0)) {
		mavlink_system.mode |= (MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED);
		mavlink_system.mode &= ~MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
	} else {
		mavlink_system.mode &= ~(MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED);
		mavlink_system.mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
	}	
	// Set HIL status
	if (status & (1 << 1)) {
		mavlink_system.mode |= MAV_MODE_FLAG_HIL_ENABLED;
	} else {
		mavlink_system.mode &= ~MAV_MODE_FLAG_HIL_ENABLED;
	}
	
	// Then the status_and_errors message is transmit

	//mavlink_message_t msg;
	//mavlink_msg_status_and_errors_pack(mavlink_system.sysid, mavlink_system.compid, &msg, status, errors);
	//len = mavlink_msg_to_send_buffer(buf, &msg);
	//uart1EnqueueData(buf, (uint8_t)len);
}

/**
 * Transmit the current mission index via UART1.
 */
void MavLinkSendCurrentMission(void)
{
	mavlink_message_t msg;
	uint8_t currentMission;
	
	GetCurrentMission(&currentMission);
	
	mavlink_msg_mission_current_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &msg, currentMission);
	len = mavlink_msg_to_send_buffer(buf, &msg);
	uart1EnqueueData(buf, (uint8_t)len);
}

/**
* @brief Receive communication packets and handle them. Should be called at the system sample rate.
*
* This function decodes packets on the protocol level and also handles
* their value by calling the appropriate functions.
*/
void MavLinkReceive(void)
{

	// Store the new size of the mission list. Used when receiving a new mission list to determine when
	// we're at the end.
	static mavlinkNewMissionListSize = -1;
	
	mavlink_message_t msg;
 
	while(GetLength(&uart1RxBuffer) > 0) {
		uint8_t c;
		Read(&uart1RxBuffer, &c);
		// Try to get a new message
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			// Handle message
			mavlink_message_t out_msg;
 
			switch(msg.msgid) {

			// Begin reception of a new mission list. We first clear the mission list in preparation for new items and then
			// request the first waypoint.
			case MAVLINK_MSG_ID_MISSION_COUNT:
				ClearMissionList();
				// Record the number of incoming missions.
				mavlinkNewMissionListSize = mavlink_msg_mission_count_get_count(&msg);
				
				mavlink_msg_mission_request_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &out_msg,
					msg.sysid, msg.compid, 0);
				len = mavlink_msg_to_send_buffer(buf, &out_msg);
				uart1EnqueueData(buf, (uint8_t)len);
				break;

			// Handle receiving a mission. Once
			case MAVLINK_MSG_ID_MISSION_ITEM:
			{
				mavlink_mission_item_t newMission;
				mavlink_msg_mission_item_decode(&msg, &newMission);
				
				// Make sure that they're coming in in the right order, and if they don't return an error in
				// the acknowledgment response. This works because the mission list was cleared when we received
				// the MISSION_COUNT message.
				uint8_t missionCount;
				GetMissionCount(&missionCount);
				if (missionCount == newMission.seq) {
				
					Mission m = {
						newMission.x, newMission.y, newMission.z,
						newMission.frame, newMission.command,
						newMission.param1, newMission.param2, newMission.param3, newMission.param4, 
						newMission.autocontinue
					};
					
					// Attempt to record this mission to the list, recording the result, which will be 0 for failure.
					uint8_t missionAddStatus;
					AppendMission(&m, &missionAddStatus);
					
					if (missionAddStatus) {
						// If this is going to be the new current mission, then we should set it as such.
						if (newMission.current) {
							SetCurrentMission(newMission.seq);
						}
					
						// If this was the last mission we were expecting, respond with an ACK confirming that we've successfully
						// received the entire mission list.
						if (missionAddStatus == mavlinkNewMissionListSize) {
							mavlink_msg_mission_ack_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &out_msg, msg.sysid, msg.compid, MAV_MISSION_ACCEPTED);
							len = mavlink_msg_to_send_buffer(buf, &out_msg);
							uart1EnqueueData(buf, (uint8_t)len);
					
						// Otherwise we just continue requesting for the subsequent missions.
						} else {
							mavlink_msg_mission_request_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &out_msg,
								msg.sysid, msg.compid, newMission.seq + 1);
							len = mavlink_msg_to_send_buffer(buf, &out_msg);
							uart1EnqueueData(buf, (uint8_t)len);
						}
					// If we've run out of space before the last message, respond saying so.
					} else {
						mavlink_msg_mission_ack_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &out_msg, msg.sysid, msg.compid, MAV_MISSION_NO_SPACE);
						len = mavlink_msg_to_send_buffer(buf, &out_msg);
						uart1EnqueueData(buf, (uint8_t)len);
					}
				} else {
					mavlink_msg_mission_ack_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &out_msg, msg.sysid, msg.compid, MAV_MISSION_INVALID_SEQUENCE);
					len = mavlink_msg_to_send_buffer(buf, &out_msg);
					uart1EnqueueData(buf, (uint8_t)len);
				}
			}
				break;

			// Processing sending of mission waypoint list to QGC.
			case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
			{
				uint8_t missionCount;
				GetMissionCount(&missionCount);
				mavlink_msg_mission_count_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &out_msg,
					msg.sysid, msg.compid, missionCount);
				len = mavlink_msg_to_send_buffer(buf, &out_msg);
				uart1EnqueueData(buf, (uint8_t)len);
			}
				break;

			// When a mission request message is received, respond with that mission information from the MissionManager
			case MAVLINK_MSG_ID_MISSION_REQUEST:
			{
				uint8_t requestedMission = mavlink_msg_mission_request_get_seq(&msg);
				Mission m;
				uint8_t result;
				GetMission(requestedMission, &m, &result);
				if (result) {
					uint8_t currMissionIndex;
					GetCurrentMission(&currMissionIndex);
					mavlink_msg_mission_item_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &out_msg,
						msg.sysid, msg.compid, mavlink_msg_mission_request_get_seq(&msg),
						m.refFrame, m.action, (requestedMission == currMissionIndex),
						m.autocontinue, m.parameters[0], m.parameters[1], m.parameters[2], m.parameters[3], m.coordinates[0], m.coordinates[1], m.coordinates[2]);
					len = mavlink_msg_to_send_buffer(buf, &out_msg);
					uart1EnqueueData(buf, (uint8_t)len);
				}
			}
				break;
			// Allow for clearing waypoints. Here we respond simply with an ACK message if we successfully
			// cleared the mission list.
			case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
				ClearMissionList();
				mavlink_msg_mission_ack_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &out_msg,
					msg.sysid, msg.compid, MAV_MISSION_ACCEPTED);
				len = mavlink_msg_to_send_buffer(buf, &out_msg);
				uart1EnqueueData(buf, (uint8_t)len);
				break;

			// Allow for the groundstation to set the current mission. This requires a WAYPOINT_CURRENT response message agreeing with the received current message index.
			case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
			{
				uint8_t newCurrentMission = mavlink_msg_mission_set_current_get_seq(&msg);
				SetCurrentMission(newCurrentMission);
				
				// Here we respond with the mission pulled via GetCurrentMission() so that we're sure we're sending the right mission #
				MavLinkSendCurrentMission();
			}
				break;

			// Ignore incoming ACK messages for now.
			case MAVLINK_MSG_ID_MISSION_ACK:
				break;
			default:
				break;
			}
		}
 
		// And get the next one
	}
 
	status.packet_rx_drop_count;
}

