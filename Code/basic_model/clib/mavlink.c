#include "uart1.h"
#include "gps.h"

#include <stdint.h>
#include <sealion/mavlink.h>
 
static mavlink_system_t mavlink_system;
static int packet_drops = 0;

// These variables are only used locally but are declared here module wise as all uses are orthogonal.
static mavlink_status_t status;

static uint8_t system_state = MAV_STATE_STANDBY;

// Declare a character buffer here to simplify things
static uint8_t buf[MAVLINK_MAX_PACKET_LEN];
static uint16_t len;

/**
 * This function creates a MAVLink heartbeat message with some basic parameters and
 * caches that message (along with its size) in the module-level variables declared
 * above. This buffer should be transmit at 1Hz back to the groundstation.
 */
void MavLinkInit(void) {
 
	mavlink_system.sysid = 20;                              // ID 20 for this vehicle
	mavlink_system.compid = MAV_COMP_ID_SYSTEM_CONTROL;     // The component sending the message is the IMU, it could be also a Linux process
	mavlink_system.type = MAV_TYPE_SURFACE_BOAT;            // This system is a surface vessel

}

/**
 * This function transmits the cached heartbeat message via UART1.
 */
void MavLinkSendHeartbeat(void) {
	mavlink_message_t msg;
	 
	// Define this as a generic autopilot
	uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
	uint8_t system_mode = MAV_MODE_PREFLIGHT;

	// Pack the message
	mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, mavlink_system.type, autopilot_type, system_mode, 0, system_state);
	 
	// Copy the message to the send buffer
	len = mavlink_msg_to_send_buffer(buf, &msg);
	uart1EnqueueData(buf, (uint8_t)len);
}

/**
 * This function transmits a MAVLink status message.
 * It takes in a single argument for the computation load on the system. It should be between 0 and 100 and be in % of usage of the mainloop time.
 * TODO: Add info on the GPS sensor.
 */
void MavLinkSendStatus(uint8_t load) {
	mavlink_message_t msg;
	
	mavlink_msg_sys_status_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 0, 0, 0, ((uint16_t)load)*10, 14000, 20000, 75, 20, 0, 0, 0, 0, 0);
	
	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	uart1EnqueueData(buf, (uint8_t)len);
}

/**
 * Pull the raw GPS sensor data from the gpsSensorData struct within the GPS module and
 * transmit it via MAVLink over UART1. This function should only be called when the GPS
 * data has been updated.
 */
void MavLinkSendRawGps(uint32_t systemTime) {
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
 */
void MavLinkSendAttitude(uint32_t systemTime, float yaw) {
	mavlink_message_t msg;

	mavlink_msg_attitude_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                                  systemTime*10, 0.0, 0.0, yaw, 0.0, 0.0, 0.0);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	uart1EnqueueData(buf, (uint8_t)len);
}

/**
 * This function takes in the system time, local position, and local velocity from
 * Matlab, unpacks it, and ships it off over a MAVLink MAVLINK_MSG_ID_LOCAL_POSITION_NED
 * message.
 */
void MavLinkSendLocalPosition(uint8_t *data) {
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

	mavlink_message_t msg;

	mavlink_msg_local_position_ned_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                                        systemTime.ulData, localPosX.flData, localPosY.flData, 0.0, localVelX.flData, localVelY.flData, 0.0);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	uart1EnqueueData(buf, (uint8_t)len);
}

void MavLinkSendGpsGlobalOrigin(void) {
	mavlink_message_t msg;

	mavlink_msg_gps_global_origin_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                                       35722, -78131, 0);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	uart1EnqueueData(buf, (uint8_t)len);
}

void MavLinkSendErrorsAndStatus(uint16_t status, uint16_t errors) {
	mavlink_message_t msg;

	mavlink_msg_status_and_errors_pack(mavlink_system.sysid, mavlink_system.compid, &msg, status, errors);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	uart1EnqueueData(buf, (uint8_t)len);
}

/**
* @brief Receive communication packets and handle them. Should be called at the system sample rate.
*
* This function decodes packets on the protocol level and also handles
* their value by calling the appropriate functions.
*/
void MavLinkReceive(void) {
 
	while(GetLength(&uart1RxBuffer) > 0) {
		uint8_t c;
		Read(&uart1RxBuffer, &c);
		// Try to get a new message
		mavlink_message_t msg;
		if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
			// Handle message
			mavlink_message_t out_msg;
 
			switch(msg.msgid) {

			// Processing reception of a new mission
			case MAVLINK_MSG_ID_MISSION_COUNT:
				mavlink_msg_mission_request_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &out_msg,
					msg.sysid, msg.compid, 0);
				len = mavlink_msg_to_send_buffer(buf, &out_msg);
				uart1EnqueueData(buf, (uint8_t)len);
				break;
			case MAVLINK_MSG_ID_MISSION_ITEM:
				mavlink_msg_mission_ack_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &out_msg, msg.sysid, msg.compid, 0);
				len = mavlink_msg_to_send_buffer(buf, &out_msg);
				uart1EnqueueData(buf, (uint8_t)len);
				break;

			// Processing sending of mission waypoint list to QGC.
			case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
				mavlink_msg_mission_count_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &out_msg,
					msg.sysid, msg.compid, 1);
				len = mavlink_msg_to_send_buffer(buf, &out_msg);
				uart1EnqueueData(buf, (uint8_t)len);
				break;
			case MAVLINK_MSG_ID_MISSION_REQUEST:
				mavlink_msg_mission_item_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &out_msg,
					msg.sysid, msg.compid, mavlink_msg_mission_request_get_seq(&msg),
					MAV_FRAME_LOCAL_NED, MAV_CMD_NAV_LOITER_TIME, 1,
					1, 0, 0, 0, 0, 44, 55, 0);
				len = mavlink_msg_to_send_buffer(buf, &out_msg);
				uart1EnqueueData(buf, (uint8_t)len);
				break;

			// Allow for clearing waypoints. Here we respond simply with an ACK message signifying no error.
			case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
				mavlink_msg_mission_ack_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &out_msg,
					msg.sysid, msg.compid, 0);
				len = mavlink_msg_to_send_buffer(buf, &out_msg);
				uart1EnqueueData(buf, (uint8_t)len);
				break;

			// Allow for the groundstation to set the current mission. This requires a WAYPOINT_CURRENT response message agreeing with the received current message index.
			case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
				mavlink_msg_mission_current_pack(mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, &out_msg,
					mavlink_msg_mission_set_current_get_seq(&msg));
				len = mavlink_msg_to_send_buffer(buf, &out_msg);
				uart1EnqueueData(buf, (uint8_t)len);
				break;

			// Catch the rest and do nothing in that case.
			default:
				c = '0';
				break;
			}
		}
 
		// And get the next one
	}
 
	// Update global packet drops counter
	packet_drops += status.packet_rx_drop_count;
}

