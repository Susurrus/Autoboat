#include "uart1.h"
#include <stdint.h>
#include <common/mavlink.h>
 
static mavlink_system_t mavlink_system;

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

	uint8_t heartbeat_buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t heartbeat_len;
	 
	// Define this as a generic autopilot
	uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
	uint8_t system_mode = MAV_MODE_PREFLIGHT;
	uint8_t system_status = MAV_STATE_STANDBY;
	
	// Initialize the required buffers
	mavlink_message_t msg;

	// Pack the message
	mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, mavlink_system.type, autopilot_type, system_mode, 0, system_status);
	 
	// Copy the message to the send buffer
	heartbeat_len = mavlink_msg_to_send_buffer(heartbeat_buf, &msg);
	uart1EnqueueData(heartbeat_buf, (uint8_t)heartbeat_len);
}

void MavLinkSendStatus(void) {
	mavlink_message_t msg;
	
	mavlink_msg_sys_status_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 0, 0, 0, 500, 14000, 20000, 75, 20, 0, 0, 0, 0, 0);

	uint8_t status_buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t status_len;
	status_len = mavlink_msg_to_send_buffer(status_buf, &msg);
	
	uart1EnqueueData(status_buf, (uint8_t)status_len);
}

void MavLinkSendRawGps(void) {
	mavlink_message_t msg;
	
	mavlink_msg_gps_raw_int_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 1, 3, 35722, -78131, 0, 65535, 65535, 200, 1000, 4);

	uint8_t status_buf[MAVLINK_MAX_PACKET_LEN];
	uint16_t status_len;
	status_len = mavlink_msg_to_send_buffer(status_buf, &msg);
	
	uart1EnqueueData(status_buf, (uint8_t)status_len);
}
