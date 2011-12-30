#include "uart1.h"
#include <stdint.h>
#include <common/mavlink.h>
 
static mavlink_system_t mavlink_system;

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
	 
	// Define this as a generic autopilot
	uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;
	uint8_t system_mode = MAV_MODE_PREFLIGHT;
	uint8_t system_status = MAV_STATE_STANDBY;
	
	// Initialize the required buffers
	mavlink_message_t msg;

	// Pack the message
	mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, mavlink_system.type, autopilot_type, system_mode, 0, system_status);
	 
	// Copy the message to the send buffer
	len = mavlink_msg_to_send_buffer(buf, &msg);
	uart1EnqueueData(buf, (uint8_t)len);
}

void MavLinkSendStatus(void) {
	mavlink_message_t msg;
	
	mavlink_msg_sys_status_pack(mavlink_system.sysid, mavlink_system.compid, &msg, 0, 0, 0, 500, 14000, 20000, 75, 20, 0, 0, 0, 0, 0);
	
	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	uart1EnqueueData(buf, (uint8_t)len);
}

void MavLinkSendRawGps(uint32_t systemTime) {
	mavlink_message_t msg;
	
	mavlink_msg_gps_raw_int_pack(mavlink_system.sysid, mavlink_system.compid, &msg, ((uint64_t)systemTime)*1000, 3, 35722, -78131, 0, 65535, 65535, 200, 1000, 4);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	uart1EnqueueData(buf, (uint8_t)len);
}

void MavLinkSendFiltGps(uint32_t systemTime) {
	mavlink_message_t msg;

	//mavlink_msg_global_position_int_pack(mavlink_system.sysid, mavlink_system.compid, &msg, systemTime, 35722, -78131, 0, 0, 1000, 1000, 0, 319);
	mavlink_msg_global_position_int_pack(mavlink_system.sysid, mavlink_system.compid, &msg, systemTime, 0, 0, 0, 0, 0, 0, 0, 0);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	uart1EnqueueData(buf, (uint8_t)len);
}

void MavLinkSendAttitude(uint32_t systemTime) {
	mavlink_message_t msg;

	mavlink_msg_attitude_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                                  systemTime, .5, .5, .5, .01, .01, .01);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	uart1EnqueueData(buf, (uint8_t)len);
}

void MavLinkSendLocalPosition(uint32_t systemTime) {
	mavlink_message_t msg;

	mavlink_msg_local_position_ned_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                                        systemTime, 1.5, 2.5, 3.5, .1, .2, .3);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	uart1EnqueueData(buf, (uint8_t)len);
}

void MavLinkSendGpsGlobalOrigin() {
	mavlink_message_t msg;

	mavlink_msg_gps_global_origin_pack(mavlink_system.sysid, mavlink_system.compid, &msg,
                                       35722, -78131, 0);

	len = mavlink_msg_to_send_buffer(buf, &msg);
	
	uart1EnqueueData(buf, (uint8_t)len);
}
