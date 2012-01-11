/** @file
 *	@brief MAVLink comm protocol testsuite generated from sealion.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef SEALION_TESTSUITE_H
#define SEALION_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_sealion(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_sealion(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_rudder_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rudder_raw_t packet_in = {
		17235,
	139,
	206,
	17,
	};
	mavlink_rudder_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.position = packet_in.position;
        	packet1.port_limit = packet_in.port_limit;
        	packet1.center_limit = packet_in.center_limit;
        	packet1.starboard_limit = packet_in.starboard_limit;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rudder_raw_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rudder_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rudder_raw_pack(system_id, component_id, &msg , packet1.position , packet1.port_limit , packet1.center_limit , packet1.starboard_limit );
	mavlink_msg_rudder_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rudder_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.position , packet1.port_limit , packet1.center_limit , packet1.starboard_limit );
	mavlink_msg_rudder_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rudder_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rudder_raw_send(MAVLINK_COMM_1 , packet1.position , packet1.port_limit , packet1.center_limit , packet1.starboard_limit );
	mavlink_msg_rudder_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_rudder_int(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rudder_int_t packet_in = {
		17235,
	17339,
	17443,
	};
	mavlink_rudder_int_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.port_limit_val = packet_in.port_limit_val;
        	packet1.starboat_limit_val = packet_in.starboat_limit_val;
        	packet1.rudder_angle = packet_in.rudder_angle;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rudder_int_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rudder_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rudder_int_pack(system_id, component_id, &msg , packet1.port_limit_val , packet1.starboat_limit_val , packet1.rudder_angle );
	mavlink_msg_rudder_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rudder_int_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.port_limit_val , packet1.starboat_limit_val , packet1.rudder_angle );
	mavlink_msg_rudder_int_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_rudder_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rudder_int_send(MAVLINK_COMM_1 , packet1.port_limit_val , packet1.starboat_limit_val , packet1.rudder_angle );
	mavlink_msg_rudder_int_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_prop_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_prop_raw_t packet_in = {
		17235,
	};
	mavlink_prop_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.speed = packet_in.speed;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_prop_raw_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_prop_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_prop_raw_pack(system_id, component_id, &msg , packet1.speed );
	mavlink_msg_prop_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_prop_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.speed );
	mavlink_msg_prop_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_prop_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_prop_raw_send(MAVLINK_COMM_1 , packet1.speed );
	mavlink_msg_prop_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_wind_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_wind_raw_t packet_in = {
		17235,
	17339,
	};
	mavlink_wind_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.speed = packet_in.speed;
        	packet1.direction = packet_in.direction;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_wind_raw_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_wind_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_wind_raw_pack(system_id, component_id, &msg , packet1.speed , packet1.direction );
	mavlink_msg_wind_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_wind_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.speed , packet1.direction );
	mavlink_msg_wind_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_wind_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_wind_raw_send(MAVLINK_COMM_1 , packet1.speed , packet1.direction );
	mavlink_msg_wind_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_air_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_air_raw_t packet_in = {
		17235,
	17339,
	17443,
	};
	mavlink_air_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.temperature = packet_in.temperature;
        	packet1.pressure = packet_in.pressure;
        	packet1.humidity = packet_in.humidity;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_air_raw_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_air_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_air_raw_pack(system_id, component_id, &msg , packet1.temperature , packet1.pressure , packet1.humidity );
	mavlink_msg_air_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_air_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.temperature , packet1.pressure , packet1.humidity );
	mavlink_msg_air_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_air_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_air_raw_send(MAVLINK_COMM_1 , packet1.temperature , packet1.pressure , packet1.humidity );
	mavlink_msg_air_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_water_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_water_raw_t packet_in = {
		17235,
	17339,
	17443,
	};
	mavlink_water_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.speed = packet_in.speed;
        	packet1.temperature = packet_in.temperature;
        	packet1.depth = packet_in.depth;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_water_raw_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_water_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_water_raw_pack(system_id, component_id, &msg , packet1.speed , packet1.temperature , packet1.depth );
	mavlink_msg_water_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_water_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.speed , packet1.temperature , packet1.depth );
	mavlink_msg_water_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_water_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_water_raw_send(MAVLINK_COMM_1 , packet1.speed , packet1.temperature , packet1.depth );
	mavlink_msg_water_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_status_and_errors(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_status_and_errors_t packet_in = {
		17235,
	17339,
	};
	mavlink_status_and_errors_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.status = packet_in.status;
        	packet1.errors = packet_in.errors;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_status_and_errors_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_status_and_errors_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_status_and_errors_pack(system_id, component_id, &msg , packet1.status , packet1.errors );
	mavlink_msg_status_and_errors_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_status_and_errors_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.status , packet1.errors );
	mavlink_msg_status_and_errors_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_status_and_errors_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_status_and_errors_send(MAVLINK_COMM_1 , packet1.status , packet1.errors );
	mavlink_msg_status_and_errors_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_actuator_commands(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_actuator_commands_t packet_in = {
		17235,
	17339,
	};
	mavlink_actuator_commands_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.rudder_angle = packet_in.rudder_angle;
        	packet1.throttle = packet_in.throttle;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_actuator_commands_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_actuator_commands_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_actuator_commands_pack(system_id, component_id, &msg , packet1.rudder_angle , packet1.throttle );
	mavlink_msg_actuator_commands_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_actuator_commands_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.rudder_angle , packet1.throttle );
	mavlink_msg_actuator_commands_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_actuator_commands_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_actuator_commands_send(MAVLINK_COMM_1 , packet1.rudder_angle , packet1.throttle );
	mavlink_msg_actuator_commands_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_l2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_l2_t packet_in = {
		17235,
	17339,
	};
	mavlink_l2_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.length = packet_in.length;
        	packet1.direction = packet_in.direction;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_l2_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_l2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_l2_pack(system_id, component_id, &msg , packet1.length , packet1.direction );
	mavlink_msg_l2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_l2_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.length , packet1.direction );
	mavlink_msg_l2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_l2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_l2_send(MAVLINK_COMM_1 , packet1.length , packet1.direction );
	mavlink_msg_l2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_sealion(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_rudder_raw(system_id, component_id, last_msg);
	mavlink_test_rudder_int(system_id, component_id, last_msg);
	mavlink_test_prop_raw(system_id, component_id, last_msg);
	mavlink_test_wind_raw(system_id, component_id, last_msg);
	mavlink_test_air_raw(system_id, component_id, last_msg);
	mavlink_test_water_raw(system_id, component_id, last_msg);
	mavlink_test_status_and_errors(system_id, component_id, last_msg);
	mavlink_test_actuator_commands(system_id, component_id, last_msg);
	mavlink_test_l2(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // SEALION_TESTSUITE_H
