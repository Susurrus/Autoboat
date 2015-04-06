/** @file
 *	@brief MAVLink comm protocol testsuite generated from seaslug.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef SEASLUG_TESTSUITE_H
#define SEASLUG_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_seaslug(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_seaslug(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_rudder_raw(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_rudder_raw_t packet_in = {
		17235,17339,17443,151,218,29
    };
	mavlink_rudder_raw_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.raw_position = packet_in.raw_position;
        	packet1.port_limit_val = packet_in.port_limit_val;
        	packet1.starboard_limit_val = packet_in.starboard_limit_val;
        	packet1.port_limit = packet_in.port_limit;
        	packet1.center_limit = packet_in.center_limit;
        	packet1.starboard_limit = packet_in.starboard_limit;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rudder_raw_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_rudder_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rudder_raw_pack(system_id, component_id, &msg , packet1.raw_position , packet1.port_limit , packet1.center_limit , packet1.starboard_limit , packet1.port_limit_val , packet1.starboard_limit_val );
	mavlink_msg_rudder_raw_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_rudder_raw_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.raw_position , packet1.port_limit , packet1.center_limit , packet1.starboard_limit , packet1.port_limit_val , packet1.starboard_limit_val );
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
	mavlink_msg_rudder_raw_send(MAVLINK_COMM_1 , packet1.raw_position , packet1.port_limit , packet1.center_limit , packet1.starboard_limit , packet1.port_limit_val , packet1.starboard_limit_val );
	mavlink_msg_rudder_raw_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_wso100(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_wso100_t packet_in = {
		17.0,45.0,73.0,101.0,129.0
    };
	mavlink_wso100_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.speed = packet_in.speed;
        	packet1.direction = packet_in.direction;
        	packet1.temperature = packet_in.temperature;
        	packet1.pressure = packet_in.pressure;
        	packet1.humidity = packet_in.humidity;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_wso100_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_wso100_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_wso100_pack(system_id, component_id, &msg , packet1.speed , packet1.direction , packet1.temperature , packet1.pressure , packet1.humidity );
	mavlink_msg_wso100_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_wso100_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.speed , packet1.direction , packet1.temperature , packet1.pressure , packet1.humidity );
	mavlink_msg_wso100_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_wso100_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_wso100_send(MAVLINK_COMM_1 , packet1.speed , packet1.direction , packet1.temperature , packet1.pressure , packet1.humidity );
	mavlink_msg_wso100_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_dst800(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_dst800_t packet_in = {
		17.0,45.0,73.0
    };
	mavlink_dst800_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.speed = packet_in.speed;
        	packet1.temperature = packet_in.temperature;
        	packet1.depth = packet_in.depth;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_dst800_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_dst800_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_dst800_pack(system_id, component_id, &msg , packet1.speed , packet1.temperature , packet1.depth );
	mavlink_msg_dst800_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_dst800_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.speed , packet1.temperature , packet1.depth );
	mavlink_msg_dst800_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_dst800_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_dst800_send(MAVLINK_COMM_1 , packet1.speed , packet1.temperature , packet1.depth );
	mavlink_msg_dst800_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_revo_gs(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_revo_gs_t packet_in = {
		17.0,45.0,73.0,101.0,18067,187,254,65
    };
	mavlink_revo_gs_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.heading = packet_in.heading;
        	packet1.pitch = packet_in.pitch;
        	packet1.roll = packet_in.roll;
        	packet1.dip = packet_in.dip;
        	packet1.mag_horiz_comp = packet_in.mag_horiz_comp;
        	packet1.mag_status = packet_in.mag_status;
        	packet1.pitch_status = packet_in.pitch_status;
        	packet1.roll_status = packet_in.roll_status;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_revo_gs_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_revo_gs_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_revo_gs_pack(system_id, component_id, &msg , packet1.heading , packet1.mag_status , packet1.pitch , packet1.pitch_status , packet1.roll , packet1.roll_status , packet1.dip , packet1.mag_horiz_comp );
	mavlink_msg_revo_gs_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_revo_gs_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.heading , packet1.mag_status , packet1.pitch , packet1.pitch_status , packet1.roll , packet1.roll_status , packet1.dip , packet1.mag_horiz_comp );
	mavlink_msg_revo_gs_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_revo_gs_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_revo_gs_send(MAVLINK_COMM_1 , packet1.heading , packet1.mag_status , packet1.pitch , packet1.pitch_status , packet1.roll , packet1.roll_status , packet1.dip , packet1.mag_horiz_comp );
	mavlink_msg_revo_gs_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_gps200(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_gps200_t packet_in = {
		17.0
    };
	mavlink_gps200_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.magnetic_variation = packet_in.magnetic_variation;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps200_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_gps200_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps200_pack(system_id, component_id, &msg , packet1.magnetic_variation );
	mavlink_msg_gps200_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps200_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.magnetic_variation );
	mavlink_msg_gps200_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_gps200_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gps200_send(MAVLINK_COMM_1 , packet1.magnetic_variation );
	mavlink_msg_gps200_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_dsp3000(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_dsp3000_t packet_in = {
		17.0
    };
	mavlink_dsp3000_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.z_rate = packet_in.z_rate;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_dsp3000_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_dsp3000_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_dsp3000_pack(system_id, component_id, &msg , packet1.z_rate );
	mavlink_msg_dsp3000_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_dsp3000_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.z_rate );
	mavlink_msg_dsp3000_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_dsp3000_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_dsp3000_send(MAVLINK_COMM_1 , packet1.z_rate );
	mavlink_msg_dsp3000_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_tokimec(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_tokimec_t packet_in = {
		963497464,963497672,963497880,963498088,18067,18171,18275,18379,18483,18587,18691,18795,18899,19003,19107,19211,19315
    };
	mavlink_tokimec_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.latitude = packet_in.latitude;
        	packet1.longitude = packet_in.longitude;
        	packet1.est_latitude = packet_in.est_latitude;
        	packet1.est_longitude = packet_in.est_longitude;
        	packet1.yaw = packet_in.yaw;
        	packet1.pitch = packet_in.pitch;
        	packet1.roll = packet_in.roll;
        	packet1.x_angle_vel = packet_in.x_angle_vel;
        	packet1.y_angle_vel = packet_in.y_angle_vel;
        	packet1.z_angle_vel = packet_in.z_angle_vel;
        	packet1.x_accel = packet_in.x_accel;
        	packet1.y_accel = packet_in.y_accel;
        	packet1.z_accel = packet_in.z_accel;
        	packet1.mag_bearing = packet_in.mag_bearing;
        	packet1.gps_heading = packet_in.gps_heading;
        	packet1.gps_speed = packet_in.gps_speed;
        	packet1.status = packet_in.status;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_tokimec_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_tokimec_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_tokimec_pack(system_id, component_id, &msg , packet1.yaw , packet1.pitch , packet1.roll , packet1.x_angle_vel , packet1.y_angle_vel , packet1.z_angle_vel , packet1.x_accel , packet1.y_accel , packet1.z_accel , packet1.mag_bearing , packet1.latitude , packet1.longitude , packet1.est_latitude , packet1.est_longitude , packet1.gps_heading , packet1.gps_speed , packet1.status );
	mavlink_msg_tokimec_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_tokimec_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.yaw , packet1.pitch , packet1.roll , packet1.x_angle_vel , packet1.y_angle_vel , packet1.z_angle_vel , packet1.x_accel , packet1.y_accel , packet1.z_accel , packet1.mag_bearing , packet1.latitude , packet1.longitude , packet1.est_latitude , packet1.est_longitude , packet1.gps_heading , packet1.gps_speed , packet1.status );
	mavlink_msg_tokimec_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_tokimec_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_tokimec_send(MAVLINK_COMM_1 , packet1.yaw , packet1.pitch , packet1.roll , packet1.x_angle_vel , packet1.y_angle_vel , packet1.z_angle_vel , packet1.x_accel , packet1.y_accel , packet1.z_accel , packet1.mag_bearing , packet1.latitude , packet1.longitude , packet1.est_latitude , packet1.est_longitude , packet1.gps_heading , packet1.gps_speed , packet1.status );
	mavlink_msg_tokimec_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_radio(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_radio_t packet_in = {
		17235,17339,17,84,151,218,29
    };
	mavlink_radio_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.rxerrors = packet_in.rxerrors;
        	packet1.fixed = packet_in.fixed;
        	packet1.rssi = packet_in.rssi;
        	packet1.remrssi = packet_in.remrssi;
        	packet1.txbuf = packet_in.txbuf;
        	packet1.noise = packet_in.noise;
        	packet1.remnoise = packet_in.remnoise;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radio_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_radio_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radio_pack(system_id, component_id, &msg , packet1.rssi , packet1.remrssi , packet1.txbuf , packet1.noise , packet1.remnoise , packet1.rxerrors , packet1.fixed );
	mavlink_msg_radio_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radio_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.rssi , packet1.remrssi , packet1.txbuf , packet1.noise , packet1.remnoise , packet1.rxerrors , packet1.fixed );
	mavlink_msg_radio_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_radio_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_radio_send(MAVLINK_COMM_1 , packet1.rssi , packet1.remrssi , packet1.txbuf , packet1.noise , packet1.remnoise , packet1.rxerrors , packet1.fixed );
	mavlink_msg_radio_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_basic_state(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_basic_state_t packet_in = {
		17.0,45.0,73.0,101.0,129.0,157.0,185.0,18691,18795,18899,19003
    };
	mavlink_basic_state_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.commanded_auto_rudder_angle = packet_in.commanded_auto_rudder_angle;
        	packet1.commanded_primary_rudder_angle = packet_in.commanded_primary_rudder_angle;
        	packet1.commanded_secondary_rudder_angle = packet_in.commanded_secondary_rudder_angle;
        	packet1.rudder_angle = packet_in.rudder_angle;
        	packet1.a_cmd = packet_in.a_cmd;
        	packet1.L2_north = packet_in.L2_north;
        	packet1.L2_east = packet_in.L2_east;
        	packet1.commanded_auto_throttle = packet_in.commanded_auto_throttle;
        	packet1.commanded_primary_throttle = packet_in.commanded_primary_throttle;
        	packet1.commanded_secondary_throttle = packet_in.commanded_secondary_throttle;
        	packet1.prop_speed = packet_in.prop_speed;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_basic_state_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_basic_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_basic_state_pack(system_id, component_id, &msg , packet1.commanded_auto_rudder_angle , packet1.commanded_primary_rudder_angle , packet1.commanded_secondary_rudder_angle , packet1.rudder_angle , packet1.commanded_auto_throttle , packet1.commanded_primary_throttle , packet1.commanded_secondary_throttle , packet1.prop_speed , packet1.a_cmd , packet1.L2_north , packet1.L2_east );
	mavlink_msg_basic_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_basic_state_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.commanded_auto_rudder_angle , packet1.commanded_primary_rudder_angle , packet1.commanded_secondary_rudder_angle , packet1.rudder_angle , packet1.commanded_auto_throttle , packet1.commanded_primary_throttle , packet1.commanded_secondary_throttle , packet1.prop_speed , packet1.a_cmd , packet1.L2_north , packet1.L2_east );
	mavlink_msg_basic_state_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_basic_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_basic_state_send(MAVLINK_COMM_1 , packet1.commanded_auto_rudder_angle , packet1.commanded_primary_rudder_angle , packet1.commanded_secondary_rudder_angle , packet1.rudder_angle , packet1.commanded_auto_throttle , packet1.commanded_primary_throttle , packet1.commanded_secondary_throttle , packet1.prop_speed , packet1.a_cmd , packet1.L2_north , packet1.L2_east );
	mavlink_msg_basic_state_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_main_power(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_main_power_t packet_in = {
		17235,17339,17443,17547,17651,17755
    };
	mavlink_main_power_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.electronics_voltage = packet_in.electronics_voltage;
        	packet1.electronics_current = packet_in.electronics_current;
        	packet1.actuator_voltage = packet_in.actuator_voltage;
        	packet1.actuator_current = packet_in.actuator_current;
        	packet1.solar_voltage = packet_in.solar_voltage;
        	packet1.solar_current = packet_in.solar_current;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_main_power_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_main_power_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_main_power_pack(system_id, component_id, &msg , packet1.electronics_voltage , packet1.electronics_current , packet1.actuator_voltage , packet1.actuator_current , packet1.solar_voltage , packet1.solar_current );
	mavlink_msg_main_power_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_main_power_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.electronics_voltage , packet1.electronics_current , packet1.actuator_voltage , packet1.actuator_current , packet1.solar_voltage , packet1.solar_current );
	mavlink_msg_main_power_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_main_power_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_main_power_send(MAVLINK_COMM_1 , packet1.electronics_voltage , packet1.electronics_current , packet1.actuator_voltage , packet1.actuator_current , packet1.solar_voltage , packet1.solar_current );
	mavlink_msg_main_power_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_node_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_node_status_t packet_in = {
		17235,17339,17443,17547,17651,17755,17859,17963,18067,18171,18275,18379,77,144,211,22,89,156,223,34,101,168,235,46,113,180,247,58,125,192
    };
	mavlink_node_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.hil_status = packet_in.hil_status;
        	packet1.hil_errors = packet_in.hil_errors;
        	packet1.imu_status = packet_in.imu_status;
        	packet1.imu_errors = packet_in.imu_errors;
        	packet1.power_status = packet_in.power_status;
        	packet1.power_errors = packet_in.power_errors;
        	packet1.primary_status = packet_in.primary_status;
        	packet1.primary_errors = packet_in.primary_errors;
        	packet1.rc_status = packet_in.rc_status;
        	packet1.rc_errors = packet_in.rc_errors;
        	packet1.rudder_status = packet_in.rudder_status;
        	packet1.rudder_errors = packet_in.rudder_errors;
        	packet1.hil_temp = packet_in.hil_temp;
        	packet1.hil_load = packet_in.hil_load;
        	packet1.hil_voltage = packet_in.hil_voltage;
        	packet1.imu_temp = packet_in.imu_temp;
        	packet1.imu_load = packet_in.imu_load;
        	packet1.imu_voltage = packet_in.imu_voltage;
        	packet1.power_temp = packet_in.power_temp;
        	packet1.power_load = packet_in.power_load;
        	packet1.power_voltage = packet_in.power_voltage;
        	packet1.primary_temp = packet_in.primary_temp;
        	packet1.primary_load = packet_in.primary_load;
        	packet1.primary_voltage = packet_in.primary_voltage;
        	packet1.rc_temp = packet_in.rc_temp;
        	packet1.rc_load = packet_in.rc_load;
        	packet1.rc_voltage = packet_in.rc_voltage;
        	packet1.rudder_temp = packet_in.rudder_temp;
        	packet1.rudder_load = packet_in.rudder_load;
        	packet1.rudder_voltage = packet_in.rudder_voltage;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_node_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_node_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_node_status_pack(system_id, component_id, &msg , packet1.hil_status , packet1.hil_errors , packet1.hil_temp , packet1.hil_load , packet1.hil_voltage , packet1.imu_status , packet1.imu_errors , packet1.imu_temp , packet1.imu_load , packet1.imu_voltage , packet1.power_status , packet1.power_errors , packet1.power_temp , packet1.power_load , packet1.power_voltage , packet1.primary_status , packet1.primary_errors , packet1.primary_temp , packet1.primary_load , packet1.primary_voltage , packet1.rc_status , packet1.rc_errors , packet1.rc_temp , packet1.rc_load , packet1.rc_voltage , packet1.rudder_status , packet1.rudder_errors , packet1.rudder_temp , packet1.rudder_load , packet1.rudder_voltage );
	mavlink_msg_node_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_node_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.hil_status , packet1.hil_errors , packet1.hil_temp , packet1.hil_load , packet1.hil_voltage , packet1.imu_status , packet1.imu_errors , packet1.imu_temp , packet1.imu_load , packet1.imu_voltage , packet1.power_status , packet1.power_errors , packet1.power_temp , packet1.power_load , packet1.power_voltage , packet1.primary_status , packet1.primary_errors , packet1.primary_temp , packet1.primary_load , packet1.primary_voltage , packet1.rc_status , packet1.rc_errors , packet1.rc_temp , packet1.rc_load , packet1.rc_voltage , packet1.rudder_status , packet1.rudder_errors , packet1.rudder_temp , packet1.rudder_load , packet1.rudder_voltage );
	mavlink_msg_node_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_node_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_node_status_send(MAVLINK_COMM_1 , packet1.hil_status , packet1.hil_errors , packet1.hil_temp , packet1.hil_load , packet1.hil_voltage , packet1.imu_status , packet1.imu_errors , packet1.imu_temp , packet1.imu_load , packet1.imu_voltage , packet1.power_status , packet1.power_errors , packet1.power_temp , packet1.power_load , packet1.power_voltage , packet1.primary_status , packet1.primary_errors , packet1.primary_temp , packet1.primary_load , packet1.primary_voltage , packet1.rc_status , packet1.rc_errors , packet1.rc_temp , packet1.rc_load , packet1.rc_voltage , packet1.rudder_status , packet1.rudder_errors , packet1.rudder_temp , packet1.rudder_load , packet1.rudder_voltage );
	mavlink_msg_node_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_waypoint_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_waypoint_status_t packet_in = {
		17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0
    };
	mavlink_waypoint_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.last_wp_lat = packet_in.last_wp_lat;
        	packet1.last_wp_lon = packet_in.last_wp_lon;
        	packet1.last_wp_north = packet_in.last_wp_north;
        	packet1.last_wp_east = packet_in.last_wp_east;
        	packet1.next_wp_lat = packet_in.next_wp_lat;
        	packet1.next_wp_lon = packet_in.next_wp_lon;
        	packet1.next_wp_north = packet_in.next_wp_north;
        	packet1.next_wp_east = packet_in.next_wp_east;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_waypoint_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_waypoint_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_waypoint_status_pack(system_id, component_id, &msg , packet1.last_wp_lat , packet1.last_wp_lon , packet1.last_wp_north , packet1.last_wp_east , packet1.next_wp_lat , packet1.next_wp_lon , packet1.next_wp_north , packet1.next_wp_east );
	mavlink_msg_waypoint_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_waypoint_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.last_wp_lat , packet1.last_wp_lon , packet1.last_wp_north , packet1.last_wp_east , packet1.next_wp_lat , packet1.next_wp_lon , packet1.next_wp_north , packet1.next_wp_east );
	mavlink_msg_waypoint_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_waypoint_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_waypoint_status_send(MAVLINK_COMM_1 , packet1.last_wp_lat , packet1.last_wp_lon , packet1.last_wp_north , packet1.last_wp_east , packet1.next_wp_lat , packet1.next_wp_lon , packet1.next_wp_north , packet1.next_wp_east );
	mavlink_msg_waypoint_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_basic_state2(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_basic_state2_t packet_in = {
		17.0,45.0,73.0,101.0,129.0,157.0,185.0,213.0,18899,19003,19107,19211,19315
    };
	mavlink_basic_state2_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.commanded_auto_rudder_angle = packet_in.commanded_auto_rudder_angle;
        	packet1.commanded_primary_rudder_angle = packet_in.commanded_primary_rudder_angle;
        	packet1.commanded_secondary_rudder_angle = packet_in.commanded_secondary_rudder_angle;
        	packet1.commanded_rudder_angle = packet_in.commanded_rudder_angle;
        	packet1.rudder_angle = packet_in.rudder_angle;
        	packet1.a_cmd = packet_in.a_cmd;
        	packet1.L2_north = packet_in.L2_north;
        	packet1.L2_east = packet_in.L2_east;
        	packet1.commanded_auto_throttle = packet_in.commanded_auto_throttle;
        	packet1.commanded_primary_throttle = packet_in.commanded_primary_throttle;
        	packet1.commanded_secondary_throttle = packet_in.commanded_secondary_throttle;
        	packet1.commanded_throttle = packet_in.commanded_throttle;
        	packet1.prop_speed = packet_in.prop_speed;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_basic_state2_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_basic_state2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_basic_state2_pack(system_id, component_id, &msg , packet1.commanded_auto_rudder_angle , packet1.commanded_primary_rudder_angle , packet1.commanded_secondary_rudder_angle , packet1.commanded_rudder_angle , packet1.rudder_angle , packet1.commanded_auto_throttle , packet1.commanded_primary_throttle , packet1.commanded_secondary_throttle , packet1.commanded_throttle , packet1.prop_speed , packet1.a_cmd , packet1.L2_north , packet1.L2_east );
	mavlink_msg_basic_state2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_basic_state2_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.commanded_auto_rudder_angle , packet1.commanded_primary_rudder_angle , packet1.commanded_secondary_rudder_angle , packet1.commanded_rudder_angle , packet1.rudder_angle , packet1.commanded_auto_throttle , packet1.commanded_primary_throttle , packet1.commanded_secondary_throttle , packet1.commanded_throttle , packet1.prop_speed , packet1.a_cmd , packet1.L2_north , packet1.L2_east );
	mavlink_msg_basic_state2_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_basic_state2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_basic_state2_send(MAVLINK_COMM_1 , packet1.commanded_auto_rudder_angle , packet1.commanded_primary_rudder_angle , packet1.commanded_secondary_rudder_angle , packet1.commanded_rudder_angle , packet1.rudder_angle , packet1.commanded_auto_throttle , packet1.commanded_primary_throttle , packet1.commanded_secondary_throttle , packet1.commanded_throttle , packet1.prop_speed , packet1.a_cmd , packet1.L2_north , packet1.L2_east );
	mavlink_msg_basic_state2_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_controller_data(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_controller_data_t packet_in = {
		963497464,963497672,963497880,963498088,963498296,18275,18379,18483,18587,18691,18795,18899,19003,19107,19211,19315,19419,19523,19627,19731,19835,19939,20043,20147,20251,20355,20459,20563,20667,209,20
    };
	mavlink_controller_data_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.lat = packet_in.lat;
        	packet1.lon = packet_in.lon;
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.north = packet_in.north;
        	packet1.east = packet_in.east;
        	packet1.last_wp_north = packet_in.last_wp_north;
        	packet1.last_wp_east = packet_in.last_wp_east;
        	packet1.next_wp_north = packet_in.next_wp_north;
        	packet1.next_wp_east = packet_in.next_wp_east;
        	packet1.yaw = packet_in.yaw;
        	packet1.pitch = packet_in.pitch;
        	packet1.roll = packet_in.roll;
        	packet1.x_angle_vel = packet_in.x_angle_vel;
        	packet1.y_angle_vel = packet_in.y_angle_vel;
        	packet1.z_angle_vel = packet_in.z_angle_vel;
        	packet1.water_speed = packet_in.water_speed;
        	packet1.sog = packet_in.sog;
        	packet1.cog = packet_in.cog;
        	packet1.hdop = packet_in.hdop;
        	packet1.north_speed = packet_in.north_speed;
        	packet1.east_speed = packet_in.east_speed;
        	packet1.a_cmd = packet_in.a_cmd;
        	packet1.aim_point_n = packet_in.aim_point_n;
        	packet1.aim_point_e = packet_in.aim_point_e;
        	packet1.yaw_rate = packet_in.yaw_rate;
        	packet1.commanded_rudder_angle = packet_in.commanded_rudder_angle;
        	packet1.commanded_throttle = packet_in.commanded_throttle;
        	packet1.rudder_angle = packet_in.rudder_angle;
        	packet1.prop_speed = packet_in.prop_speed;
        	packet1.new_gps_fix = packet_in.new_gps_fix;
        	packet1.reset = packet_in.reset;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_controller_data_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_controller_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_controller_data_pack(system_id, component_id, &msg , packet1.last_wp_north , packet1.last_wp_east , packet1.next_wp_north , packet1.next_wp_east , packet1.yaw , packet1.pitch , packet1.roll , packet1.x_angle_vel , packet1.y_angle_vel , packet1.z_angle_vel , packet1.water_speed , packet1.new_gps_fix , packet1.lat , packet1.lon , packet1.sog , packet1.cog , packet1.hdop , packet1.reset , packet1.time_boot_ms , packet1.north , packet1.east , packet1.north_speed , packet1.east_speed , packet1.a_cmd , packet1.aim_point_n , packet1.aim_point_e , packet1.yaw_rate , packet1.commanded_rudder_angle , packet1.commanded_throttle , packet1.rudder_angle , packet1.prop_speed );
	mavlink_msg_controller_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_controller_data_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.last_wp_north , packet1.last_wp_east , packet1.next_wp_north , packet1.next_wp_east , packet1.yaw , packet1.pitch , packet1.roll , packet1.x_angle_vel , packet1.y_angle_vel , packet1.z_angle_vel , packet1.water_speed , packet1.new_gps_fix , packet1.lat , packet1.lon , packet1.sog , packet1.cog , packet1.hdop , packet1.reset , packet1.time_boot_ms , packet1.north , packet1.east , packet1.north_speed , packet1.east_speed , packet1.a_cmd , packet1.aim_point_n , packet1.aim_point_e , packet1.yaw_rate , packet1.commanded_rudder_angle , packet1.commanded_throttle , packet1.rudder_angle , packet1.prop_speed );
	mavlink_msg_controller_data_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_controller_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_controller_data_send(MAVLINK_COMM_1 , packet1.last_wp_north , packet1.last_wp_east , packet1.next_wp_north , packet1.next_wp_east , packet1.yaw , packet1.pitch , packet1.roll , packet1.x_angle_vel , packet1.y_angle_vel , packet1.z_angle_vel , packet1.water_speed , packet1.new_gps_fix , packet1.lat , packet1.lon , packet1.sog , packet1.cog , packet1.hdop , packet1.reset , packet1.time_boot_ms , packet1.north , packet1.east , packet1.north_speed , packet1.east_speed , packet1.a_cmd , packet1.aim_point_n , packet1.aim_point_e , packet1.yaw_rate , packet1.commanded_rudder_angle , packet1.commanded_throttle , packet1.rudder_angle , packet1.prop_speed );
	mavlink_msg_controller_data_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_tokimec_with_time(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_tokimec_with_time_t packet_in = {
		963497464,963497672,963497880,963498088,963498296,18275,18379,18483,18587,18691,18795,18899,19003,19107,19211,19315,19419,19523
    };
	mavlink_tokimec_with_time_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.latitude = packet_in.latitude;
        	packet1.longitude = packet_in.longitude;
        	packet1.est_latitude = packet_in.est_latitude;
        	packet1.est_longitude = packet_in.est_longitude;
        	packet1.yaw = packet_in.yaw;
        	packet1.pitch = packet_in.pitch;
        	packet1.roll = packet_in.roll;
        	packet1.x_angle_vel = packet_in.x_angle_vel;
        	packet1.y_angle_vel = packet_in.y_angle_vel;
        	packet1.z_angle_vel = packet_in.z_angle_vel;
        	packet1.x_accel = packet_in.x_accel;
        	packet1.y_accel = packet_in.y_accel;
        	packet1.z_accel = packet_in.z_accel;
        	packet1.mag_bearing = packet_in.mag_bearing;
        	packet1.gps_heading = packet_in.gps_heading;
        	packet1.gps_speed = packet_in.gps_speed;
        	packet1.status = packet_in.status;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_tokimec_with_time_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_tokimec_with_time_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_tokimec_with_time_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.yaw , packet1.pitch , packet1.roll , packet1.x_angle_vel , packet1.y_angle_vel , packet1.z_angle_vel , packet1.x_accel , packet1.y_accel , packet1.z_accel , packet1.mag_bearing , packet1.latitude , packet1.longitude , packet1.est_latitude , packet1.est_longitude , packet1.gps_heading , packet1.gps_speed , packet1.status );
	mavlink_msg_tokimec_with_time_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_tokimec_with_time_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.yaw , packet1.pitch , packet1.roll , packet1.x_angle_vel , packet1.y_angle_vel , packet1.z_angle_vel , packet1.x_accel , packet1.y_accel , packet1.z_accel , packet1.mag_bearing , packet1.latitude , packet1.longitude , packet1.est_latitude , packet1.est_longitude , packet1.gps_heading , packet1.gps_speed , packet1.status );
	mavlink_msg_tokimec_with_time_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_tokimec_with_time_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_tokimec_with_time_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.yaw , packet1.pitch , packet1.roll , packet1.x_angle_vel , packet1.y_angle_vel , packet1.z_angle_vel , packet1.x_accel , packet1.y_accel , packet1.z_accel , packet1.mag_bearing , packet1.latitude , packet1.longitude , packet1.est_latitude , packet1.est_longitude , packet1.gps_heading , packet1.gps_speed , packet1.status );
	mavlink_msg_tokimec_with_time_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_param_value_with_time(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_param_value_with_time_t packet_in = {
		963497464,45.0,17651,17755,"MNOPQRSTUVWXYZA",89
    };
	mavlink_param_value_with_time_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.time_boot_ms = packet_in.time_boot_ms;
        	packet1.param_value = packet_in.param_value;
        	packet1.param_count = packet_in.param_count;
        	packet1.param_index = packet_in.param_index;
        	packet1.param_type = packet_in.param_type;
        
        	mav_array_memcpy(packet1.param_id, packet_in.param_id, sizeof(char)*16);
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_param_value_with_time_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_param_value_with_time_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_param_value_with_time_pack(system_id, component_id, &msg , packet1.time_boot_ms , packet1.param_id , packet1.param_value , packet1.param_type , packet1.param_count , packet1.param_index );
	mavlink_msg_param_value_with_time_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_param_value_with_time_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.time_boot_ms , packet1.param_id , packet1.param_value , packet1.param_type , packet1.param_count , packet1.param_index );
	mavlink_msg_param_value_with_time_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_param_value_with_time_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_param_value_with_time_send(MAVLINK_COMM_1 , packet1.time_boot_ms , packet1.param_id , packet1.param_value , packet1.param_type , packet1.param_count , packet1.param_index );
	mavlink_msg_param_value_with_time_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_seaslug(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_rudder_raw(system_id, component_id, last_msg);
	mavlink_test_wso100(system_id, component_id, last_msg);
	mavlink_test_dst800(system_id, component_id, last_msg);
	mavlink_test_revo_gs(system_id, component_id, last_msg);
	mavlink_test_gps200(system_id, component_id, last_msg);
	mavlink_test_dsp3000(system_id, component_id, last_msg);
	mavlink_test_tokimec(system_id, component_id, last_msg);
	mavlink_test_radio(system_id, component_id, last_msg);
	mavlink_test_basic_state(system_id, component_id, last_msg);
	mavlink_test_main_power(system_id, component_id, last_msg);
	mavlink_test_node_status(system_id, component_id, last_msg);
	mavlink_test_waypoint_status(system_id, component_id, last_msg);
	mavlink_test_basic_state2(system_id, component_id, last_msg);
	mavlink_test_controller_data(system_id, component_id, last_msg);
	mavlink_test_tokimec_with_time(system_id, component_id, last_msg);
	mavlink_test_param_value_with_time(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // SEASLUG_TESTSUITE_H
