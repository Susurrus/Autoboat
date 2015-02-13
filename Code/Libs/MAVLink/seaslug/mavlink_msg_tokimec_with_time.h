// MESSAGE TOKIMEC_WITH_TIME PACKING

#define MAVLINK_MSG_ID_TOKIMEC_WITH_TIME 181

typedef struct __mavlink_tokimec_with_time_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 int32_t latitude; ///< Raw GPS latitude. Units are 2e-29 rads.
 int32_t longitude; ///< Raw GPS longitude. Units are 2e-29 rads.
 int32_t est_latitude; ///< Estimated GPS latitude. Units are 2e-29 rads.
 int32_t est_longitude; ///< Estimated GPS longitude. Units are 2e-29 rads.
 int16_t yaw; ///< Heading in degrees eastward from north
 int16_t pitch; ///< Magnetometer status
 int16_t roll; ///< Pitch angle in degrees.
 int16_t x_angle_vel; ///< Angular velocity around the X-axis. Units are 2e-12 rads/s.
 int16_t y_angle_vel; ///< Angular velocity around the Y-axis. Units are 2e-12 rads/s.
 int16_t z_angle_vel; ///< Angular velocity around the Z-axis. Units are 2e-12 rads/s.
 int16_t x_accel; ///< Acceleration along the X-axis. Units are 2e-8 m/s^2.
 int16_t y_accel; ///< Acceleration along the Y-axis. Units are 2e-8 m/s^2.
 int16_t z_accel; ///< Acceleration along the Z-axis. Units are 2e-8 m/s^2.
 int16_t mag_bearing; ///< Magnetic bearing. Units are 2^-13 rads.
 int16_t gps_heading; ///< Raw GPS heading. Units are 2e-13 rads.
 int16_t gps_speed; ///< Raw GPS speed. Units are 2e-6 m/s.
 int16_t status; ///< Status bitfield. See Tokimec documentation.
} mavlink_tokimec_with_time_t;

#define MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN 46
#define MAVLINK_MSG_ID_181_LEN 46

#define MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_CRC 82
#define MAVLINK_MSG_ID_181_CRC 82



#define MAVLINK_MESSAGE_INFO_TOKIMEC_WITH_TIME { \
	"TOKIMEC_WITH_TIME", \
	18, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_tokimec_with_time_t, time_boot_ms) }, \
         { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_tokimec_with_time_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_tokimec_with_time_t, longitude) }, \
         { "est_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_tokimec_with_time_t, est_latitude) }, \
         { "est_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 16, offsetof(mavlink_tokimec_with_time_t, est_longitude) }, \
         { "yaw", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_tokimec_with_time_t, yaw) }, \
         { "pitch", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_tokimec_with_time_t, pitch) }, \
         { "roll", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_tokimec_with_time_t, roll) }, \
         { "x_angle_vel", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_tokimec_with_time_t, x_angle_vel) }, \
         { "y_angle_vel", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_tokimec_with_time_t, y_angle_vel) }, \
         { "z_angle_vel", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_tokimec_with_time_t, z_angle_vel) }, \
         { "x_accel", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_tokimec_with_time_t, x_accel) }, \
         { "y_accel", NULL, MAVLINK_TYPE_INT16_T, 0, 34, offsetof(mavlink_tokimec_with_time_t, y_accel) }, \
         { "z_accel", NULL, MAVLINK_TYPE_INT16_T, 0, 36, offsetof(mavlink_tokimec_with_time_t, z_accel) }, \
         { "mag_bearing", NULL, MAVLINK_TYPE_INT16_T, 0, 38, offsetof(mavlink_tokimec_with_time_t, mag_bearing) }, \
         { "gps_heading", NULL, MAVLINK_TYPE_INT16_T, 0, 40, offsetof(mavlink_tokimec_with_time_t, gps_heading) }, \
         { "gps_speed", NULL, MAVLINK_TYPE_INT16_T, 0, 42, offsetof(mavlink_tokimec_with_time_t, gps_speed) }, \
         { "status", NULL, MAVLINK_TYPE_INT16_T, 0, 44, offsetof(mavlink_tokimec_with_time_t, status) }, \
         } \
}


/**
 * @brief Pack a tokimec_with_time message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param yaw Heading in degrees eastward from north
 * @param pitch Magnetometer status
 * @param roll Pitch angle in degrees.
 * @param x_angle_vel Angular velocity around the X-axis. Units are 2e-12 rads/s.
 * @param y_angle_vel Angular velocity around the Y-axis. Units are 2e-12 rads/s.
 * @param z_angle_vel Angular velocity around the Z-axis. Units are 2e-12 rads/s.
 * @param x_accel Acceleration along the X-axis. Units are 2e-8 m/s^2.
 * @param y_accel Acceleration along the Y-axis. Units are 2e-8 m/s^2.
 * @param z_accel Acceleration along the Z-axis. Units are 2e-8 m/s^2.
 * @param mag_bearing Magnetic bearing. Units are 2^-13 rads.
 * @param latitude Raw GPS latitude. Units are 2e-29 rads.
 * @param longitude Raw GPS longitude. Units are 2e-29 rads.
 * @param est_latitude Estimated GPS latitude. Units are 2e-29 rads.
 * @param est_longitude Estimated GPS longitude. Units are 2e-29 rads.
 * @param gps_heading Raw GPS heading. Units are 2e-13 rads.
 * @param gps_speed Raw GPS speed. Units are 2e-6 m/s.
 * @param status Status bitfield. See Tokimec documentation.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tokimec_with_time_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, int16_t yaw, int16_t pitch, int16_t roll, int16_t x_angle_vel, int16_t y_angle_vel, int16_t z_angle_vel, int16_t x_accel, int16_t y_accel, int16_t z_accel, int16_t mag_bearing, int32_t latitude, int32_t longitude, int32_t est_latitude, int32_t est_longitude, int16_t gps_heading, int16_t gps_speed, int16_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, latitude);
	_mav_put_int32_t(buf, 8, longitude);
	_mav_put_int32_t(buf, 12, est_latitude);
	_mav_put_int32_t(buf, 16, est_longitude);
	_mav_put_int16_t(buf, 20, yaw);
	_mav_put_int16_t(buf, 22, pitch);
	_mav_put_int16_t(buf, 24, roll);
	_mav_put_int16_t(buf, 26, x_angle_vel);
	_mav_put_int16_t(buf, 28, y_angle_vel);
	_mav_put_int16_t(buf, 30, z_angle_vel);
	_mav_put_int16_t(buf, 32, x_accel);
	_mav_put_int16_t(buf, 34, y_accel);
	_mav_put_int16_t(buf, 36, z_accel);
	_mav_put_int16_t(buf, 38, mag_bearing);
	_mav_put_int16_t(buf, 40, gps_heading);
	_mav_put_int16_t(buf, 42, gps_speed);
	_mav_put_int16_t(buf, 44, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN);
#else
	mavlink_tokimec_with_time_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.est_latitude = est_latitude;
	packet.est_longitude = est_longitude;
	packet.yaw = yaw;
	packet.pitch = pitch;
	packet.roll = roll;
	packet.x_angle_vel = x_angle_vel;
	packet.y_angle_vel = y_angle_vel;
	packet.z_angle_vel = z_angle_vel;
	packet.x_accel = x_accel;
	packet.y_accel = y_accel;
	packet.z_accel = z_accel;
	packet.mag_bearing = mag_bearing;
	packet.gps_heading = gps_heading;
	packet.gps_speed = gps_speed;
	packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TOKIMEC_WITH_TIME;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN);
#endif
}

/**
 * @brief Pack a tokimec_with_time message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param yaw Heading in degrees eastward from north
 * @param pitch Magnetometer status
 * @param roll Pitch angle in degrees.
 * @param x_angle_vel Angular velocity around the X-axis. Units are 2e-12 rads/s.
 * @param y_angle_vel Angular velocity around the Y-axis. Units are 2e-12 rads/s.
 * @param z_angle_vel Angular velocity around the Z-axis. Units are 2e-12 rads/s.
 * @param x_accel Acceleration along the X-axis. Units are 2e-8 m/s^2.
 * @param y_accel Acceleration along the Y-axis. Units are 2e-8 m/s^2.
 * @param z_accel Acceleration along the Z-axis. Units are 2e-8 m/s^2.
 * @param mag_bearing Magnetic bearing. Units are 2^-13 rads.
 * @param latitude Raw GPS latitude. Units are 2e-29 rads.
 * @param longitude Raw GPS longitude. Units are 2e-29 rads.
 * @param est_latitude Estimated GPS latitude. Units are 2e-29 rads.
 * @param est_longitude Estimated GPS longitude. Units are 2e-29 rads.
 * @param gps_heading Raw GPS heading. Units are 2e-13 rads.
 * @param gps_speed Raw GPS speed. Units are 2e-6 m/s.
 * @param status Status bitfield. See Tokimec documentation.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tokimec_with_time_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,int16_t yaw,int16_t pitch,int16_t roll,int16_t x_angle_vel,int16_t y_angle_vel,int16_t z_angle_vel,int16_t x_accel,int16_t y_accel,int16_t z_accel,int16_t mag_bearing,int32_t latitude,int32_t longitude,int32_t est_latitude,int32_t est_longitude,int16_t gps_heading,int16_t gps_speed,int16_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, latitude);
	_mav_put_int32_t(buf, 8, longitude);
	_mav_put_int32_t(buf, 12, est_latitude);
	_mav_put_int32_t(buf, 16, est_longitude);
	_mav_put_int16_t(buf, 20, yaw);
	_mav_put_int16_t(buf, 22, pitch);
	_mav_put_int16_t(buf, 24, roll);
	_mav_put_int16_t(buf, 26, x_angle_vel);
	_mav_put_int16_t(buf, 28, y_angle_vel);
	_mav_put_int16_t(buf, 30, z_angle_vel);
	_mav_put_int16_t(buf, 32, x_accel);
	_mav_put_int16_t(buf, 34, y_accel);
	_mav_put_int16_t(buf, 36, z_accel);
	_mav_put_int16_t(buf, 38, mag_bearing);
	_mav_put_int16_t(buf, 40, gps_heading);
	_mav_put_int16_t(buf, 42, gps_speed);
	_mav_put_int16_t(buf, 44, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN);
#else
	mavlink_tokimec_with_time_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.est_latitude = est_latitude;
	packet.est_longitude = est_longitude;
	packet.yaw = yaw;
	packet.pitch = pitch;
	packet.roll = roll;
	packet.x_angle_vel = x_angle_vel;
	packet.y_angle_vel = y_angle_vel;
	packet.z_angle_vel = z_angle_vel;
	packet.x_accel = x_accel;
	packet.y_accel = y_accel;
	packet.z_accel = z_accel;
	packet.mag_bearing = mag_bearing;
	packet.gps_heading = gps_heading;
	packet.gps_speed = gps_speed;
	packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TOKIMEC_WITH_TIME;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN);
#endif
}

/**
 * @brief Encode a tokimec_with_time struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tokimec_with_time C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tokimec_with_time_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tokimec_with_time_t* tokimec_with_time)
{
	return mavlink_msg_tokimec_with_time_pack(system_id, component_id, msg, tokimec_with_time->time_boot_ms, tokimec_with_time->yaw, tokimec_with_time->pitch, tokimec_with_time->roll, tokimec_with_time->x_angle_vel, tokimec_with_time->y_angle_vel, tokimec_with_time->z_angle_vel, tokimec_with_time->x_accel, tokimec_with_time->y_accel, tokimec_with_time->z_accel, tokimec_with_time->mag_bearing, tokimec_with_time->latitude, tokimec_with_time->longitude, tokimec_with_time->est_latitude, tokimec_with_time->est_longitude, tokimec_with_time->gps_heading, tokimec_with_time->gps_speed, tokimec_with_time->status);
}

/**
 * @brief Encode a tokimec_with_time struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tokimec_with_time C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tokimec_with_time_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tokimec_with_time_t* tokimec_with_time)
{
	return mavlink_msg_tokimec_with_time_pack_chan(system_id, component_id, chan, msg, tokimec_with_time->time_boot_ms, tokimec_with_time->yaw, tokimec_with_time->pitch, tokimec_with_time->roll, tokimec_with_time->x_angle_vel, tokimec_with_time->y_angle_vel, tokimec_with_time->z_angle_vel, tokimec_with_time->x_accel, tokimec_with_time->y_accel, tokimec_with_time->z_accel, tokimec_with_time->mag_bearing, tokimec_with_time->latitude, tokimec_with_time->longitude, tokimec_with_time->est_latitude, tokimec_with_time->est_longitude, tokimec_with_time->gps_heading, tokimec_with_time->gps_speed, tokimec_with_time->status);
}

/**
 * @brief Send a tokimec_with_time message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param yaw Heading in degrees eastward from north
 * @param pitch Magnetometer status
 * @param roll Pitch angle in degrees.
 * @param x_angle_vel Angular velocity around the X-axis. Units are 2e-12 rads/s.
 * @param y_angle_vel Angular velocity around the Y-axis. Units are 2e-12 rads/s.
 * @param z_angle_vel Angular velocity around the Z-axis. Units are 2e-12 rads/s.
 * @param x_accel Acceleration along the X-axis. Units are 2e-8 m/s^2.
 * @param y_accel Acceleration along the Y-axis. Units are 2e-8 m/s^2.
 * @param z_accel Acceleration along the Z-axis. Units are 2e-8 m/s^2.
 * @param mag_bearing Magnetic bearing. Units are 2^-13 rads.
 * @param latitude Raw GPS latitude. Units are 2e-29 rads.
 * @param longitude Raw GPS longitude. Units are 2e-29 rads.
 * @param est_latitude Estimated GPS latitude. Units are 2e-29 rads.
 * @param est_longitude Estimated GPS longitude. Units are 2e-29 rads.
 * @param gps_heading Raw GPS heading. Units are 2e-13 rads.
 * @param gps_speed Raw GPS speed. Units are 2e-6 m/s.
 * @param status Status bitfield. See Tokimec documentation.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tokimec_with_time_send(mavlink_channel_t chan, uint32_t time_boot_ms, int16_t yaw, int16_t pitch, int16_t roll, int16_t x_angle_vel, int16_t y_angle_vel, int16_t z_angle_vel, int16_t x_accel, int16_t y_accel, int16_t z_accel, int16_t mag_bearing, int32_t latitude, int32_t longitude, int32_t est_latitude, int32_t est_longitude, int16_t gps_heading, int16_t gps_speed, int16_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, latitude);
	_mav_put_int32_t(buf, 8, longitude);
	_mav_put_int32_t(buf, 12, est_latitude);
	_mav_put_int32_t(buf, 16, est_longitude);
	_mav_put_int16_t(buf, 20, yaw);
	_mav_put_int16_t(buf, 22, pitch);
	_mav_put_int16_t(buf, 24, roll);
	_mav_put_int16_t(buf, 26, x_angle_vel);
	_mav_put_int16_t(buf, 28, y_angle_vel);
	_mav_put_int16_t(buf, 30, z_angle_vel);
	_mav_put_int16_t(buf, 32, x_accel);
	_mav_put_int16_t(buf, 34, y_accel);
	_mav_put_int16_t(buf, 36, z_accel);
	_mav_put_int16_t(buf, 38, mag_bearing);
	_mav_put_int16_t(buf, 40, gps_heading);
	_mav_put_int16_t(buf, 42, gps_speed);
	_mav_put_int16_t(buf, 44, status);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME, buf, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME, buf, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN);
#endif
#else
	mavlink_tokimec_with_time_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.latitude = latitude;
	packet.longitude = longitude;
	packet.est_latitude = est_latitude;
	packet.est_longitude = est_longitude;
	packet.yaw = yaw;
	packet.pitch = pitch;
	packet.roll = roll;
	packet.x_angle_vel = x_angle_vel;
	packet.y_angle_vel = y_angle_vel;
	packet.z_angle_vel = z_angle_vel;
	packet.x_accel = x_accel;
	packet.y_accel = y_accel;
	packet.z_accel = z_accel;
	packet.mag_bearing = mag_bearing;
	packet.gps_heading = gps_heading;
	packet.gps_speed = gps_speed;
	packet.status = status;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME, (const char *)&packet, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME, (const char *)&packet, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tokimec_with_time_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, int16_t yaw, int16_t pitch, int16_t roll, int16_t x_angle_vel, int16_t y_angle_vel, int16_t z_angle_vel, int16_t x_accel, int16_t y_accel, int16_t z_accel, int16_t mag_bearing, int32_t latitude, int32_t longitude, int32_t est_latitude, int32_t est_longitude, int16_t gps_heading, int16_t gps_speed, int16_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_int32_t(buf, 4, latitude);
	_mav_put_int32_t(buf, 8, longitude);
	_mav_put_int32_t(buf, 12, est_latitude);
	_mav_put_int32_t(buf, 16, est_longitude);
	_mav_put_int16_t(buf, 20, yaw);
	_mav_put_int16_t(buf, 22, pitch);
	_mav_put_int16_t(buf, 24, roll);
	_mav_put_int16_t(buf, 26, x_angle_vel);
	_mav_put_int16_t(buf, 28, y_angle_vel);
	_mav_put_int16_t(buf, 30, z_angle_vel);
	_mav_put_int16_t(buf, 32, x_accel);
	_mav_put_int16_t(buf, 34, y_accel);
	_mav_put_int16_t(buf, 36, z_accel);
	_mav_put_int16_t(buf, 38, mag_bearing);
	_mav_put_int16_t(buf, 40, gps_heading);
	_mav_put_int16_t(buf, 42, gps_speed);
	_mav_put_int16_t(buf, 44, status);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME, buf, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME, buf, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN);
#endif
#else
	mavlink_tokimec_with_time_t *packet = (mavlink_tokimec_with_time_t *)msgbuf;
	packet->time_boot_ms = time_boot_ms;
	packet->latitude = latitude;
	packet->longitude = longitude;
	packet->est_latitude = est_latitude;
	packet->est_longitude = est_longitude;
	packet->yaw = yaw;
	packet->pitch = pitch;
	packet->roll = roll;
	packet->x_angle_vel = x_angle_vel;
	packet->y_angle_vel = y_angle_vel;
	packet->z_angle_vel = z_angle_vel;
	packet->x_accel = x_accel;
	packet->y_accel = y_accel;
	packet->z_accel = z_accel;
	packet->mag_bearing = mag_bearing;
	packet->gps_heading = gps_heading;
	packet->gps_speed = gps_speed;
	packet->status = status;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME, (const char *)packet, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME, (const char *)packet, MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE TOKIMEC_WITH_TIME UNPACKING


/**
 * @brief Get field time_boot_ms from tokimec_with_time message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_tokimec_with_time_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field yaw from tokimec_with_time message
 *
 * @return Heading in degrees eastward from north
 */
static inline int16_t mavlink_msg_tokimec_with_time_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field pitch from tokimec_with_time message
 *
 * @return Magnetometer status
 */
static inline int16_t mavlink_msg_tokimec_with_time_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field roll from tokimec_with_time message
 *
 * @return Pitch angle in degrees.
 */
static inline int16_t mavlink_msg_tokimec_with_time_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field x_angle_vel from tokimec_with_time message
 *
 * @return Angular velocity around the X-axis. Units are 2e-12 rads/s.
 */
static inline int16_t mavlink_msg_tokimec_with_time_get_x_angle_vel(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  26);
}

/**
 * @brief Get field y_angle_vel from tokimec_with_time message
 *
 * @return Angular velocity around the Y-axis. Units are 2e-12 rads/s.
 */
static inline int16_t mavlink_msg_tokimec_with_time_get_y_angle_vel(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  28);
}

/**
 * @brief Get field z_angle_vel from tokimec_with_time message
 *
 * @return Angular velocity around the Z-axis. Units are 2e-12 rads/s.
 */
static inline int16_t mavlink_msg_tokimec_with_time_get_z_angle_vel(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  30);
}

/**
 * @brief Get field x_accel from tokimec_with_time message
 *
 * @return Acceleration along the X-axis. Units are 2e-8 m/s^2.
 */
static inline int16_t mavlink_msg_tokimec_with_time_get_x_accel(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  32);
}

/**
 * @brief Get field y_accel from tokimec_with_time message
 *
 * @return Acceleration along the Y-axis. Units are 2e-8 m/s^2.
 */
static inline int16_t mavlink_msg_tokimec_with_time_get_y_accel(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  34);
}

/**
 * @brief Get field z_accel from tokimec_with_time message
 *
 * @return Acceleration along the Z-axis. Units are 2e-8 m/s^2.
 */
static inline int16_t mavlink_msg_tokimec_with_time_get_z_accel(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  36);
}

/**
 * @brief Get field mag_bearing from tokimec_with_time message
 *
 * @return Magnetic bearing. Units are 2^-13 rads.
 */
static inline int16_t mavlink_msg_tokimec_with_time_get_mag_bearing(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  38);
}

/**
 * @brief Get field latitude from tokimec_with_time message
 *
 * @return Raw GPS latitude. Units are 2e-29 rads.
 */
static inline int32_t mavlink_msg_tokimec_with_time_get_latitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field longitude from tokimec_with_time message
 *
 * @return Raw GPS longitude. Units are 2e-29 rads.
 */
static inline int32_t mavlink_msg_tokimec_with_time_get_longitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field est_latitude from tokimec_with_time message
 *
 * @return Estimated GPS latitude. Units are 2e-29 rads.
 */
static inline int32_t mavlink_msg_tokimec_with_time_get_est_latitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field est_longitude from tokimec_with_time message
 *
 * @return Estimated GPS longitude. Units are 2e-29 rads.
 */
static inline int32_t mavlink_msg_tokimec_with_time_get_est_longitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  16);
}

/**
 * @brief Get field gps_heading from tokimec_with_time message
 *
 * @return Raw GPS heading. Units are 2e-13 rads.
 */
static inline int16_t mavlink_msg_tokimec_with_time_get_gps_heading(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  40);
}

/**
 * @brief Get field gps_speed from tokimec_with_time message
 *
 * @return Raw GPS speed. Units are 2e-6 m/s.
 */
static inline int16_t mavlink_msg_tokimec_with_time_get_gps_speed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  42);
}

/**
 * @brief Get field status from tokimec_with_time message
 *
 * @return Status bitfield. See Tokimec documentation.
 */
static inline int16_t mavlink_msg_tokimec_with_time_get_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  44);
}

/**
 * @brief Decode a tokimec_with_time message into a struct
 *
 * @param msg The message to decode
 * @param tokimec_with_time C-struct to decode the message contents into
 */
static inline void mavlink_msg_tokimec_with_time_decode(const mavlink_message_t* msg, mavlink_tokimec_with_time_t* tokimec_with_time)
{
#if MAVLINK_NEED_BYTE_SWAP
	tokimec_with_time->time_boot_ms = mavlink_msg_tokimec_with_time_get_time_boot_ms(msg);
	tokimec_with_time->latitude = mavlink_msg_tokimec_with_time_get_latitude(msg);
	tokimec_with_time->longitude = mavlink_msg_tokimec_with_time_get_longitude(msg);
	tokimec_with_time->est_latitude = mavlink_msg_tokimec_with_time_get_est_latitude(msg);
	tokimec_with_time->est_longitude = mavlink_msg_tokimec_with_time_get_est_longitude(msg);
	tokimec_with_time->yaw = mavlink_msg_tokimec_with_time_get_yaw(msg);
	tokimec_with_time->pitch = mavlink_msg_tokimec_with_time_get_pitch(msg);
	tokimec_with_time->roll = mavlink_msg_tokimec_with_time_get_roll(msg);
	tokimec_with_time->x_angle_vel = mavlink_msg_tokimec_with_time_get_x_angle_vel(msg);
	tokimec_with_time->y_angle_vel = mavlink_msg_tokimec_with_time_get_y_angle_vel(msg);
	tokimec_with_time->z_angle_vel = mavlink_msg_tokimec_with_time_get_z_angle_vel(msg);
	tokimec_with_time->x_accel = mavlink_msg_tokimec_with_time_get_x_accel(msg);
	tokimec_with_time->y_accel = mavlink_msg_tokimec_with_time_get_y_accel(msg);
	tokimec_with_time->z_accel = mavlink_msg_tokimec_with_time_get_z_accel(msg);
	tokimec_with_time->mag_bearing = mavlink_msg_tokimec_with_time_get_mag_bearing(msg);
	tokimec_with_time->gps_heading = mavlink_msg_tokimec_with_time_get_gps_heading(msg);
	tokimec_with_time->gps_speed = mavlink_msg_tokimec_with_time_get_gps_speed(msg);
	tokimec_with_time->status = mavlink_msg_tokimec_with_time_get_status(msg);
#else
	memcpy(tokimec_with_time, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_TOKIMEC_WITH_TIME_LEN);
#endif
}
