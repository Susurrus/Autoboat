// MESSAGE TOKIMEC PACKING

#define MAVLINK_MSG_ID_TOKIMEC 165

typedef struct __mavlink_tokimec_t
{
 int32_t latitude; ///< Raw GPS latitude. Units are 2e-29 rads.
 int32_t longitude; ///< Raw GPS longitude. Units are 2e-29 rads.
 int32_t est_latitude; ///< Estimated GPS latitude. Units are 2e-29 rads.
 int32_t est_longitude; ///< Estimated GPS longitude. Units are 2e-29 rads.
 int16_t yaw; ///< Yaw angle. Units are 2e-13 rads.
 int16_t pitch; ///< Pitch angle. Units are 2e-13 rads.
 int16_t roll; ///< Roll angle. Units are 2e-13 rads.
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
} mavlink_tokimec_t;

#define MAVLINK_MSG_ID_TOKIMEC_LEN 42
#define MAVLINK_MSG_ID_165_LEN 42

#define MAVLINK_MSG_ID_TOKIMEC_CRC 111
#define MAVLINK_MSG_ID_165_CRC 111



#define MAVLINK_MESSAGE_INFO_TOKIMEC { \
	"TOKIMEC", \
	17, \
	{  { "latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 0, offsetof(mavlink_tokimec_t, latitude) }, \
         { "longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 4, offsetof(mavlink_tokimec_t, longitude) }, \
         { "est_latitude", NULL, MAVLINK_TYPE_INT32_T, 0, 8, offsetof(mavlink_tokimec_t, est_latitude) }, \
         { "est_longitude", NULL, MAVLINK_TYPE_INT32_T, 0, 12, offsetof(mavlink_tokimec_t, est_longitude) }, \
         { "yaw", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_tokimec_t, yaw) }, \
         { "pitch", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_tokimec_t, pitch) }, \
         { "roll", NULL, MAVLINK_TYPE_INT16_T, 0, 20, offsetof(mavlink_tokimec_t, roll) }, \
         { "x_angle_vel", NULL, MAVLINK_TYPE_INT16_T, 0, 22, offsetof(mavlink_tokimec_t, x_angle_vel) }, \
         { "y_angle_vel", NULL, MAVLINK_TYPE_INT16_T, 0, 24, offsetof(mavlink_tokimec_t, y_angle_vel) }, \
         { "z_angle_vel", NULL, MAVLINK_TYPE_INT16_T, 0, 26, offsetof(mavlink_tokimec_t, z_angle_vel) }, \
         { "x_accel", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_tokimec_t, x_accel) }, \
         { "y_accel", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_tokimec_t, y_accel) }, \
         { "z_accel", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_tokimec_t, z_accel) }, \
         { "mag_bearing", NULL, MAVLINK_TYPE_INT16_T, 0, 34, offsetof(mavlink_tokimec_t, mag_bearing) }, \
         { "gps_heading", NULL, MAVLINK_TYPE_INT16_T, 0, 36, offsetof(mavlink_tokimec_t, gps_heading) }, \
         { "gps_speed", NULL, MAVLINK_TYPE_INT16_T, 0, 38, offsetof(mavlink_tokimec_t, gps_speed) }, \
         { "status", NULL, MAVLINK_TYPE_INT16_T, 0, 40, offsetof(mavlink_tokimec_t, status) }, \
         } \
}


/**
 * @brief Pack a tokimec message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param yaw Yaw angle. Units are 2e-13 rads.
 * @param pitch Pitch angle. Units are 2e-13 rads.
 * @param roll Roll angle. Units are 2e-13 rads.
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
static inline uint16_t mavlink_msg_tokimec_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t yaw, int16_t pitch, int16_t roll, int16_t x_angle_vel, int16_t y_angle_vel, int16_t z_angle_vel, int16_t x_accel, int16_t y_accel, int16_t z_accel, int16_t mag_bearing, int32_t latitude, int32_t longitude, int32_t est_latitude, int32_t est_longitude, int16_t gps_heading, int16_t gps_speed, int16_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TOKIMEC_LEN];
	_mav_put_int32_t(buf, 0, latitude);
	_mav_put_int32_t(buf, 4, longitude);
	_mav_put_int32_t(buf, 8, est_latitude);
	_mav_put_int32_t(buf, 12, est_longitude);
	_mav_put_int16_t(buf, 16, yaw);
	_mav_put_int16_t(buf, 18, pitch);
	_mav_put_int16_t(buf, 20, roll);
	_mav_put_int16_t(buf, 22, x_angle_vel);
	_mav_put_int16_t(buf, 24, y_angle_vel);
	_mav_put_int16_t(buf, 26, z_angle_vel);
	_mav_put_int16_t(buf, 28, x_accel);
	_mav_put_int16_t(buf, 30, y_accel);
	_mav_put_int16_t(buf, 32, z_accel);
	_mav_put_int16_t(buf, 34, mag_bearing);
	_mav_put_int16_t(buf, 36, gps_heading);
	_mav_put_int16_t(buf, 38, gps_speed);
	_mav_put_int16_t(buf, 40, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TOKIMEC_LEN);
#else
	mavlink_tokimec_t packet;
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

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TOKIMEC_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TOKIMEC;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TOKIMEC_LEN, MAVLINK_MSG_ID_TOKIMEC_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_TOKIMEC_LEN);
#endif
}

/**
 * @brief Pack a tokimec message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param yaw Yaw angle. Units are 2e-13 rads.
 * @param pitch Pitch angle. Units are 2e-13 rads.
 * @param roll Roll angle. Units are 2e-13 rads.
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
static inline uint16_t mavlink_msg_tokimec_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t yaw,int16_t pitch,int16_t roll,int16_t x_angle_vel,int16_t y_angle_vel,int16_t z_angle_vel,int16_t x_accel,int16_t y_accel,int16_t z_accel,int16_t mag_bearing,int32_t latitude,int32_t longitude,int32_t est_latitude,int32_t est_longitude,int16_t gps_heading,int16_t gps_speed,int16_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TOKIMEC_LEN];
	_mav_put_int32_t(buf, 0, latitude);
	_mav_put_int32_t(buf, 4, longitude);
	_mav_put_int32_t(buf, 8, est_latitude);
	_mav_put_int32_t(buf, 12, est_longitude);
	_mav_put_int16_t(buf, 16, yaw);
	_mav_put_int16_t(buf, 18, pitch);
	_mav_put_int16_t(buf, 20, roll);
	_mav_put_int16_t(buf, 22, x_angle_vel);
	_mav_put_int16_t(buf, 24, y_angle_vel);
	_mav_put_int16_t(buf, 26, z_angle_vel);
	_mav_put_int16_t(buf, 28, x_accel);
	_mav_put_int16_t(buf, 30, y_accel);
	_mav_put_int16_t(buf, 32, z_accel);
	_mav_put_int16_t(buf, 34, mag_bearing);
	_mav_put_int16_t(buf, 36, gps_heading);
	_mav_put_int16_t(buf, 38, gps_speed);
	_mav_put_int16_t(buf, 40, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_TOKIMEC_LEN);
#else
	mavlink_tokimec_t packet;
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

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_TOKIMEC_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_TOKIMEC;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TOKIMEC_LEN, MAVLINK_MSG_ID_TOKIMEC_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_TOKIMEC_LEN);
#endif
}

/**
 * @brief Encode a tokimec struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tokimec C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tokimec_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tokimec_t* tokimec)
{
	return mavlink_msg_tokimec_pack(system_id, component_id, msg, tokimec->yaw, tokimec->pitch, tokimec->roll, tokimec->x_angle_vel, tokimec->y_angle_vel, tokimec->z_angle_vel, tokimec->x_accel, tokimec->y_accel, tokimec->z_accel, tokimec->mag_bearing, tokimec->latitude, tokimec->longitude, tokimec->est_latitude, tokimec->est_longitude, tokimec->gps_heading, tokimec->gps_speed, tokimec->status);
}

/**
 * @brief Encode a tokimec struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param tokimec C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tokimec_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_tokimec_t* tokimec)
{
	return mavlink_msg_tokimec_pack_chan(system_id, component_id, chan, msg, tokimec->yaw, tokimec->pitch, tokimec->roll, tokimec->x_angle_vel, tokimec->y_angle_vel, tokimec->z_angle_vel, tokimec->x_accel, tokimec->y_accel, tokimec->z_accel, tokimec->mag_bearing, tokimec->latitude, tokimec->longitude, tokimec->est_latitude, tokimec->est_longitude, tokimec->gps_heading, tokimec->gps_speed, tokimec->status);
}

/**
 * @brief Send a tokimec message
 * @param chan MAVLink channel to send the message
 *
 * @param yaw Yaw angle. Units are 2e-13 rads.
 * @param pitch Pitch angle. Units are 2e-13 rads.
 * @param roll Roll angle. Units are 2e-13 rads.
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

static inline void mavlink_msg_tokimec_send(mavlink_channel_t chan, int16_t yaw, int16_t pitch, int16_t roll, int16_t x_angle_vel, int16_t y_angle_vel, int16_t z_angle_vel, int16_t x_accel, int16_t y_accel, int16_t z_accel, int16_t mag_bearing, int32_t latitude, int32_t longitude, int32_t est_latitude, int32_t est_longitude, int16_t gps_heading, int16_t gps_speed, int16_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_TOKIMEC_LEN];
	_mav_put_int32_t(buf, 0, latitude);
	_mav_put_int32_t(buf, 4, longitude);
	_mav_put_int32_t(buf, 8, est_latitude);
	_mav_put_int32_t(buf, 12, est_longitude);
	_mav_put_int16_t(buf, 16, yaw);
	_mav_put_int16_t(buf, 18, pitch);
	_mav_put_int16_t(buf, 20, roll);
	_mav_put_int16_t(buf, 22, x_angle_vel);
	_mav_put_int16_t(buf, 24, y_angle_vel);
	_mav_put_int16_t(buf, 26, z_angle_vel);
	_mav_put_int16_t(buf, 28, x_accel);
	_mav_put_int16_t(buf, 30, y_accel);
	_mav_put_int16_t(buf, 32, z_accel);
	_mav_put_int16_t(buf, 34, mag_bearing);
	_mav_put_int16_t(buf, 36, gps_heading);
	_mav_put_int16_t(buf, 38, gps_speed);
	_mav_put_int16_t(buf, 40, status);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOKIMEC, buf, MAVLINK_MSG_ID_TOKIMEC_LEN, MAVLINK_MSG_ID_TOKIMEC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOKIMEC, buf, MAVLINK_MSG_ID_TOKIMEC_LEN);
#endif
#else
	mavlink_tokimec_t packet;
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
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOKIMEC, (const char *)&packet, MAVLINK_MSG_ID_TOKIMEC_LEN, MAVLINK_MSG_ID_TOKIMEC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOKIMEC, (const char *)&packet, MAVLINK_MSG_ID_TOKIMEC_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_TOKIMEC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_tokimec_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  int16_t yaw, int16_t pitch, int16_t roll, int16_t x_angle_vel, int16_t y_angle_vel, int16_t z_angle_vel, int16_t x_accel, int16_t y_accel, int16_t z_accel, int16_t mag_bearing, int32_t latitude, int32_t longitude, int32_t est_latitude, int32_t est_longitude, int16_t gps_heading, int16_t gps_speed, int16_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_int32_t(buf, 0, latitude);
	_mav_put_int32_t(buf, 4, longitude);
	_mav_put_int32_t(buf, 8, est_latitude);
	_mav_put_int32_t(buf, 12, est_longitude);
	_mav_put_int16_t(buf, 16, yaw);
	_mav_put_int16_t(buf, 18, pitch);
	_mav_put_int16_t(buf, 20, roll);
	_mav_put_int16_t(buf, 22, x_angle_vel);
	_mav_put_int16_t(buf, 24, y_angle_vel);
	_mav_put_int16_t(buf, 26, z_angle_vel);
	_mav_put_int16_t(buf, 28, x_accel);
	_mav_put_int16_t(buf, 30, y_accel);
	_mav_put_int16_t(buf, 32, z_accel);
	_mav_put_int16_t(buf, 34, mag_bearing);
	_mav_put_int16_t(buf, 36, gps_heading);
	_mav_put_int16_t(buf, 38, gps_speed);
	_mav_put_int16_t(buf, 40, status);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOKIMEC, buf, MAVLINK_MSG_ID_TOKIMEC_LEN, MAVLINK_MSG_ID_TOKIMEC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOKIMEC, buf, MAVLINK_MSG_ID_TOKIMEC_LEN);
#endif
#else
	mavlink_tokimec_t *packet = (mavlink_tokimec_t *)msgbuf;
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
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOKIMEC, (const char *)packet, MAVLINK_MSG_ID_TOKIMEC_LEN, MAVLINK_MSG_ID_TOKIMEC_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TOKIMEC, (const char *)packet, MAVLINK_MSG_ID_TOKIMEC_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE TOKIMEC UNPACKING


/**
 * @brief Get field yaw from tokimec message
 *
 * @return Yaw angle. Units are 2e-13 rads.
 */
static inline int16_t mavlink_msg_tokimec_get_yaw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field pitch from tokimec message
 *
 * @return Pitch angle. Units are 2e-13 rads.
 */
static inline int16_t mavlink_msg_tokimec_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field roll from tokimec message
 *
 * @return Roll angle. Units are 2e-13 rads.
 */
static inline int16_t mavlink_msg_tokimec_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  20);
}

/**
 * @brief Get field x_angle_vel from tokimec message
 *
 * @return Angular velocity around the X-axis. Units are 2e-12 rads/s.
 */
static inline int16_t mavlink_msg_tokimec_get_x_angle_vel(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  22);
}

/**
 * @brief Get field y_angle_vel from tokimec message
 *
 * @return Angular velocity around the Y-axis. Units are 2e-12 rads/s.
 */
static inline int16_t mavlink_msg_tokimec_get_y_angle_vel(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  24);
}

/**
 * @brief Get field z_angle_vel from tokimec message
 *
 * @return Angular velocity around the Z-axis. Units are 2e-12 rads/s.
 */
static inline int16_t mavlink_msg_tokimec_get_z_angle_vel(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  26);
}

/**
 * @brief Get field x_accel from tokimec message
 *
 * @return Acceleration along the X-axis. Units are 2e-8 m/s^2.
 */
static inline int16_t mavlink_msg_tokimec_get_x_accel(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  28);
}

/**
 * @brief Get field y_accel from tokimec message
 *
 * @return Acceleration along the Y-axis. Units are 2e-8 m/s^2.
 */
static inline int16_t mavlink_msg_tokimec_get_y_accel(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  30);
}

/**
 * @brief Get field z_accel from tokimec message
 *
 * @return Acceleration along the Z-axis. Units are 2e-8 m/s^2.
 */
static inline int16_t mavlink_msg_tokimec_get_z_accel(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  32);
}

/**
 * @brief Get field mag_bearing from tokimec message
 *
 * @return Magnetic bearing. Units are 2^-13 rads.
 */
static inline int16_t mavlink_msg_tokimec_get_mag_bearing(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  34);
}

/**
 * @brief Get field latitude from tokimec message
 *
 * @return Raw GPS latitude. Units are 2e-29 rads.
 */
static inline int32_t mavlink_msg_tokimec_get_latitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  0);
}

/**
 * @brief Get field longitude from tokimec message
 *
 * @return Raw GPS longitude. Units are 2e-29 rads.
 */
static inline int32_t mavlink_msg_tokimec_get_longitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  4);
}

/**
 * @brief Get field est_latitude from tokimec message
 *
 * @return Estimated GPS latitude. Units are 2e-29 rads.
 */
static inline int32_t mavlink_msg_tokimec_get_est_latitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  8);
}

/**
 * @brief Get field est_longitude from tokimec message
 *
 * @return Estimated GPS longitude. Units are 2e-29 rads.
 */
static inline int32_t mavlink_msg_tokimec_get_est_longitude(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int32_t(msg,  12);
}

/**
 * @brief Get field gps_heading from tokimec message
 *
 * @return Raw GPS heading. Units are 2e-13 rads.
 */
static inline int16_t mavlink_msg_tokimec_get_gps_heading(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  36);
}

/**
 * @brief Get field gps_speed from tokimec message
 *
 * @return Raw GPS speed. Units are 2e-6 m/s.
 */
static inline int16_t mavlink_msg_tokimec_get_gps_speed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  38);
}

/**
 * @brief Get field status from tokimec message
 *
 * @return Status bitfield. See Tokimec documentation.
 */
static inline int16_t mavlink_msg_tokimec_get_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  40);
}

/**
 * @brief Decode a tokimec message into a struct
 *
 * @param msg The message to decode
 * @param tokimec C-struct to decode the message contents into
 */
static inline void mavlink_msg_tokimec_decode(const mavlink_message_t* msg, mavlink_tokimec_t* tokimec)
{
#if MAVLINK_NEED_BYTE_SWAP
	tokimec->latitude = mavlink_msg_tokimec_get_latitude(msg);
	tokimec->longitude = mavlink_msg_tokimec_get_longitude(msg);
	tokimec->est_latitude = mavlink_msg_tokimec_get_est_latitude(msg);
	tokimec->est_longitude = mavlink_msg_tokimec_get_est_longitude(msg);
	tokimec->yaw = mavlink_msg_tokimec_get_yaw(msg);
	tokimec->pitch = mavlink_msg_tokimec_get_pitch(msg);
	tokimec->roll = mavlink_msg_tokimec_get_roll(msg);
	tokimec->x_angle_vel = mavlink_msg_tokimec_get_x_angle_vel(msg);
	tokimec->y_angle_vel = mavlink_msg_tokimec_get_y_angle_vel(msg);
	tokimec->z_angle_vel = mavlink_msg_tokimec_get_z_angle_vel(msg);
	tokimec->x_accel = mavlink_msg_tokimec_get_x_accel(msg);
	tokimec->y_accel = mavlink_msg_tokimec_get_y_accel(msg);
	tokimec->z_accel = mavlink_msg_tokimec_get_z_accel(msg);
	tokimec->mag_bearing = mavlink_msg_tokimec_get_mag_bearing(msg);
	tokimec->gps_heading = mavlink_msg_tokimec_get_gps_heading(msg);
	tokimec->gps_speed = mavlink_msg_tokimec_get_gps_speed(msg);
	tokimec->status = mavlink_msg_tokimec_get_status(msg);
#else
	memcpy(tokimec, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_TOKIMEC_LEN);
#endif
}
