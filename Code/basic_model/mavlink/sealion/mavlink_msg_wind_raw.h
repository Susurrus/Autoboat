// MESSAGE WIND_RAW PACKING

#define MAVLINK_MSG_ID_WIND_RAW 160

typedef struct __mavlink_wind_raw_t
{
 uint16_t speed; ///< Wind speed in cm/s.
 uint16_t direction; ///< Wind direction in e-4 rads east from north.
} mavlink_wind_raw_t;

#define MAVLINK_MSG_ID_WIND_RAW_LEN 4
#define MAVLINK_MSG_ID_160_LEN 4



#define MAVLINK_MESSAGE_INFO_WIND_RAW { \
	"WIND_RAW", \
	2, \
	{  { "speed", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_wind_raw_t, speed) }, \
         { "direction", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_wind_raw_t, direction) }, \
         } \
}


/**
 * @brief Pack a wind_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param speed Wind speed in cm/s.
 * @param direction Wind direction in e-4 rads east from north.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wind_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t speed, uint16_t direction)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, speed);
	_mav_put_uint16_t(buf, 2, direction);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 4);
#else
	mavlink_wind_raw_t packet;
	packet.speed = speed;
	packet.direction = direction;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_WIND_RAW;
	return mavlink_finalize_message(msg, system_id, component_id, 4, 119);
}

/**
 * @brief Pack a wind_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param speed Wind speed in cm/s.
 * @param direction Wind direction in e-4 rads east from north.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wind_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t speed,uint16_t direction)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, speed);
	_mav_put_uint16_t(buf, 2, direction);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 4);
#else
	mavlink_wind_raw_t packet;
	packet.speed = speed;
	packet.direction = direction;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_WIND_RAW;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4, 119);
}

/**
 * @brief Encode a wind_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param wind_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wind_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_wind_raw_t* wind_raw)
{
	return mavlink_msg_wind_raw_pack(system_id, component_id, msg, wind_raw->speed, wind_raw->direction);
}

/**
 * @brief Send a wind_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param speed Wind speed in cm/s.
 * @param direction Wind direction in e-4 rads east from north.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_wind_raw_send(mavlink_channel_t chan, uint16_t speed, uint16_t direction)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, speed);
	_mav_put_uint16_t(buf, 2, direction);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND_RAW, buf, 4, 119);
#else
	mavlink_wind_raw_t packet;
	packet.speed = speed;
	packet.direction = direction;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WIND_RAW, (const char *)&packet, 4, 119);
#endif
}

#endif

// MESSAGE WIND_RAW UNPACKING


/**
 * @brief Get field speed from wind_raw message
 *
 * @return Wind speed in cm/s.
 */
static inline uint16_t mavlink_msg_wind_raw_get_speed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field direction from wind_raw message
 *
 * @return Wind direction in e-4 rads east from north.
 */
static inline uint16_t mavlink_msg_wind_raw_get_direction(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Decode a wind_raw message into a struct
 *
 * @param msg The message to decode
 * @param wind_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_wind_raw_decode(const mavlink_message_t* msg, mavlink_wind_raw_t* wind_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	wind_raw->speed = mavlink_msg_wind_raw_get_speed(msg);
	wind_raw->direction = mavlink_msg_wind_raw_get_direction(msg);
#else
	memcpy(wind_raw, _MAV_PAYLOAD(msg), 4);
#endif
}
