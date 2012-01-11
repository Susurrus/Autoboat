// MESSAGE RUDDER_RAW PACKING

#define MAVLINK_MSG_ID_RUDDER_RAW 150

typedef struct __mavlink_rudder_raw_t
{
 uint16_t position; ///< The raw data from the position sensor, generally a potentiometer.
 uint8_t port_limit; ///< Status of the rudder limit sensor, port side. 0 indicates off and 1 indicates that the limit is hit. If this sensor is inactive set to 0xFF.
 uint8_t center_limit; ///< Status of the rudder limit sensor, port side. 0 indicates off and 1 indicates that the limit is hit. If this sensor is inactive set to 0xFF.
 uint8_t starboard_limit; ///< Status of the rudder limit sensor, starboard side. 0 indicates off and 1 indicates that the limit is hit. If this sensor is inactive set to 0xFF.
} mavlink_rudder_raw_t;

#define MAVLINK_MSG_ID_RUDDER_RAW_LEN 5
#define MAVLINK_MSG_ID_150_LEN 5



#define MAVLINK_MESSAGE_INFO_RUDDER_RAW { \
	"RUDDER_RAW", \
	4, \
	{  { "position", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_rudder_raw_t, position) }, \
         { "port_limit", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_rudder_raw_t, port_limit) }, \
         { "center_limit", NULL, MAVLINK_TYPE_UINT8_T, 0, 3, offsetof(mavlink_rudder_raw_t, center_limit) }, \
         { "starboard_limit", NULL, MAVLINK_TYPE_UINT8_T, 0, 4, offsetof(mavlink_rudder_raw_t, starboard_limit) }, \
         } \
}


/**
 * @brief Pack a rudder_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param position The raw data from the position sensor, generally a potentiometer.
 * @param port_limit Status of the rudder limit sensor, port side. 0 indicates off and 1 indicates that the limit is hit. If this sensor is inactive set to 0xFF.
 * @param center_limit Status of the rudder limit sensor, port side. 0 indicates off and 1 indicates that the limit is hit. If this sensor is inactive set to 0xFF.
 * @param starboard_limit Status of the rudder limit sensor, starboard side. 0 indicates off and 1 indicates that the limit is hit. If this sensor is inactive set to 0xFF.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rudder_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t position, uint8_t port_limit, uint8_t center_limit, uint8_t starboard_limit)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[5];
	_mav_put_uint16_t(buf, 0, position);
	_mav_put_uint8_t(buf, 2, port_limit);
	_mav_put_uint8_t(buf, 3, center_limit);
	_mav_put_uint8_t(buf, 4, starboard_limit);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 5);
#else
	mavlink_rudder_raw_t packet;
	packet.position = position;
	packet.port_limit = port_limit;
	packet.center_limit = center_limit;
	packet.starboard_limit = starboard_limit;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 5);
#endif

	msg->msgid = MAVLINK_MSG_ID_RUDDER_RAW;
	return mavlink_finalize_message(msg, system_id, component_id, 5, 32);
}

/**
 * @brief Pack a rudder_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param position The raw data from the position sensor, generally a potentiometer.
 * @param port_limit Status of the rudder limit sensor, port side. 0 indicates off and 1 indicates that the limit is hit. If this sensor is inactive set to 0xFF.
 * @param center_limit Status of the rudder limit sensor, port side. 0 indicates off and 1 indicates that the limit is hit. If this sensor is inactive set to 0xFF.
 * @param starboard_limit Status of the rudder limit sensor, starboard side. 0 indicates off and 1 indicates that the limit is hit. If this sensor is inactive set to 0xFF.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rudder_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t position,uint8_t port_limit,uint8_t center_limit,uint8_t starboard_limit)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[5];
	_mav_put_uint16_t(buf, 0, position);
	_mav_put_uint8_t(buf, 2, port_limit);
	_mav_put_uint8_t(buf, 3, center_limit);
	_mav_put_uint8_t(buf, 4, starboard_limit);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 5);
#else
	mavlink_rudder_raw_t packet;
	packet.position = position;
	packet.port_limit = port_limit;
	packet.center_limit = center_limit;
	packet.starboard_limit = starboard_limit;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 5);
#endif

	msg->msgid = MAVLINK_MSG_ID_RUDDER_RAW;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 5, 32);
}

/**
 * @brief Encode a rudder_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rudder_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rudder_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rudder_raw_t* rudder_raw)
{
	return mavlink_msg_rudder_raw_pack(system_id, component_id, msg, rudder_raw->position, rudder_raw->port_limit, rudder_raw->center_limit, rudder_raw->starboard_limit);
}

/**
 * @brief Send a rudder_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param position The raw data from the position sensor, generally a potentiometer.
 * @param port_limit Status of the rudder limit sensor, port side. 0 indicates off and 1 indicates that the limit is hit. If this sensor is inactive set to 0xFF.
 * @param center_limit Status of the rudder limit sensor, port side. 0 indicates off and 1 indicates that the limit is hit. If this sensor is inactive set to 0xFF.
 * @param starboard_limit Status of the rudder limit sensor, starboard side. 0 indicates off and 1 indicates that the limit is hit. If this sensor is inactive set to 0xFF.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rudder_raw_send(mavlink_channel_t chan, uint16_t position, uint8_t port_limit, uint8_t center_limit, uint8_t starboard_limit)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[5];
	_mav_put_uint16_t(buf, 0, position);
	_mav_put_uint8_t(buf, 2, port_limit);
	_mav_put_uint8_t(buf, 3, center_limit);
	_mav_put_uint8_t(buf, 4, starboard_limit);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RUDDER_RAW, buf, 5, 32);
#else
	mavlink_rudder_raw_t packet;
	packet.position = position;
	packet.port_limit = port_limit;
	packet.center_limit = center_limit;
	packet.starboard_limit = starboard_limit;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RUDDER_RAW, (const char *)&packet, 5, 32);
#endif
}

#endif

// MESSAGE RUDDER_RAW UNPACKING


/**
 * @brief Get field position from rudder_raw message
 *
 * @return The raw data from the position sensor, generally a potentiometer.
 */
static inline uint16_t mavlink_msg_rudder_raw_get_position(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field port_limit from rudder_raw message
 *
 * @return Status of the rudder limit sensor, port side. 0 indicates off and 1 indicates that the limit is hit. If this sensor is inactive set to 0xFF.
 */
static inline uint8_t mavlink_msg_rudder_raw_get_port_limit(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Get field center_limit from rudder_raw message
 *
 * @return Status of the rudder limit sensor, port side. 0 indicates off and 1 indicates that the limit is hit. If this sensor is inactive set to 0xFF.
 */
static inline uint8_t mavlink_msg_rudder_raw_get_center_limit(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  3);
}

/**
 * @brief Get field starboard_limit from rudder_raw message
 *
 * @return Status of the rudder limit sensor, starboard side. 0 indicates off and 1 indicates that the limit is hit. If this sensor is inactive set to 0xFF.
 */
static inline uint8_t mavlink_msg_rudder_raw_get_starboard_limit(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  4);
}

/**
 * @brief Decode a rudder_raw message into a struct
 *
 * @param msg The message to decode
 * @param rudder_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_rudder_raw_decode(const mavlink_message_t* msg, mavlink_rudder_raw_t* rudder_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	rudder_raw->position = mavlink_msg_rudder_raw_get_position(msg);
	rudder_raw->port_limit = mavlink_msg_rudder_raw_get_port_limit(msg);
	rudder_raw->center_limit = mavlink_msg_rudder_raw_get_center_limit(msg);
	rudder_raw->starboard_limit = mavlink_msg_rudder_raw_get_starboard_limit(msg);
#else
	memcpy(rudder_raw, _MAV_PAYLOAD(msg), 5);
#endif
}
