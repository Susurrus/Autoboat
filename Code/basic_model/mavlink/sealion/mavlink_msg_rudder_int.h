// MESSAGE RUDDER_INT PACKING

#define MAVLINK_MSG_ID_RUDDER_INT 151

typedef struct __mavlink_rudder_int_t
{
 uint16_t port_limit_val; ///< Holds the raw value that indicates the port limit. Used for mapping the raw ADC values into real units.
 uint16_t starboat_limit_val; ///< Holds the raw value that indicates the starboard limit. Used for mapping the raw ADC values into real units.
 uint16_t rudder_angle; ///< The interpreted rudder angle in milliradians.
} mavlink_rudder_int_t;

#define MAVLINK_MSG_ID_RUDDER_INT_LEN 6
#define MAVLINK_MSG_ID_151_LEN 6



#define MAVLINK_MESSAGE_INFO_RUDDER_INT { \
	"RUDDER_INT", \
	3, \
	{  { "port_limit_val", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_rudder_int_t, port_limit_val) }, \
         { "starboat_limit_val", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_rudder_int_t, starboat_limit_val) }, \
         { "rudder_angle", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_rudder_int_t, rudder_angle) }, \
         } \
}


/**
 * @brief Pack a rudder_int message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param port_limit_val Holds the raw value that indicates the port limit. Used for mapping the raw ADC values into real units.
 * @param starboat_limit_val Holds the raw value that indicates the starboard limit. Used for mapping the raw ADC values into real units.
 * @param rudder_angle The interpreted rudder angle in milliradians.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rudder_int_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t port_limit_val, uint16_t starboat_limit_val, uint16_t rudder_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_uint16_t(buf, 0, port_limit_val);
	_mav_put_uint16_t(buf, 2, starboat_limit_val);
	_mav_put_uint16_t(buf, 4, rudder_angle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 6);
#else
	mavlink_rudder_int_t packet;
	packet.port_limit_val = port_limit_val;
	packet.starboat_limit_val = starboat_limit_val;
	packet.rudder_angle = rudder_angle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 6);
#endif

	msg->msgid = MAVLINK_MSG_ID_RUDDER_INT;
	return mavlink_finalize_message(msg, system_id, component_id, 6, 238);
}

/**
 * @brief Pack a rudder_int message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param port_limit_val Holds the raw value that indicates the port limit. Used for mapping the raw ADC values into real units.
 * @param starboat_limit_val Holds the raw value that indicates the starboard limit. Used for mapping the raw ADC values into real units.
 * @param rudder_angle The interpreted rudder angle in milliradians.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_rudder_int_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t port_limit_val,uint16_t starboat_limit_val,uint16_t rudder_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_uint16_t(buf, 0, port_limit_val);
	_mav_put_uint16_t(buf, 2, starboat_limit_val);
	_mav_put_uint16_t(buf, 4, rudder_angle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 6);
#else
	mavlink_rudder_int_t packet;
	packet.port_limit_val = port_limit_val;
	packet.starboat_limit_val = starboat_limit_val;
	packet.rudder_angle = rudder_angle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 6);
#endif

	msg->msgid = MAVLINK_MSG_ID_RUDDER_INT;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 6, 238);
}

/**
 * @brief Encode a rudder_int struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param rudder_int C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_rudder_int_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_rudder_int_t* rudder_int)
{
	return mavlink_msg_rudder_int_pack(system_id, component_id, msg, rudder_int->port_limit_val, rudder_int->starboat_limit_val, rudder_int->rudder_angle);
}

/**
 * @brief Send a rudder_int message
 * @param chan MAVLink channel to send the message
 *
 * @param port_limit_val Holds the raw value that indicates the port limit. Used for mapping the raw ADC values into real units.
 * @param starboat_limit_val Holds the raw value that indicates the starboard limit. Used for mapping the raw ADC values into real units.
 * @param rudder_angle The interpreted rudder angle in milliradians.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_rudder_int_send(mavlink_channel_t chan, uint16_t port_limit_val, uint16_t starboat_limit_val, uint16_t rudder_angle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_uint16_t(buf, 0, port_limit_val);
	_mav_put_uint16_t(buf, 2, starboat_limit_val);
	_mav_put_uint16_t(buf, 4, rudder_angle);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RUDDER_INT, buf, 6, 238);
#else
	mavlink_rudder_int_t packet;
	packet.port_limit_val = port_limit_val;
	packet.starboat_limit_val = starboat_limit_val;
	packet.rudder_angle = rudder_angle;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_RUDDER_INT, (const char *)&packet, 6, 238);
#endif
}

#endif

// MESSAGE RUDDER_INT UNPACKING


/**
 * @brief Get field port_limit_val from rudder_int message
 *
 * @return Holds the raw value that indicates the port limit. Used for mapping the raw ADC values into real units.
 */
static inline uint16_t mavlink_msg_rudder_int_get_port_limit_val(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field starboat_limit_val from rudder_int message
 *
 * @return Holds the raw value that indicates the starboard limit. Used for mapping the raw ADC values into real units.
 */
static inline uint16_t mavlink_msg_rudder_int_get_starboat_limit_val(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field rudder_angle from rudder_int message
 *
 * @return The interpreted rudder angle in milliradians.
 */
static inline uint16_t mavlink_msg_rudder_int_get_rudder_angle(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Decode a rudder_int message into a struct
 *
 * @param msg The message to decode
 * @param rudder_int C-struct to decode the message contents into
 */
static inline void mavlink_msg_rudder_int_decode(const mavlink_message_t* msg, mavlink_rudder_int_t* rudder_int)
{
#if MAVLINK_NEED_BYTE_SWAP
	rudder_int->port_limit_val = mavlink_msg_rudder_int_get_port_limit_val(msg);
	rudder_int->starboat_limit_val = mavlink_msg_rudder_int_get_starboat_limit_val(msg);
	rudder_int->rudder_angle = mavlink_msg_rudder_int_get_rudder_angle(msg);
#else
	memcpy(rudder_int, _MAV_PAYLOAD(msg), 6);
#endif
}
