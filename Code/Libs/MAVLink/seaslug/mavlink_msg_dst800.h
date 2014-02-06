// MESSAGE DST800 PACKING

#define MAVLINK_MSG_ID_DST800 161

typedef struct __mavlink_dst800_t
{
 float speed; ///< Water speed in m/s.
 float temperature; ///< Water temperature in degrees Celsius.
 float depth; ///< Water depth in m (DST800 range is 100m). Invalid measurements are 0.0.
} mavlink_dst800_t;

#define MAVLINK_MSG_ID_DST800_LEN 12
#define MAVLINK_MSG_ID_161_LEN 12

#define MAVLINK_MSG_ID_DST800_CRC 43
#define MAVLINK_MSG_ID_161_CRC 43



#define MAVLINK_MESSAGE_INFO_DST800 { \
	"DST800", \
	3, \
	{  { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_dst800_t, speed) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_dst800_t, temperature) }, \
         { "depth", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_dst800_t, depth) }, \
         } \
}


/**
 * @brief Pack a dst800 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param speed Water speed in m/s.
 * @param temperature Water temperature in degrees Celsius.
 * @param depth Water depth in m (DST800 range is 100m). Invalid measurements are 0.0.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dst800_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float speed, float temperature, float depth)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DST800_LEN];
	_mav_put_float(buf, 0, speed);
	_mav_put_float(buf, 4, temperature);
	_mav_put_float(buf, 8, depth);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DST800_LEN);
#else
	mavlink_dst800_t packet;
	packet.speed = speed;
	packet.temperature = temperature;
	packet.depth = depth;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DST800_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DST800;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DST800_LEN, MAVLINK_MSG_ID_DST800_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DST800_LEN);
#endif
}

/**
 * @brief Pack a dst800 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param speed Water speed in m/s.
 * @param temperature Water temperature in degrees Celsius.
 * @param depth Water depth in m (DST800 range is 100m). Invalid measurements are 0.0.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dst800_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float speed,float temperature,float depth)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DST800_LEN];
	_mav_put_float(buf, 0, speed);
	_mav_put_float(buf, 4, temperature);
	_mav_put_float(buf, 8, depth);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DST800_LEN);
#else
	mavlink_dst800_t packet;
	packet.speed = speed;
	packet.temperature = temperature;
	packet.depth = depth;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DST800_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DST800;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DST800_LEN, MAVLINK_MSG_ID_DST800_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DST800_LEN);
#endif
}

/**
 * @brief Encode a dst800 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param dst800 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dst800_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_dst800_t* dst800)
{
	return mavlink_msg_dst800_pack(system_id, component_id, msg, dst800->speed, dst800->temperature, dst800->depth);
}

/**
 * @brief Encode a dst800 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param dst800 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dst800_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_dst800_t* dst800)
{
	return mavlink_msg_dst800_pack_chan(system_id, component_id, chan, msg, dst800->speed, dst800->temperature, dst800->depth);
}

/**
 * @brief Send a dst800 message
 * @param chan MAVLink channel to send the message
 *
 * @param speed Water speed in m/s.
 * @param temperature Water temperature in degrees Celsius.
 * @param depth Water depth in m (DST800 range is 100m). Invalid measurements are 0.0.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_dst800_send(mavlink_channel_t chan, float speed, float temperature, float depth)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DST800_LEN];
	_mav_put_float(buf, 0, speed);
	_mav_put_float(buf, 4, temperature);
	_mav_put_float(buf, 8, depth);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DST800, buf, MAVLINK_MSG_ID_DST800_LEN, MAVLINK_MSG_ID_DST800_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DST800, buf, MAVLINK_MSG_ID_DST800_LEN);
#endif
#else
	mavlink_dst800_t packet;
	packet.speed = speed;
	packet.temperature = temperature;
	packet.depth = depth;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DST800, (const char *)&packet, MAVLINK_MSG_ID_DST800_LEN, MAVLINK_MSG_ID_DST800_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DST800, (const char *)&packet, MAVLINK_MSG_ID_DST800_LEN);
#endif
#endif
}

#endif

// MESSAGE DST800 UNPACKING


/**
 * @brief Get field speed from dst800 message
 *
 * @return Water speed in m/s.
 */
static inline float mavlink_msg_dst800_get_speed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field temperature from dst800 message
 *
 * @return Water temperature in degrees Celsius.
 */
static inline float mavlink_msg_dst800_get_temperature(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field depth from dst800 message
 *
 * @return Water depth in m (DST800 range is 100m). Invalid measurements are 0.0.
 */
static inline float mavlink_msg_dst800_get_depth(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Decode a dst800 message into a struct
 *
 * @param msg The message to decode
 * @param dst800 C-struct to decode the message contents into
 */
static inline void mavlink_msg_dst800_decode(const mavlink_message_t* msg, mavlink_dst800_t* dst800)
{
#if MAVLINK_NEED_BYTE_SWAP
	dst800->speed = mavlink_msg_dst800_get_speed(msg);
	dst800->temperature = mavlink_msg_dst800_get_temperature(msg);
	dst800->depth = mavlink_msg_dst800_get_depth(msg);
#else
	memcpy(dst800, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_DST800_LEN);
#endif
}
