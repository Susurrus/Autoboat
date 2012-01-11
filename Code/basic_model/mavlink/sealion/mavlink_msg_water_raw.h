// MESSAGE WATER_RAW PACKING

#define MAVLINK_MSG_ID_WATER_RAW 162

typedef struct __mavlink_water_raw_t
{
 int16_t speed; ///< Water speed in mm/s.
 int16_t temperature; ///< Water temperature in centidegrees Celsius.
 uint16_t depth; ///< Water depth in centimeters (DST800 range is 100m). Invalid measurements are 0xFFFF.
} mavlink_water_raw_t;

#define MAVLINK_MSG_ID_WATER_RAW_LEN 6
#define MAVLINK_MSG_ID_162_LEN 6



#define MAVLINK_MESSAGE_INFO_WATER_RAW { \
	"WATER_RAW", \
	3, \
	{  { "speed", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_water_raw_t, speed) }, \
         { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_water_raw_t, temperature) }, \
         { "depth", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_water_raw_t, depth) }, \
         } \
}


/**
 * @brief Pack a water_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param speed Water speed in mm/s.
 * @param temperature Water temperature in centidegrees Celsius.
 * @param depth Water depth in centimeters (DST800 range is 100m). Invalid measurements are 0xFFFF.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_water_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t speed, int16_t temperature, uint16_t depth)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_int16_t(buf, 0, speed);
	_mav_put_int16_t(buf, 2, temperature);
	_mav_put_uint16_t(buf, 4, depth);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 6);
#else
	mavlink_water_raw_t packet;
	packet.speed = speed;
	packet.temperature = temperature;
	packet.depth = depth;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 6);
#endif

	msg->msgid = MAVLINK_MSG_ID_WATER_RAW;
	return mavlink_finalize_message(msg, system_id, component_id, 6, 90);
}

/**
 * @brief Pack a water_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param speed Water speed in mm/s.
 * @param temperature Water temperature in centidegrees Celsius.
 * @param depth Water depth in centimeters (DST800 range is 100m). Invalid measurements are 0xFFFF.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_water_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t speed,int16_t temperature,uint16_t depth)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_int16_t(buf, 0, speed);
	_mav_put_int16_t(buf, 2, temperature);
	_mav_put_uint16_t(buf, 4, depth);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 6);
#else
	mavlink_water_raw_t packet;
	packet.speed = speed;
	packet.temperature = temperature;
	packet.depth = depth;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 6);
#endif

	msg->msgid = MAVLINK_MSG_ID_WATER_RAW;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 6, 90);
}

/**
 * @brief Encode a water_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param water_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_water_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_water_raw_t* water_raw)
{
	return mavlink_msg_water_raw_pack(system_id, component_id, msg, water_raw->speed, water_raw->temperature, water_raw->depth);
}

/**
 * @brief Send a water_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param speed Water speed in mm/s.
 * @param temperature Water temperature in centidegrees Celsius.
 * @param depth Water depth in centimeters (DST800 range is 100m). Invalid measurements are 0xFFFF.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_water_raw_send(mavlink_channel_t chan, int16_t speed, int16_t temperature, uint16_t depth)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_int16_t(buf, 0, speed);
	_mav_put_int16_t(buf, 2, temperature);
	_mav_put_uint16_t(buf, 4, depth);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WATER_RAW, buf, 6, 90);
#else
	mavlink_water_raw_t packet;
	packet.speed = speed;
	packet.temperature = temperature;
	packet.depth = depth;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WATER_RAW, (const char *)&packet, 6, 90);
#endif
}

#endif

// MESSAGE WATER_RAW UNPACKING


/**
 * @brief Get field speed from water_raw message
 *
 * @return Water speed in mm/s.
 */
static inline int16_t mavlink_msg_water_raw_get_speed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field temperature from water_raw message
 *
 * @return Water temperature in centidegrees Celsius.
 */
static inline int16_t mavlink_msg_water_raw_get_temperature(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Get field depth from water_raw message
 *
 * @return Water depth in centimeters (DST800 range is 100m). Invalid measurements are 0xFFFF.
 */
static inline uint16_t mavlink_msg_water_raw_get_depth(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Decode a water_raw message into a struct
 *
 * @param msg The message to decode
 * @param water_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_water_raw_decode(const mavlink_message_t* msg, mavlink_water_raw_t* water_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	water_raw->speed = mavlink_msg_water_raw_get_speed(msg);
	water_raw->temperature = mavlink_msg_water_raw_get_temperature(msg);
	water_raw->depth = mavlink_msg_water_raw_get_depth(msg);
#else
	memcpy(water_raw, _MAV_PAYLOAD(msg), 6);
#endif
}
