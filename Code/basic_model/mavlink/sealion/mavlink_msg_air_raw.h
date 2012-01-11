// MESSAGE AIR_RAW PACKING

#define MAVLINK_MSG_ID_AIR_RAW 161

typedef struct __mavlink_air_raw_t
{
 int16_t temperature; ///< Air temperature in cdegrees Celsius.
 uint16_t pressure; ///< Air pressure in hectorPascals.
 uint16_t humidity; ///< Air humidity in units of .0004%.
} mavlink_air_raw_t;

#define MAVLINK_MSG_ID_AIR_RAW_LEN 6
#define MAVLINK_MSG_ID_161_LEN 6



#define MAVLINK_MESSAGE_INFO_AIR_RAW { \
	"AIR_RAW", \
	3, \
	{  { "temperature", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_air_raw_t, temperature) }, \
         { "pressure", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_air_raw_t, pressure) }, \
         { "humidity", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_air_raw_t, humidity) }, \
         } \
}


/**
 * @brief Pack a air_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param temperature Air temperature in cdegrees Celsius.
 * @param pressure Air pressure in hectorPascals.
 * @param humidity Air humidity in units of .0004%.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_air_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t temperature, uint16_t pressure, uint16_t humidity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_int16_t(buf, 0, temperature);
	_mav_put_uint16_t(buf, 2, pressure);
	_mav_put_uint16_t(buf, 4, humidity);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 6);
#else
	mavlink_air_raw_t packet;
	packet.temperature = temperature;
	packet.pressure = pressure;
	packet.humidity = humidity;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 6);
#endif

	msg->msgid = MAVLINK_MSG_ID_AIR_RAW;
	return mavlink_finalize_message(msg, system_id, component_id, 6, 77);
}

/**
 * @brief Pack a air_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param temperature Air temperature in cdegrees Celsius.
 * @param pressure Air pressure in hectorPascals.
 * @param humidity Air humidity in units of .0004%.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_air_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t temperature,uint16_t pressure,uint16_t humidity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_int16_t(buf, 0, temperature);
	_mav_put_uint16_t(buf, 2, pressure);
	_mav_put_uint16_t(buf, 4, humidity);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 6);
#else
	mavlink_air_raw_t packet;
	packet.temperature = temperature;
	packet.pressure = pressure;
	packet.humidity = humidity;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 6);
#endif

	msg->msgid = MAVLINK_MSG_ID_AIR_RAW;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 6, 77);
}

/**
 * @brief Encode a air_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param air_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_air_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_air_raw_t* air_raw)
{
	return mavlink_msg_air_raw_pack(system_id, component_id, msg, air_raw->temperature, air_raw->pressure, air_raw->humidity);
}

/**
 * @brief Send a air_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param temperature Air temperature in cdegrees Celsius.
 * @param pressure Air pressure in hectorPascals.
 * @param humidity Air humidity in units of .0004%.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_air_raw_send(mavlink_channel_t chan, int16_t temperature, uint16_t pressure, uint16_t humidity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[6];
	_mav_put_int16_t(buf, 0, temperature);
	_mav_put_uint16_t(buf, 2, pressure);
	_mav_put_uint16_t(buf, 4, humidity);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIR_RAW, buf, 6, 77);
#else
	mavlink_air_raw_t packet;
	packet.temperature = temperature;
	packet.pressure = pressure;
	packet.humidity = humidity;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_AIR_RAW, (const char *)&packet, 6, 77);
#endif
}

#endif

// MESSAGE AIR_RAW UNPACKING


/**
 * @brief Get field temperature from air_raw message
 *
 * @return Air temperature in cdegrees Celsius.
 */
static inline int16_t mavlink_msg_air_raw_get_temperature(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field pressure from air_raw message
 *
 * @return Air pressure in hectorPascals.
 */
static inline uint16_t mavlink_msg_air_raw_get_pressure(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field humidity from air_raw message
 *
 * @return Air humidity in units of .0004%.
 */
static inline uint16_t mavlink_msg_air_raw_get_humidity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Decode a air_raw message into a struct
 *
 * @param msg The message to decode
 * @param air_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_air_raw_decode(const mavlink_message_t* msg, mavlink_air_raw_t* air_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	air_raw->temperature = mavlink_msg_air_raw_get_temperature(msg);
	air_raw->pressure = mavlink_msg_air_raw_get_pressure(msg);
	air_raw->humidity = mavlink_msg_air_raw_get_humidity(msg);
#else
	memcpy(air_raw, _MAV_PAYLOAD(msg), 6);
#endif
}
