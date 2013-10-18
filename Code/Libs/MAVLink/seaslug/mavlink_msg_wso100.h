// MESSAGE WSO100 PACKING

#define MAVLINK_MSG_ID_WSO100 160

typedef struct __mavlink_wso100_t
{
 float speed; ///< Wind speed in m/s.
 float direction; ///< Wind direction in rads east from north.
 float temperature; ///< Air temperature in degrees Celsius.
 float pressure; ///< Air pressure in Pascals.
 float humidity; ///< Air humidity in %.
} mavlink_wso100_t;

#define MAVLINK_MSG_ID_WSO100_LEN 20
#define MAVLINK_MSG_ID_160_LEN 20

#define MAVLINK_MSG_ID_WSO100_CRC 236
#define MAVLINK_MSG_ID_160_CRC 236



#define MAVLINK_MESSAGE_INFO_WSO100 { \
	"WSO100", \
	5, \
	{  { "speed", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_wso100_t, speed) }, \
         { "direction", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_wso100_t, direction) }, \
         { "temperature", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_wso100_t, temperature) }, \
         { "pressure", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_wso100_t, pressure) }, \
         { "humidity", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_wso100_t, humidity) }, \
         } \
}


/**
 * @brief Pack a wso100 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param speed Wind speed in m/s.
 * @param direction Wind direction in rads east from north.
 * @param temperature Air temperature in degrees Celsius.
 * @param pressure Air pressure in Pascals.
 * @param humidity Air humidity in %.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wso100_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float speed, float direction, float temperature, float pressure, float humidity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_WSO100_LEN];
	_mav_put_float(buf, 0, speed);
	_mav_put_float(buf, 4, direction);
	_mav_put_float(buf, 8, temperature);
	_mav_put_float(buf, 12, pressure);
	_mav_put_float(buf, 16, humidity);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WSO100_LEN);
#else
	mavlink_wso100_t packet;
	packet.speed = speed;
	packet.direction = direction;
	packet.temperature = temperature;
	packet.pressure = pressure;
	packet.humidity = humidity;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WSO100_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_WSO100;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WSO100_LEN, MAVLINK_MSG_ID_WSO100_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WSO100_LEN);
#endif
}

/**
 * @brief Pack a wso100 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param speed Wind speed in m/s.
 * @param direction Wind direction in rads east from north.
 * @param temperature Air temperature in degrees Celsius.
 * @param pressure Air pressure in Pascals.
 * @param humidity Air humidity in %.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_wso100_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float speed,float direction,float temperature,float pressure,float humidity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_WSO100_LEN];
	_mav_put_float(buf, 0, speed);
	_mav_put_float(buf, 4, direction);
	_mav_put_float(buf, 8, temperature);
	_mav_put_float(buf, 12, pressure);
	_mav_put_float(buf, 16, humidity);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WSO100_LEN);
#else
	mavlink_wso100_t packet;
	packet.speed = speed;
	packet.direction = direction;
	packet.temperature = temperature;
	packet.pressure = pressure;
	packet.humidity = humidity;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WSO100_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_WSO100;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WSO100_LEN, MAVLINK_MSG_ID_WSO100_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WSO100_LEN);
#endif
}

/**
 * @brief Encode a wso100 struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param wso100 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_wso100_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_wso100_t* wso100)
{
	return mavlink_msg_wso100_pack(system_id, component_id, msg, wso100->speed, wso100->direction, wso100->temperature, wso100->pressure, wso100->humidity);
}

/**
 * @brief Send a wso100 message
 * @param chan MAVLink channel to send the message
 *
 * @param speed Wind speed in m/s.
 * @param direction Wind direction in rads east from north.
 * @param temperature Air temperature in degrees Celsius.
 * @param pressure Air pressure in Pascals.
 * @param humidity Air humidity in %.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_wso100_send(mavlink_channel_t chan, float speed, float direction, float temperature, float pressure, float humidity)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_WSO100_LEN];
	_mav_put_float(buf, 0, speed);
	_mav_put_float(buf, 4, direction);
	_mav_put_float(buf, 8, temperature);
	_mav_put_float(buf, 12, pressure);
	_mav_put_float(buf, 16, humidity);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WSO100, buf, MAVLINK_MSG_ID_WSO100_LEN, MAVLINK_MSG_ID_WSO100_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WSO100, buf, MAVLINK_MSG_ID_WSO100_LEN);
#endif
#else
	mavlink_wso100_t packet;
	packet.speed = speed;
	packet.direction = direction;
	packet.temperature = temperature;
	packet.pressure = pressure;
	packet.humidity = humidity;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WSO100, (const char *)&packet, MAVLINK_MSG_ID_WSO100_LEN, MAVLINK_MSG_ID_WSO100_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WSO100, (const char *)&packet, MAVLINK_MSG_ID_WSO100_LEN);
#endif
#endif
}

#endif

// MESSAGE WSO100 UNPACKING


/**
 * @brief Get field speed from wso100 message
 *
 * @return Wind speed in m/s.
 */
static inline float mavlink_msg_wso100_get_speed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field direction from wso100 message
 *
 * @return Wind direction in rads east from north.
 */
static inline float mavlink_msg_wso100_get_direction(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field temperature from wso100 message
 *
 * @return Air temperature in degrees Celsius.
 */
static inline float mavlink_msg_wso100_get_temperature(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pressure from wso100 message
 *
 * @return Air pressure in Pascals.
 */
static inline float mavlink_msg_wso100_get_pressure(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field humidity from wso100 message
 *
 * @return Air humidity in %.
 */
static inline float mavlink_msg_wso100_get_humidity(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a wso100 message into a struct
 *
 * @param msg The message to decode
 * @param wso100 C-struct to decode the message contents into
 */
static inline void mavlink_msg_wso100_decode(const mavlink_message_t* msg, mavlink_wso100_t* wso100)
{
#if MAVLINK_NEED_BYTE_SWAP
	wso100->speed = mavlink_msg_wso100_get_speed(msg);
	wso100->direction = mavlink_msg_wso100_get_direction(msg);
	wso100->temperature = mavlink_msg_wso100_get_temperature(msg);
	wso100->pressure = mavlink_msg_wso100_get_pressure(msg);
	wso100->humidity = mavlink_msg_wso100_get_humidity(msg);
#else
	memcpy(wso100, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_WSO100_LEN);
#endif
}
