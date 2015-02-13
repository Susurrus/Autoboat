// MESSAGE PARAM_VALUE_WITH_TIME PACKING

#define MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME 182

typedef struct __mavlink_param_value_with_time_t
{
 uint32_t time_boot_ms; ///< Timestamp (milliseconds since system boot)
 float param_value; ///< Onboard parameter value
 uint16_t param_count; ///< Total number of onboard parameters
 uint16_t param_index; ///< Index of this onboard parameter
 char param_id[16]; ///< Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 uint8_t param_type; ///< Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
} mavlink_param_value_with_time_t;

#define MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN 29
#define MAVLINK_MSG_ID_182_LEN 29

#define MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_CRC 189
#define MAVLINK_MSG_ID_182_CRC 189

#define MAVLINK_MSG_PARAM_VALUE_WITH_TIME_FIELD_PARAM_ID_LEN 16

#define MAVLINK_MESSAGE_INFO_PARAM_VALUE_WITH_TIME { \
	"PARAM_VALUE_WITH_TIME", \
	6, \
	{  { "time_boot_ms", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_param_value_with_time_t, time_boot_ms) }, \
         { "param_value", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_param_value_with_time_t, param_value) }, \
         { "param_count", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_param_value_with_time_t, param_count) }, \
         { "param_index", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_param_value_with_time_t, param_index) }, \
         { "param_id", NULL, MAVLINK_TYPE_CHAR, 16, 12, offsetof(mavlink_param_value_with_time_t, param_id) }, \
         { "param_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_param_value_with_time_t, param_type) }, \
         } \
}


/**
 * @brief Pack a param_value_with_time message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param param_id Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param param_value Onboard parameter value
 * @param param_type Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
 * @param param_count Total number of onboard parameters
 * @param param_index Index of this onboard parameter
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_value_with_time_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t time_boot_ms, const char *param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, param_value);
	_mav_put_uint16_t(buf, 8, param_count);
	_mav_put_uint16_t(buf, 10, param_index);
	_mav_put_uint8_t(buf, 28, param_type);
	_mav_put_char_array(buf, 12, param_id, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN);
#else
	mavlink_param_value_with_time_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.param_value = param_value;
	packet.param_count = param_count;
	packet.param_index = param_index;
	packet.param_type = param_type;
	mav_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN);
#endif
}

/**
 * @brief Pack a param_value_with_time message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param param_id Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param param_value Onboard parameter value
 * @param param_type Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
 * @param param_count Total number of onboard parameters
 * @param param_index Index of this onboard parameter
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_param_value_with_time_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t time_boot_ms,const char *param_id,float param_value,uint8_t param_type,uint16_t param_count,uint16_t param_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, param_value);
	_mav_put_uint16_t(buf, 8, param_count);
	_mav_put_uint16_t(buf, 10, param_index);
	_mav_put_uint8_t(buf, 28, param_type);
	_mav_put_char_array(buf, 12, param_id, 16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN);
#else
	mavlink_param_value_with_time_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.param_value = param_value;
	packet.param_count = param_count;
	packet.param_index = param_index;
	packet.param_type = param_type;
	mav_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN);
#endif
}

/**
 * @brief Encode a param_value_with_time struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param param_value_with_time C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_param_value_with_time_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_param_value_with_time_t* param_value_with_time)
{
	return mavlink_msg_param_value_with_time_pack(system_id, component_id, msg, param_value_with_time->time_boot_ms, param_value_with_time->param_id, param_value_with_time->param_value, param_value_with_time->param_type, param_value_with_time->param_count, param_value_with_time->param_index);
}

/**
 * @brief Encode a param_value_with_time struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param param_value_with_time C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_param_value_with_time_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_param_value_with_time_t* param_value_with_time)
{
	return mavlink_msg_param_value_with_time_pack_chan(system_id, component_id, chan, msg, param_value_with_time->time_boot_ms, param_value_with_time->param_id, param_value_with_time->param_value, param_value_with_time->param_type, param_value_with_time->param_count, param_value_with_time->param_index);
}

/**
 * @brief Send a param_value_with_time message
 * @param chan MAVLink channel to send the message
 *
 * @param time_boot_ms Timestamp (milliseconds since system boot)
 * @param param_id Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 * @param param_value Onboard parameter value
 * @param param_type Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
 * @param param_count Total number of onboard parameters
 * @param param_index Index of this onboard parameter
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_param_value_with_time_send(mavlink_channel_t chan, uint32_t time_boot_ms, const char *param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN];
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, param_value);
	_mav_put_uint16_t(buf, 8, param_count);
	_mav_put_uint16_t(buf, 10, param_index);
	_mav_put_uint8_t(buf, 28, param_type);
	_mav_put_char_array(buf, 12, param_id, 16);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME, buf, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME, buf, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN);
#endif
#else
	mavlink_param_value_with_time_t packet;
	packet.time_boot_ms = time_boot_ms;
	packet.param_value = param_value;
	packet.param_count = param_count;
	packet.param_index = param_index;
	packet.param_type = param_type;
	mav_array_memcpy(packet.param_id, param_id, sizeof(char)*16);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME, (const char *)&packet, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME, (const char *)&packet, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_param_value_with_time_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t time_boot_ms, const char *param_id, float param_value, uint8_t param_type, uint16_t param_count, uint16_t param_index)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, time_boot_ms);
	_mav_put_float(buf, 4, param_value);
	_mav_put_uint16_t(buf, 8, param_count);
	_mav_put_uint16_t(buf, 10, param_index);
	_mav_put_uint8_t(buf, 28, param_type);
	_mav_put_char_array(buf, 12, param_id, 16);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME, buf, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME, buf, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN);
#endif
#else
	mavlink_param_value_with_time_t *packet = (mavlink_param_value_with_time_t *)msgbuf;
	packet->time_boot_ms = time_boot_ms;
	packet->param_value = param_value;
	packet->param_count = param_count;
	packet->param_index = param_index;
	packet->param_type = param_type;
	mav_array_memcpy(packet->param_id, param_id, sizeof(char)*16);
#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME, (const char *)packet, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME, (const char *)packet, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE PARAM_VALUE_WITH_TIME UNPACKING


/**
 * @brief Get field time_boot_ms from param_value_with_time message
 *
 * @return Timestamp (milliseconds since system boot)
 */
static inline uint32_t mavlink_msg_param_value_with_time_get_time_boot_ms(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Get field param_id from param_value_with_time message
 *
 * @return Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
 */
static inline uint16_t mavlink_msg_param_value_with_time_get_param_id(const mavlink_message_t* msg, char *param_id)
{
	return _MAV_RETURN_char_array(msg, param_id, 16,  12);
}

/**
 * @brief Get field param_value from param_value_with_time message
 *
 * @return Onboard parameter value
 */
static inline float mavlink_msg_param_value_with_time_get_param_value(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field param_type from param_value_with_time message
 *
 * @return Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
 */
static inline uint8_t mavlink_msg_param_value_with_time_get_param_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field param_count from param_value_with_time message
 *
 * @return Total number of onboard parameters
 */
static inline uint16_t mavlink_msg_param_value_with_time_get_param_count(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field param_index from param_value_with_time message
 *
 * @return Index of this onboard parameter
 */
static inline uint16_t mavlink_msg_param_value_with_time_get_param_index(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Decode a param_value_with_time message into a struct
 *
 * @param msg The message to decode
 * @param param_value_with_time C-struct to decode the message contents into
 */
static inline void mavlink_msg_param_value_with_time_decode(const mavlink_message_t* msg, mavlink_param_value_with_time_t* param_value_with_time)
{
#if MAVLINK_NEED_BYTE_SWAP
	param_value_with_time->time_boot_ms = mavlink_msg_param_value_with_time_get_time_boot_ms(msg);
	param_value_with_time->param_value = mavlink_msg_param_value_with_time_get_param_value(msg);
	param_value_with_time->param_count = mavlink_msg_param_value_with_time_get_param_count(msg);
	param_value_with_time->param_index = mavlink_msg_param_value_with_time_get_param_index(msg);
	mavlink_msg_param_value_with_time_get_param_id(msg, param_value_with_time->param_id);
	param_value_with_time->param_type = mavlink_msg_param_value_with_time_get_param_type(msg);
#else
	memcpy(param_value_with_time, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME_LEN);
#endif
}
