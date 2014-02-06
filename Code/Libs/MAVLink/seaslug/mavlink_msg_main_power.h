// MESSAGE MAIN_POWER PACKING

#define MAVLINK_MSG_ID_MAIN_POWER 172

typedef struct __mavlink_main_power_t
{
 uint16_t voltage; ///< The current main battery rail voltage. In units of 0.01V.
 uint16_t current_draw; ///< The current being drawn from the main battery bank. In units of 0.1A.
} mavlink_main_power_t;

#define MAVLINK_MSG_ID_MAIN_POWER_LEN 4
#define MAVLINK_MSG_ID_172_LEN 4

#define MAVLINK_MSG_ID_MAIN_POWER_CRC 171
#define MAVLINK_MSG_ID_172_CRC 171



#define MAVLINK_MESSAGE_INFO_MAIN_POWER { \
	"MAIN_POWER", \
	2, \
	{  { "voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_main_power_t, voltage) }, \
         { "current_draw", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_main_power_t, current_draw) }, \
         } \
}


/**
 * @brief Pack a main_power message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param voltage The current main battery rail voltage. In units of 0.01V.
 * @param current_draw The current being drawn from the main battery bank. In units of 0.1A.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_main_power_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t voltage, uint16_t current_draw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAIN_POWER_LEN];
	_mav_put_uint16_t(buf, 0, voltage);
	_mav_put_uint16_t(buf, 2, current_draw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAIN_POWER_LEN);
#else
	mavlink_main_power_t packet;
	packet.voltage = voltage;
	packet.current_draw = current_draw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAIN_POWER_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAIN_POWER;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAIN_POWER_LEN, MAVLINK_MSG_ID_MAIN_POWER_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_MAIN_POWER_LEN);
#endif
}

/**
 * @brief Pack a main_power message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param voltage The current main battery rail voltage. In units of 0.01V.
 * @param current_draw The current being drawn from the main battery bank. In units of 0.1A.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_main_power_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t voltage,uint16_t current_draw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAIN_POWER_LEN];
	_mav_put_uint16_t(buf, 0, voltage);
	_mav_put_uint16_t(buf, 2, current_draw);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAIN_POWER_LEN);
#else
	mavlink_main_power_t packet;
	packet.voltage = voltage;
	packet.current_draw = current_draw;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_MAIN_POWER_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_MAIN_POWER;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAIN_POWER_LEN, MAVLINK_MSG_ID_MAIN_POWER_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_MAIN_POWER_LEN);
#endif
}

/**
 * @brief Encode a main_power struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param main_power C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_main_power_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_main_power_t* main_power)
{
	return mavlink_msg_main_power_pack(system_id, component_id, msg, main_power->voltage, main_power->current_draw);
}

/**
 * @brief Encode a main_power struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param main_power C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_main_power_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_main_power_t* main_power)
{
	return mavlink_msg_main_power_pack_chan(system_id, component_id, chan, msg, main_power->voltage, main_power->current_draw);
}

/**
 * @brief Send a main_power message
 * @param chan MAVLink channel to send the message
 *
 * @param voltage The current main battery rail voltage. In units of 0.01V.
 * @param current_draw The current being drawn from the main battery bank. In units of 0.1A.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_main_power_send(mavlink_channel_t chan, uint16_t voltage, uint16_t current_draw)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAIN_POWER_LEN];
	_mav_put_uint16_t(buf, 0, voltage);
	_mav_put_uint16_t(buf, 2, current_draw);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAIN_POWER, buf, MAVLINK_MSG_ID_MAIN_POWER_LEN, MAVLINK_MSG_ID_MAIN_POWER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAIN_POWER, buf, MAVLINK_MSG_ID_MAIN_POWER_LEN);
#endif
#else
	mavlink_main_power_t packet;
	packet.voltage = voltage;
	packet.current_draw = current_draw;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAIN_POWER, (const char *)&packet, MAVLINK_MSG_ID_MAIN_POWER_LEN, MAVLINK_MSG_ID_MAIN_POWER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAIN_POWER, (const char *)&packet, MAVLINK_MSG_ID_MAIN_POWER_LEN);
#endif
#endif
}

#endif

// MESSAGE MAIN_POWER UNPACKING


/**
 * @brief Get field voltage from main_power message
 *
 * @return The current main battery rail voltage. In units of 0.01V.
 */
static inline uint16_t mavlink_msg_main_power_get_voltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field current_draw from main_power message
 *
 * @return The current being drawn from the main battery bank. In units of 0.1A.
 */
static inline uint16_t mavlink_msg_main_power_get_current_draw(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Decode a main_power message into a struct
 *
 * @param msg The message to decode
 * @param main_power C-struct to decode the message contents into
 */
static inline void mavlink_msg_main_power_decode(const mavlink_message_t* msg, mavlink_main_power_t* main_power)
{
#if MAVLINK_NEED_BYTE_SWAP
	main_power->voltage = mavlink_msg_main_power_get_voltage(msg);
	main_power->current_draw = mavlink_msg_main_power_get_current_draw(msg);
#else
	memcpy(main_power, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MAIN_POWER_LEN);
#endif
}
