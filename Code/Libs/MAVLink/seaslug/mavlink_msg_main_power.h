// MESSAGE MAIN_POWER PACKING

#define MAVLINK_MSG_ID_MAIN_POWER 172

typedef struct __mavlink_main_power_t
{
 uint16_t electronics_voltage; ///< The voltage for the electronics battery bank. In units of 0.001V.
 uint16_t electronics_current; ///< The current being drawn from the electronics battery bank. In units of 0.001A.
 uint16_t actuator_voltage; ///< The voltage of the actuator battery bank. In units of 0.001V.
 uint16_t actuator_current; ///< The current being drawn from the main battery bank. In units of 0.001A.
} mavlink_main_power_t;

#define MAVLINK_MSG_ID_MAIN_POWER_LEN 8
#define MAVLINK_MSG_ID_172_LEN 8

#define MAVLINK_MSG_ID_MAIN_POWER_CRC 43
#define MAVLINK_MSG_ID_172_CRC 43



#define MAVLINK_MESSAGE_INFO_MAIN_POWER { \
	"MAIN_POWER", \
	4, \
	{  { "electronics_voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_main_power_t, electronics_voltage) }, \
         { "electronics_current", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_main_power_t, electronics_current) }, \
         { "actuator_voltage", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_main_power_t, actuator_voltage) }, \
         { "actuator_current", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_main_power_t, actuator_current) }, \
         } \
}


/**
 * @brief Pack a main_power message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param electronics_voltage The voltage for the electronics battery bank. In units of 0.001V.
 * @param electronics_current The current being drawn from the electronics battery bank. In units of 0.001A.
 * @param actuator_voltage The voltage of the actuator battery bank. In units of 0.001V.
 * @param actuator_current The current being drawn from the main battery bank. In units of 0.001A.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_main_power_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t electronics_voltage, uint16_t electronics_current, uint16_t actuator_voltage, uint16_t actuator_current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAIN_POWER_LEN];
	_mav_put_uint16_t(buf, 0, electronics_voltage);
	_mav_put_uint16_t(buf, 2, electronics_current);
	_mav_put_uint16_t(buf, 4, actuator_voltage);
	_mav_put_uint16_t(buf, 6, actuator_current);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAIN_POWER_LEN);
#else
	mavlink_main_power_t packet;
	packet.electronics_voltage = electronics_voltage;
	packet.electronics_current = electronics_current;
	packet.actuator_voltage = actuator_voltage;
	packet.actuator_current = actuator_current;

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
 * @param electronics_voltage The voltage for the electronics battery bank. In units of 0.001V.
 * @param electronics_current The current being drawn from the electronics battery bank. In units of 0.001A.
 * @param actuator_voltage The voltage of the actuator battery bank. In units of 0.001V.
 * @param actuator_current The current being drawn from the main battery bank. In units of 0.001A.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_main_power_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t electronics_voltage,uint16_t electronics_current,uint16_t actuator_voltage,uint16_t actuator_current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAIN_POWER_LEN];
	_mav_put_uint16_t(buf, 0, electronics_voltage);
	_mav_put_uint16_t(buf, 2, electronics_current);
	_mav_put_uint16_t(buf, 4, actuator_voltage);
	_mav_put_uint16_t(buf, 6, actuator_current);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_MAIN_POWER_LEN);
#else
	mavlink_main_power_t packet;
	packet.electronics_voltage = electronics_voltage;
	packet.electronics_current = electronics_current;
	packet.actuator_voltage = actuator_voltage;
	packet.actuator_current = actuator_current;

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
	return mavlink_msg_main_power_pack(system_id, component_id, msg, main_power->electronics_voltage, main_power->electronics_current, main_power->actuator_voltage, main_power->actuator_current);
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
	return mavlink_msg_main_power_pack_chan(system_id, component_id, chan, msg, main_power->electronics_voltage, main_power->electronics_current, main_power->actuator_voltage, main_power->actuator_current);
}

/**
 * @brief Send a main_power message
 * @param chan MAVLink channel to send the message
 *
 * @param electronics_voltage The voltage for the electronics battery bank. In units of 0.001V.
 * @param electronics_current The current being drawn from the electronics battery bank. In units of 0.001A.
 * @param actuator_voltage The voltage of the actuator battery bank. In units of 0.001V.
 * @param actuator_current The current being drawn from the main battery bank. In units of 0.001A.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_main_power_send(mavlink_channel_t chan, uint16_t electronics_voltage, uint16_t electronics_current, uint16_t actuator_voltage, uint16_t actuator_current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_MAIN_POWER_LEN];
	_mav_put_uint16_t(buf, 0, electronics_voltage);
	_mav_put_uint16_t(buf, 2, electronics_current);
	_mav_put_uint16_t(buf, 4, actuator_voltage);
	_mav_put_uint16_t(buf, 6, actuator_current);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAIN_POWER, buf, MAVLINK_MSG_ID_MAIN_POWER_LEN, MAVLINK_MSG_ID_MAIN_POWER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAIN_POWER, buf, MAVLINK_MSG_ID_MAIN_POWER_LEN);
#endif
#else
	mavlink_main_power_t packet;
	packet.electronics_voltage = electronics_voltage;
	packet.electronics_current = electronics_current;
	packet.actuator_voltage = actuator_voltage;
	packet.actuator_current = actuator_current;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAIN_POWER, (const char *)&packet, MAVLINK_MSG_ID_MAIN_POWER_LEN, MAVLINK_MSG_ID_MAIN_POWER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAIN_POWER, (const char *)&packet, MAVLINK_MSG_ID_MAIN_POWER_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_MAIN_POWER_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_main_power_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t electronics_voltage, uint16_t electronics_current, uint16_t actuator_voltage, uint16_t actuator_current)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, electronics_voltage);
	_mav_put_uint16_t(buf, 2, electronics_current);
	_mav_put_uint16_t(buf, 4, actuator_voltage);
	_mav_put_uint16_t(buf, 6, actuator_current);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAIN_POWER, buf, MAVLINK_MSG_ID_MAIN_POWER_LEN, MAVLINK_MSG_ID_MAIN_POWER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAIN_POWER, buf, MAVLINK_MSG_ID_MAIN_POWER_LEN);
#endif
#else
	mavlink_main_power_t *packet = (mavlink_main_power_t *)msgbuf;
	packet->electronics_voltage = electronics_voltage;
	packet->electronics_current = electronics_current;
	packet->actuator_voltage = actuator_voltage;
	packet->actuator_current = actuator_current;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAIN_POWER, (const char *)packet, MAVLINK_MSG_ID_MAIN_POWER_LEN, MAVLINK_MSG_ID_MAIN_POWER_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_MAIN_POWER, (const char *)packet, MAVLINK_MSG_ID_MAIN_POWER_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE MAIN_POWER UNPACKING


/**
 * @brief Get field electronics_voltage from main_power message
 *
 * @return The voltage for the electronics battery bank. In units of 0.001V.
 */
static inline uint16_t mavlink_msg_main_power_get_electronics_voltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field electronics_current from main_power message
 *
 * @return The current being drawn from the electronics battery bank. In units of 0.001A.
 */
static inline uint16_t mavlink_msg_main_power_get_electronics_current(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field actuator_voltage from main_power message
 *
 * @return The voltage of the actuator battery bank. In units of 0.001V.
 */
static inline uint16_t mavlink_msg_main_power_get_actuator_voltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field actuator_current from main_power message
 *
 * @return The current being drawn from the main battery bank. In units of 0.001A.
 */
static inline uint16_t mavlink_msg_main_power_get_actuator_current(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
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
	main_power->electronics_voltage = mavlink_msg_main_power_get_electronics_voltage(msg);
	main_power->electronics_current = mavlink_msg_main_power_get_electronics_current(msg);
	main_power->actuator_voltage = mavlink_msg_main_power_get_actuator_voltage(msg);
	main_power->actuator_current = mavlink_msg_main_power_get_actuator_current(msg);
#else
	memcpy(main_power, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_MAIN_POWER_LEN);
#endif
}
