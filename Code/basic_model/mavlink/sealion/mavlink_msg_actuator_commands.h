// MESSAGE ACTUATOR_COMMANDS PACKING

#define MAVLINK_MSG_ID_ACTUATOR_COMMANDS 171

typedef struct __mavlink_actuator_commands_t
{
 int16_t rudder_angle; ///< Commanded rudder angle in milliradians where positive indicates port-side.
 int16_t throttle; ///< Commanded throttle speed in units of 1/1023*100% of max current and positive values propel the vehicle forward.
} mavlink_actuator_commands_t;

#define MAVLINK_MSG_ID_ACTUATOR_COMMANDS_LEN 4
#define MAVLINK_MSG_ID_171_LEN 4



#define MAVLINK_MESSAGE_INFO_ACTUATOR_COMMANDS { \
	"ACTUATOR_COMMANDS", \
	2, \
	{  { "rudder_angle", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_actuator_commands_t, rudder_angle) }, \
         { "throttle", NULL, MAVLINK_TYPE_INT16_T, 0, 2, offsetof(mavlink_actuator_commands_t, throttle) }, \
         } \
}


/**
 * @brief Pack a actuator_commands message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param rudder_angle Commanded rudder angle in milliradians where positive indicates port-side.
 * @param throttle Commanded throttle speed in units of 1/1023*100% of max current and positive values propel the vehicle forward.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_actuator_commands_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t rudder_angle, int16_t throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_int16_t(buf, 0, rudder_angle);
	_mav_put_int16_t(buf, 2, throttle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 4);
#else
	mavlink_actuator_commands_t packet;
	packet.rudder_angle = rudder_angle;
	packet.throttle = throttle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_ACTUATOR_COMMANDS;
	return mavlink_finalize_message(msg, system_id, component_id, 4, 94);
}

/**
 * @brief Pack a actuator_commands message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param rudder_angle Commanded rudder angle in milliradians where positive indicates port-side.
 * @param throttle Commanded throttle speed in units of 1/1023*100% of max current and positive values propel the vehicle forward.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_actuator_commands_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t rudder_angle,int16_t throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_int16_t(buf, 0, rudder_angle);
	_mav_put_int16_t(buf, 2, throttle);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 4);
#else
	mavlink_actuator_commands_t packet;
	packet.rudder_angle = rudder_angle;
	packet.throttle = throttle;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_ACTUATOR_COMMANDS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4, 94);
}

/**
 * @brief Encode a actuator_commands struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param actuator_commands C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_actuator_commands_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_actuator_commands_t* actuator_commands)
{
	return mavlink_msg_actuator_commands_pack(system_id, component_id, msg, actuator_commands->rudder_angle, actuator_commands->throttle);
}

/**
 * @brief Send a actuator_commands message
 * @param chan MAVLink channel to send the message
 *
 * @param rudder_angle Commanded rudder angle in milliradians where positive indicates port-side.
 * @param throttle Commanded throttle speed in units of 1/1023*100% of max current and positive values propel the vehicle forward.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_actuator_commands_send(mavlink_channel_t chan, int16_t rudder_angle, int16_t throttle)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_int16_t(buf, 0, rudder_angle);
	_mav_put_int16_t(buf, 2, throttle);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_COMMANDS, buf, 4, 94);
#else
	mavlink_actuator_commands_t packet;
	packet.rudder_angle = rudder_angle;
	packet.throttle = throttle;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACTUATOR_COMMANDS, (const char *)&packet, 4, 94);
#endif
}

#endif

// MESSAGE ACTUATOR_COMMANDS UNPACKING


/**
 * @brief Get field rudder_angle from actuator_commands message
 *
 * @return Commanded rudder angle in milliradians where positive indicates port-side.
 */
static inline int16_t mavlink_msg_actuator_commands_get_rudder_angle(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field throttle from actuator_commands message
 *
 * @return Commanded throttle speed in units of 1/1023*100% of max current and positive values propel the vehicle forward.
 */
static inline int16_t mavlink_msg_actuator_commands_get_throttle(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  2);
}

/**
 * @brief Decode a actuator_commands message into a struct
 *
 * @param msg The message to decode
 * @param actuator_commands C-struct to decode the message contents into
 */
static inline void mavlink_msg_actuator_commands_decode(const mavlink_message_t* msg, mavlink_actuator_commands_t* actuator_commands)
{
#if MAVLINK_NEED_BYTE_SWAP
	actuator_commands->rudder_angle = mavlink_msg_actuator_commands_get_rudder_angle(msg);
	actuator_commands->throttle = mavlink_msg_actuator_commands_get_throttle(msg);
#else
	memcpy(actuator_commands, _MAV_PAYLOAD(msg), 4);
#endif
}
