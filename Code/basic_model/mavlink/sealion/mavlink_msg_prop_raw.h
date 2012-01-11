// MESSAGE PROP_RAW PACKING

#define MAVLINK_MSG_ID_PROP_RAW 152

typedef struct __mavlink_prop_raw_t
{
 int16_t speed; ///< Propeller speed, positive values mean the vessel will be propelled forward. Units are in RPM.
} mavlink_prop_raw_t;

#define MAVLINK_MSG_ID_PROP_RAW_LEN 2
#define MAVLINK_MSG_ID_152_LEN 2



#define MAVLINK_MESSAGE_INFO_PROP_RAW { \
	"PROP_RAW", \
	1, \
	{  { "speed", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_prop_raw_t, speed) }, \
         } \
}


/**
 * @brief Pack a prop_raw message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param speed Propeller speed, positive values mean the vessel will be propelled forward. Units are in RPM.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_prop_raw_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_int16_t(buf, 0, speed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 2);
#else
	mavlink_prop_raw_t packet;
	packet.speed = speed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 2);
#endif

	msg->msgid = MAVLINK_MSG_ID_PROP_RAW;
	return mavlink_finalize_message(msg, system_id, component_id, 2, 210);
}

/**
 * @brief Pack a prop_raw message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param speed Propeller speed, positive values mean the vessel will be propelled forward. Units are in RPM.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_prop_raw_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_int16_t(buf, 0, speed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 2);
#else
	mavlink_prop_raw_t packet;
	packet.speed = speed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 2);
#endif

	msg->msgid = MAVLINK_MSG_ID_PROP_RAW;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 2, 210);
}

/**
 * @brief Encode a prop_raw struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param prop_raw C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_prop_raw_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_prop_raw_t* prop_raw)
{
	return mavlink_msg_prop_raw_pack(system_id, component_id, msg, prop_raw->speed);
}

/**
 * @brief Send a prop_raw message
 * @param chan MAVLink channel to send the message
 *
 * @param speed Propeller speed, positive values mean the vessel will be propelled forward. Units are in RPM.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_prop_raw_send(mavlink_channel_t chan, int16_t speed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_int16_t(buf, 0, speed);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROP_RAW, buf, 2, 210);
#else
	mavlink_prop_raw_t packet;
	packet.speed = speed;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PROP_RAW, (const char *)&packet, 2, 210);
#endif
}

#endif

// MESSAGE PROP_RAW UNPACKING


/**
 * @brief Get field speed from prop_raw message
 *
 * @return Propeller speed, positive values mean the vessel will be propelled forward. Units are in RPM.
 */
static inline int16_t mavlink_msg_prop_raw_get_speed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Decode a prop_raw message into a struct
 *
 * @param msg The message to decode
 * @param prop_raw C-struct to decode the message contents into
 */
static inline void mavlink_msg_prop_raw_decode(const mavlink_message_t* msg, mavlink_prop_raw_t* prop_raw)
{
#if MAVLINK_NEED_BYTE_SWAP
	prop_raw->speed = mavlink_msg_prop_raw_get_speed(msg);
#else
	memcpy(prop_raw, _MAV_PAYLOAD(msg), 2);
#endif
}
