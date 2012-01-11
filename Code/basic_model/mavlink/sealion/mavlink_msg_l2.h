// MESSAGE L2 PACKING

#define MAVLINK_MSG_ID_L2 172

typedef struct __mavlink_l2_t
{
 uint16_t length; ///< Length of the L2 vector in mm.
 uint16_t direction; ///< The direction that the L2 vector points. Units are milliradians eastward from north.
} mavlink_l2_t;

#define MAVLINK_MSG_ID_L2_LEN 4
#define MAVLINK_MSG_ID_172_LEN 4



#define MAVLINK_MESSAGE_INFO_L2 { \
	"L2", \
	2, \
	{  { "length", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_l2_t, length) }, \
         { "direction", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_l2_t, direction) }, \
         } \
}


/**
 * @brief Pack a l2 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param length Length of the L2 vector in mm.
 * @param direction The direction that the L2 vector points. Units are milliradians eastward from north.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_l2_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t length, uint16_t direction)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, length);
	_mav_put_uint16_t(buf, 2, direction);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 4);
#else
	mavlink_l2_t packet;
	packet.length = length;
	packet.direction = direction;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_L2;
	return mavlink_finalize_message(msg, system_id, component_id, 4, 228);
}

/**
 * @brief Pack a l2 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param length Length of the L2 vector in mm.
 * @param direction The direction that the L2 vector points. Units are milliradians eastward from north.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_l2_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t length,uint16_t direction)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, length);
	_mav_put_uint16_t(buf, 2, direction);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 4);
#else
	mavlink_l2_t packet;
	packet.length = length;
	packet.direction = direction;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_L2;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4, 228);
}

/**
 * @brief Encode a l2 struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param l2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_l2_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_l2_t* l2)
{
	return mavlink_msg_l2_pack(system_id, component_id, msg, l2->length, l2->direction);
}

/**
 * @brief Send a l2 message
 * @param chan MAVLink channel to send the message
 *
 * @param length Length of the L2 vector in mm.
 * @param direction The direction that the L2 vector points. Units are milliradians eastward from north.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_l2_send(mavlink_channel_t chan, uint16_t length, uint16_t direction)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, length);
	_mav_put_uint16_t(buf, 2, direction);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_L2, buf, 4, 228);
#else
	mavlink_l2_t packet;
	packet.length = length;
	packet.direction = direction;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_L2, (const char *)&packet, 4, 228);
#endif
}

#endif

// MESSAGE L2 UNPACKING


/**
 * @brief Get field length from l2 message
 *
 * @return Length of the L2 vector in mm.
 */
static inline uint16_t mavlink_msg_l2_get_length(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field direction from l2 message
 *
 * @return The direction that the L2 vector points. Units are milliradians eastward from north.
 */
static inline uint16_t mavlink_msg_l2_get_direction(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Decode a l2 message into a struct
 *
 * @param msg The message to decode
 * @param l2 C-struct to decode the message contents into
 */
static inline void mavlink_msg_l2_decode(const mavlink_message_t* msg, mavlink_l2_t* l2)
{
#if MAVLINK_NEED_BYTE_SWAP
	l2->length = mavlink_msg_l2_get_length(msg);
	l2->direction = mavlink_msg_l2_get_direction(msg);
#else
	memcpy(l2, _MAV_PAYLOAD(msg), 4);
#endif
}
