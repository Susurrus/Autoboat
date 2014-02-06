// MESSAGE GPS200 PACKING

#define MAVLINK_MSG_ID_GPS200 163

typedef struct __mavlink_gps200_t
{
 float magnetic_variation; ///< Magnetic variation for the current GPS location. In units of degrees.
} mavlink_gps200_t;

#define MAVLINK_MSG_ID_GPS200_LEN 4
#define MAVLINK_MSG_ID_163_LEN 4

#define MAVLINK_MSG_ID_GPS200_CRC 61
#define MAVLINK_MSG_ID_163_CRC 61



#define MAVLINK_MESSAGE_INFO_GPS200 { \
	"GPS200", \
	1, \
	{  { "magnetic_variation", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_gps200_t, magnetic_variation) }, \
         } \
}


/**
 * @brief Pack a gps200 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param magnetic_variation Magnetic variation for the current GPS location. In units of degrees.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps200_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float magnetic_variation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GPS200_LEN];
	_mav_put_float(buf, 0, magnetic_variation);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS200_LEN);
#else
	mavlink_gps200_t packet;
	packet.magnetic_variation = magnetic_variation;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS200_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GPS200;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GPS200_LEN, MAVLINK_MSG_ID_GPS200_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_GPS200_LEN);
#endif
}

/**
 * @brief Pack a gps200 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param magnetic_variation Magnetic variation for the current GPS location. In units of degrees.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gps200_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float magnetic_variation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GPS200_LEN];
	_mav_put_float(buf, 0, magnetic_variation);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_GPS200_LEN);
#else
	mavlink_gps200_t packet;
	packet.magnetic_variation = magnetic_variation;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_GPS200_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_GPS200;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GPS200_LEN, MAVLINK_MSG_ID_GPS200_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_GPS200_LEN);
#endif
}

/**
 * @brief Encode a gps200 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gps200 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps200_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gps200_t* gps200)
{
	return mavlink_msg_gps200_pack(system_id, component_id, msg, gps200->magnetic_variation);
}

/**
 * @brief Encode a gps200 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param gps200 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gps200_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_gps200_t* gps200)
{
	return mavlink_msg_gps200_pack_chan(system_id, component_id, chan, msg, gps200->magnetic_variation);
}

/**
 * @brief Send a gps200 message
 * @param chan MAVLink channel to send the message
 *
 * @param magnetic_variation Magnetic variation for the current GPS location. In units of degrees.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gps200_send(mavlink_channel_t chan, float magnetic_variation)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_GPS200_LEN];
	_mav_put_float(buf, 0, magnetic_variation);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS200, buf, MAVLINK_MSG_ID_GPS200_LEN, MAVLINK_MSG_ID_GPS200_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS200, buf, MAVLINK_MSG_ID_GPS200_LEN);
#endif
#else
	mavlink_gps200_t packet;
	packet.magnetic_variation = magnetic_variation;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS200, (const char *)&packet, MAVLINK_MSG_ID_GPS200_LEN, MAVLINK_MSG_ID_GPS200_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GPS200, (const char *)&packet, MAVLINK_MSG_ID_GPS200_LEN);
#endif
#endif
}

#endif

// MESSAGE GPS200 UNPACKING


/**
 * @brief Get field magnetic_variation from gps200 message
 *
 * @return Magnetic variation for the current GPS location. In units of degrees.
 */
static inline float mavlink_msg_gps200_get_magnetic_variation(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a gps200 message into a struct
 *
 * @param msg The message to decode
 * @param gps200 C-struct to decode the message contents into
 */
static inline void mavlink_msg_gps200_decode(const mavlink_message_t* msg, mavlink_gps200_t* gps200)
{
#if MAVLINK_NEED_BYTE_SWAP
	gps200->magnetic_variation = mavlink_msg_gps200_get_magnetic_variation(msg);
#else
	memcpy(gps200, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_GPS200_LEN);
#endif
}
