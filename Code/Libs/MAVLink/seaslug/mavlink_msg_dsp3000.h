// MESSAGE DSP3000 PACKING

#define MAVLINK_MSG_ID_DSP3000 164

typedef struct __mavlink_dsp3000_t
{
 float z_rate; ///< Z-axis rotation rate in degrees/s, clockwise positive.
} mavlink_dsp3000_t;

#define MAVLINK_MSG_ID_DSP3000_LEN 4
#define MAVLINK_MSG_ID_164_LEN 4

#define MAVLINK_MSG_ID_DSP3000_CRC 39
#define MAVLINK_MSG_ID_164_CRC 39



#define MAVLINK_MESSAGE_INFO_DSP3000 { \
	"DSP3000", \
	1, \
	{  { "z_rate", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_dsp3000_t, z_rate) }, \
         } \
}


/**
 * @brief Pack a dsp3000 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param z_rate Z-axis rotation rate in degrees/s, clockwise positive.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dsp3000_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float z_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DSP3000_LEN];
	_mav_put_float(buf, 0, z_rate);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DSP3000_LEN);
#else
	mavlink_dsp3000_t packet;
	packet.z_rate = z_rate;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DSP3000_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DSP3000;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DSP3000_LEN, MAVLINK_MSG_ID_DSP3000_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DSP3000_LEN);
#endif
}

/**
 * @brief Pack a dsp3000 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param z_rate Z-axis rotation rate in degrees/s, clockwise positive.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_dsp3000_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float z_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DSP3000_LEN];
	_mav_put_float(buf, 0, z_rate);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DSP3000_LEN);
#else
	mavlink_dsp3000_t packet;
	packet.z_rate = z_rate;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DSP3000_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_DSP3000;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DSP3000_LEN, MAVLINK_MSG_ID_DSP3000_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DSP3000_LEN);
#endif
}

/**
 * @brief Encode a dsp3000 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param dsp3000 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dsp3000_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_dsp3000_t* dsp3000)
{
	return mavlink_msg_dsp3000_pack(system_id, component_id, msg, dsp3000->z_rate);
}

/**
 * @brief Encode a dsp3000 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param dsp3000 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_dsp3000_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_dsp3000_t* dsp3000)
{
	return mavlink_msg_dsp3000_pack_chan(system_id, component_id, chan, msg, dsp3000->z_rate);
}

/**
 * @brief Send a dsp3000 message
 * @param chan MAVLink channel to send the message
 *
 * @param z_rate Z-axis rotation rate in degrees/s, clockwise positive.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_dsp3000_send(mavlink_channel_t chan, float z_rate)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_DSP3000_LEN];
	_mav_put_float(buf, 0, z_rate);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DSP3000, buf, MAVLINK_MSG_ID_DSP3000_LEN, MAVLINK_MSG_ID_DSP3000_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DSP3000, buf, MAVLINK_MSG_ID_DSP3000_LEN);
#endif
#else
	mavlink_dsp3000_t packet;
	packet.z_rate = z_rate;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DSP3000, (const char *)&packet, MAVLINK_MSG_ID_DSP3000_LEN, MAVLINK_MSG_ID_DSP3000_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DSP3000, (const char *)&packet, MAVLINK_MSG_ID_DSP3000_LEN);
#endif
#endif
}

#endif

// MESSAGE DSP3000 UNPACKING


/**
 * @brief Get field z_rate from dsp3000 message
 *
 * @return Z-axis rotation rate in degrees/s, clockwise positive.
 */
static inline float mavlink_msg_dsp3000_get_z_rate(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Decode a dsp3000 message into a struct
 *
 * @param msg The message to decode
 * @param dsp3000 C-struct to decode the message contents into
 */
static inline void mavlink_msg_dsp3000_decode(const mavlink_message_t* msg, mavlink_dsp3000_t* dsp3000)
{
#if MAVLINK_NEED_BYTE_SWAP
	dsp3000->z_rate = mavlink_msg_dsp3000_get_z_rate(msg);
#else
	memcpy(dsp3000, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_DSP3000_LEN);
#endif
}
