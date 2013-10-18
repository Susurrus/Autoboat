// MESSAGE REVO_GS PACKING

#define MAVLINK_MSG_ID_REVO_GS 162

typedef struct __mavlink_revo_gs_t
{
 float heading; ///< Heading in degrees eastward from north
 float pitch; ///< Pitch angle in degrees.
 float roll; ///< Roll angle in degrees.
 float dip; ///< Dip angle in degrees.
 uint16_t mag_horiz_comp; ///< Horizontal component of the earth's magnetic field.
 uint8_t mag_status; ///< Magnetometer status
 uint8_t pitch_status; ///< Pitch status
 uint8_t roll_status; ///< Roll status
} mavlink_revo_gs_t;

#define MAVLINK_MSG_ID_REVO_GS_LEN 21
#define MAVLINK_MSG_ID_162_LEN 21

#define MAVLINK_MSG_ID_REVO_GS_CRC 44
#define MAVLINK_MSG_ID_162_CRC 44



#define MAVLINK_MESSAGE_INFO_REVO_GS { \
	"REVO_GS", \
	8, \
	{  { "heading", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_revo_gs_t, heading) }, \
         { "pitch", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_revo_gs_t, pitch) }, \
         { "roll", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_revo_gs_t, roll) }, \
         { "dip", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_revo_gs_t, dip) }, \
         { "mag_horiz_comp", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_revo_gs_t, mag_horiz_comp) }, \
         { "mag_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 18, offsetof(mavlink_revo_gs_t, mag_status) }, \
         { "pitch_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 19, offsetof(mavlink_revo_gs_t, pitch_status) }, \
         { "roll_status", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_revo_gs_t, roll_status) }, \
         } \
}


/**
 * @brief Pack a revo_gs message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param heading Heading in degrees eastward from north
 * @param mag_status Magnetometer status
 * @param pitch Pitch angle in degrees.
 * @param pitch_status Pitch status
 * @param roll Roll angle in degrees.
 * @param roll_status Roll status
 * @param dip Dip angle in degrees.
 * @param mag_horiz_comp Horizontal component of the earth's magnetic field.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_revo_gs_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float heading, uint8_t mag_status, float pitch, uint8_t pitch_status, float roll, uint8_t roll_status, float dip, uint16_t mag_horiz_comp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_REVO_GS_LEN];
	_mav_put_float(buf, 0, heading);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, roll);
	_mav_put_float(buf, 12, dip);
	_mav_put_uint16_t(buf, 16, mag_horiz_comp);
	_mav_put_uint8_t(buf, 18, mag_status);
	_mav_put_uint8_t(buf, 19, pitch_status);
	_mav_put_uint8_t(buf, 20, roll_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REVO_GS_LEN);
#else
	mavlink_revo_gs_t packet;
	packet.heading = heading;
	packet.pitch = pitch;
	packet.roll = roll;
	packet.dip = dip;
	packet.mag_horiz_comp = mag_horiz_comp;
	packet.mag_status = mag_status;
	packet.pitch_status = pitch_status;
	packet.roll_status = roll_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REVO_GS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_REVO_GS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_REVO_GS_LEN, MAVLINK_MSG_ID_REVO_GS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_REVO_GS_LEN);
#endif
}

/**
 * @brief Pack a revo_gs message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param heading Heading in degrees eastward from north
 * @param mag_status Magnetometer status
 * @param pitch Pitch angle in degrees.
 * @param pitch_status Pitch status
 * @param roll Roll angle in degrees.
 * @param roll_status Roll status
 * @param dip Dip angle in degrees.
 * @param mag_horiz_comp Horizontal component of the earth's magnetic field.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_revo_gs_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float heading,uint8_t mag_status,float pitch,uint8_t pitch_status,float roll,uint8_t roll_status,float dip,uint16_t mag_horiz_comp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_REVO_GS_LEN];
	_mav_put_float(buf, 0, heading);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, roll);
	_mav_put_float(buf, 12, dip);
	_mav_put_uint16_t(buf, 16, mag_horiz_comp);
	_mav_put_uint8_t(buf, 18, mag_status);
	_mav_put_uint8_t(buf, 19, pitch_status);
	_mav_put_uint8_t(buf, 20, roll_status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_REVO_GS_LEN);
#else
	mavlink_revo_gs_t packet;
	packet.heading = heading;
	packet.pitch = pitch;
	packet.roll = roll;
	packet.dip = dip;
	packet.mag_horiz_comp = mag_horiz_comp;
	packet.mag_status = mag_status;
	packet.pitch_status = pitch_status;
	packet.roll_status = roll_status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_REVO_GS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_REVO_GS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_REVO_GS_LEN, MAVLINK_MSG_ID_REVO_GS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_REVO_GS_LEN);
#endif
}

/**
 * @brief Encode a revo_gs struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param revo_gs C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_revo_gs_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_revo_gs_t* revo_gs)
{
	return mavlink_msg_revo_gs_pack(system_id, component_id, msg, revo_gs->heading, revo_gs->mag_status, revo_gs->pitch, revo_gs->pitch_status, revo_gs->roll, revo_gs->roll_status, revo_gs->dip, revo_gs->mag_horiz_comp);
}

/**
 * @brief Send a revo_gs message
 * @param chan MAVLink channel to send the message
 *
 * @param heading Heading in degrees eastward from north
 * @param mag_status Magnetometer status
 * @param pitch Pitch angle in degrees.
 * @param pitch_status Pitch status
 * @param roll Roll angle in degrees.
 * @param roll_status Roll status
 * @param dip Dip angle in degrees.
 * @param mag_horiz_comp Horizontal component of the earth's magnetic field.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_revo_gs_send(mavlink_channel_t chan, float heading, uint8_t mag_status, float pitch, uint8_t pitch_status, float roll, uint8_t roll_status, float dip, uint16_t mag_horiz_comp)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_REVO_GS_LEN];
	_mav_put_float(buf, 0, heading);
	_mav_put_float(buf, 4, pitch);
	_mav_put_float(buf, 8, roll);
	_mav_put_float(buf, 12, dip);
	_mav_put_uint16_t(buf, 16, mag_horiz_comp);
	_mav_put_uint8_t(buf, 18, mag_status);
	_mav_put_uint8_t(buf, 19, pitch_status);
	_mav_put_uint8_t(buf, 20, roll_status);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REVO_GS, buf, MAVLINK_MSG_ID_REVO_GS_LEN, MAVLINK_MSG_ID_REVO_GS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REVO_GS, buf, MAVLINK_MSG_ID_REVO_GS_LEN);
#endif
#else
	mavlink_revo_gs_t packet;
	packet.heading = heading;
	packet.pitch = pitch;
	packet.roll = roll;
	packet.dip = dip;
	packet.mag_horiz_comp = mag_horiz_comp;
	packet.mag_status = mag_status;
	packet.pitch_status = pitch_status;
	packet.roll_status = roll_status;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REVO_GS, (const char *)&packet, MAVLINK_MSG_ID_REVO_GS_LEN, MAVLINK_MSG_ID_REVO_GS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_REVO_GS, (const char *)&packet, MAVLINK_MSG_ID_REVO_GS_LEN);
#endif
#endif
}

#endif

// MESSAGE REVO_GS UNPACKING


/**
 * @brief Get field heading from revo_gs message
 *
 * @return Heading in degrees eastward from north
 */
static inline float mavlink_msg_revo_gs_get_heading(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field mag_status from revo_gs message
 *
 * @return Magnetometer status
 */
static inline uint8_t mavlink_msg_revo_gs_get_mag_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  18);
}

/**
 * @brief Get field pitch from revo_gs message
 *
 * @return Pitch angle in degrees.
 */
static inline float mavlink_msg_revo_gs_get_pitch(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field pitch_status from revo_gs message
 *
 * @return Pitch status
 */
static inline uint8_t mavlink_msg_revo_gs_get_pitch_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  19);
}

/**
 * @brief Get field roll from revo_gs message
 *
 * @return Roll angle in degrees.
 */
static inline float mavlink_msg_revo_gs_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field roll_status from revo_gs message
 *
 * @return Roll status
 */
static inline uint8_t mavlink_msg_revo_gs_get_roll_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field dip from revo_gs message
 *
 * @return Dip angle in degrees.
 */
static inline float mavlink_msg_revo_gs_get_dip(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field mag_horiz_comp from revo_gs message
 *
 * @return Horizontal component of the earth's magnetic field.
 */
static inline uint16_t mavlink_msg_revo_gs_get_mag_horiz_comp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Decode a revo_gs message into a struct
 *
 * @param msg The message to decode
 * @param revo_gs C-struct to decode the message contents into
 */
static inline void mavlink_msg_revo_gs_decode(const mavlink_message_t* msg, mavlink_revo_gs_t* revo_gs)
{
#if MAVLINK_NEED_BYTE_SWAP
	revo_gs->heading = mavlink_msg_revo_gs_get_heading(msg);
	revo_gs->pitch = mavlink_msg_revo_gs_get_pitch(msg);
	revo_gs->roll = mavlink_msg_revo_gs_get_roll(msg);
	revo_gs->dip = mavlink_msg_revo_gs_get_dip(msg);
	revo_gs->mag_horiz_comp = mavlink_msg_revo_gs_get_mag_horiz_comp(msg);
	revo_gs->mag_status = mavlink_msg_revo_gs_get_mag_status(msg);
	revo_gs->pitch_status = mavlink_msg_revo_gs_get_pitch_status(msg);
	revo_gs->roll_status = mavlink_msg_revo_gs_get_roll_status(msg);
#else
	memcpy(revo_gs, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_REVO_GS_LEN);
#endif
}
