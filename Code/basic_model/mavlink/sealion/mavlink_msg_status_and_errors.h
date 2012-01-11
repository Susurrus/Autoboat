// MESSAGE STATUS_AND_ERRORS PACKING

#define MAVLINK_MSG_ID_STATUS_AND_ERRORS 170

typedef struct __mavlink_status_and_errors_t
{
 uint16_t status; ///< Status bitfield. Bits are active-high and ordered as: 0-reset/startup, 1-reset_hil_toggle, 2-reset_HIL_discon, 3-reset_GPS-discon, 4-reset_track, 5-reset_calibrating, 6-reset_uncalibrated, 7-reset_estrop, 8-auto_mode, 9-hil_sensors_mode, 10-rc_disconnected.
 uint16_t errors; ///< Errors bitfield. Bits are active-high and ordered as: 0-ECAN_TX_err, 1-ECAN_RX_err.
} mavlink_status_and_errors_t;

#define MAVLINK_MSG_ID_STATUS_AND_ERRORS_LEN 4
#define MAVLINK_MSG_ID_170_LEN 4



#define MAVLINK_MESSAGE_INFO_STATUS_AND_ERRORS { \
	"STATUS_AND_ERRORS", \
	2, \
	{  { "status", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_status_and_errors_t, status) }, \
         { "errors", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_status_and_errors_t, errors) }, \
         } \
}


/**
 * @brief Pack a status_and_errors message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param status Status bitfield. Bits are active-high and ordered as: 0-reset/startup, 1-reset_hil_toggle, 2-reset_HIL_discon, 3-reset_GPS-discon, 4-reset_track, 5-reset_calibrating, 6-reset_uncalibrated, 7-reset_estrop, 8-auto_mode, 9-hil_sensors_mode, 10-rc_disconnected.
 * @param errors Errors bitfield. Bits are active-high and ordered as: 0-ECAN_TX_err, 1-ECAN_RX_err.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_status_and_errors_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t status, uint16_t errors)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, status);
	_mav_put_uint16_t(buf, 2, errors);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 4);
#else
	mavlink_status_and_errors_t packet;
	packet.status = status;
	packet.errors = errors;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_STATUS_AND_ERRORS;
	return mavlink_finalize_message(msg, system_id, component_id, 4, 155);
}

/**
 * @brief Pack a status_and_errors message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param status Status bitfield. Bits are active-high and ordered as: 0-reset/startup, 1-reset_hil_toggle, 2-reset_HIL_discon, 3-reset_GPS-discon, 4-reset_track, 5-reset_calibrating, 6-reset_uncalibrated, 7-reset_estrop, 8-auto_mode, 9-hil_sensors_mode, 10-rc_disconnected.
 * @param errors Errors bitfield. Bits are active-high and ordered as: 0-ECAN_TX_err, 1-ECAN_RX_err.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_status_and_errors_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t status,uint16_t errors)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, status);
	_mav_put_uint16_t(buf, 2, errors);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 4);
#else
	mavlink_status_and_errors_t packet;
	packet.status = status;
	packet.errors = errors;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 4);
#endif

	msg->msgid = MAVLINK_MSG_ID_STATUS_AND_ERRORS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 4, 155);
}

/**
 * @brief Encode a status_and_errors struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param status_and_errors C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_status_and_errors_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_status_and_errors_t* status_and_errors)
{
	return mavlink_msg_status_and_errors_pack(system_id, component_id, msg, status_and_errors->status, status_and_errors->errors);
}

/**
 * @brief Send a status_and_errors message
 * @param chan MAVLink channel to send the message
 *
 * @param status Status bitfield. Bits are active-high and ordered as: 0-reset/startup, 1-reset_hil_toggle, 2-reset_HIL_discon, 3-reset_GPS-discon, 4-reset_track, 5-reset_calibrating, 6-reset_uncalibrated, 7-reset_estrop, 8-auto_mode, 9-hil_sensors_mode, 10-rc_disconnected.
 * @param errors Errors bitfield. Bits are active-high and ordered as: 0-ECAN_TX_err, 1-ECAN_RX_err.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_status_and_errors_send(mavlink_channel_t chan, uint16_t status, uint16_t errors)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[4];
	_mav_put_uint16_t(buf, 0, status);
	_mav_put_uint16_t(buf, 2, errors);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUS_AND_ERRORS, buf, 4, 155);
#else
	mavlink_status_and_errors_t packet;
	packet.status = status;
	packet.errors = errors;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STATUS_AND_ERRORS, (const char *)&packet, 4, 155);
#endif
}

#endif

// MESSAGE STATUS_AND_ERRORS UNPACKING


/**
 * @brief Get field status from status_and_errors message
 *
 * @return Status bitfield. Bits are active-high and ordered as: 0-reset/startup, 1-reset_hil_toggle, 2-reset_HIL_discon, 3-reset_GPS-discon, 4-reset_track, 5-reset_calibrating, 6-reset_uncalibrated, 7-reset_estrop, 8-auto_mode, 9-hil_sensors_mode, 10-rc_disconnected.
 */
static inline uint16_t mavlink_msg_status_and_errors_get_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field errors from status_and_errors message
 *
 * @return Errors bitfield. Bits are active-high and ordered as: 0-ECAN_TX_err, 1-ECAN_RX_err.
 */
static inline uint16_t mavlink_msg_status_and_errors_get_errors(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Decode a status_and_errors message into a struct
 *
 * @param msg The message to decode
 * @param status_and_errors C-struct to decode the message contents into
 */
static inline void mavlink_msg_status_and_errors_decode(const mavlink_message_t* msg, mavlink_status_and_errors_t* status_and_errors)
{
#if MAVLINK_NEED_BYTE_SWAP
	status_and_errors->status = mavlink_msg_status_and_errors_get_status(msg);
	status_and_errors->errors = mavlink_msg_status_and_errors_get_errors(msg);
#else
	memcpy(status_and_errors, _MAV_PAYLOAD(msg), 4);
#endif
}
