// MESSAGE BASIC_STATE PACKING

#define MAVLINK_MSG_ID_BASIC_STATE 171

typedef struct __mavlink_basic_state_t
{
 float commanded_auto_rudder_angle; ///< This is the rudder angle command as commanded by the onboard autonomous controller. It's in milliradians where positive indicates port-side.
 float commanded_primary_rudder_angle; ///< This is the rudder angle command as commanded by the primary manual controller. It's in milliradians where positive indicates port-side.
 float commanded_secondary_rudder_angle; ///< This is the rudder angle command as commanded by the secondary/emergency manual controller. It's in milliradians where positive indicates port-side.
 float rudder_angle; ///< The interpreted rudder angle in radians.
 float a_cmd; ///< This is the lateral acceleration as commanded by the onboard L2+ controller. Units are in m/s^2.
 float L2_north; ///< North-coordinate of the L2 vector in mm.
 float L2_east; ///< East-coordinate of the L2 vector in mm.
 int16_t commanded_auto_throttle; ///< This is the throttle command as commanded by the onboard autonomous controller. It's in units of 1/1023*100% of max current and positive values propel the vehicle forward.
 int16_t commanded_primary_throttle; ///< This is the throttle command as commanded by the primary manual controller. It's in units of 1/1023*100% of max current and positive values propel the vehicle forward.
 int16_t commanded_secondary_throttle; ///< This is the throttle command as commanded by the secondary/emergency manual controller. It's in units of 1/1023*100% of max current and positive values propel the vehicle forward.
 int16_t prop_speed; ///< Propeller speed, positive values mean the vessel will be propelled forward. Units are in RPM.
} mavlink_basic_state_t;

#define MAVLINK_MSG_ID_BASIC_STATE_LEN 36
#define MAVLINK_MSG_ID_171_LEN 36

#define MAVLINK_MSG_ID_BASIC_STATE_CRC 136
#define MAVLINK_MSG_ID_171_CRC 136



#define MAVLINK_MESSAGE_INFO_BASIC_STATE { \
	"BASIC_STATE", \
	11, \
	{  { "commanded_auto_rudder_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_basic_state_t, commanded_auto_rudder_angle) }, \
         { "commanded_primary_rudder_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_basic_state_t, commanded_primary_rudder_angle) }, \
         { "commanded_secondary_rudder_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_basic_state_t, commanded_secondary_rudder_angle) }, \
         { "rudder_angle", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_basic_state_t, rudder_angle) }, \
         { "a_cmd", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_basic_state_t, a_cmd) }, \
         { "L2_north", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_basic_state_t, L2_north) }, \
         { "L2_east", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_basic_state_t, L2_east) }, \
         { "commanded_auto_throttle", NULL, MAVLINK_TYPE_INT16_T, 0, 28, offsetof(mavlink_basic_state_t, commanded_auto_throttle) }, \
         { "commanded_primary_throttle", NULL, MAVLINK_TYPE_INT16_T, 0, 30, offsetof(mavlink_basic_state_t, commanded_primary_throttle) }, \
         { "commanded_secondary_throttle", NULL, MAVLINK_TYPE_INT16_T, 0, 32, offsetof(mavlink_basic_state_t, commanded_secondary_throttle) }, \
         { "prop_speed", NULL, MAVLINK_TYPE_INT16_T, 0, 34, offsetof(mavlink_basic_state_t, prop_speed) }, \
         } \
}


/**
 * @brief Pack a basic_state message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param commanded_auto_rudder_angle This is the rudder angle command as commanded by the onboard autonomous controller. It's in milliradians where positive indicates port-side.
 * @param commanded_primary_rudder_angle This is the rudder angle command as commanded by the primary manual controller. It's in milliradians where positive indicates port-side.
 * @param commanded_secondary_rudder_angle This is the rudder angle command as commanded by the secondary/emergency manual controller. It's in milliradians where positive indicates port-side.
 * @param rudder_angle The interpreted rudder angle in radians.
 * @param commanded_auto_throttle This is the throttle command as commanded by the onboard autonomous controller. It's in units of 1/1023*100% of max current and positive values propel the vehicle forward.
 * @param commanded_primary_throttle This is the throttle command as commanded by the primary manual controller. It's in units of 1/1023*100% of max current and positive values propel the vehicle forward.
 * @param commanded_secondary_throttle This is the throttle command as commanded by the secondary/emergency manual controller. It's in units of 1/1023*100% of max current and positive values propel the vehicle forward.
 * @param prop_speed Propeller speed, positive values mean the vessel will be propelled forward. Units are in RPM.
 * @param a_cmd This is the lateral acceleration as commanded by the onboard L2+ controller. Units are in m/s^2.
 * @param L2_north North-coordinate of the L2 vector in mm.
 * @param L2_east East-coordinate of the L2 vector in mm.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_basic_state_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float commanded_auto_rudder_angle, float commanded_primary_rudder_angle, float commanded_secondary_rudder_angle, float rudder_angle, int16_t commanded_auto_throttle, int16_t commanded_primary_throttle, int16_t commanded_secondary_throttle, int16_t prop_speed, float a_cmd, float L2_north, float L2_east)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BASIC_STATE_LEN];
	_mav_put_float(buf, 0, commanded_auto_rudder_angle);
	_mav_put_float(buf, 4, commanded_primary_rudder_angle);
	_mav_put_float(buf, 8, commanded_secondary_rudder_angle);
	_mav_put_float(buf, 12, rudder_angle);
	_mav_put_float(buf, 16, a_cmd);
	_mav_put_float(buf, 20, L2_north);
	_mav_put_float(buf, 24, L2_east);
	_mav_put_int16_t(buf, 28, commanded_auto_throttle);
	_mav_put_int16_t(buf, 30, commanded_primary_throttle);
	_mav_put_int16_t(buf, 32, commanded_secondary_throttle);
	_mav_put_int16_t(buf, 34, prop_speed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BASIC_STATE_LEN);
#else
	mavlink_basic_state_t packet;
	packet.commanded_auto_rudder_angle = commanded_auto_rudder_angle;
	packet.commanded_primary_rudder_angle = commanded_primary_rudder_angle;
	packet.commanded_secondary_rudder_angle = commanded_secondary_rudder_angle;
	packet.rudder_angle = rudder_angle;
	packet.a_cmd = a_cmd;
	packet.L2_north = L2_north;
	packet.L2_east = L2_east;
	packet.commanded_auto_throttle = commanded_auto_throttle;
	packet.commanded_primary_throttle = commanded_primary_throttle;
	packet.commanded_secondary_throttle = commanded_secondary_throttle;
	packet.prop_speed = prop_speed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BASIC_STATE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_BASIC_STATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BASIC_STATE_LEN, MAVLINK_MSG_ID_BASIC_STATE_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BASIC_STATE_LEN);
#endif
}

/**
 * @brief Pack a basic_state message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param commanded_auto_rudder_angle This is the rudder angle command as commanded by the onboard autonomous controller. It's in milliradians where positive indicates port-side.
 * @param commanded_primary_rudder_angle This is the rudder angle command as commanded by the primary manual controller. It's in milliradians where positive indicates port-side.
 * @param commanded_secondary_rudder_angle This is the rudder angle command as commanded by the secondary/emergency manual controller. It's in milliradians where positive indicates port-side.
 * @param rudder_angle The interpreted rudder angle in radians.
 * @param commanded_auto_throttle This is the throttle command as commanded by the onboard autonomous controller. It's in units of 1/1023*100% of max current and positive values propel the vehicle forward.
 * @param commanded_primary_throttle This is the throttle command as commanded by the primary manual controller. It's in units of 1/1023*100% of max current and positive values propel the vehicle forward.
 * @param commanded_secondary_throttle This is the throttle command as commanded by the secondary/emergency manual controller. It's in units of 1/1023*100% of max current and positive values propel the vehicle forward.
 * @param prop_speed Propeller speed, positive values mean the vessel will be propelled forward. Units are in RPM.
 * @param a_cmd This is the lateral acceleration as commanded by the onboard L2+ controller. Units are in m/s^2.
 * @param L2_north North-coordinate of the L2 vector in mm.
 * @param L2_east East-coordinate of the L2 vector in mm.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_basic_state_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float commanded_auto_rudder_angle,float commanded_primary_rudder_angle,float commanded_secondary_rudder_angle,float rudder_angle,int16_t commanded_auto_throttle,int16_t commanded_primary_throttle,int16_t commanded_secondary_throttle,int16_t prop_speed,float a_cmd,float L2_north,float L2_east)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BASIC_STATE_LEN];
	_mav_put_float(buf, 0, commanded_auto_rudder_angle);
	_mav_put_float(buf, 4, commanded_primary_rudder_angle);
	_mav_put_float(buf, 8, commanded_secondary_rudder_angle);
	_mav_put_float(buf, 12, rudder_angle);
	_mav_put_float(buf, 16, a_cmd);
	_mav_put_float(buf, 20, L2_north);
	_mav_put_float(buf, 24, L2_east);
	_mav_put_int16_t(buf, 28, commanded_auto_throttle);
	_mav_put_int16_t(buf, 30, commanded_primary_throttle);
	_mav_put_int16_t(buf, 32, commanded_secondary_throttle);
	_mav_put_int16_t(buf, 34, prop_speed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BASIC_STATE_LEN);
#else
	mavlink_basic_state_t packet;
	packet.commanded_auto_rudder_angle = commanded_auto_rudder_angle;
	packet.commanded_primary_rudder_angle = commanded_primary_rudder_angle;
	packet.commanded_secondary_rudder_angle = commanded_secondary_rudder_angle;
	packet.rudder_angle = rudder_angle;
	packet.a_cmd = a_cmd;
	packet.L2_north = L2_north;
	packet.L2_east = L2_east;
	packet.commanded_auto_throttle = commanded_auto_throttle;
	packet.commanded_primary_throttle = commanded_primary_throttle;
	packet.commanded_secondary_throttle = commanded_secondary_throttle;
	packet.prop_speed = prop_speed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BASIC_STATE_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_BASIC_STATE;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BASIC_STATE_LEN, MAVLINK_MSG_ID_BASIC_STATE_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BASIC_STATE_LEN);
#endif
}

/**
 * @brief Encode a basic_state struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param basic_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_basic_state_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_basic_state_t* basic_state)
{
	return mavlink_msg_basic_state_pack(system_id, component_id, msg, basic_state->commanded_auto_rudder_angle, basic_state->commanded_primary_rudder_angle, basic_state->commanded_secondary_rudder_angle, basic_state->rudder_angle, basic_state->commanded_auto_throttle, basic_state->commanded_primary_throttle, basic_state->commanded_secondary_throttle, basic_state->prop_speed, basic_state->a_cmd, basic_state->L2_north, basic_state->L2_east);
}

/**
 * @brief Encode a basic_state struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param basic_state C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_basic_state_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_basic_state_t* basic_state)
{
	return mavlink_msg_basic_state_pack_chan(system_id, component_id, chan, msg, basic_state->commanded_auto_rudder_angle, basic_state->commanded_primary_rudder_angle, basic_state->commanded_secondary_rudder_angle, basic_state->rudder_angle, basic_state->commanded_auto_throttle, basic_state->commanded_primary_throttle, basic_state->commanded_secondary_throttle, basic_state->prop_speed, basic_state->a_cmd, basic_state->L2_north, basic_state->L2_east);
}

/**
 * @brief Send a basic_state message
 * @param chan MAVLink channel to send the message
 *
 * @param commanded_auto_rudder_angle This is the rudder angle command as commanded by the onboard autonomous controller. It's in milliradians where positive indicates port-side.
 * @param commanded_primary_rudder_angle This is the rudder angle command as commanded by the primary manual controller. It's in milliradians where positive indicates port-side.
 * @param commanded_secondary_rudder_angle This is the rudder angle command as commanded by the secondary/emergency manual controller. It's in milliradians where positive indicates port-side.
 * @param rudder_angle The interpreted rudder angle in radians.
 * @param commanded_auto_throttle This is the throttle command as commanded by the onboard autonomous controller. It's in units of 1/1023*100% of max current and positive values propel the vehicle forward.
 * @param commanded_primary_throttle This is the throttle command as commanded by the primary manual controller. It's in units of 1/1023*100% of max current and positive values propel the vehicle forward.
 * @param commanded_secondary_throttle This is the throttle command as commanded by the secondary/emergency manual controller. It's in units of 1/1023*100% of max current and positive values propel the vehicle forward.
 * @param prop_speed Propeller speed, positive values mean the vessel will be propelled forward. Units are in RPM.
 * @param a_cmd This is the lateral acceleration as commanded by the onboard L2+ controller. Units are in m/s^2.
 * @param L2_north North-coordinate of the L2 vector in mm.
 * @param L2_east East-coordinate of the L2 vector in mm.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_basic_state_send(mavlink_channel_t chan, float commanded_auto_rudder_angle, float commanded_primary_rudder_angle, float commanded_secondary_rudder_angle, float rudder_angle, int16_t commanded_auto_throttle, int16_t commanded_primary_throttle, int16_t commanded_secondary_throttle, int16_t prop_speed, float a_cmd, float L2_north, float L2_east)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_BASIC_STATE_LEN];
	_mav_put_float(buf, 0, commanded_auto_rudder_angle);
	_mav_put_float(buf, 4, commanded_primary_rudder_angle);
	_mav_put_float(buf, 8, commanded_secondary_rudder_angle);
	_mav_put_float(buf, 12, rudder_angle);
	_mav_put_float(buf, 16, a_cmd);
	_mav_put_float(buf, 20, L2_north);
	_mav_put_float(buf, 24, L2_east);
	_mav_put_int16_t(buf, 28, commanded_auto_throttle);
	_mav_put_int16_t(buf, 30, commanded_primary_throttle);
	_mav_put_int16_t(buf, 32, commanded_secondary_throttle);
	_mav_put_int16_t(buf, 34, prop_speed);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BASIC_STATE, buf, MAVLINK_MSG_ID_BASIC_STATE_LEN, MAVLINK_MSG_ID_BASIC_STATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BASIC_STATE, buf, MAVLINK_MSG_ID_BASIC_STATE_LEN);
#endif
#else
	mavlink_basic_state_t packet;
	packet.commanded_auto_rudder_angle = commanded_auto_rudder_angle;
	packet.commanded_primary_rudder_angle = commanded_primary_rudder_angle;
	packet.commanded_secondary_rudder_angle = commanded_secondary_rudder_angle;
	packet.rudder_angle = rudder_angle;
	packet.a_cmd = a_cmd;
	packet.L2_north = L2_north;
	packet.L2_east = L2_east;
	packet.commanded_auto_throttle = commanded_auto_throttle;
	packet.commanded_primary_throttle = commanded_primary_throttle;
	packet.commanded_secondary_throttle = commanded_secondary_throttle;
	packet.prop_speed = prop_speed;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BASIC_STATE, (const char *)&packet, MAVLINK_MSG_ID_BASIC_STATE_LEN, MAVLINK_MSG_ID_BASIC_STATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BASIC_STATE, (const char *)&packet, MAVLINK_MSG_ID_BASIC_STATE_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_BASIC_STATE_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_basic_state_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float commanded_auto_rudder_angle, float commanded_primary_rudder_angle, float commanded_secondary_rudder_angle, float rudder_angle, int16_t commanded_auto_throttle, int16_t commanded_primary_throttle, int16_t commanded_secondary_throttle, int16_t prop_speed, float a_cmd, float L2_north, float L2_east)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, commanded_auto_rudder_angle);
	_mav_put_float(buf, 4, commanded_primary_rudder_angle);
	_mav_put_float(buf, 8, commanded_secondary_rudder_angle);
	_mav_put_float(buf, 12, rudder_angle);
	_mav_put_float(buf, 16, a_cmd);
	_mav_put_float(buf, 20, L2_north);
	_mav_put_float(buf, 24, L2_east);
	_mav_put_int16_t(buf, 28, commanded_auto_throttle);
	_mav_put_int16_t(buf, 30, commanded_primary_throttle);
	_mav_put_int16_t(buf, 32, commanded_secondary_throttle);
	_mav_put_int16_t(buf, 34, prop_speed);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BASIC_STATE, buf, MAVLINK_MSG_ID_BASIC_STATE_LEN, MAVLINK_MSG_ID_BASIC_STATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BASIC_STATE, buf, MAVLINK_MSG_ID_BASIC_STATE_LEN);
#endif
#else
	mavlink_basic_state_t *packet = (mavlink_basic_state_t *)msgbuf;
	packet->commanded_auto_rudder_angle = commanded_auto_rudder_angle;
	packet->commanded_primary_rudder_angle = commanded_primary_rudder_angle;
	packet->commanded_secondary_rudder_angle = commanded_secondary_rudder_angle;
	packet->rudder_angle = rudder_angle;
	packet->a_cmd = a_cmd;
	packet->L2_north = L2_north;
	packet->L2_east = L2_east;
	packet->commanded_auto_throttle = commanded_auto_throttle;
	packet->commanded_primary_throttle = commanded_primary_throttle;
	packet->commanded_secondary_throttle = commanded_secondary_throttle;
	packet->prop_speed = prop_speed;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BASIC_STATE, (const char *)packet, MAVLINK_MSG_ID_BASIC_STATE_LEN, MAVLINK_MSG_ID_BASIC_STATE_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BASIC_STATE, (const char *)packet, MAVLINK_MSG_ID_BASIC_STATE_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE BASIC_STATE UNPACKING


/**
 * @brief Get field commanded_auto_rudder_angle from basic_state message
 *
 * @return This is the rudder angle command as commanded by the onboard autonomous controller. It's in milliradians where positive indicates port-side.
 */
static inline float mavlink_msg_basic_state_get_commanded_auto_rudder_angle(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field commanded_primary_rudder_angle from basic_state message
 *
 * @return This is the rudder angle command as commanded by the primary manual controller. It's in milliradians where positive indicates port-side.
 */
static inline float mavlink_msg_basic_state_get_commanded_primary_rudder_angle(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field commanded_secondary_rudder_angle from basic_state message
 *
 * @return This is the rudder angle command as commanded by the secondary/emergency manual controller. It's in milliradians where positive indicates port-side.
 */
static inline float mavlink_msg_basic_state_get_commanded_secondary_rudder_angle(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field rudder_angle from basic_state message
 *
 * @return The interpreted rudder angle in radians.
 */
static inline float mavlink_msg_basic_state_get_rudder_angle(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field commanded_auto_throttle from basic_state message
 *
 * @return This is the throttle command as commanded by the onboard autonomous controller. It's in units of 1/1023*100% of max current and positive values propel the vehicle forward.
 */
static inline int16_t mavlink_msg_basic_state_get_commanded_auto_throttle(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  28);
}

/**
 * @brief Get field commanded_primary_throttle from basic_state message
 *
 * @return This is the throttle command as commanded by the primary manual controller. It's in units of 1/1023*100% of max current and positive values propel the vehicle forward.
 */
static inline int16_t mavlink_msg_basic_state_get_commanded_primary_throttle(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  30);
}

/**
 * @brief Get field commanded_secondary_throttle from basic_state message
 *
 * @return This is the throttle command as commanded by the secondary/emergency manual controller. It's in units of 1/1023*100% of max current and positive values propel the vehicle forward.
 */
static inline int16_t mavlink_msg_basic_state_get_commanded_secondary_throttle(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  32);
}

/**
 * @brief Get field prop_speed from basic_state message
 *
 * @return Propeller speed, positive values mean the vessel will be propelled forward. Units are in RPM.
 */
static inline int16_t mavlink_msg_basic_state_get_prop_speed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  34);
}

/**
 * @brief Get field a_cmd from basic_state message
 *
 * @return This is the lateral acceleration as commanded by the onboard L2+ controller. Units are in m/s^2.
 */
static inline float mavlink_msg_basic_state_get_a_cmd(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field L2_north from basic_state message
 *
 * @return North-coordinate of the L2 vector in mm.
 */
static inline float mavlink_msg_basic_state_get_L2_north(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field L2_east from basic_state message
 *
 * @return East-coordinate of the L2 vector in mm.
 */
static inline float mavlink_msg_basic_state_get_L2_east(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Decode a basic_state message into a struct
 *
 * @param msg The message to decode
 * @param basic_state C-struct to decode the message contents into
 */
static inline void mavlink_msg_basic_state_decode(const mavlink_message_t* msg, mavlink_basic_state_t* basic_state)
{
#if MAVLINK_NEED_BYTE_SWAP
	basic_state->commanded_auto_rudder_angle = mavlink_msg_basic_state_get_commanded_auto_rudder_angle(msg);
	basic_state->commanded_primary_rudder_angle = mavlink_msg_basic_state_get_commanded_primary_rudder_angle(msg);
	basic_state->commanded_secondary_rudder_angle = mavlink_msg_basic_state_get_commanded_secondary_rudder_angle(msg);
	basic_state->rudder_angle = mavlink_msg_basic_state_get_rudder_angle(msg);
	basic_state->a_cmd = mavlink_msg_basic_state_get_a_cmd(msg);
	basic_state->L2_north = mavlink_msg_basic_state_get_L2_north(msg);
	basic_state->L2_east = mavlink_msg_basic_state_get_L2_east(msg);
	basic_state->commanded_auto_throttle = mavlink_msg_basic_state_get_commanded_auto_throttle(msg);
	basic_state->commanded_primary_throttle = mavlink_msg_basic_state_get_commanded_primary_throttle(msg);
	basic_state->commanded_secondary_throttle = mavlink_msg_basic_state_get_commanded_secondary_throttle(msg);
	basic_state->prop_speed = mavlink_msg_basic_state_get_prop_speed(msg);
#else
	memcpy(basic_state, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_BASIC_STATE_LEN);
#endif
}
