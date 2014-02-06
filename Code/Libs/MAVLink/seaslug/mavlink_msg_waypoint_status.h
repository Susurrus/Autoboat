// MESSAGE WAYPOINT_STATUS PACKING

#define MAVLINK_MSG_ID_WAYPOINT_STATUS 174

typedef struct __mavlink_waypoint_status_t
{
 float last_wp_lat; ///< The latitude of the last waypoint.
 float last_wp_lon; ///< The longitude of the last waypoint.
 float last_wp_north; ///< The north component of the local coordinates of the last waypoint.
 float last_wp_east; ///< The east component of the local coordinates of the last waypoint.
 float next_wp_lat; ///< The latitude of the next waypoint.
 float next_wp_lon; ///< The longitude of the next waypoint.
 float next_wp_north; ///< The north component of the local coordinates of the next waypoint.
 float next_wp_east; ///< The east component of the local coordinates of the next waypoint.
} mavlink_waypoint_status_t;

#define MAVLINK_MSG_ID_WAYPOINT_STATUS_LEN 32
#define MAVLINK_MSG_ID_174_LEN 32

#define MAVLINK_MSG_ID_WAYPOINT_STATUS_CRC 220
#define MAVLINK_MSG_ID_174_CRC 220



#define MAVLINK_MESSAGE_INFO_WAYPOINT_STATUS { \
	"WAYPOINT_STATUS", \
	8, \
	{  { "last_wp_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_waypoint_status_t, last_wp_lat) }, \
         { "last_wp_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_waypoint_status_t, last_wp_lon) }, \
         { "last_wp_north", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_waypoint_status_t, last_wp_north) }, \
         { "last_wp_east", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_waypoint_status_t, last_wp_east) }, \
         { "next_wp_lat", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_waypoint_status_t, next_wp_lat) }, \
         { "next_wp_lon", NULL, MAVLINK_TYPE_FLOAT, 0, 20, offsetof(mavlink_waypoint_status_t, next_wp_lon) }, \
         { "next_wp_north", NULL, MAVLINK_TYPE_FLOAT, 0, 24, offsetof(mavlink_waypoint_status_t, next_wp_north) }, \
         { "next_wp_east", NULL, MAVLINK_TYPE_FLOAT, 0, 28, offsetof(mavlink_waypoint_status_t, next_wp_east) }, \
         } \
}


/**
 * @brief Pack a waypoint_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param last_wp_lat The latitude of the last waypoint.
 * @param last_wp_lon The longitude of the last waypoint.
 * @param last_wp_north The north component of the local coordinates of the last waypoint.
 * @param last_wp_east The east component of the local coordinates of the last waypoint.
 * @param next_wp_lat The latitude of the next waypoint.
 * @param next_wp_lon The longitude of the next waypoint.
 * @param next_wp_north The north component of the local coordinates of the next waypoint.
 * @param next_wp_east The east component of the local coordinates of the next waypoint.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float last_wp_lat, float last_wp_lon, float last_wp_north, float last_wp_east, float next_wp_lat, float next_wp_lon, float next_wp_north, float next_wp_east)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_WAYPOINT_STATUS_LEN];
	_mav_put_float(buf, 0, last_wp_lat);
	_mav_put_float(buf, 4, last_wp_lon);
	_mav_put_float(buf, 8, last_wp_north);
	_mav_put_float(buf, 12, last_wp_east);
	_mav_put_float(buf, 16, next_wp_lat);
	_mav_put_float(buf, 20, next_wp_lon);
	_mav_put_float(buf, 24, next_wp_north);
	_mav_put_float(buf, 28, next_wp_east);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WAYPOINT_STATUS_LEN);
#else
	mavlink_waypoint_status_t packet;
	packet.last_wp_lat = last_wp_lat;
	packet.last_wp_lon = last_wp_lon;
	packet.last_wp_north = last_wp_north;
	packet.last_wp_east = last_wp_east;
	packet.next_wp_lat = next_wp_lat;
	packet.next_wp_lon = next_wp_lon;
	packet.next_wp_north = next_wp_north;
	packet.next_wp_east = next_wp_east;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WAYPOINT_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_WAYPOINT_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WAYPOINT_STATUS_LEN, MAVLINK_MSG_ID_WAYPOINT_STATUS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_WAYPOINT_STATUS_LEN);
#endif
}

/**
 * @brief Pack a waypoint_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param last_wp_lat The latitude of the last waypoint.
 * @param last_wp_lon The longitude of the last waypoint.
 * @param last_wp_north The north component of the local coordinates of the last waypoint.
 * @param last_wp_east The east component of the local coordinates of the last waypoint.
 * @param next_wp_lat The latitude of the next waypoint.
 * @param next_wp_lon The longitude of the next waypoint.
 * @param next_wp_north The north component of the local coordinates of the next waypoint.
 * @param next_wp_east The east component of the local coordinates of the next waypoint.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_waypoint_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float last_wp_lat,float last_wp_lon,float last_wp_north,float last_wp_east,float next_wp_lat,float next_wp_lon,float next_wp_north,float next_wp_east)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_WAYPOINT_STATUS_LEN];
	_mav_put_float(buf, 0, last_wp_lat);
	_mav_put_float(buf, 4, last_wp_lon);
	_mav_put_float(buf, 8, last_wp_north);
	_mav_put_float(buf, 12, last_wp_east);
	_mav_put_float(buf, 16, next_wp_lat);
	_mav_put_float(buf, 20, next_wp_lon);
	_mav_put_float(buf, 24, next_wp_north);
	_mav_put_float(buf, 28, next_wp_east);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_WAYPOINT_STATUS_LEN);
#else
	mavlink_waypoint_status_t packet;
	packet.last_wp_lat = last_wp_lat;
	packet.last_wp_lon = last_wp_lon;
	packet.last_wp_north = last_wp_north;
	packet.last_wp_east = last_wp_east;
	packet.next_wp_lat = next_wp_lat;
	packet.next_wp_lon = next_wp_lon;
	packet.next_wp_north = next_wp_north;
	packet.next_wp_east = next_wp_east;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_WAYPOINT_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_WAYPOINT_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WAYPOINT_STATUS_LEN, MAVLINK_MSG_ID_WAYPOINT_STATUS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_WAYPOINT_STATUS_LEN);
#endif
}

/**
 * @brief Encode a waypoint_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param waypoint_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_waypoint_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_waypoint_status_t* waypoint_status)
{
	return mavlink_msg_waypoint_status_pack(system_id, component_id, msg, waypoint_status->last_wp_lat, waypoint_status->last_wp_lon, waypoint_status->last_wp_north, waypoint_status->last_wp_east, waypoint_status->next_wp_lat, waypoint_status->next_wp_lon, waypoint_status->next_wp_north, waypoint_status->next_wp_east);
}

/**
 * @brief Encode a waypoint_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param waypoint_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_waypoint_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_waypoint_status_t* waypoint_status)
{
	return mavlink_msg_waypoint_status_pack_chan(system_id, component_id, chan, msg, waypoint_status->last_wp_lat, waypoint_status->last_wp_lon, waypoint_status->last_wp_north, waypoint_status->last_wp_east, waypoint_status->next_wp_lat, waypoint_status->next_wp_lon, waypoint_status->next_wp_north, waypoint_status->next_wp_east);
}

/**
 * @brief Send a waypoint_status message
 * @param chan MAVLink channel to send the message
 *
 * @param last_wp_lat The latitude of the last waypoint.
 * @param last_wp_lon The longitude of the last waypoint.
 * @param last_wp_north The north component of the local coordinates of the last waypoint.
 * @param last_wp_east The east component of the local coordinates of the last waypoint.
 * @param next_wp_lat The latitude of the next waypoint.
 * @param next_wp_lon The longitude of the next waypoint.
 * @param next_wp_north The north component of the local coordinates of the next waypoint.
 * @param next_wp_east The east component of the local coordinates of the next waypoint.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_waypoint_status_send(mavlink_channel_t chan, float last_wp_lat, float last_wp_lon, float last_wp_north, float last_wp_east, float next_wp_lat, float next_wp_lon, float next_wp_north, float next_wp_east)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_WAYPOINT_STATUS_LEN];
	_mav_put_float(buf, 0, last_wp_lat);
	_mav_put_float(buf, 4, last_wp_lon);
	_mav_put_float(buf, 8, last_wp_north);
	_mav_put_float(buf, 12, last_wp_east);
	_mav_put_float(buf, 16, next_wp_lat);
	_mav_put_float(buf, 20, next_wp_lon);
	_mav_put_float(buf, 24, next_wp_north);
	_mav_put_float(buf, 28, next_wp_east);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_STATUS, buf, MAVLINK_MSG_ID_WAYPOINT_STATUS_LEN, MAVLINK_MSG_ID_WAYPOINT_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_STATUS, buf, MAVLINK_MSG_ID_WAYPOINT_STATUS_LEN);
#endif
#else
	mavlink_waypoint_status_t packet;
	packet.last_wp_lat = last_wp_lat;
	packet.last_wp_lon = last_wp_lon;
	packet.last_wp_north = last_wp_north;
	packet.last_wp_east = last_wp_east;
	packet.next_wp_lat = next_wp_lat;
	packet.next_wp_lon = next_wp_lon;
	packet.next_wp_north = next_wp_north;
	packet.next_wp_east = next_wp_east;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_STATUS, (const char *)&packet, MAVLINK_MSG_ID_WAYPOINT_STATUS_LEN, MAVLINK_MSG_ID_WAYPOINT_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_WAYPOINT_STATUS, (const char *)&packet, MAVLINK_MSG_ID_WAYPOINT_STATUS_LEN);
#endif
#endif
}

#endif

// MESSAGE WAYPOINT_STATUS UNPACKING


/**
 * @brief Get field last_wp_lat from waypoint_status message
 *
 * @return The latitude of the last waypoint.
 */
static inline float mavlink_msg_waypoint_status_get_last_wp_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field last_wp_lon from waypoint_status message
 *
 * @return The longitude of the last waypoint.
 */
static inline float mavlink_msg_waypoint_status_get_last_wp_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field last_wp_north from waypoint_status message
 *
 * @return The north component of the local coordinates of the last waypoint.
 */
static inline float mavlink_msg_waypoint_status_get_last_wp_north(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field last_wp_east from waypoint_status message
 *
 * @return The east component of the local coordinates of the last waypoint.
 */
static inline float mavlink_msg_waypoint_status_get_last_wp_east(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field next_wp_lat from waypoint_status message
 *
 * @return The latitude of the next waypoint.
 */
static inline float mavlink_msg_waypoint_status_get_next_wp_lat(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Get field next_wp_lon from waypoint_status message
 *
 * @return The longitude of the next waypoint.
 */
static inline float mavlink_msg_waypoint_status_get_next_wp_lon(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  20);
}

/**
 * @brief Get field next_wp_north from waypoint_status message
 *
 * @return The north component of the local coordinates of the next waypoint.
 */
static inline float mavlink_msg_waypoint_status_get_next_wp_north(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  24);
}

/**
 * @brief Get field next_wp_east from waypoint_status message
 *
 * @return The east component of the local coordinates of the next waypoint.
 */
static inline float mavlink_msg_waypoint_status_get_next_wp_east(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  28);
}

/**
 * @brief Decode a waypoint_status message into a struct
 *
 * @param msg The message to decode
 * @param waypoint_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_waypoint_status_decode(const mavlink_message_t* msg, mavlink_waypoint_status_t* waypoint_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	waypoint_status->last_wp_lat = mavlink_msg_waypoint_status_get_last_wp_lat(msg);
	waypoint_status->last_wp_lon = mavlink_msg_waypoint_status_get_last_wp_lon(msg);
	waypoint_status->last_wp_north = mavlink_msg_waypoint_status_get_last_wp_north(msg);
	waypoint_status->last_wp_east = mavlink_msg_waypoint_status_get_last_wp_east(msg);
	waypoint_status->next_wp_lat = mavlink_msg_waypoint_status_get_next_wp_lat(msg);
	waypoint_status->next_wp_lon = mavlink_msg_waypoint_status_get_next_wp_lon(msg);
	waypoint_status->next_wp_north = mavlink_msg_waypoint_status_get_next_wp_north(msg);
	waypoint_status->next_wp_east = mavlink_msg_waypoint_status_get_next_wp_east(msg);
#else
	memcpy(waypoint_status, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_WAYPOINT_STATUS_LEN);
#endif
}
