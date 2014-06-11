// MESSAGE NODE_STATUS PACKING

#define MAVLINK_MSG_ID_NODE_STATUS 173

typedef struct __mavlink_node_status_t
{
 uint16_t hil_status; ///< Status bitfield for the HIL node. Consult HilNode.h for details.
 uint16_t hil_errors; ///< Reset bitfield for the HIL node. Consult HilNode.h for details.
 uint16_t imu_status; ///< Status bitfield for the imu node. Consult ImuNode.h for details.
 uint16_t imu_errors; ///< Reset bitfield for the imu node. Consult ImutNode.h for details.
 uint16_t power_status; ///< Status bitfield for the power node. Consult PowerNode.h for details.
 uint16_t power_errors; ///< Reset bitfield for the power node. Consult PowerNode.h for details.
 uint16_t primary_status; ///< Status bitfield for the Primary Node. Consult PrimaryNode.h for details.
 uint16_t primary_errors; ///< Reset bitfield for the Primary Node. Consult PrimaryNode.h for details.
 uint16_t rc_status; ///< Status bitfield for the RC node. Consult RcNode.h for details.
 uint16_t rc_errors; ///< Reset bitfield for the RC node. Consult RcNode.h for details.
 uint16_t rudder_status; ///< Status bitfield for the rudder node. Consult RudderNode.h for details.
 uint16_t rudder_errors; ///< Reset bitfield for the rudder node. Consult RudderNode.h for details.
 int8_t hil_temp; ///< The onboard temperature of the HIL node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 uint8_t hil_load; ///< The onboard CPU load of the HIL node in units of 1%. UINT8_MAX if unmeasured/invalid.
 uint8_t hil_voltage; ///< The onboard unregulated voltage of the HIL node in units of .1V. UINT8_MAX if unmeasured/invalid.
 int8_t imu_temp; ///< The onboard temperature of the imu node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 uint8_t imu_load; ///< The onboard CPU load of the imu node in units of 1%. UINT8_MAX if unmeasured/invalid.
 uint8_t imu_voltage; ///< The onboard unregulated voltage of the IMU node in units of .1V. UINT8_MAX if unmeasured/invalid.
 int8_t power_temp; ///< The onboard temperature of the power node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 uint8_t power_load; ///< The onboard CPU load of the power node in units of 1%. UINT8_MAX if unmeasured/invalid.
 uint8_t power_voltage; ///< The onboard unregulated voltage of the power node in units of .1V. UINT8_MAX if unmeasured/invalid.
 int8_t primary_temp; ///< The onboard temperature of the primary node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 uint8_t primary_load; ///< The onboard CPU load of the primary node in units of 1%. UINT8_MAX if unmeasured/invalid.
 uint8_t primary_voltage; ///< The onboard unregulated voltage of the primary node in units of .1V. UINT8_MAX if unmeasured/invalid.
 int8_t rc_temp; ///< The onboard temperature of the RC node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 uint8_t rc_load; ///< The onboard CPU load of the RC node in units of 1%. UINT8_MAX if unmeasured/invalid.
 uint8_t rc_voltage; ///< The onboard unregulated voltage of the RC node in units of .1V. UINT8_MAX if unmeasured/invalid.
 int8_t rudder_temp; ///< The onboard temperature of the rudder node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 uint8_t rudder_load; ///< The onboard CPU load of the rudder node in units of 1%. UINT8_MAX if unmeasured/invalid.
 uint8_t rudder_voltage; ///< The onboard unregulated voltage of the rudder node in units of .1V. UINT8_MAX if unmeasured/invalid.
} mavlink_node_status_t;

#define MAVLINK_MSG_ID_NODE_STATUS_LEN 42
#define MAVLINK_MSG_ID_173_LEN 42

#define MAVLINK_MSG_ID_NODE_STATUS_CRC 78
#define MAVLINK_MSG_ID_173_CRC 78



#define MAVLINK_MESSAGE_INFO_NODE_STATUS { \
	"NODE_STATUS", \
	30, \
	{  { "hil_status", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_node_status_t, hil_status) }, \
         { "hil_errors", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_node_status_t, hil_errors) }, \
         { "imu_status", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_node_status_t, imu_status) }, \
         { "imu_errors", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_node_status_t, imu_errors) }, \
         { "power_status", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_node_status_t, power_status) }, \
         { "power_errors", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_node_status_t, power_errors) }, \
         { "primary_status", NULL, MAVLINK_TYPE_UINT16_T, 0, 12, offsetof(mavlink_node_status_t, primary_status) }, \
         { "primary_errors", NULL, MAVLINK_TYPE_UINT16_T, 0, 14, offsetof(mavlink_node_status_t, primary_errors) }, \
         { "rc_status", NULL, MAVLINK_TYPE_UINT16_T, 0, 16, offsetof(mavlink_node_status_t, rc_status) }, \
         { "rc_errors", NULL, MAVLINK_TYPE_UINT16_T, 0, 18, offsetof(mavlink_node_status_t, rc_errors) }, \
         { "rudder_status", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_node_status_t, rudder_status) }, \
         { "rudder_errors", NULL, MAVLINK_TYPE_UINT16_T, 0, 22, offsetof(mavlink_node_status_t, rudder_errors) }, \
         { "hil_temp", NULL, MAVLINK_TYPE_INT8_T, 0, 24, offsetof(mavlink_node_status_t, hil_temp) }, \
         { "hil_load", NULL, MAVLINK_TYPE_UINT8_T, 0, 25, offsetof(mavlink_node_status_t, hil_load) }, \
         { "hil_voltage", NULL, MAVLINK_TYPE_UINT8_T, 0, 26, offsetof(mavlink_node_status_t, hil_voltage) }, \
         { "imu_temp", NULL, MAVLINK_TYPE_INT8_T, 0, 27, offsetof(mavlink_node_status_t, imu_temp) }, \
         { "imu_load", NULL, MAVLINK_TYPE_UINT8_T, 0, 28, offsetof(mavlink_node_status_t, imu_load) }, \
         { "imu_voltage", NULL, MAVLINK_TYPE_UINT8_T, 0, 29, offsetof(mavlink_node_status_t, imu_voltage) }, \
         { "power_temp", NULL, MAVLINK_TYPE_INT8_T, 0, 30, offsetof(mavlink_node_status_t, power_temp) }, \
         { "power_load", NULL, MAVLINK_TYPE_UINT8_T, 0, 31, offsetof(mavlink_node_status_t, power_load) }, \
         { "power_voltage", NULL, MAVLINK_TYPE_UINT8_T, 0, 32, offsetof(mavlink_node_status_t, power_voltage) }, \
         { "primary_temp", NULL, MAVLINK_TYPE_INT8_T, 0, 33, offsetof(mavlink_node_status_t, primary_temp) }, \
         { "primary_load", NULL, MAVLINK_TYPE_UINT8_T, 0, 34, offsetof(mavlink_node_status_t, primary_load) }, \
         { "primary_voltage", NULL, MAVLINK_TYPE_UINT8_T, 0, 35, offsetof(mavlink_node_status_t, primary_voltage) }, \
         { "rc_temp", NULL, MAVLINK_TYPE_INT8_T, 0, 36, offsetof(mavlink_node_status_t, rc_temp) }, \
         { "rc_load", NULL, MAVLINK_TYPE_UINT8_T, 0, 37, offsetof(mavlink_node_status_t, rc_load) }, \
         { "rc_voltage", NULL, MAVLINK_TYPE_UINT8_T, 0, 38, offsetof(mavlink_node_status_t, rc_voltage) }, \
         { "rudder_temp", NULL, MAVLINK_TYPE_INT8_T, 0, 39, offsetof(mavlink_node_status_t, rudder_temp) }, \
         { "rudder_load", NULL, MAVLINK_TYPE_UINT8_T, 0, 40, offsetof(mavlink_node_status_t, rudder_load) }, \
         { "rudder_voltage", NULL, MAVLINK_TYPE_UINT8_T, 0, 41, offsetof(mavlink_node_status_t, rudder_voltage) }, \
         } \
}


/**
 * @brief Pack a node_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param hil_status Status bitfield for the HIL node. Consult HilNode.h for details.
 * @param hil_errors Reset bitfield for the HIL node. Consult HilNode.h for details.
 * @param hil_temp The onboard temperature of the HIL node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 * @param hil_load The onboard CPU load of the HIL node in units of 1%. UINT8_MAX if unmeasured/invalid.
 * @param hil_voltage The onboard unregulated voltage of the HIL node in units of .1V. UINT8_MAX if unmeasured/invalid.
 * @param imu_status Status bitfield for the imu node. Consult ImuNode.h for details.
 * @param imu_errors Reset bitfield for the imu node. Consult ImutNode.h for details.
 * @param imu_temp The onboard temperature of the imu node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 * @param imu_load The onboard CPU load of the imu node in units of 1%. UINT8_MAX if unmeasured/invalid.
 * @param imu_voltage The onboard unregulated voltage of the IMU node in units of .1V. UINT8_MAX if unmeasured/invalid.
 * @param power_status Status bitfield for the power node. Consult PowerNode.h for details.
 * @param power_errors Reset bitfield for the power node. Consult PowerNode.h for details.
 * @param power_temp The onboard temperature of the power node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 * @param power_load The onboard CPU load of the power node in units of 1%. UINT8_MAX if unmeasured/invalid.
 * @param power_voltage The onboard unregulated voltage of the power node in units of .1V. UINT8_MAX if unmeasured/invalid.
 * @param primary_status Status bitfield for the Primary Node. Consult PrimaryNode.h for details.
 * @param primary_errors Reset bitfield for the Primary Node. Consult PrimaryNode.h for details.
 * @param primary_temp The onboard temperature of the primary node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 * @param primary_load The onboard CPU load of the primary node in units of 1%. UINT8_MAX if unmeasured/invalid.
 * @param primary_voltage The onboard unregulated voltage of the primary node in units of .1V. UINT8_MAX if unmeasured/invalid.
 * @param rc_status Status bitfield for the RC node. Consult RcNode.h for details.
 * @param rc_errors Reset bitfield for the RC node. Consult RcNode.h for details.
 * @param rc_temp The onboard temperature of the RC node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 * @param rc_load The onboard CPU load of the RC node in units of 1%. UINT8_MAX if unmeasured/invalid.
 * @param rc_voltage The onboard unregulated voltage of the RC node in units of .1V. UINT8_MAX if unmeasured/invalid.
 * @param rudder_status Status bitfield for the rudder node. Consult RudderNode.h for details.
 * @param rudder_errors Reset bitfield for the rudder node. Consult RudderNode.h for details.
 * @param rudder_temp The onboard temperature of the rudder node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 * @param rudder_load The onboard CPU load of the rudder node in units of 1%. UINT8_MAX if unmeasured/invalid.
 * @param rudder_voltage The onboard unregulated voltage of the rudder node in units of .1V. UINT8_MAX if unmeasured/invalid.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_node_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t hil_status, uint16_t hil_errors, int8_t hil_temp, uint8_t hil_load, uint8_t hil_voltage, uint16_t imu_status, uint16_t imu_errors, int8_t imu_temp, uint8_t imu_load, uint8_t imu_voltage, uint16_t power_status, uint16_t power_errors, int8_t power_temp, uint8_t power_load, uint8_t power_voltage, uint16_t primary_status, uint16_t primary_errors, int8_t primary_temp, uint8_t primary_load, uint8_t primary_voltage, uint16_t rc_status, uint16_t rc_errors, int8_t rc_temp, uint8_t rc_load, uint8_t rc_voltage, uint16_t rudder_status, uint16_t rudder_errors, int8_t rudder_temp, uint8_t rudder_load, uint8_t rudder_voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NODE_STATUS_LEN];
	_mav_put_uint16_t(buf, 0, hil_status);
	_mav_put_uint16_t(buf, 2, hil_errors);
	_mav_put_uint16_t(buf, 4, imu_status);
	_mav_put_uint16_t(buf, 6, imu_errors);
	_mav_put_uint16_t(buf, 8, power_status);
	_mav_put_uint16_t(buf, 10, power_errors);
	_mav_put_uint16_t(buf, 12, primary_status);
	_mav_put_uint16_t(buf, 14, primary_errors);
	_mav_put_uint16_t(buf, 16, rc_status);
	_mav_put_uint16_t(buf, 18, rc_errors);
	_mav_put_uint16_t(buf, 20, rudder_status);
	_mav_put_uint16_t(buf, 22, rudder_errors);
	_mav_put_int8_t(buf, 24, hil_temp);
	_mav_put_uint8_t(buf, 25, hil_load);
	_mav_put_uint8_t(buf, 26, hil_voltage);
	_mav_put_int8_t(buf, 27, imu_temp);
	_mav_put_uint8_t(buf, 28, imu_load);
	_mav_put_uint8_t(buf, 29, imu_voltage);
	_mav_put_int8_t(buf, 30, power_temp);
	_mav_put_uint8_t(buf, 31, power_load);
	_mav_put_uint8_t(buf, 32, power_voltage);
	_mav_put_int8_t(buf, 33, primary_temp);
	_mav_put_uint8_t(buf, 34, primary_load);
	_mav_put_uint8_t(buf, 35, primary_voltage);
	_mav_put_int8_t(buf, 36, rc_temp);
	_mav_put_uint8_t(buf, 37, rc_load);
	_mav_put_uint8_t(buf, 38, rc_voltage);
	_mav_put_int8_t(buf, 39, rudder_temp);
	_mav_put_uint8_t(buf, 40, rudder_load);
	_mav_put_uint8_t(buf, 41, rudder_voltage);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NODE_STATUS_LEN);
#else
	mavlink_node_status_t packet;
	packet.hil_status = hil_status;
	packet.hil_errors = hil_errors;
	packet.imu_status = imu_status;
	packet.imu_errors = imu_errors;
	packet.power_status = power_status;
	packet.power_errors = power_errors;
	packet.primary_status = primary_status;
	packet.primary_errors = primary_errors;
	packet.rc_status = rc_status;
	packet.rc_errors = rc_errors;
	packet.rudder_status = rudder_status;
	packet.rudder_errors = rudder_errors;
	packet.hil_temp = hil_temp;
	packet.hil_load = hil_load;
	packet.hil_voltage = hil_voltage;
	packet.imu_temp = imu_temp;
	packet.imu_load = imu_load;
	packet.imu_voltage = imu_voltage;
	packet.power_temp = power_temp;
	packet.power_load = power_load;
	packet.power_voltage = power_voltage;
	packet.primary_temp = primary_temp;
	packet.primary_load = primary_load;
	packet.primary_voltage = primary_voltage;
	packet.rc_temp = rc_temp;
	packet.rc_load = rc_load;
	packet.rc_voltage = rc_voltage;
	packet.rudder_temp = rudder_temp;
	packet.rudder_load = rudder_load;
	packet.rudder_voltage = rudder_voltage;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NODE_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_NODE_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NODE_STATUS_LEN, MAVLINK_MSG_ID_NODE_STATUS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_NODE_STATUS_LEN);
#endif
}

/**
 * @brief Pack a node_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param hil_status Status bitfield for the HIL node. Consult HilNode.h for details.
 * @param hil_errors Reset bitfield for the HIL node. Consult HilNode.h for details.
 * @param hil_temp The onboard temperature of the HIL node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 * @param hil_load The onboard CPU load of the HIL node in units of 1%. UINT8_MAX if unmeasured/invalid.
 * @param hil_voltage The onboard unregulated voltage of the HIL node in units of .1V. UINT8_MAX if unmeasured/invalid.
 * @param imu_status Status bitfield for the imu node. Consult ImuNode.h for details.
 * @param imu_errors Reset bitfield for the imu node. Consult ImutNode.h for details.
 * @param imu_temp The onboard temperature of the imu node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 * @param imu_load The onboard CPU load of the imu node in units of 1%. UINT8_MAX if unmeasured/invalid.
 * @param imu_voltage The onboard unregulated voltage of the IMU node in units of .1V. UINT8_MAX if unmeasured/invalid.
 * @param power_status Status bitfield for the power node. Consult PowerNode.h for details.
 * @param power_errors Reset bitfield for the power node. Consult PowerNode.h for details.
 * @param power_temp The onboard temperature of the power node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 * @param power_load The onboard CPU load of the power node in units of 1%. UINT8_MAX if unmeasured/invalid.
 * @param power_voltage The onboard unregulated voltage of the power node in units of .1V. UINT8_MAX if unmeasured/invalid.
 * @param primary_status Status bitfield for the Primary Node. Consult PrimaryNode.h for details.
 * @param primary_errors Reset bitfield for the Primary Node. Consult PrimaryNode.h for details.
 * @param primary_temp The onboard temperature of the primary node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 * @param primary_load The onboard CPU load of the primary node in units of 1%. UINT8_MAX if unmeasured/invalid.
 * @param primary_voltage The onboard unregulated voltage of the primary node in units of .1V. UINT8_MAX if unmeasured/invalid.
 * @param rc_status Status bitfield for the RC node. Consult RcNode.h for details.
 * @param rc_errors Reset bitfield for the RC node. Consult RcNode.h for details.
 * @param rc_temp The onboard temperature of the RC node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 * @param rc_load The onboard CPU load of the RC node in units of 1%. UINT8_MAX if unmeasured/invalid.
 * @param rc_voltage The onboard unregulated voltage of the RC node in units of .1V. UINT8_MAX if unmeasured/invalid.
 * @param rudder_status Status bitfield for the rudder node. Consult RudderNode.h for details.
 * @param rudder_errors Reset bitfield for the rudder node. Consult RudderNode.h for details.
 * @param rudder_temp The onboard temperature of the rudder node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 * @param rudder_load The onboard CPU load of the rudder node in units of 1%. UINT8_MAX if unmeasured/invalid.
 * @param rudder_voltage The onboard unregulated voltage of the rudder node in units of .1V. UINT8_MAX if unmeasured/invalid.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_node_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t hil_status,uint16_t hil_errors,int8_t hil_temp,uint8_t hil_load,uint8_t hil_voltage,uint16_t imu_status,uint16_t imu_errors,int8_t imu_temp,uint8_t imu_load,uint8_t imu_voltage,uint16_t power_status,uint16_t power_errors,int8_t power_temp,uint8_t power_load,uint8_t power_voltage,uint16_t primary_status,uint16_t primary_errors,int8_t primary_temp,uint8_t primary_load,uint8_t primary_voltage,uint16_t rc_status,uint16_t rc_errors,int8_t rc_temp,uint8_t rc_load,uint8_t rc_voltage,uint16_t rudder_status,uint16_t rudder_errors,int8_t rudder_temp,uint8_t rudder_load,uint8_t rudder_voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NODE_STATUS_LEN];
	_mav_put_uint16_t(buf, 0, hil_status);
	_mav_put_uint16_t(buf, 2, hil_errors);
	_mav_put_uint16_t(buf, 4, imu_status);
	_mav_put_uint16_t(buf, 6, imu_errors);
	_mav_put_uint16_t(buf, 8, power_status);
	_mav_put_uint16_t(buf, 10, power_errors);
	_mav_put_uint16_t(buf, 12, primary_status);
	_mav_put_uint16_t(buf, 14, primary_errors);
	_mav_put_uint16_t(buf, 16, rc_status);
	_mav_put_uint16_t(buf, 18, rc_errors);
	_mav_put_uint16_t(buf, 20, rudder_status);
	_mav_put_uint16_t(buf, 22, rudder_errors);
	_mav_put_int8_t(buf, 24, hil_temp);
	_mav_put_uint8_t(buf, 25, hil_load);
	_mav_put_uint8_t(buf, 26, hil_voltage);
	_mav_put_int8_t(buf, 27, imu_temp);
	_mav_put_uint8_t(buf, 28, imu_load);
	_mav_put_uint8_t(buf, 29, imu_voltage);
	_mav_put_int8_t(buf, 30, power_temp);
	_mav_put_uint8_t(buf, 31, power_load);
	_mav_put_uint8_t(buf, 32, power_voltage);
	_mav_put_int8_t(buf, 33, primary_temp);
	_mav_put_uint8_t(buf, 34, primary_load);
	_mav_put_uint8_t(buf, 35, primary_voltage);
	_mav_put_int8_t(buf, 36, rc_temp);
	_mav_put_uint8_t(buf, 37, rc_load);
	_mav_put_uint8_t(buf, 38, rc_voltage);
	_mav_put_int8_t(buf, 39, rudder_temp);
	_mav_put_uint8_t(buf, 40, rudder_load);
	_mav_put_uint8_t(buf, 41, rudder_voltage);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_NODE_STATUS_LEN);
#else
	mavlink_node_status_t packet;
	packet.hil_status = hil_status;
	packet.hil_errors = hil_errors;
	packet.imu_status = imu_status;
	packet.imu_errors = imu_errors;
	packet.power_status = power_status;
	packet.power_errors = power_errors;
	packet.primary_status = primary_status;
	packet.primary_errors = primary_errors;
	packet.rc_status = rc_status;
	packet.rc_errors = rc_errors;
	packet.rudder_status = rudder_status;
	packet.rudder_errors = rudder_errors;
	packet.hil_temp = hil_temp;
	packet.hil_load = hil_load;
	packet.hil_voltage = hil_voltage;
	packet.imu_temp = imu_temp;
	packet.imu_load = imu_load;
	packet.imu_voltage = imu_voltage;
	packet.power_temp = power_temp;
	packet.power_load = power_load;
	packet.power_voltage = power_voltage;
	packet.primary_temp = primary_temp;
	packet.primary_load = primary_load;
	packet.primary_voltage = primary_voltage;
	packet.rc_temp = rc_temp;
	packet.rc_load = rc_load;
	packet.rc_voltage = rc_voltage;
	packet.rudder_temp = rudder_temp;
	packet.rudder_load = rudder_load;
	packet.rudder_voltage = rudder_voltage;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_NODE_STATUS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_NODE_STATUS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NODE_STATUS_LEN, MAVLINK_MSG_ID_NODE_STATUS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_NODE_STATUS_LEN);
#endif
}

/**
 * @brief Encode a node_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param node_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_node_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_node_status_t* node_status)
{
	return mavlink_msg_node_status_pack(system_id, component_id, msg, node_status->hil_status, node_status->hil_errors, node_status->hil_temp, node_status->hil_load, node_status->hil_voltage, node_status->imu_status, node_status->imu_errors, node_status->imu_temp, node_status->imu_load, node_status->imu_voltage, node_status->power_status, node_status->power_errors, node_status->power_temp, node_status->power_load, node_status->power_voltage, node_status->primary_status, node_status->primary_errors, node_status->primary_temp, node_status->primary_load, node_status->primary_voltage, node_status->rc_status, node_status->rc_errors, node_status->rc_temp, node_status->rc_load, node_status->rc_voltage, node_status->rudder_status, node_status->rudder_errors, node_status->rudder_temp, node_status->rudder_load, node_status->rudder_voltage);
}

/**
 * @brief Encode a node_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param node_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_node_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_node_status_t* node_status)
{
	return mavlink_msg_node_status_pack_chan(system_id, component_id, chan, msg, node_status->hil_status, node_status->hil_errors, node_status->hil_temp, node_status->hil_load, node_status->hil_voltage, node_status->imu_status, node_status->imu_errors, node_status->imu_temp, node_status->imu_load, node_status->imu_voltage, node_status->power_status, node_status->power_errors, node_status->power_temp, node_status->power_load, node_status->power_voltage, node_status->primary_status, node_status->primary_errors, node_status->primary_temp, node_status->primary_load, node_status->primary_voltage, node_status->rc_status, node_status->rc_errors, node_status->rc_temp, node_status->rc_load, node_status->rc_voltage, node_status->rudder_status, node_status->rudder_errors, node_status->rudder_temp, node_status->rudder_load, node_status->rudder_voltage);
}

/**
 * @brief Send a node_status message
 * @param chan MAVLink channel to send the message
 *
 * @param hil_status Status bitfield for the HIL node. Consult HilNode.h for details.
 * @param hil_errors Reset bitfield for the HIL node. Consult HilNode.h for details.
 * @param hil_temp The onboard temperature of the HIL node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 * @param hil_load The onboard CPU load of the HIL node in units of 1%. UINT8_MAX if unmeasured/invalid.
 * @param hil_voltage The onboard unregulated voltage of the HIL node in units of .1V. UINT8_MAX if unmeasured/invalid.
 * @param imu_status Status bitfield for the imu node. Consult ImuNode.h for details.
 * @param imu_errors Reset bitfield for the imu node. Consult ImutNode.h for details.
 * @param imu_temp The onboard temperature of the imu node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 * @param imu_load The onboard CPU load of the imu node in units of 1%. UINT8_MAX if unmeasured/invalid.
 * @param imu_voltage The onboard unregulated voltage of the IMU node in units of .1V. UINT8_MAX if unmeasured/invalid.
 * @param power_status Status bitfield for the power node. Consult PowerNode.h for details.
 * @param power_errors Reset bitfield for the power node. Consult PowerNode.h for details.
 * @param power_temp The onboard temperature of the power node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 * @param power_load The onboard CPU load of the power node in units of 1%. UINT8_MAX if unmeasured/invalid.
 * @param power_voltage The onboard unregulated voltage of the power node in units of .1V. UINT8_MAX if unmeasured/invalid.
 * @param primary_status Status bitfield for the Primary Node. Consult PrimaryNode.h for details.
 * @param primary_errors Reset bitfield for the Primary Node. Consult PrimaryNode.h for details.
 * @param primary_temp The onboard temperature of the primary node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 * @param primary_load The onboard CPU load of the primary node in units of 1%. UINT8_MAX if unmeasured/invalid.
 * @param primary_voltage The onboard unregulated voltage of the primary node in units of .1V. UINT8_MAX if unmeasured/invalid.
 * @param rc_status Status bitfield for the RC node. Consult RcNode.h for details.
 * @param rc_errors Reset bitfield for the RC node. Consult RcNode.h for details.
 * @param rc_temp The onboard temperature of the RC node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 * @param rc_load The onboard CPU load of the RC node in units of 1%. UINT8_MAX if unmeasured/invalid.
 * @param rc_voltage The onboard unregulated voltage of the RC node in units of .1V. UINT8_MAX if unmeasured/invalid.
 * @param rudder_status Status bitfield for the rudder node. Consult RudderNode.h for details.
 * @param rudder_errors Reset bitfield for the rudder node. Consult RudderNode.h for details.
 * @param rudder_temp The onboard temperature of the rudder node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 * @param rudder_load The onboard CPU load of the rudder node in units of 1%. UINT8_MAX if unmeasured/invalid.
 * @param rudder_voltage The onboard unregulated voltage of the rudder node in units of .1V. UINT8_MAX if unmeasured/invalid.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_node_status_send(mavlink_channel_t chan, uint16_t hil_status, uint16_t hil_errors, int8_t hil_temp, uint8_t hil_load, uint8_t hil_voltage, uint16_t imu_status, uint16_t imu_errors, int8_t imu_temp, uint8_t imu_load, uint8_t imu_voltage, uint16_t power_status, uint16_t power_errors, int8_t power_temp, uint8_t power_load, uint8_t power_voltage, uint16_t primary_status, uint16_t primary_errors, int8_t primary_temp, uint8_t primary_load, uint8_t primary_voltage, uint16_t rc_status, uint16_t rc_errors, int8_t rc_temp, uint8_t rc_load, uint8_t rc_voltage, uint16_t rudder_status, uint16_t rudder_errors, int8_t rudder_temp, uint8_t rudder_load, uint8_t rudder_voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_NODE_STATUS_LEN];
	_mav_put_uint16_t(buf, 0, hil_status);
	_mav_put_uint16_t(buf, 2, hil_errors);
	_mav_put_uint16_t(buf, 4, imu_status);
	_mav_put_uint16_t(buf, 6, imu_errors);
	_mav_put_uint16_t(buf, 8, power_status);
	_mav_put_uint16_t(buf, 10, power_errors);
	_mav_put_uint16_t(buf, 12, primary_status);
	_mav_put_uint16_t(buf, 14, primary_errors);
	_mav_put_uint16_t(buf, 16, rc_status);
	_mav_put_uint16_t(buf, 18, rc_errors);
	_mav_put_uint16_t(buf, 20, rudder_status);
	_mav_put_uint16_t(buf, 22, rudder_errors);
	_mav_put_int8_t(buf, 24, hil_temp);
	_mav_put_uint8_t(buf, 25, hil_load);
	_mav_put_uint8_t(buf, 26, hil_voltage);
	_mav_put_int8_t(buf, 27, imu_temp);
	_mav_put_uint8_t(buf, 28, imu_load);
	_mav_put_uint8_t(buf, 29, imu_voltage);
	_mav_put_int8_t(buf, 30, power_temp);
	_mav_put_uint8_t(buf, 31, power_load);
	_mav_put_uint8_t(buf, 32, power_voltage);
	_mav_put_int8_t(buf, 33, primary_temp);
	_mav_put_uint8_t(buf, 34, primary_load);
	_mav_put_uint8_t(buf, 35, primary_voltage);
	_mav_put_int8_t(buf, 36, rc_temp);
	_mav_put_uint8_t(buf, 37, rc_load);
	_mav_put_uint8_t(buf, 38, rc_voltage);
	_mav_put_int8_t(buf, 39, rudder_temp);
	_mav_put_uint8_t(buf, 40, rudder_load);
	_mav_put_uint8_t(buf, 41, rudder_voltage);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_STATUS, buf, MAVLINK_MSG_ID_NODE_STATUS_LEN, MAVLINK_MSG_ID_NODE_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_STATUS, buf, MAVLINK_MSG_ID_NODE_STATUS_LEN);
#endif
#else
	mavlink_node_status_t packet;
	packet.hil_status = hil_status;
	packet.hil_errors = hil_errors;
	packet.imu_status = imu_status;
	packet.imu_errors = imu_errors;
	packet.power_status = power_status;
	packet.power_errors = power_errors;
	packet.primary_status = primary_status;
	packet.primary_errors = primary_errors;
	packet.rc_status = rc_status;
	packet.rc_errors = rc_errors;
	packet.rudder_status = rudder_status;
	packet.rudder_errors = rudder_errors;
	packet.hil_temp = hil_temp;
	packet.hil_load = hil_load;
	packet.hil_voltage = hil_voltage;
	packet.imu_temp = imu_temp;
	packet.imu_load = imu_load;
	packet.imu_voltage = imu_voltage;
	packet.power_temp = power_temp;
	packet.power_load = power_load;
	packet.power_voltage = power_voltage;
	packet.primary_temp = primary_temp;
	packet.primary_load = primary_load;
	packet.primary_voltage = primary_voltage;
	packet.rc_temp = rc_temp;
	packet.rc_load = rc_load;
	packet.rc_voltage = rc_voltage;
	packet.rudder_temp = rudder_temp;
	packet.rudder_load = rudder_load;
	packet.rudder_voltage = rudder_voltage;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_NODE_STATUS_LEN, MAVLINK_MSG_ID_NODE_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_STATUS, (const char *)&packet, MAVLINK_MSG_ID_NODE_STATUS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_NODE_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_node_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t hil_status, uint16_t hil_errors, int8_t hil_temp, uint8_t hil_load, uint8_t hil_voltage, uint16_t imu_status, uint16_t imu_errors, int8_t imu_temp, uint8_t imu_load, uint8_t imu_voltage, uint16_t power_status, uint16_t power_errors, int8_t power_temp, uint8_t power_load, uint8_t power_voltage, uint16_t primary_status, uint16_t primary_errors, int8_t primary_temp, uint8_t primary_load, uint8_t primary_voltage, uint16_t rc_status, uint16_t rc_errors, int8_t rc_temp, uint8_t rc_load, uint8_t rc_voltage, uint16_t rudder_status, uint16_t rudder_errors, int8_t rudder_temp, uint8_t rudder_load, uint8_t rudder_voltage)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, hil_status);
	_mav_put_uint16_t(buf, 2, hil_errors);
	_mav_put_uint16_t(buf, 4, imu_status);
	_mav_put_uint16_t(buf, 6, imu_errors);
	_mav_put_uint16_t(buf, 8, power_status);
	_mav_put_uint16_t(buf, 10, power_errors);
	_mav_put_uint16_t(buf, 12, primary_status);
	_mav_put_uint16_t(buf, 14, primary_errors);
	_mav_put_uint16_t(buf, 16, rc_status);
	_mav_put_uint16_t(buf, 18, rc_errors);
	_mav_put_uint16_t(buf, 20, rudder_status);
	_mav_put_uint16_t(buf, 22, rudder_errors);
	_mav_put_int8_t(buf, 24, hil_temp);
	_mav_put_uint8_t(buf, 25, hil_load);
	_mav_put_uint8_t(buf, 26, hil_voltage);
	_mav_put_int8_t(buf, 27, imu_temp);
	_mav_put_uint8_t(buf, 28, imu_load);
	_mav_put_uint8_t(buf, 29, imu_voltage);
	_mav_put_int8_t(buf, 30, power_temp);
	_mav_put_uint8_t(buf, 31, power_load);
	_mav_put_uint8_t(buf, 32, power_voltage);
	_mav_put_int8_t(buf, 33, primary_temp);
	_mav_put_uint8_t(buf, 34, primary_load);
	_mav_put_uint8_t(buf, 35, primary_voltage);
	_mav_put_int8_t(buf, 36, rc_temp);
	_mav_put_uint8_t(buf, 37, rc_load);
	_mav_put_uint8_t(buf, 38, rc_voltage);
	_mav_put_int8_t(buf, 39, rudder_temp);
	_mav_put_uint8_t(buf, 40, rudder_load);
	_mav_put_uint8_t(buf, 41, rudder_voltage);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_STATUS, buf, MAVLINK_MSG_ID_NODE_STATUS_LEN, MAVLINK_MSG_ID_NODE_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_STATUS, buf, MAVLINK_MSG_ID_NODE_STATUS_LEN);
#endif
#else
	mavlink_node_status_t *packet = (mavlink_node_status_t *)msgbuf;
	packet->hil_status = hil_status;
	packet->hil_errors = hil_errors;
	packet->imu_status = imu_status;
	packet->imu_errors = imu_errors;
	packet->power_status = power_status;
	packet->power_errors = power_errors;
	packet->primary_status = primary_status;
	packet->primary_errors = primary_errors;
	packet->rc_status = rc_status;
	packet->rc_errors = rc_errors;
	packet->rudder_status = rudder_status;
	packet->rudder_errors = rudder_errors;
	packet->hil_temp = hil_temp;
	packet->hil_load = hil_load;
	packet->hil_voltage = hil_voltage;
	packet->imu_temp = imu_temp;
	packet->imu_load = imu_load;
	packet->imu_voltage = imu_voltage;
	packet->power_temp = power_temp;
	packet->power_load = power_load;
	packet->power_voltage = power_voltage;
	packet->primary_temp = primary_temp;
	packet->primary_load = primary_load;
	packet->primary_voltage = primary_voltage;
	packet->rc_temp = rc_temp;
	packet->rc_load = rc_load;
	packet->rc_voltage = rc_voltage;
	packet->rudder_temp = rudder_temp;
	packet->rudder_load = rudder_load;
	packet->rudder_voltage = rudder_voltage;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_STATUS, (const char *)packet, MAVLINK_MSG_ID_NODE_STATUS_LEN, MAVLINK_MSG_ID_NODE_STATUS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_NODE_STATUS, (const char *)packet, MAVLINK_MSG_ID_NODE_STATUS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE NODE_STATUS UNPACKING


/**
 * @brief Get field hil_status from node_status message
 *
 * @return Status bitfield for the HIL node. Consult HilNode.h for details.
 */
static inline uint16_t mavlink_msg_node_status_get_hil_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field hil_errors from node_status message
 *
 * @return Reset bitfield for the HIL node. Consult HilNode.h for details.
 */
static inline uint16_t mavlink_msg_node_status_get_hil_errors(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field hil_temp from node_status message
 *
 * @return The onboard temperature of the HIL node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 */
static inline int8_t mavlink_msg_node_status_get_hil_temp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  24);
}

/**
 * @brief Get field hil_load from node_status message
 *
 * @return The onboard CPU load of the HIL node in units of 1%. UINT8_MAX if unmeasured/invalid.
 */
static inline uint8_t mavlink_msg_node_status_get_hil_load(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  25);
}

/**
 * @brief Get field hil_voltage from node_status message
 *
 * @return The onboard unregulated voltage of the HIL node in units of .1V. UINT8_MAX if unmeasured/invalid.
 */
static inline uint8_t mavlink_msg_node_status_get_hil_voltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  26);
}

/**
 * @brief Get field imu_status from node_status message
 *
 * @return Status bitfield for the imu node. Consult ImuNode.h for details.
 */
static inline uint16_t mavlink_msg_node_status_get_imu_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field imu_errors from node_status message
 *
 * @return Reset bitfield for the imu node. Consult ImutNode.h for details.
 */
static inline uint16_t mavlink_msg_node_status_get_imu_errors(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field imu_temp from node_status message
 *
 * @return The onboard temperature of the imu node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 */
static inline int8_t mavlink_msg_node_status_get_imu_temp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  27);
}

/**
 * @brief Get field imu_load from node_status message
 *
 * @return The onboard CPU load of the imu node in units of 1%. UINT8_MAX if unmeasured/invalid.
 */
static inline uint8_t mavlink_msg_node_status_get_imu_load(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  28);
}

/**
 * @brief Get field imu_voltage from node_status message
 *
 * @return The onboard unregulated voltage of the IMU node in units of .1V. UINT8_MAX if unmeasured/invalid.
 */
static inline uint8_t mavlink_msg_node_status_get_imu_voltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  29);
}

/**
 * @brief Get field power_status from node_status message
 *
 * @return Status bitfield for the power node. Consult PowerNode.h for details.
 */
static inline uint16_t mavlink_msg_node_status_get_power_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field power_errors from node_status message
 *
 * @return Reset bitfield for the power node. Consult PowerNode.h for details.
 */
static inline uint16_t mavlink_msg_node_status_get_power_errors(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field power_temp from node_status message
 *
 * @return The onboard temperature of the power node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 */
static inline int8_t mavlink_msg_node_status_get_power_temp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  30);
}

/**
 * @brief Get field power_load from node_status message
 *
 * @return The onboard CPU load of the power node in units of 1%. UINT8_MAX if unmeasured/invalid.
 */
static inline uint8_t mavlink_msg_node_status_get_power_load(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  31);
}

/**
 * @brief Get field power_voltage from node_status message
 *
 * @return The onboard unregulated voltage of the power node in units of .1V. UINT8_MAX if unmeasured/invalid.
 */
static inline uint8_t mavlink_msg_node_status_get_power_voltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  32);
}

/**
 * @brief Get field primary_status from node_status message
 *
 * @return Status bitfield for the Primary Node. Consult PrimaryNode.h for details.
 */
static inline uint16_t mavlink_msg_node_status_get_primary_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  12);
}

/**
 * @brief Get field primary_errors from node_status message
 *
 * @return Reset bitfield for the Primary Node. Consult PrimaryNode.h for details.
 */
static inline uint16_t mavlink_msg_node_status_get_primary_errors(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  14);
}

/**
 * @brief Get field primary_temp from node_status message
 *
 * @return The onboard temperature of the primary node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 */
static inline int8_t mavlink_msg_node_status_get_primary_temp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  33);
}

/**
 * @brief Get field primary_load from node_status message
 *
 * @return The onboard CPU load of the primary node in units of 1%. UINT8_MAX if unmeasured/invalid.
 */
static inline uint8_t mavlink_msg_node_status_get_primary_load(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  34);
}

/**
 * @brief Get field primary_voltage from node_status message
 *
 * @return The onboard unregulated voltage of the primary node in units of .1V. UINT8_MAX if unmeasured/invalid.
 */
static inline uint8_t mavlink_msg_node_status_get_primary_voltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  35);
}

/**
 * @brief Get field rc_status from node_status message
 *
 * @return Status bitfield for the RC node. Consult RcNode.h for details.
 */
static inline uint16_t mavlink_msg_node_status_get_rc_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  16);
}

/**
 * @brief Get field rc_errors from node_status message
 *
 * @return Reset bitfield for the RC node. Consult RcNode.h for details.
 */
static inline uint16_t mavlink_msg_node_status_get_rc_errors(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  18);
}

/**
 * @brief Get field rc_temp from node_status message
 *
 * @return The onboard temperature of the RC node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 */
static inline int8_t mavlink_msg_node_status_get_rc_temp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  36);
}

/**
 * @brief Get field rc_load from node_status message
 *
 * @return The onboard CPU load of the RC node in units of 1%. UINT8_MAX if unmeasured/invalid.
 */
static inline uint8_t mavlink_msg_node_status_get_rc_load(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  37);
}

/**
 * @brief Get field rc_voltage from node_status message
 *
 * @return The onboard unregulated voltage of the RC node in units of .1V. UINT8_MAX if unmeasured/invalid.
 */
static inline uint8_t mavlink_msg_node_status_get_rc_voltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  38);
}

/**
 * @brief Get field rudder_status from node_status message
 *
 * @return Status bitfield for the rudder node. Consult RudderNode.h for details.
 */
static inline uint16_t mavlink_msg_node_status_get_rudder_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field rudder_errors from node_status message
 *
 * @return Reset bitfield for the rudder node. Consult RudderNode.h for details.
 */
static inline uint16_t mavlink_msg_node_status_get_rudder_errors(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  22);
}

/**
 * @brief Get field rudder_temp from node_status message
 *
 * @return The onboard temperature of the rudder node in units of degrees Celsius. INT8_MAX if unmeasured/invalid.
 */
static inline int8_t mavlink_msg_node_status_get_rudder_temp(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int8_t(msg,  39);
}

/**
 * @brief Get field rudder_load from node_status message
 *
 * @return The onboard CPU load of the rudder node in units of 1%. UINT8_MAX if unmeasured/invalid.
 */
static inline uint8_t mavlink_msg_node_status_get_rudder_load(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  40);
}

/**
 * @brief Get field rudder_voltage from node_status message
 *
 * @return The onboard unregulated voltage of the rudder node in units of .1V. UINT8_MAX if unmeasured/invalid.
 */
static inline uint8_t mavlink_msg_node_status_get_rudder_voltage(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  41);
}

/**
 * @brief Decode a node_status message into a struct
 *
 * @param msg The message to decode
 * @param node_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_node_status_decode(const mavlink_message_t* msg, mavlink_node_status_t* node_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	node_status->hil_status = mavlink_msg_node_status_get_hil_status(msg);
	node_status->hil_errors = mavlink_msg_node_status_get_hil_errors(msg);
	node_status->imu_status = mavlink_msg_node_status_get_imu_status(msg);
	node_status->imu_errors = mavlink_msg_node_status_get_imu_errors(msg);
	node_status->power_status = mavlink_msg_node_status_get_power_status(msg);
	node_status->power_errors = mavlink_msg_node_status_get_power_errors(msg);
	node_status->primary_status = mavlink_msg_node_status_get_primary_status(msg);
	node_status->primary_errors = mavlink_msg_node_status_get_primary_errors(msg);
	node_status->rc_status = mavlink_msg_node_status_get_rc_status(msg);
	node_status->rc_errors = mavlink_msg_node_status_get_rc_errors(msg);
	node_status->rudder_status = mavlink_msg_node_status_get_rudder_status(msg);
	node_status->rudder_errors = mavlink_msg_node_status_get_rudder_errors(msg);
	node_status->hil_temp = mavlink_msg_node_status_get_hil_temp(msg);
	node_status->hil_load = mavlink_msg_node_status_get_hil_load(msg);
	node_status->hil_voltage = mavlink_msg_node_status_get_hil_voltage(msg);
	node_status->imu_temp = mavlink_msg_node_status_get_imu_temp(msg);
	node_status->imu_load = mavlink_msg_node_status_get_imu_load(msg);
	node_status->imu_voltage = mavlink_msg_node_status_get_imu_voltage(msg);
	node_status->power_temp = mavlink_msg_node_status_get_power_temp(msg);
	node_status->power_load = mavlink_msg_node_status_get_power_load(msg);
	node_status->power_voltage = mavlink_msg_node_status_get_power_voltage(msg);
	node_status->primary_temp = mavlink_msg_node_status_get_primary_temp(msg);
	node_status->primary_load = mavlink_msg_node_status_get_primary_load(msg);
	node_status->primary_voltage = mavlink_msg_node_status_get_primary_voltage(msg);
	node_status->rc_temp = mavlink_msg_node_status_get_rc_temp(msg);
	node_status->rc_load = mavlink_msg_node_status_get_rc_load(msg);
	node_status->rc_voltage = mavlink_msg_node_status_get_rc_voltage(msg);
	node_status->rudder_temp = mavlink_msg_node_status_get_rudder_temp(msg);
	node_status->rudder_load = mavlink_msg_node_status_get_rudder_load(msg);
	node_status->rudder_voltage = mavlink_msg_node_status_get_rudder_voltage(msg);
#else
	memcpy(node_status, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_NODE_STATUS_LEN);
#endif
}
