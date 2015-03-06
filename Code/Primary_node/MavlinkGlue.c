/**
 * This file contains all of the MAVLink interfacing necessary by SeaSlug.
 * It relies heavily on the MavlinkMessageScheduler for scheduling transmission
 * of MAVLink messages such as to not overload the interface.
 *
 * The main functions are at the bottom: MavLinkTransmit() and MavLinkReceive()
 * handle the dispatching of messages (and sending of non-FSM reliant ones) and
 * the reception of messages and forwarding of reception events to the relevent
 * FSMs. The two state machine functions (MavLinkEvaluateMissionState and
 * MavLinkEvaluateParameterState) both contain all of the state logic for the
 * MAVLink mission and parameter protocols. As the specifications for those two
 * protocols are not fully defined they have been tested with QGroundControl to
 * work correctly.
 *
 * This code was written to be as generic as possible. If you remove all of the
 * custom messages and switch the transmission from uart1EnqueueData() it should
 * be almost exclusively relient on modules like MissionManager and the scheduler.
 */

#include "Ecan1.h"

// C standard library includes
#include <stdio.h>

// User code includes
#include "Uart1.h"
#include "Uart2.h"
#include "MessageScheduler.h"
#include "EcanSensors.h"
#include "Rudder.h"
#include "MavlinkGlue.h"
#include "MissionManager.h"
#include "Node.h"
#include "PrimaryNode.h"
#include "Parameters.h"
#include "DataStore.h"

// MATLAB-generated code is included here, really only required for the declaration of the
// InternalVariables struct.
#include "controller.h"

// Declare our internal variable data store for some miscellaneous data output over MAVLink.
InternalVariables controllerVars;

// Store a single txMessage copy here. This is shared by all of the Send*() functions in this file
// to reduce stack usage. Note that since all Send*() functions use this shared memory, NONE of them
// are re-entrant.
// This fixes an AddressError trap bug I was experiencing when each function had their own local
// copy. This was an odd error to receive, since compilation with Stack.s forces the stack below
// the 32K boundary. This code is redundant as of the xc16-1.20 release that defaults to keeping the
// stack below this boundary and outside of extended data space.
static mavlink_message_t txMessage;

/**
 * This function converts latitude/longitude/altitude into a north/east/down local tangent plane. The
 * code I use by default is auto-generated C code from a Simulink block in the autonomous controller.
 * @param[in] lat/lon/alt in units of 1e7deg/1e7deg/1e3m.
 * @param[out] Output in a north/east/down coordinate frame in units of meters.
 */
extern void lla2ltp(const int32_t[3], float[3]);

// Set up some state machine variables for the parameter protocol.
enum PARAM_STATE {
	PARAM_STATE_INACTIVE = 0,

	PARAM_STATE_SINGLETON_SEND_VALUE,

	PARAM_STATE_STREAM_SEND_VALUE,
	PARAM_STATE_STREAM_DELAY
};

// Events that trigger changes in the parameter protocol state machine.
// This follows almost exactly from MAVLink's parameter protocol as described in their docs.
// Note, however, the addition of the TRANSMIT_ALL differs from that.
enum PARAM_EVENT {
	PARAM_EVENT_NONE,
	PARAM_EVENT_ENTER_STATE,
	PARAM_EVENT_EXIT_STATE,

	PARAM_EVENT_REQUEST_LIST_RECEIVED,
	PARAM_EVENT_REQUEST_READ_RECEIVED,
	PARAM_EVENT_SET_RECEIVED
};

// Set up state machine variables for the mission protocol
enum MISSION_STATE {
	MISSION_STATE_INACTIVE = 0,

	// States handling transmitting the mission count. It needs to be retransmit twice.
	MISSION_STATE_SEND_MISSION_COUNT,
	MISSION_STATE_MISSION_COUNT_TIMEOUT,
	MISSION_STATE_SEND_MISSION_COUNT2,
	MISSION_STATE_MISSION_COUNT_TIMEOUT2,
	MISSION_STATE_SEND_MISSION_COUNT3,
	MISSION_STATE_MISSION_COUNT_TIMEOUT3,

	// States handling transmitting a mission item. It needs to be retransmit twice.
	MISSION_STATE_SEND_MISSION_ITEM,
	MISSION_STATE_MISSION_ITEM_TIMEOUT,
	MISSION_STATE_SEND_MISSION_ITEM2,
	MISSION_STATE_MISSION_ITEM_TIMEOUT2,
	MISSION_STATE_SEND_MISSION_ITEM3,
	MISSION_STATE_MISSION_ITEM_TIMEOUT3,

	// States handling transmission of the current mission
	MISSION_STATE_SEND_CURRENT,
	MISSION_STATE_CURRENT_TIMEOUT,
	MISSION_STATE_SEND_CURRENT2,
	MISSION_STATE_CURRENT_TIMEOUT2,

	// States handling sending a mission request
	MISSION_STATE_SEND_MISSION_REQUEST,
	MISSION_STATE_MISSION_REQUEST_TIMEOUT,
	MISSION_STATE_SEND_MISSION_REQUEST2,
	MISSION_STATE_MISSION_REQUEST_TIMEOUT2,
	MISSION_STATE_SEND_MISSION_REQUEST3,
	MISSION_STATE_MISSION_REQUEST_TIMEOUT3
};

// Set up the events necessary for the mission protocol state machine.
enum MISSION_EVENT {
	MISSION_EVENT_NONE = 0,
	MISSION_EVENT_ENTER_STATE,
	MISSION_EVENT_EXIT_STATE,

	// Message reception events
	MISSION_EVENT_COUNT_RECEIVED,
	MISSION_EVENT_ACK_RECEIVED,
	MISSION_EVENT_REQUEST_RECEIVED,
	MISSION_EVENT_REQUEST_LIST_RECEIVED,
	MISSION_EVENT_CLEAR_ALL_RECEIVED,
	MISSION_EVENT_SET_CURRENT_RECEIVED,
	MISSION_EVENT_ITEM_RECEIVED
};

// These flags are for use with the SYS_STATUS MAVLink message as a mapping from the Autoboat's
// sensors to the sensors/controllers available in SYS_STATUS.
enum ONBOARD_SENSORS {
	ONBOARD_SENSORS_IMU = (1 << 0) | (1 << 1) | (1 << 2),
	ONBOARD_SENSORS_WSO100 = 1 << 3,
	ONBOARD_SENSORS_GPS = 1 << 5,
	ONBOARD_CONTROL_YAW_POS = 1 << 12,
	ONBOARD_CONTROL_XY_POS = 1 << 14,
	ONBOARD_CONTROL_MOTOR = 1 << 15
};

typedef struct {
    uint8_t sysid;
    uint8_t compid;
    uint8_t type;
    uint8_t autopilot;
    uint8_t state;
    uint8_t mode;
    uint32_t custom_mode;
} MavlinkVehicle;

// Store a module-wide variable for common MAVLink system variables.
static MavlinkVehicle mavlink_system = {
	20, // Arbitrarily chosen MAV number
	MAV_COMP_ID_ALL,
	MAV_TYPE_SURFACE_BOAT,
        MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY,
	MAV_STATE_UNINIT,
	MAV_MODE_PREFLIGHT | MAV_MODE_FLAG_MANUAL_INPUT_ENABLED // The vehicle is booting up and have manual control enabled.
};

/** manual control parameters **/
// Specify the button that must be pressed for throttle values to be recorded.
// On the Logitech F710 we use, it's the Button, which in D-mode is button 4.
#define TRIGGER_ENABLE_BUTTON 1 << 4
// Specify the button that activates rudder calibration.
// On the Logitech F710 we use, it's the Button, which in D-mode is button 8.
#define RUDDER_CAL_BUTTON     1 << 8
// Autonomous/manual switching is done through QGC

// Latch onto the first groundstation unit and only receive and transmit to it.
static uint8_t groundStationSystemId = 0;
static uint8_t groundStationComponentId = 0;

// Declare a character buffer here to prevent continual allocation/deallocation of MAVLink buffers.
static uint8_t buf[MAVLINK_MAX_PACKET_LEN];
static uint16_t len;

// Variable for counting timesteps for the delaying parameter transmission
static uint8_t parameterTimeoutCounter = 0;

// Specify how long between transmitting parameters in a parameter transmission stream. This is in
// units of timesteps. Conversion to real time depends on call rate of IncrementParameterCounter().
#define INTRA_PARAM_DELAY 1

// Internal counter variable for use with the COUNTDOWN state
static uint8_t missionTimeoutCounter = 0;

// Define a timeout (in units of main timesteps of MavlinkReceive()) for transmitting
// MAVLink messages as part of the MISSION protocol. Messages will be retransmit
// twice before giving up.
#define MISSION_RESEND_TIMEOUT 100

// This is a variable declared in Simulink that contains the GPS origin used for global/local
// coordinate conversions. It's organized as latitude (1e7 degrees), longitude (1e7 degrees), and
// altitude (1e6 meters). 
extern int32_t gpsOrigin[3];

// Track how well MAVLink decoding is going.
// WARN: Possible overflow over long-enough duration, but this is the size of these variables in
// the MAVLink code.
static uint16_t mavLinkMessagesReceived = 0;
static uint16_t mavLinkMessagesFailedParsing = 0;

// Store radio telemetry information from the 3DRs.
static mavlink_radio_status_t radioStatus;

// Track manual control data transmit via MAVLink
static struct {
	int16_t Rudder;
	int16_t Throttle;
	uint16_t Buttons; 
} mavlinkManualControlData;

// Track the last transmission received from the groundstation. This can be used to check for a
// disconnection. This is in .01s units, the same as that used by the nodeSystemTime variable from
// the Node.h library.
static uint32_t gcsLastTimeSeen = UINT32_MAX;

// Set up the message scheduler for MAVLink transmission to the groundstation
#define GROUNDSTATION_SCHEDULE_NUM_MSGS 17
static uint8_t groundstationMavlinkScheduleIds[GROUNDSTATION_SCHEDULE_NUM_MSGS] = {
	MAVLINK_MSG_ID_HEARTBEAT,
	MAVLINK_MSG_ID_SYS_STATUS,
	MAVLINK_MSG_ID_SYSTEM_TIME,
	MAVLINK_MSG_ID_LOCAL_POSITION_NED,
	MAVLINK_MSG_ID_ATTITUDE,
	MAVLINK_MSG_ID_GPS_RAW_INT,
	MAVLINK_MSG_ID_WSO100,
	MAVLINK_MSG_ID_BASIC_STATE2,
	MAVLINK_MSG_ID_RUDDER_RAW,
	MAVLINK_MSG_ID_DST800,
	MAVLINK_MSG_ID_MAIN_POWER,
	MAVLINK_MSG_ID_GPS200,
	MAVLINK_MSG_ID_NODE_STATUS,
	MAVLINK_MSG_ID_WAYPOINT_STATUS,
	MAVLINK_MSG_ID_TOKIMEC,
	MAVLINK_MSG_ID_RADIO_STATUS,
	MAVLINK_MSG_ID_VFR_HUD
};
static uint16_t groundstationMavlinkScheduleTSteps[GROUNDSTATION_SCHEDULE_NUM_MSGS][2][8] = {};
static uint8_t  groundstationMavlinkScheduleSizes[GROUNDSTATION_SCHEDULE_NUM_MSGS];
static MessageSchedule groundstationMavlinkSchedule = {
	GROUNDSTATION_SCHEDULE_NUM_MSGS,
	groundstationMavlinkScheduleIds,
	groundstationMavlinkScheduleSizes,
	0,
	groundstationMavlinkScheduleTSteps
};

// Specify how many times each parameter should be transmit to the datalogger for reference.
#define DATALOGGER_PARAM_TRANSMIT_COUNT 2

// Set up the message scheduler for MAVLink transmission to the datalogger
#define DATALOGGER_SCHEDULE_NUM_MSGS 7
static uint8_t dataloggerMavlinkScheduleIds[DATALOGGER_SCHEDULE_NUM_MSGS] = {
	MAVLINK_MSG_ID_HEARTBEAT,
	MAVLINK_MSG_ID_SYS_STATUS,
	MAVLINK_MSG_ID_NODE_STATUS,
	MAVLINK_MSG_ID_TOKIMEC_WITH_TIME,
	MAVLINK_MSG_ID_CONTROLLER_DATA,
        MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME,
        MAVLINK_MSG_ID_SYSTEM_TIME
};
static uint16_t dataloggerMavlinkScheduleTSteps[DATALOGGER_SCHEDULE_NUM_MSGS][2][8] = {};
static uint8_t  dataloggerMavlinkScheduleSizes[DATALOGGER_SCHEDULE_NUM_MSGS];
static MessageSchedule dataloggerMavlinkSchedule = {
	DATALOGGER_SCHEDULE_NUM_MSGS,
	dataloggerMavlinkScheduleIds,
	dataloggerMavlinkScheduleSizes,
	0,
	dataloggerMavlinkScheduleTSteps
};

void MavLinkSendMissionCount(void);
void MavLinkSendMissionItem(uint8_t currentMissionIndex);
void MavLinkSendMissionRequest(uint8_t currentMissionIndex);
void MavLinkSendMissionAck(uint8_t type);
void MavLinkSendGpsGlobalOrigin(void);
void MavLinkSendLocalPosition(void);
void MavLinkSendStatus(uint8_t channel);
void MavLinkSendNodeStatus(uint8_t channel);
void MavLinkSendRawGps(void);
void MavLinkSendMainPower(void);
void MavLinkSendBasicState2(void);
void MavLinkSendAttitude(void);
void MavLinkSendSystemTime(uint8_t channel);
void MavLinkSendVfrHud(void);
void MavLinkSendParamValue(uint16_t id);
int MavLinkAppendMission(const mavlink_mission_item_t *mission);
void MavLinkSendDataloggerParameters(bool reset);


/**
 * Initialize MAVLink transmission. This just sets up the MAVLink scheduler with the basic
 * repeatedly-transmit messages.
 */
void MavLinkInit(void)
{
    const uint8_t const mavMessageSizes[] = MAVLINK_MESSAGE_LENGTHS;

    // First initialize the MessageSchedule struct with the proper sizes.
    {
        int i;
        for (i = 0; i < GROUNDSTATION_SCHEDULE_NUM_MSGS; ++i) {
            groundstationMavlinkSchedule.MessageSizes[i] = mavMessageSizes[groundstationMavlinkScheduleIds[i]];
        }

        // We only report things that the GUI needs at 2Hz because it only updates at 1 or 2Hz.
        // We output the VFR_HUD message at a fast 5Hz because it has the throttle value and that's
        // nice to have quick response to.
        const uint8_t const periodicities[GROUNDSTATION_SCHEDULE_NUM_MSGS] = {2, 2, 1, 2, 4, 2, 1, 1, 1, 1, 1, 0, 1, 1, 1, 0, 5};
        for (i = 0; i < GROUNDSTATION_SCHEDULE_NUM_MSGS; ++i) {
            if (periodicities[i] && !AddMessageRepeating(&groundstationMavlinkSchedule, groundstationMavlinkScheduleIds[i], periodicities[i])) {
                FATAL_ERROR();
            }
        }

        // Make sure that we haven't exceeded the total number of bytes/s available on this connection.
        // While we're connecting at 115200, we expect the airspeed of the radios to be 64kbps.
        // Additionally, ECC should be turned on, so that halves that data rate. And I don't want to
        // exceed 80% of that total bandwidth. This makes sure we have space for transient messages like
        // missions, parameters, or waypoint/state changes.
        uint32_t bps = GetBps(&groundstationMavlinkSchedule);
        if (bps > (64000 / 10) * 4 / 5 / 2) {
            FATAL_ERROR();
        }
    }

    // Initialize the MAVLink message scheduler for the datalogger
    {
	// First initialize the MessageSchedule struct with the proper sizes.
	int i;
	for (i = 0; i < DATALOGGER_SCHEDULE_NUM_MSGS; ++i) {
            dataloggerMavlinkSchedule.MessageSizes[i] = mavMessageSizes[dataloggerMavlinkScheduleIds[i]];
	}

        // We want the HEARTBEAT/SYS_STATUS messages so this stream can be used with QGC. And then
        // for datalogging having the status of all nodes at 5Hz + the controller's input/output at
        // 100Hz is awesome.
        const uint8_t const periodicities[DATALOGGER_SCHEDULE_NUM_MSGS] = {2, 2, 5, 0, 100, 0, 1};
	for (i = 0; i < DATALOGGER_SCHEDULE_NUM_MSGS; ++i) {
            if (periodicities[i] && !AddMessageRepeating(&dataloggerMavlinkSchedule, dataloggerMavlinkScheduleIds[i], periodicities[i])) {
                FATAL_ERROR();
            }
	}

        // Make sure that we haven't exceeded the total number of bytes/s available on this connection.
        // We're connecting at 115200, with all bandwidth available to us, almost all are scheduled.
        // Every so often some SEASLUG_PARAMETER messages will be sent, so if we don't exceed 90%,
        // it'll be fine.
        uint32_t bps = GetBps(&dataloggerMavlinkSchedule);
        if (bps > (115200 / 10) * 9 / 10) {
            FATAL_ERROR();
        }
    }
}

uint32_t MavLinkTimeSinceLastGcsMessage(void)
{
    return nodeSystemTime - gcsLastTimeSeen;
}

/**
 * This function creates a MAVLink heartbeat message with some basic parameters and
 * caches that message (along with its size) in the module-level variables declared
 * above. This buffer should be transmit at 1Hz back to the groundstation.
 */
void MavLinkSendHeartbeat(uint8_t channel)
{
	// Update MAVLink state and run mode based on the system state.

	// If the vehicle is in ESTOP switch into emergency mode.
	if (nodeErrors & PRIMARY_NODE_RESET_ESTOP) {
		mavlink_system.state = MAV_STATE_EMERGENCY;
		mavlink_system.mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
	}
	// If the startup reset line is triggered, indicate we're booting up. This is the only unarmed state
	// although that's not technically true with this controller.
	else if (nodeErrors & PRIMARY_NODE_RESET_STARTUP) {
		mavlink_system.state = MAV_STATE_BOOT;
		mavlink_system.mode &= ~MAV_MODE_FLAG_SAFETY_ARMED;
	}
	// Otherwise if we're undergoing calibration indicate that
	else if (nodeErrors & PRIMARY_NODE_RESET_CALIBRATING) {
		mavlink_system.state = MAV_STATE_CALIBRATING;
		mavlink_system.mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}
	// Otherwise if there're any other errors we're in standby
	else if (nodeErrors) {
		mavlink_system.state = MAV_STATE_STANDBY;
		mavlink_system.mode |= MAV_MODE_FLAG_SAFETY_ARMED;
	}
	// Finally we're active if there're no errors. Also indicate within the mode that we're armed.
	else {
            mavlink_system.state = MAV_STATE_ACTIVE;
	}

        // And then we update the system mode with the control method, either manual or autonomous.
        // Note that they're not mutually exclusive within the MAVLink protocol, though I treat them
        // as such for my autopilot.
        if (IS_AUTONOMOUS()) {
                mavlink_system.mode |= (MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED);
                mavlink_system.mode &= ~MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        } else {
                mavlink_system.mode &= ~(MAV_MODE_FLAG_AUTO_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED);
                mavlink_system.mode |= MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
        }

        // Since we have the uint32 field for storing custom data, we're going to pack both the
        // nodeStatus and nodeErrors bitfields into it.
        mavlink_system.custom_mode = (((uint32_t)nodeStatus) << 16) | (uint32_t)nodeErrors;

	// Pack the message
	mavlink_msg_heartbeat_pack_chan(mavlink_system.sysid, mavlink_system.compid, channel,
            &txMessage,
            mavlink_system.type, mavlink_system.autopilot, mavlink_system.mode,
            mavlink_system.custom_mode, mavlink_system.state);

	// Copy the message to the send buffer
	len = mavlink_msg_to_send_buffer(buf, &txMessage);

        // Send to the correct channel
        if (channel == MAVLINK_CHAN_DATALOGGER) {
            Uart2WriteData(buf, (uint8_t)len);
        } else {
            Uart1WriteData(buf, (uint8_t)len);
        }
}

/**
 * This function transmits the system time. Looks like it's necessary for QGC to
 * record timestamps on data reliably. For some reason it doesn't just use the local
 * time of message reception. Hopefully this fixes that.
 */
void MavLinkSendSystemTime(uint8_t channel)
{
    // Pack the message
    mavlink_msg_system_time_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage,
        dateTimeDataStore.usecSinceEpoch, nodeSystemTime*10);

    // Copy the message to the send buffer
    len = mavlink_msg_to_send_buffer(buf, &txMessage);

    // Send to the correct channel
    if (channel == MAVLINK_CHAN_DATALOGGER) {
        Uart2WriteData(buf, (uint8_t)len);
    } else {
        Uart1WriteData(buf, (uint8_t)len);
    }
}

/**
 * This function transmits a MAVLink SYS_STATUS message. It relies on various external information such as sensor/actuator status
 * from ecanSensors.h, the controllerVars struct exported by Simulink, and the drop rate calculated within ecanSensors.c.
 */
void MavLinkSendStatus(uint8_t channel)
{
	// Declare that we have onboard sensors: 3D gyro, 3D accelerometer, 3D magnetometer, absolute pressure, GPS
	// And that we have the following controllers: yaw position, x/y position control, motor outputs/control.
	uint32_t systemsPresent = ONBOARD_SENSORS_IMU |
	                          ONBOARD_SENSORS_WSO100  |
	                          ONBOARD_SENSORS_GPS     |
	                          ONBOARD_CONTROL_YAW_POS |
	                          ONBOARD_CONTROL_XY_POS  |
	                          ONBOARD_CONTROL_MOTOR;

        // These are systems which are connected.
	uint32_t systemsEnabled = ONBOARD_CONTROL_YAW_POS;
	systemsEnabled |= sensorAvailability.gps.enabled?ONBOARD_SENSORS_GPS:0;
	systemsEnabled |= sensorAvailability.imu.enabled?ONBOARD_SENSORS_IMU:0;
	systemsEnabled |= sensorAvailability.wso100.enabled?ONBOARD_SENSORS_WSO100:0;
	// The DST800 doesn't map into this bitfield.
	// The power node doesn't map into this bitfield.
	systemsEnabled |= sensorAvailability.prop.enabled?(ONBOARD_CONTROL_XY_POS|ONBOARD_CONTROL_MOTOR):0;

        // And these are systems which are transmitting good data
	uint32_t systemsActive = ONBOARD_CONTROL_YAW_POS;
	systemsActive |= sensorAvailability.gps.active?ONBOARD_SENSORS_GPS:0;
	systemsActive |= sensorAvailability.imu.active?ONBOARD_SENSORS_IMU:0;
	systemsActive |= sensorAvailability.wso100.active?ONBOARD_SENSORS_WSO100:0;
	// The DST800 doesn't map into this bitfield.
	// The power node doesn't map into this bitfield.
	systemsActive |= sensorAvailability.prop.active?(ONBOARD_CONTROL_XY_POS|ONBOARD_CONTROL_MOTOR):0;

	// Grab the globally-declared battery sensor data and map into the values necessary for transmission.
	uint16_t voltage = (uint16_t)(GetPowerRailVoltage() * 1000);
	int16_t amperage = (int16_t)(GetPowerRailCurrent() * 100);

	// Calculate the drop rate
	uint16_t dropRate = 0;
	if (mavLinkMessagesFailedParsing) {
            dropRate = (uint16_t)(((float)mavLinkMessagesFailedParsing) * 10000.0f / ((float)(mavLinkMessagesReceived + mavLinkMessagesFailedParsing)));
	}

        // Get the ECAN error count to transmit that as well:
        //  * errors_count1 - ecan1 tx error count
        //  * errors_count2 - ecan1 rx error count
        uint8_t ecanTxErrorCount, ecanRxErrorCount;
        Ecan1GetErrorCounts(&ecanTxErrorCount, &ecanRxErrorCount);

	mavlink_msg_sys_status_pack_chan(mavlink_system.sysid, mavlink_system.compid, channel,
            &txMessage,
            systemsPresent, systemsEnabled, systemsActive,
            (uint16_t)(nodeCpuLoad)*10,
            voltage, amperage, -1,
            dropRate, mavLinkMessagesFailedParsing,
            ecanTxErrorCount, ecanRxErrorCount, 0, 0);
	len = mavlink_msg_to_send_buffer(buf, &txMessage);

        if (channel == MAVLINK_CHAN_DATALOGGER) {
            Uart2WriteData(buf, (uint8_t)len);
        } else {
            Uart1WriteData(buf, (uint8_t)len);
        }
}

void MavLinkSendStatusText(enum MAV_SEVERITY severity, const char *text)
{
	char msgText[MAVLINK_MSG_ID_STATUSTEXT_LEN] = {};
	strncpy(msgText, text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);
	mavlink_msg_statustext_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage, severity, msgText);

	len = mavlink_msg_to_send_buffer(buf, &txMessage);

	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendTokimec(void)
{
    mavlink_msg_tokimec_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage,
        tokimecDataStore.yaw, tokimecDataStore.pitch, tokimecDataStore.roll,
        tokimecDataStore.x_angle_vel, tokimecDataStore.y_angle_vel, tokimecDataStore.z_angle_vel,
        tokimecDataStore.x_accel, tokimecDataStore.y_accel, tokimecDataStore.z_accel,
        tokimecDataStore.magneticBearing,
        tokimecDataStore.latitude, tokimecDataStore.longitude,
        tokimecDataStore.est_latitude, tokimecDataStore.est_longitude,
        tokimecDataStore.gpsDirection, tokimecDataStore.gpsSpeed,
        tokimecDataStore.status);

    len = mavlink_msg_to_send_buffer(buf, &txMessage);

    Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendTokimecWithTime(void)
{
    mavlink_msg_tokimec_with_time_pack_chan(mavlink_system.sysid, mavlink_system.compid, MAVLINK_CHAN_DATALOGGER, &txMessage,
        nodeSystemTime*10,
        tokimecDataStore.yaw, tokimecDataStore.pitch, tokimecDataStore.roll,
        tokimecDataStore.x_angle_vel, tokimecDataStore.y_angle_vel, tokimecDataStore.z_angle_vel,
        tokimecDataStore.x_accel, tokimecDataStore.y_accel, tokimecDataStore.z_accel,
        tokimecDataStore.magneticBearing,
        tokimecDataStore.latitude, tokimecDataStore.longitude,
        tokimecDataStore.est_latitude, tokimecDataStore.est_longitude,
        tokimecDataStore.gpsDirection, tokimecDataStore.gpsSpeed,
        tokimecDataStore.status);

    len = mavlink_msg_to_send_buffer(buf, &txMessage);

    Uart2WriteData(buf, (uint8_t)len);
}

/**
 * Transmits the VFR_HUD message, which updates the UI of the groundstation. This is currently only
 * used to update the throttle value.
 */
void MavLinkSendVfrHud(void)
{
    float actRudderCommand;
    int16_t actThrottleCommand;
    GetCurrentActuatorCommands(&actRudderCommand, &actThrottleCommand);
    mavlink_msg_vfr_hud_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage,
        waterDataStore.speed, gpsDataStore.sog / 100.0,
        (uint16_t)((float)tokimecDataStore.yaw / 8192.0),
        (uint16_t)(fabs(actThrottleCommand / 1023.0) * 100),
        gpsDataStore.altitude / 1000000.0,
        0);

    len = mavlink_msg_to_send_buffer(buf, &txMessage);

    Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendRadioStatus(void)
{
    mavlink_msg_radio_status_encode(mavlink_system.sysid, mavlink_system.compid, &txMessage, &radioStatus);

    len = mavlink_msg_to_send_buffer(buf, &txMessage);

    Uart1WriteData(buf, (uint8_t)len);
}

/**
 * Pull the raw GPS sensor data from the gpsDataStore struct within the GPS module and
 * transmit it via MAVLink over UART1.
 */
void MavLinkSendRawGps(void)
{

	// We need to made the mode received from NMEA2000 messages to NMEA0183 fix type.
	// NMEA2000    | NMEA0183 | Meaning
	// 0,3,4,5,6,7 |   0      | invalid/no fix
	//    2        |   3      | 3D fix
	//    1        |   2      | 2D fix
	uint8_t mavlinkGpsMode = gpsDataStore.mode == 2?3:(gpsDataStore.mode == 1?2:0);

	mavlink_msg_gps_raw_int_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage, ((uint64_t)nodeSystemTime)*10000,
		mavlinkGpsMode, gpsDataStore.latitude, gpsDataStore.longitude, gpsDataStore.altitude,
		gpsDataStore.hdop, gpsDataStore.vdop,
		gpsDataStore.sog, (uint16_t)(((float)gpsDataStore.cog) * 180 / M_PI / 100),
		0xFF);

	len = mavlink_msg_to_send_buffer(buf, &txMessage);

	Uart1WriteData(buf, (uint8_t)len);
}

/**
  * Transmit the main battery state as obtained from the power node via the CAN bus.
  */
void MavLinkSendMainPower(void)
{
        mavlink_msg_main_power_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage,
		(uint16_t)(powerDataStore.voltage * 100.0f),(uint16_t)(powerDataStore.current * 10.0f));

	len = mavlink_msg_to_send_buffer(buf, &txMessage);

	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * Transmits the custom BASIC_STATE2 message. This just transmits a bunch of random variables
 * that are good to know but arbitrarily grouped.
 */
void MavLinkSendBasicState2(void)
{
    float actRudderAngleCommand;
    int16_t actThrottleCommand;
    GetCurrentActuatorCommands(&actRudderAngleCommand, &actThrottleCommand);
    mavlink_msg_basic_state2_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage,
        currentCommands.autonomousRudderCommand, currentCommands.primaryManualRudderCommand, currentCommands.secondaryManualRudderCommand,
        actRudderAngleCommand,
        rudderSensorData.RudderAngle,
        currentCommands.autonomousThrottleCommand, currentCommands.primaryManualThrottleCommand, currentCommands.secondaryManualThrottleCommand,
        actThrottleCommand,
        0,
        controllerVars.Acmd,
        controllerVars.L2Vector[0], controllerVars.L2Vector[1]
    );

    len = mavlink_msg_to_send_buffer(buf, &txMessage);

    Uart1WriteData(buf, (uint8_t)len);
}

/**
 * Transmits the z-axis rotation rate from the DSP3000. Note that this is in the body frame. Data is
 * in rads/s and clockwise positive.
 */
void MavLinkSendDsp3000(void)
{
	mavlink_msg_dsp3000_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage, gyroDataStore.zRate);

	len = mavlink_msg_to_send_buffer(buf, &txMessage);

	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * Transmits the vehicle attitude pulled from the Revolution GS 3-axis compass. This functions e
 * xpects nodeSystemTime to be in centiseconds which are then converted to ms for transmission.
 * Yaw should be in radians where positive is eastward from north.
 */
void MavLinkSendAttitude(void)
{
	float roll = (float)tokimecDataStore.roll / 8192.0;
	float pitch = (float)tokimecDataStore.pitch / 8192.0;
	float yaw = (float)tokimecDataStore.yaw / 8192.0;
        float rollRate = (float)tokimecDataStore.x_angle_vel / 4096.0;
        float pitchRate = (float)tokimecDataStore.y_angle_vel / 4096.0;
        float yawRate = (float)tokimecDataStore.z_angle_vel / 4096.0;
	mavlink_msg_attitude_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage,
	                          nodeSystemTime*10,
                                  roll, pitch, yaw,
                                  rollRate, pitchRate, yawRate);

	len = mavlink_msg_to_send_buffer(buf, &txMessage);

	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * This function outputs a  MAVLink MAVLINK_MSG_ID_LOCAL_POSITION_NED
 * message using the filtered data generated by the controller.
 */
void MavLinkSendLocalPosition(void)
{
	mavlink_msg_local_position_ned_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage,
	                                    nodeSystemTime*10,
	                                    controllerVars.LocalPosition[0], controllerVars.LocalPosition[1], controllerVars.LocalPosition[2],
	                                    controllerVars.Velocity[0], controllerVars.Velocity[1], controllerVars.Velocity[2]);

	len = mavlink_msg_to_send_buffer(buf, &txMessage);

	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * Transmits the current GPS position of the origin of the local coordinate frame that the North-East-Down
 * coordinates are all relative too. They should be in units of 1e-7 degrees.
 */
void MavLinkSendGpsGlobalOrigin(void)
{
	mavlink_msg_gps_global_origin_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage,
	                                   gpsOrigin[0], gpsOrigin[1], gpsOrigin[2]);

	len = mavlink_msg_to_send_buffer(buf, &txMessage);

	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * Transmit the current mission index via UART1. GetCurrentMission returns a -1 if there're no missions,
 * so we check and only transmit valid current missions.
 */
void MavLinkSendCurrentMission(void)
{
	int8_t currentMission;

	GetCurrentMission(&currentMission);

	if (currentMission != -1) {
		mavlink_msg_mission_current_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage, (uint16_t)currentMission);
		len = mavlink_msg_to_send_buffer(buf, &txMessage);
		Uart1WriteData(buf, (uint8_t)len);
	}
}

/**
 * Transmit a mission acknowledgement message. The type of message is the sole argument to this
 * function (see enum MAV_MISSIONRESULT).
 */
void MavLinkSendMissionAck(uint8_t type)
{
	mavlink_msg_mission_ack_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage,
	                             groundStationSystemId, groundStationComponentId, type);
	len = mavlink_msg_to_send_buffer(buf, &txMessage);
	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * Transmit a command acknowledgement message. The type of message is the sole argument to this
 * function (see enum MAV_RESULT).
 */
void MavLinkSendCommandAck(uint8_t command, uint8_t result)
{
	mavlink_msg_command_ack_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage,
	                             command, result);
	len = mavlink_msg_to_send_buffer(buf, &txMessage);
	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * Transmit all data input/output from the central controller loop to the datalogger.
 * The IMU data is passed in to make sure we transmit the exact same data that the controller
 * processed.
 */
void MavLinkSendControllerData(const ImuData *imu, const GpsData *gps, float waterSpeed, float rudderAngle, float propSpeed, bool reset, float commandedRudder, int16_t commandedThrottle)
{
    // We need to make sure we clamp the acceleration command, because invalid values are represented
    // as +-99, while its normal range is < +-1.
    int16_t clampedACmd;
    if (controllerVars.Acmd > 50) {
        clampedACmd = INT16_MAX;
    } else if (controllerVars.Acmd < -50) {
        clampedACmd = INT16_MIN;
    } else {
        clampedACmd = controllerVars.Acmd * 1e5;
    }

    // Note that GPS mode checking is already done for us when new GPS messages are received
    mavlink_msg_controller_data_pack_chan(
        mavlink_system.sysid, mavlink_system.compid, MAVLINK_CHAN_DATALOGGER, &txMessage,
        controllerVars.wp0[0] * 10, controllerVars.wp0[1] * 10,
        controllerVars.wp1[0] * 10, controllerVars.wp1[1] * 10,
        imu->attitude[0] * 8192.0, imu->attitude[1] * 8192.0, imu->attitude[2] * 8192.0,
        imu->gyros[0] * 4096.0, imu->gyros[1] * 4096.0, imu->gyros[2] * 4096.0,
        waterSpeed * 1e4,
        gps->newData, gps->latitude, gps->longitude, gps->sog, gps->cog, gps->hdop,
        reset,
        nodeSystemTime*10,
        controllerVars.LocalPosition[0] * 1e3, controllerVars.LocalPosition[1] * 1e3,
        controllerVars.Velocity[0] * 1e3, controllerVars.Velocity[1] * 1e3,
        clampedACmd,
        controllerVars.AimPoint[0] * 10, controllerVars.AimPoint[1] * 10,
        controllerVars.sensedYawRate * 4096.0,
        commandedRudder * 1e4,
        commandedThrottle,
        rudderAngle * 1e4,
        propSpeed * 100
    );

    len = mavlink_msg_to_send_buffer(buf, &txMessage);

    Uart2WriteData(buf, (uint8_t)len);
}

void MavLinkSendMissionCount(void)
{
	uint8_t missionCount;
	GetMissionCount(&missionCount);
	mavlink_msg_mission_count_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage,
	                               groundStationSystemId, groundStationComponentId, missionCount);
	len = mavlink_msg_to_send_buffer(buf, &txMessage);
	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * Output the desired mission. On receiving mission, the autopilot converts them to LOCAL_NED, before
 * they're stored onboard the controller. The original global coordinates are preserved in the
 * `otherCoordinates` member. This is not done for missions already in the local frame. This function
 * prioritizes the global frame version, however, so if the mission was converted, it is output in
 * the global frame. Otherwise the original mission is returned.
 * @param currentMissionIndex The 0-based index of the current mission.
 */
void MavLinkSendMissionItem(uint8_t currentMissionIndex)
{
	Mission m;
	uint8_t result;
	GetMission(currentMissionIndex, &m, &result);
	if (result) {
		int8_t missionManagerCurrentIndex;
		GetCurrentMission(&missionManagerCurrentIndex);

                // Try to send the mission back in the global frame. This makes the mission viewable
                // on the main map, but not on the Horizontal Situation Indicator, which sucks, but
                // the map is more important.
                if (m.otherCoordinates[0] || m.otherCoordinates[1] || m.otherCoordinates[2]) {
                    mavlink_msg_mission_item_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage,
                                                  groundStationSystemId, groundStationComponentId, currentMissionIndex,
                                                  MAV_FRAME_GLOBAL, m.action, (currentMissionIndex == (uint8_t)missionManagerCurrentIndex),
                                                  m.autocontinue, m.parameters[0], m.parameters[1], m.parameters[2], m.parameters[3],
                                                  m.otherCoordinates[0], m.otherCoordinates[1], m.otherCoordinates[2]);
                }
                // Otherwise, if this message wasn't originally in the global frame, just output it
                // as we received it.
                else {
                    mavlink_msg_mission_item_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage,
                                                  groundStationSystemId, groundStationComponentId, currentMissionIndex,
                                                  m.refFrame, m.action, (currentMissionIndex == (uint8_t)missionManagerCurrentIndex),
                                                  m.autocontinue, m.parameters[0], m.parameters[1], m.parameters[2], m.parameters[3],
                                                  m.coordinates[0], m.coordinates[1], m.coordinates[2]);
                }
		len = mavlink_msg_to_send_buffer(buf, &txMessage);
		Uart1WriteData(buf, (uint8_t)len);
	}
}

void MavLinkSendMissionRequest(uint8_t currentMissionIndex)
{
	mavlink_msg_mission_request_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage,
	                                 groundStationSystemId, groundStationComponentId, currentMissionIndex);
	len = mavlink_msg_to_send_buffer(buf, &txMessage);
	Uart1WriteData(buf, (uint8_t)len);
}

/**
 * The following functions are helper functions for reading the various parameters aboard the boat.
 * @param id The ID of this parameter.
 */
void MavLinkSendParamValue(uint16_t id)
{
    if (id < PARAMETERS_TOTAL) {
        // Then use the helper functions from Parameters.h to get the current value. If there was an
        // error, just return having done nothing.
        float param_value = 0.0;
        ParameterGetValueById(id, &param_value);

        // Finally encode the message and transmit.
        mavlink_msg_param_value_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage,
            onboardParameters[id].name, param_value, onboardParameters[id].dataType,
            PARAMETERS_TOTAL, id);
        len = mavlink_msg_to_send_buffer(buf, &txMessage);
        Uart1WriteData(buf, (uint8_t)len);
    }
}

void MavLinkTransmitAllParameters(void)
{
    // To transmit all parameters we schedule a custom event. This lets us defer transmission for 1s
    // which should resolve issues when setting the autonomous mode via the parameter interface.
    AddMessageOnce(&dataloggerMavlinkSchedule, MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME, ADD_METHOD_SOONEST);
}

/** Custom SeaSlug Messages **/

void MavLinkSendRudderRaw(void)
{
	mavlink_msg_rudder_raw_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage,
                                rudderSensorData.RudderPotValue, rudderSensorData.LimitHitPort, 0, rudderSensorData.LimitHitStarboard,
                                rudderSensorData.RudderPotLimitPort, rudderSensorData.RudderPotLimitStarboard);

	len = mavlink_msg_to_send_buffer(buf, &txMessage);

	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendWindAirData(void)
{
	mavlink_msg_wso100_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage,
		windDataStore.speed, windDataStore.direction,
		airDataStore.temp, airDataStore.pressure, airDataStore.humidity);
	len = mavlink_msg_to_send_buffer(buf, &txMessage);
	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendDst800Data(void)
{
	mavlink_msg_dst800_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage,
	                        waterDataStore.speed, waterDataStore.temp, waterDataStore.depth);
	len = mavlink_msg_to_send_buffer(buf, &txMessage);
	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendRevoGsData(void)
{
	mavlink_msg_revo_gs_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage,
		revoGsDataStore.heading, revoGsDataStore.magStatus,
		revoGsDataStore.pitch, revoGsDataStore.pitchStatus,
		revoGsDataStore.roll, revoGsDataStore.rollStatus,
		revoGsDataStore.dip, revoGsDataStore.magneticMagnitude);
	len = mavlink_msg_to_send_buffer(buf, &txMessage);
	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendGps200Data(void)
{
	mavlink_msg_gps200_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage,
	                        gpsDataStore.variation);
	len = mavlink_msg_to_send_buffer(buf, &txMessage);
	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkSendNodeStatus(uint8_t channel)
{
    mavlink_msg_node_status_pack_chan(mavlink_system.sysid, mavlink_system.compid, channel,
        &txMessage,
        nodeStatusDataStore[CAN_NODE_HIL - 1].status,
        nodeStatusDataStore[CAN_NODE_HIL - 1].errors,
        nodeStatusDataStore[CAN_NODE_HIL - 1].temp,
        nodeStatusDataStore[CAN_NODE_HIL - 1].load,
        nodeStatusDataStore[CAN_NODE_HIL - 1].voltage,
        nodeStatusDataStore[CAN_NODE_IMU_SENSOR - 1].status,
        nodeStatusDataStore[CAN_NODE_IMU_SENSOR - 1].errors,
        nodeStatusDataStore[CAN_NODE_IMU_SENSOR - 1].temp,
        nodeStatusDataStore[CAN_NODE_IMU_SENSOR - 1].load,
        nodeStatusDataStore[CAN_NODE_IMU_SENSOR - 1].voltage,
        nodeStatusDataStore[CAN_NODE_POWER_SENSOR - 1].status,
        nodeStatusDataStore[CAN_NODE_POWER_SENSOR - 1].errors,
        nodeStatusDataStore[CAN_NODE_POWER_SENSOR - 1].temp,
        nodeStatusDataStore[CAN_NODE_POWER_SENSOR - 1].load,
        nodeStatusDataStore[CAN_NODE_POWER_SENSOR - 1].voltage,
        nodeStatus,
        nodeErrors,
        nodeTemp,
        nodeCpuLoad,
        nodeVoltage,
        nodeStatusDataStore[CAN_NODE_RC - 1].status,
        nodeStatusDataStore[CAN_NODE_RC - 1].errors,
        nodeStatusDataStore[CAN_NODE_RC - 1].temp,
        nodeStatusDataStore[CAN_NODE_RC - 1].load,
        nodeStatusDataStore[CAN_NODE_RC - 1].voltage,
        nodeStatusDataStore[CAN_NODE_RUDDER_CONTROLLER - 1].status,
        nodeStatusDataStore[CAN_NODE_RUDDER_CONTROLLER - 1].errors,
        nodeStatusDataStore[CAN_NODE_RUDDER_CONTROLLER - 1].temp,
        nodeStatusDataStore[CAN_NODE_RUDDER_CONTROLLER - 1].load,
        nodeStatusDataStore[CAN_NODE_RUDDER_CONTROLLER - 1].voltage);
    len = mavlink_msg_to_send_buffer(buf, &txMessage);

    // Send to the correct channel
    if (channel == MAVLINK_CHAN_DATALOGGER) {
        Uart2WriteData(buf, (uint8_t)len);
    } else {
        Uart1WriteData(buf, (uint8_t)len);
    }
}

void MavLinkSendWaypointStatusData(void)
{
	int8_t missionIndex;
	Mission cMission = {}, nMission;
	boolean_T wasFound; // Using stupid Simulink datatypes to avoid errors
	GetCurrentMission(&missionIndex);
	GetMission(missionIndex, &nMission, &wasFound);
	if (!wasFound) {
		return; // Error out if we couldn't successfully fetch a mission.
	}
	// Fetch either the first waypoint or the starting point depending on what the next waypoint is.
	if (missionIndex > 0) {
		GetMission(missionIndex - 1, &cMission, &wasFound);
	} else {
		GetStartingPoint(&cMission);
	}
	mavlink_msg_waypoint_status_pack(mavlink_system.sysid, mavlink_system.compid, &txMessage,
	                                 cMission.otherCoordinates[0], cMission.otherCoordinates[1], cMission.coordinates[0], cMission.coordinates[1],
									 nMission.otherCoordinates[0], nMission.otherCoordinates[1], nMission.coordinates[0], nMission.coordinates[1]);
	len = mavlink_msg_to_send_buffer(buf, &txMessage);
	Uart1WriteData(buf, (uint8_t)len);
}

void MavLinkReceiveCommandLong(const mavlink_command_long_t *msg)
{
    if (msg->target_system == mavlink_system.sysid) {
        switch (msg->command) {
        case MAV_CMD_PREFLIGHT_STORAGE:
        {
            uint8_t result = MAV_RESULT_FAILED;
            if (msg->param1) {
                if (DataStoreSaveParameters()) {
                    result = MAV_RESULT_ACCEPTED;
                }
            } else {
                if (DataStoreLoadParameters()) {
                    result = MAV_RESULT_ACCEPTED;
                }
            }
            MavLinkSendCommandAck(msg->command, result);
        }
            break;
        default:
            MavLinkSendCommandAck(msg->command, MAV_RESULT_UNSUPPORTED);
            break;
        }
    }
}

void MavLinkReceiveManualControl(const mavlink_manual_control_t *msg)
{
    static uint16_t lastButtons = 0;
    if (msg->target == mavlink_system.sysid) {
        // Record the rudder angle
        if (msg->r != INT16_MAX) {
            mavlinkManualControlData.Rudder = msg->r;
        }

        // If the trigger has been pulled was part of this data packet, update the throttle value.
        if ((msg->buttons & TRIGGER_ENABLE_BUTTON) != 0 && msg->z != INT16_MAX) {
            mavlinkManualControlData.Throttle = msg->z;
        }

        // Record the buttons that are pressed
        mavlinkManualControlData.Buttons = msg->buttons;

        // If the rudder calibration button has been pressed, send that command.
        if (!(lastButtons & RUDDER_CAL_BUTTON) && (msg->buttons & RUDDER_CAL_BUTTON)) {
            RudderStartCalibration();
        }

        // Keep track of what buttons are currently pressed so that up- and down-events can be
        // tracked.
        lastButtons = msg->buttons;
    }
}

void MavLinkReceiveSetMode(const mavlink_set_mode_t *msg)
{
    if (msg->target_system == mavlink_system.sysid) {
        // Set autonomous mode.
        if ((msg->base_mode & MAV_MODE_FLAG_AUTO_ENABLED) &&
            !(msg->base_mode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)) {
            SetAutoMode(PRIMARY_MODE_AUTONOMOUS);
        }
        // Or set manual mode
        else if (!(msg->base_mode & MAV_MODE_FLAG_AUTO_ENABLED) &&
            (msg->base_mode & MAV_MODE_FLAG_MANUAL_INPUT_ENABLED)) {
            SetAutoMode(PRIMARY_MODE_MANUAL);
        }
    }
}

/**
 * Returns the throttle and rudder manual control commands received over MAVLink.
 * @param rc The rudder command (floating point radians)
 * @param tc The throttle command (16-bit integer, -1000=full reverse, 1000=full forward)
 */
void GetMavLinkManualControl(float *rc, int16_t *tc)
{
	if (rc) {
		*rc = mavlinkManualControlData.Rudder;
	}
	if (tc) {
		*tc = mavlinkManualControlData.Throttle;
	}
}

/** Core MAVLink functions handling transmission and state machines **/

/**
 *
 * @param event An event from PARAM_EVENT.
 * @param data A pointer to data, its meaning depends on the current state of the parameter protocol.
 */
void MavLinkEvaluateParameterState(enum PARAM_EVENT event, const void *data)
{
	// Track the parameter protocol state
	static uint8_t state = PARAM_STATE_INACTIVE;

	// Keep a record of the current parameter being used
	static uint16_t currentParameter;

	// Store the state to change into
	uint8_t nextState = state;

	// First check the parameter protocol state
	switch (state) {
		case PARAM_STATE_INACTIVE:
			if (event == PARAM_EVENT_REQUEST_LIST_RECEIVED) {
				currentParameter = 0;
				nextState = PARAM_STATE_STREAM_SEND_VALUE;
			} else if (event == PARAM_EVENT_SET_RECEIVED) {
                            // Decode the message
                            mavlink_param_set_t *x = (mavlink_param_set_t *)data;

                            // Only allow setting a parameter if we're in manual mode unless it's
                            // the automode parameter.
                            if (!IS_AUTONOMOUS() || strcmp(x->param_id, "ModeAuto") == 0) {
                                currentParameter = ParameterSetValueByName(x->param_id, &x->param_value);
                                // If there was an error, just reset.
                                if (currentParameter == UINT16_MAX) {
                                    currentParameter = 0;
                                }
                                nextState = PARAM_STATE_SINGLETON_SEND_VALUE;
                            } else {
                                // There no way to report errors with the parameter protocol.
                                // See MAVLink issue #337: https://github.com/mavlink/mavlink/issues/337
                            }
			} else if (event == PARAM_EVENT_REQUEST_READ_RECEIVED) {
				currentParameter = *(uint16_t*)data;
				nextState = PARAM_STATE_SINGLETON_SEND_VALUE;
			}
		break;

		case PARAM_STATE_SINGLETON_SEND_VALUE: {
			if (event == PARAM_EVENT_NONE) {
				MavLinkSendParamValue(currentParameter);
				nextState = PARAM_STATE_INACTIVE;
			}
		} break;

		case PARAM_STATE_STREAM_SEND_VALUE: {
			if (event == PARAM_EVENT_NONE) {
				MavLinkSendParamValue(currentParameter);

				// And increment the current parameter index for the next iteration and
				// we finish if we've hit the limit of parameters.
				if (++currentParameter == PARAMETERS_TOTAL) {
					nextState = PARAM_STATE_INACTIVE;
				} else {
					nextState = PARAM_STATE_STREAM_DELAY;
				}
			}
		} break;

		// Add a delay of INTRA_PARAM_DELAY timesteps before attempting to schedule another one
                // This counter variable should be incremented by the `IncrementParameterCounter()`
                // function, which should be called at a constant rate. This rate should be the same
                // as the units for INTRA_PARAM_DELAY.
		case PARAM_STATE_STREAM_DELAY: {
			if (event == PARAM_EVENT_ENTER_STATE) {
					parameterTimeoutCounter = 0;
			} else if (event == PARAM_EVENT_NONE) {
				if (parameterTimeoutCounter >= INTRA_PARAM_DELAY) {
					nextState = PARAM_STATE_STREAM_SEND_VALUE;
				}
			}
		} break;

		default: break;
	}

	// Here is when we actually transition between states, calling init/exit code as necessary
	if (nextState != state) {
		MavLinkEvaluateParameterState(PARAM_EVENT_EXIT_STATE, NULL);
		state = nextState;
		MavLinkEvaluateParameterState(PARAM_EVENT_ENTER_STATE, NULL);
	}
}

void IncrementParameterCounter(void)
{
    if (parameterTimeoutCounter < UINT8_MAX) {
        ++parameterTimeoutCounter;
    }
}

/**
 * Set the starting point for the mission manager to the boat's current location.
 */
void SetStartingPointToCurrentLocation(void)
{
	// Update the starting point for the track to be the current vehicle position.
	// We tack on GPS coordinates if we have some.
	Mission newStartPoint = {};
	newStartPoint.coordinates[0] = controllerVars.LocalPosition[0];
	newStartPoint.coordinates[1] = controllerVars.LocalPosition[1];
	newStartPoint.coordinates[2] = controllerVars.LocalPosition[2];
	if (gpsDataStore.mode == 1 || gpsDataStore.mode == 2) {
		newStartPoint.otherCoordinates[0] = gpsDataStore.latitude;
		newStartPoint.otherCoordinates[1] = gpsDataStore.longitude;
		newStartPoint.otherCoordinates[2] = gpsDataStore.altitude;
	}
	SetStartingPoint(&newStartPoint);
}

/**
 * This function implements the mission protocol state machine for the MAVLink protocol.
 * events can be passed as the first argument, or NO_EVENT if desired. data is a pointer
 * to data if there is any to be passed to the state logic. data is not guaranteed to persist
 * beyond the single call to this function.
 */
void MavLinkEvaluateMissionState(enum MISSION_EVENT event, const void *data)
{
	// Keep track of the expected length of the incoming mission list
	static uint16_t mavlinkNewMissionListSize;

	// Track a mission index for some multi-state loops.
	static uint8_t currentMissionIndex;

	// Track the state
	static uint8_t state = MISSION_STATE_INACTIVE;

	// Keep track of the next state to transition into
	uint8_t nextState = state;

	// Then check the mission protocol state
	switch (state) {
		case MISSION_STATE_INACTIVE:
			// If a REQUEST_LIST is received, reset the current mission and move into the receive
			// missions mode.
			if (event == MISSION_EVENT_REQUEST_LIST_RECEIVED) {
				currentMissionIndex = 0;
				nextState = MISSION_STATE_SEND_MISSION_COUNT;
			}
			// Otherwise if a mission count was received, prepare to receive new missions.
			else if (event == MISSION_EVENT_COUNT_RECEIVED) {
				// Don't allow for writing of new missions if we're in autonomous mode.
				if (IS_AUTONOMOUS()) {
					MavLinkSendMissionAck(MAV_MISSION_ERROR);
					nextState = MISSION_STATE_INACTIVE;
				}

				uint8_t newListSize = *(uint8_t *)data;

				// If we received a 0-length mission list, just respond with a MISSION_ACK error.
				if (newListSize == 0) {
					MavLinkSendMissionAck(MAV_MISSION_ERROR);
					nextState = MISSION_STATE_INACTIVE;
				}
				// If there isn't enough room, respond with a MISSION_ACK error.
				else if (newListSize > mList.maxSize) { // mList is exported by MATLAB code.
					MavLinkSendMissionAck(MAV_MISSION_NO_SPACE);
					nextState = MISSION_STATE_INACTIVE;
				}
				// Otherwise we're set to start retrieving a new mission list so we request the first mission.
				else {
					// Update the size of the mission list to the new list size.
					mavlinkNewMissionListSize = newListSize;

					// Clear all the old waypoints.
					ClearMissionList();

					// Update the starting point to the vehicle's current location
					SetStartingPointToCurrentLocation();

					// And wait for info on the first mission.
					currentMissionIndex = 0;

					// And finally trigger the proper response.
					nextState = MISSION_STATE_SEND_MISSION_REQUEST;
				}
			} else if (event == MISSION_EVENT_CLEAR_ALL_RECEIVED) {
				// If we're in autonomous mode, don't allow for clearing the mission list
				if (IS_AUTONOMOUS()) {
					MavLinkSendMissionAck(MAV_MISSION_ERROR);
					nextState = MISSION_STATE_INACTIVE;
				}
				// But if we're in manual mode, go ahead and clear everything.
				else {
					// Clear the old list
					ClearMissionList();

					// Update the starting point to the vehicle's current location
					SetStartingPointToCurrentLocation();

					// And then send our acknowledgement.
					MavLinkSendMissionAck(MAV_MISSION_ACCEPTED);
					nextState = MISSION_STATE_INACTIVE;
				}
			} else if (event == MISSION_EVENT_SET_CURRENT_RECEIVED) {
				SetCurrentMission(*(uint8_t*)data);
				MavLinkSendCurrentMission();
				nextState = MISSION_STATE_INACTIVE;
			}
                        // At this point we shouldn't see anything else, so report an error if we do.
                        else if (event > MISSION_EVENT_EXIT_STATE) {
                                MavLinkSendMissionAck(MAV_MISSION_ERROR);
                                nextState = MISSION_STATE_INACTIVE;
                        }
		break;

		case MISSION_STATE_SEND_MISSION_COUNT:
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionCount();
				nextState = MISSION_STATE_MISSION_COUNT_TIMEOUT;
			}
		break;

		case MISSION_STATE_MISSION_COUNT_TIMEOUT:
			if (event == MISSION_EVENT_ENTER_STATE) {
				missionTimeoutCounter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				if (missionTimeoutCounter >= MISSION_RESEND_TIMEOUT) {
					nextState = MISSION_STATE_SEND_MISSION_COUNT2;
				}
			} else if (event == MISSION_EVENT_REQUEST_LIST_RECEIVED) {
				nextState = MISSION_STATE_SEND_MISSION_COUNT;
			} else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
				// If the current mission is requested, send it.
				if (data && *(uint8_t *)data == currentMissionIndex) {
					MavLinkSendMissionItem(currentMissionIndex);
					nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT;
				} else {
					MavLinkSendMissionAck(MAV_MISSION_INVALID_SEQUENCE);
					nextState = MISSION_STATE_INACTIVE;
				}
			}
		break;

		case MISSION_STATE_SEND_MISSION_COUNT2:
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionCount();
				nextState = MISSION_STATE_MISSION_COUNT_TIMEOUT2;
			}
		break;

		case MISSION_STATE_MISSION_COUNT_TIMEOUT2:
			if (event == MISSION_EVENT_ENTER_STATE) {
				missionTimeoutCounter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				if (missionTimeoutCounter >= MISSION_RESEND_TIMEOUT) {
					nextState = MISSION_STATE_SEND_MISSION_COUNT3;
				}
			} else if (event == MISSION_EVENT_REQUEST_LIST_RECEIVED) {
				nextState = MISSION_STATE_SEND_MISSION_COUNT;
			} else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
				// If the current mission is requested, send it.
				if (data && *(uint8_t *)data == currentMissionIndex) {
					MavLinkSendMissionItem(currentMissionIndex);
					nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT;
				} else {
					MavLinkSendMissionAck(MAV_MISSION_INVALID_SEQUENCE);
					nextState = MISSION_STATE_INACTIVE;
				}
			}
		break;

		case MISSION_STATE_SEND_MISSION_COUNT3:
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionCount();
				nextState = MISSION_STATE_MISSION_COUNT_TIMEOUT3;
			}
		break;

		case MISSION_STATE_MISSION_COUNT_TIMEOUT3:
			if (event == MISSION_EVENT_ENTER_STATE) {
				missionTimeoutCounter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				if (missionTimeoutCounter >= MISSION_RESEND_TIMEOUT) {
					nextState = MISSION_STATE_INACTIVE;
				}
			} else if (event == MISSION_EVENT_REQUEST_LIST_RECEIVED) {
				nextState = MISSION_STATE_SEND_MISSION_COUNT;
			} else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
				// If the current mission is requested, send it.
				if (data && *(uint8_t *)data == currentMissionIndex) {
					MavLinkSendMissionItem(currentMissionIndex);
					nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT;
				} else {
					MavLinkSendMissionAck(MAV_MISSION_INVALID_SEQUENCE);
					nextState = MISSION_STATE_INACTIVE;
				}
			}
		break;

		case MISSION_STATE_SEND_MISSION_ITEM: {
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionItem(currentMissionIndex);
				nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT2;
			}
		} break;

		case MISSION_STATE_MISSION_ITEM_TIMEOUT:
			if (event == MISSION_EVENT_ENTER_STATE) {
				missionTimeoutCounter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				// Keep track of how long it's taking for a request to be received so we can timeout
				// if necessary.
				if (missionTimeoutCounter >= MISSION_RESEND_TIMEOUT) {
					nextState = MISSION_STATE_SEND_MISSION_ITEM2;
				}
			} else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
				// If the mission that was already sent was requested again, retransmit it.
				if (*(uint8_t *)data == currentMissionIndex) {
					nextState = MISSION_STATE_SEND_MISSION_ITEM;
				}
				// Otherwise if the next mission was requested, move on to sending that one.
				else if (*(uint8_t *)data == currentMissionIndex + 1) {
					++currentMissionIndex;
					nextState = MISSION_STATE_SEND_MISSION_ITEM;
				} else {
					MavLinkSendMissionAck(MAV_MISSION_INVALID_SEQUENCE);
					nextState = MISSION_STATE_INACTIVE;
				}
			} else if (event == MISSION_EVENT_ACK_RECEIVED) {
				nextState = MISSION_STATE_INACTIVE;
			}
		break;

		case MISSION_STATE_SEND_MISSION_ITEM2: {
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionItem(currentMissionIndex);
				nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT2;
			}
		} break;

		case MISSION_STATE_MISSION_ITEM_TIMEOUT2:
			if (event == MISSION_EVENT_ENTER_STATE) {
				missionTimeoutCounter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				// Keep track of how long it's taking for a request to be received so we can timeout
				// if necessary.
				if (missionTimeoutCounter >= MISSION_RESEND_TIMEOUT) {
					nextState = MISSION_STATE_SEND_MISSION_ITEM3;
				}
			} else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
				// If the mission that was already sent was requested again, retransmit it.
				if (*(uint8_t *)data == currentMissionIndex) {
					nextState = MISSION_STATE_SEND_MISSION_ITEM;
				}
				// Otherwise if the next mission was requested, move on to sending that one.
				else if (*(uint8_t *)data == currentMissionIndex + 1) {
					++currentMissionIndex;
					nextState = MISSION_STATE_SEND_MISSION_ITEM;
				} else {
					MavLinkSendMissionAck(MAV_MISSION_INVALID_SEQUENCE);
					nextState = MISSION_STATE_INACTIVE;
				}
			} else if (event == MISSION_EVENT_ACK_RECEIVED) {
				nextState = MISSION_STATE_INACTIVE;
			}
		break;

		case MISSION_STATE_SEND_MISSION_ITEM3: {
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionItem(currentMissionIndex);
				nextState = MISSION_STATE_MISSION_ITEM_TIMEOUT3;
			}
		} break;

		case MISSION_STATE_MISSION_ITEM_TIMEOUT3:
			if (event == MISSION_EVENT_ENTER_STATE) {
				missionTimeoutCounter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				// Keep track of how long it's taking for a request to be received so we can timeout
				// if necessary.
				if (missionTimeoutCounter >= MISSION_RESEND_TIMEOUT) {
					nextState = MISSION_STATE_INACTIVE;
				}
			} else if (event == MISSION_EVENT_REQUEST_RECEIVED) {
				// If the mission that was already sent was requested again, retransmit it.
				if (*(uint8_t *)data == currentMissionIndex) {
					nextState = MISSION_STATE_SEND_MISSION_ITEM;
				}
				// Otherwise if the next mission was requested, move on to sending that one.
				else if (*(uint8_t *)data == currentMissionIndex + 1) {
					++currentMissionIndex;
					nextState = MISSION_STATE_SEND_MISSION_ITEM;
				} else {
					MavLinkSendMissionAck(MAV_MISSION_INVALID_SEQUENCE);
					nextState = MISSION_STATE_INACTIVE;
				}
			} else if (event == MISSION_EVENT_ACK_RECEIVED) {
				nextState = MISSION_STATE_INACTIVE;
			}
		break;

		case MISSION_STATE_SEND_MISSION_REQUEST: {
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionRequest(currentMissionIndex);
				nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT;
			}
		} break;

		// Implement the countdown timer for receiving a mission item
		case MISSION_STATE_MISSION_REQUEST_TIMEOUT:
			if (event == MISSION_EVENT_ENTER_STATE) {
				missionTimeoutCounter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				// Keep track of how long it's taking for a request to be received so we can timeout
				// if necessary.
				if (missionTimeoutCounter >= MISSION_RESEND_TIMEOUT) {
					nextState = MISSION_STATE_SEND_MISSION_REQUEST2;
				}
			}
			// If an ACK was received, we just stop. One shouldn't have been received, so just stop
			// for now.
			else if (event == MISSION_EVENT_ACK_RECEIVED) {
				nextState = MISSION_STATE_INACTIVE;
			}
			//
			else if (event == MISSION_EVENT_ITEM_RECEIVED) {
				const mavlink_mission_item_t *incomingMission = (mavlink_mission_item_t *)data;

				// Make sure that they're coming in in the right order, and if they don't return an error in
				// the acknowledgment response.
				if (currentMissionIndex == incomingMission->seq) {
					int missionAddStatus = MavLinkAppendMission(incomingMission);
					if (missionAddStatus != -1) {
						// If this is going to be the new current mission, then we should set it as such.
						if (incomingMission->current) {
							SetCurrentMission(incomingMission->seq);
						}

						// If this was the last mission we were expecting, respond with an ACK
						// confirming that we've successfully received the entire mission list.
						if (currentMissionIndex == mavlinkNewMissionListSize - 1) {
							MavLinkSendMissionAck(MAV_MISSION_ACCEPTED);
							nextState = MISSION_STATE_INACTIVE;
						}
						// Otherwise we just increment and request the next mission.
						else {
							++currentMissionIndex;
							MavLinkSendMissionRequest(currentMissionIndex);
							nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT;
						}
					}
					// If we've run out of space before the last message, respond saying so.
					else {
						MavLinkSendMissionAck(MAV_MISSION_NO_SPACE);
						nextState = MISSION_STATE_INACTIVE;
					}
				}
			}
		break;

		case MISSION_STATE_SEND_MISSION_REQUEST2: {
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionRequest(currentMissionIndex);
				nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT2;
			}
		} break;

		// Implement the countdown timer for receiving a mission item
		case MISSION_STATE_MISSION_REQUEST_TIMEOUT2:
			if (event == MISSION_EVENT_ENTER_STATE) {
				missionTimeoutCounter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				// Keep track of how long it's taking for a request to be received so we can timeout
				// if necessary.
				if (missionTimeoutCounter >= MISSION_RESEND_TIMEOUT) {
					nextState = MISSION_STATE_SEND_MISSION_REQUEST3;
				}
			}
			// If an ACK was received, we just stop. One shouldn't have been received, so just stop
			// for now.
			else if (event == MISSION_EVENT_ACK_RECEIVED) {
				nextState = MISSION_STATE_INACTIVE;
			}
			//
			else if (event == MISSION_EVENT_ITEM_RECEIVED) {
				const mavlink_mission_item_t *incomingMission = (mavlink_mission_item_t *)data;

				// Make sure that they're coming in in the right order, and if they don't return an error in
				// the acknowledgment response.
				if (currentMissionIndex == incomingMission->seq) {
					int missionAddStatus = MavLinkAppendMission(incomingMission);
					if (missionAddStatus != -1) {
						// If this is going to be the new current mission, then we should set it as such.
						if (incomingMission->current) {
							SetCurrentMission(incomingMission->seq);
						}

						// If this was the last mission we were expecting, respond with an ACK
						// confirming that we've successfully received the entire mission list.
						if (currentMissionIndex == mavlinkNewMissionListSize - 1) {
							MavLinkSendMissionAck(MAV_MISSION_ACCEPTED);
							nextState = MISSION_STATE_INACTIVE;
						}
						// Otherwise we just increment and request the next mission.
						else {
							++currentMissionIndex;
							MavLinkSendMissionRequest(currentMissionIndex);
							nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT;
						}
					}
					// If we've run out of space before the last message, respond saying so.
					else {
						MavLinkSendMissionAck(MAV_MISSION_NO_SPACE);
						nextState = MISSION_STATE_INACTIVE;
					}
				}
			}
		break;

		case MISSION_STATE_SEND_MISSION_REQUEST3: {
			if (event == MISSION_EVENT_NONE) {
				MavLinkSendMissionRequest(currentMissionIndex);
				nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT3;
			}
		} break;

		// Implement the countdown timer for receiving a mission item
		case MISSION_STATE_MISSION_REQUEST_TIMEOUT3:
			if (event == MISSION_EVENT_ENTER_STATE) {
				missionTimeoutCounter = 0;
			} else if (event == MISSION_EVENT_NONE) {
				// Keep track of how long it's taking for a request to be received so we can timeout
				// if necessary.
				if (missionTimeoutCounter >= MISSION_RESEND_TIMEOUT) {
					nextState = MISSION_STATE_INACTIVE;
				}
			}
			// If an ACK was received, we just stop. One shouldn't have been received, so just stop
			// for now.
			else if (event == MISSION_EVENT_ACK_RECEIVED) {
				nextState = MISSION_STATE_INACTIVE;
			}
			//
			else if (event == MISSION_EVENT_ITEM_RECEIVED) {
				const mavlink_mission_item_t *incomingMission = (mavlink_mission_item_t *)data;

				// Make sure that they're coming in in the right order, and if they don't return an error in
				// the acknowledgment response.
				if (currentMissionIndex == incomingMission->seq) {
					int missionAddStatus = MavLinkAppendMission(incomingMission);
					if (missionAddStatus != -1) {
						// If this is going to be the new current mission, then we should set it as such.
						if (incomingMission->current) {
							SetCurrentMission(incomingMission->seq);
						}

						// If this was the last mission we were expecting, respond with an ACK
						// confirming that we've successfully received the entire mission list.
						if (currentMissionIndex == mavlinkNewMissionListSize - 1) {
							MavLinkSendMissionAck(MAV_MISSION_ACCEPTED);
							nextState = MISSION_STATE_INACTIVE;
						}
						// Otherwise we just increment and request the next mission.
						else {
							++currentMissionIndex;
							MavLinkSendMissionRequest(currentMissionIndex);
							nextState = MISSION_STATE_MISSION_REQUEST_TIMEOUT;
						}
					}
					// If we've run out of space before the last message, respond saying so.
					else {
						MavLinkSendMissionAck(MAV_MISSION_NO_SPACE);
						nextState = MISSION_STATE_INACTIVE;
					}
				}
			}
		break;
	}

	// Here is when we actually transition between states, calling init/exit code as necessary
	if (nextState != state) {
		MavLinkEvaluateMissionState(MISSION_EVENT_EXIT_STATE, NULL);
		state = nextState;
		MavLinkEvaluateMissionState(MISSION_EVENT_ENTER_STATE, NULL);
	}
}

void IncrementMissionCounter(void)
{
    if (missionTimeoutCounter < UINT8_MAX) {
        ++missionTimeoutCounter;
    }
}

int MavLinkAppendMission(const mavlink_mission_item_t *mission)
{
	// We first copy the incoming mission data into our version of a Mission struct,
	// which does not have some fields that are unnecessary.
	Mission m = {
		{
			mission->x,
			mission->y,
			mission->z
		},
		{
			0.0,
			0.0,
			0.0
		},
		mission->frame,
		mission->command,
		{
			mission->param1,
			mission->param2,
			mission->param3,
			mission->param4
		},
		mission->autocontinue
	};

	// Attempt to record this mission to the list, recording the result, which will be 0 for failure.
	// We also map all incoming Global Lat/Long/Alt messages to North-East-Down here.
	// These can be created in QGroundControl by just double-clicking on the Map. While the NED coordinates
	// are stored, the global coordinates are as well so that they can be transmit as global coordinates
	// to QGC, which doesn't display local waypoints on the primary map.
	if (m.refFrame == MAV_FRAME_GLOBAL || m.refFrame == MAV_FRAME_GLOBAL_RELATIVE_ALT) {
		m.refFrame = MAV_FRAME_LOCAL_NED;
		// Preserve the global coordinates in the "otherCoordinates" members.
		m.otherCoordinates[0] = m.coordinates[0];
		m.otherCoordinates[1] = m.coordinates[1];
		m.otherCoordinates[2] = m.coordinates[2];
		const int32_t x[3] = {
			(int32_t)(m.coordinates[0] * 1e7), // Stored in 1e-7 degrees
			(int32_t)(m.coordinates[1] * 1e7), // Stored in 1e-7 degres
			(int32_t)(m.coordinates[2] * 1e3)  // Stored in 1e-3 meters
		};
		lla2ltp(x, m.coordinates);
	}

	int8_t missionAddStatus;
	AppendMission(&m, &missionAddStatus);
	return missionAddStatus;
}

/**
* @brief Receive communication packets and handle them. Should be called at the system sample rate.
*
* This function decodes packets on the protocol level and also handles
* their value by calling the appropriate functions.
*/
void MavLinkReceive(void)
{
	mavlink_message_t rxMessage;
	mavlink_status_t status;

	// Track if a mission message was processed in this call. This is used to determine if a
	// NONE_EVENT should be sent to the mission manager. The manager needs to be called every
	// timestep such that its internal state machine works properly.
	bool processedMissionMessage = false;

	// Track if a parameter message was processed in this call. This is used to determine if a
	// NONE_EVENT should be sent to the parameter manager. The manager needs to be called every
	// timestep such that its internal state machine works properly.
	bool processedParameterMessage = false;

        uint8_t c;
	while (Uart1ReadByte(&c)) {
		// Parse another byte and if there's a message found process it.
		if (mavlink_parse_char(MAVLINK_COMM_0, c, &rxMessage, &status)) {

			// Latch the groundstation system and component ID if we haven't yet. We exclude the
			// combination of systemid:3/compid:D, because that's the combo used by the 3DR radios.
			if (!groundStationSystemId && !groundStationComponentId &&
			    (rxMessage.sysid != '3' && rxMessage.compid != 'D')) {
				groundStationSystemId = rxMessage.sysid;
				groundStationComponentId = rxMessage.compid;
			}

                        // If the message is from the groundstation, update the received time
                        if (rxMessage.sysid == groundStationSystemId &&
                            rxMessage.compid == groundStationComponentId) {
                            gcsLastTimeSeen = nodeSystemTime;
                        }

			switch(rxMessage.msgid) {

				// Check for commands like write data to EEPROM
				case MAVLINK_MSG_ID_COMMAND_LONG: {
					mavlink_command_long_t mavCommand;
					mavlink_msg_command_long_decode(&rxMessage, &mavCommand);
					MavLinkReceiveCommandLong(&mavCommand);
				} break;

				case MAVLINK_MSG_ID_SET_MODE: {
                                        mavlink_set_mode_t modeMessage;
                                        mavlink_msg_set_mode_decode(&rxMessage, &modeMessage);
                                        MavLinkReceiveSetMode(&modeMessage);
				} break;

				// Check for manual commands via Joystick from QGC.
				case MAVLINK_MSG_ID_MANUAL_CONTROL: {
					mavlink_manual_control_t manualControl;
					mavlink_msg_manual_control_decode(&rxMessage, &manualControl);
					MavLinkReceiveManualControl(&manualControl);
				} break;

				// If we are not doing any mission protocol operations, record the size of the incoming mission
				// list and transition into the write missions state machine loop.
				case MAVLINK_MSG_ID_MISSION_COUNT: {
					uint8_t mavlinkNewMissionListSize = mavlink_msg_mission_count_get_count(&rxMessage);
					MavLinkEvaluateMissionState(MISSION_EVENT_COUNT_RECEIVED, &mavlinkNewMissionListSize);
					processedMissionMessage = true;
				} break;

				// Handle receiving a mission.
				case MAVLINK_MSG_ID_MISSION_ITEM: {
					mavlink_mission_item_t currentMission;
					mavlink_msg_mission_item_decode(&rxMessage, &currentMission);
					MavLinkEvaluateMissionState(MISSION_EVENT_ITEM_RECEIVED, &currentMission);
					processedMissionMessage = true;
				} break;

				// Responding to a mission request entails moving into the first active state and scheduling a MISSION_COUNT message.
				// Will also schedule a transmission of a GPS_ORIGIN message. This is used for translating global to local coordinates
				// in QGC.
				case MAVLINK_MSG_ID_MISSION_REQUEST_LIST: {
					MavLinkSendGpsGlobalOrigin();
					MavLinkEvaluateMissionState(MISSION_EVENT_REQUEST_LIST_RECEIVED, NULL);
					processedMissionMessage = true;
				} break;

				// When a mission request message is received, respond with that mission information from the MissionManager
				case MAVLINK_MSG_ID_MISSION_REQUEST: {
					uint8_t receivedMissionIndex = mavlink_msg_mission_request_get_seq(&rxMessage);
					MavLinkEvaluateMissionState(MISSION_EVENT_REQUEST_RECEIVED, &receivedMissionIndex);
					processedMissionMessage = true;
				} break;

				// Allow for clearing waypoints. Here we respond simply with an ACK message if we successfully
				// cleared the mission list.
				case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
					MavLinkEvaluateMissionState(MISSION_EVENT_CLEAR_ALL_RECEIVED, NULL);
					processedMissionMessage = true;
				break;

				// Allow for the groundstation to set the current mission. This requires a WAYPOINT_CURRENT response message agreeing with the received current message index.
				case MAVLINK_MSG_ID_MISSION_SET_CURRENT: {
					uint8_t newCurrentMission = mavlink_msg_mission_set_current_get_seq(&rxMessage);
					MavLinkEvaluateMissionState(MISSION_EVENT_SET_CURRENT_RECEIVED, &newCurrentMission);
					processedMissionMessage = true;
				} break;

				case MAVLINK_MSG_ID_MISSION_ACK: {
					uint8_t type = mavlink_msg_mission_ack_get_type(&rxMessage);
					MavLinkEvaluateMissionState(MISSION_EVENT_ACK_RECEIVED, &type);
					processedMissionMessage = true;
				} break;

				// If they're requesting a list of all parameters, call a separate function that'll track the state and transmit the necessary messages.
				// This reason that this is an external function is so that it can be run separately at 20Hz.
				case MAVLINK_MSG_ID_PARAM_REQUEST_LIST: {
					MavLinkEvaluateParameterState(PARAM_EVENT_REQUEST_LIST_RECEIVED, NULL);
					processedParameterMessage = true;
				} break;

				// If a request comes for a single parameter then set that to be the current parameter and move into the proper state.
				case MAVLINK_MSG_ID_PARAM_REQUEST_READ: {
					uint16_t currentParameter = mavlink_msg_param_request_read_get_param_index(&rxMessage);
					MavLinkEvaluateParameterState(PARAM_EVENT_REQUEST_READ_RECEIVED, &currentParameter);
					processedParameterMessage = true;
				} break;

				case MAVLINK_MSG_ID_PARAM_SET: {
					mavlink_param_set_t p;
					mavlink_msg_param_set_decode(&rxMessage, &p);
					MavLinkEvaluateParameterState(PARAM_EVENT_SET_RECEIVED, &p);
					processedParameterMessage = true;
				} break;

				case MAVLINK_MSG_ID_RADIO_STATUS:
					mavlink_msg_radio_status_decode(&rxMessage, &radioStatus);
				break;

                                default:
                                    break;
			}
		}

                // Update our stats of both messages received and number of message failures.
                // The `packet_rx_success_count` is a global and persistent count of successfully-
                // received messages, while `packet_rx_drop_count` is a local value and will be 0 or
                // 1 depending on if the character decoded successfully.
                mavLinkMessagesReceived = status.packet_rx_success_count;
                mavLinkMessagesFailedParsing += status.packet_rx_drop_count;
        }

	// Now if no mission messages were received, trigger the Mission Manager anyways with a NONE
	// event.
	if (!processedMissionMessage) {
		MavLinkEvaluateMissionState(MISSION_EVENT_NONE, NULL);
	}

	// Now if no parameter messages were received, trigger the Parameter Manager anyways with a NONE
	// event.
	if (!processedParameterMessage) {
		MavLinkEvaluateParameterState(PARAM_EVENT_NONE, NULL);
	}
}

/**
 * This function handles transmission of MavLink messages taking into account transmission
 * speed, message size, and desired transmission rate.
 */
void MavLinkTransmitGroundstation(void)
{
	// And now transmit all messages for this timestep
	uint8_t msgs[GROUNDSTATION_SCHEDULE_NUM_MSGS];
	uint8_t count = GetMessagesForTimestep(&groundstationMavlinkSchedule, msgs);
	int i;
	for (i = 0; i < count; ++i) {

		switch (msgs[i]) {

			/** Common Messages **/

			case MAVLINK_MSG_ID_HEARTBEAT: {
				MavLinkSendHeartbeat(MAVLINK_CHAN_GROUNDSTATION);
			} break;

			case MAVLINK_MSG_ID_SYSTEM_TIME: {
				MavLinkSendSystemTime(MAVLINK_CHAN_GROUNDSTATION);
			} break;

			case MAVLINK_MSG_ID_SYS_STATUS: {
				MavLinkSendStatus(MAVLINK_CHAN_GROUNDSTATION);
			} break;

			case MAVLINK_MSG_ID_ATTITUDE: {
				MavLinkSendAttitude();
			} break;

			case MAVLINK_MSG_ID_LOCAL_POSITION_NED: {
				MavLinkSendLocalPosition();
			} break;

			case MAVLINK_MSG_ID_GPS_RAW_INT: {
				MavLinkSendRawGps();
			} break;

			case MAVLINK_MSG_ID_RADIO_STATUS: {
				MavLinkSendRadioStatus();
			} break;

                case MAVLINK_MSG_ID_VFR_HUD:
                    MavLinkSendVfrHud();
                    break;

			/** SeaSlug Messages **/

			case MAVLINK_MSG_ID_NODE_STATUS: {
				MavLinkSendNodeStatus(MAVLINK_CHAN_GROUNDSTATION);
			} break;

			case MAVLINK_MSG_ID_WAYPOINT_STATUS: {
				MavLinkSendWaypointStatusData();
			} break;

			case MAVLINK_MSG_ID_WSO100: {
				MavLinkSendWindAirData();
			} break;

			case MAVLINK_MSG_ID_BASIC_STATE2:
				MavLinkSendBasicState2();
			break;

			case MAVLINK_MSG_ID_RUDDER_RAW:
				MavLinkSendRudderRaw();
			break;

			case MAVLINK_MSG_ID_DST800:
				MavLinkSendDst800Data();
			break;

			case MAVLINK_MSG_ID_GPS200:
				MavLinkSendGps200Data();
			break;

			case MAVLINK_MSG_ID_TOKIMEC:
				MavLinkSendTokimec();
			break;

			case MAVLINK_MSG_ID_MAIN_POWER:
				MavLinkSendMainPower();
			break;

			default: {

			} break;
		}
	}
}

/**
 * This function handles transmission of MavLink messages taking into account transmission
 * speed, message size, and desired transmission rate.
 */
void MavLinkTransmitDatalogger(void)
{
    uint8_t msgs[DATALOGGER_SCHEDULE_NUM_MSGS];
    uint8_t count = GetMessagesForTimestep(&dataloggerMavlinkSchedule, msgs);
    int i;
    for (i = 0; i < count; ++i) {
        switch (msgs[i]) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                MavLinkSendHeartbeat(MAVLINK_CHAN_DATALOGGER);
                break;
            case MAVLINK_MSG_ID_SYS_STATUS:
                MavLinkSendStatus(MAVLINK_CHAN_DATALOGGER);
                break;
            case MAVLINK_MSG_ID_NODE_STATUS:
                MavLinkSendNodeStatus(MAVLINK_CHAN_DATALOGGER);
                break;
            case MAVLINK_MSG_ID_SYSTEM_TIME:
                MavLinkSendSystemTime(MAVLINK_CHAN_DATALOGGER);
                break;
            case MAVLINK_MSG_ID_TOKIMEC_WITH_TIME:
                MavLinkSendTokimecWithTime();
                break;
            case MAVLINK_MSG_ID_PARAM_VALUE_WITH_TIME:
                MavLinkSendDataloggerParameters(true);
                break;
             default:
                 break;
         }
     }

    // Always attempt to send the datalogger parameters. This simplifies the logic somewhat.
    MavLinkSendDataloggerParameters(false);
}

/**
 * Every call to this function transmits another parameter using the timestamped
 * PARAM_VALUE_WITH_TIME message. These are transmit out on the datalogger channel
 * @param reset True to reset the internal counter. If true, this **will not** transmit a message.
 */
void MavLinkSendDataloggerParameters(bool reset)
{
    static uint8_t pid = 0, count = 0;
    if (reset) {
        pid = 0;
        count = 0;
    } else if (pid < PARAMETERS_TOTAL) {
        // Get the value from the Parameter library using it's provided numeric ID
        float param_value = 0.0;
        ParameterGetValueById(pid, &param_value);

        // Finally encode the message and transmit.
        mavlink_msg_param_value_with_time_pack_chan(
            mavlink_system.sysid, mavlink_system.compid, MAVLINK_CHAN_DATALOGGER,
            &txMessage,
            nodeSystemTime * 10,
            onboardParameters[pid].name, param_value, onboardParameters[pid].dataType,
            PARAMETERS_TOTAL, pid);
        len = mavlink_msg_to_send_buffer(buf, &txMessage);
        Uart2WriteData(buf, (uint8_t)len);

        // Track how many times this message had been sent.
        ++count;
        if (count == DATALOGGER_PARAM_TRANSMIT_COUNT) {
            // Reset the counter
            count = 0;

            // And increment the pid for next time
            ++pid;
        }
    }
}