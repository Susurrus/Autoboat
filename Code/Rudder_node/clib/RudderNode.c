#include <math.h>

#include "Ecan1.h"
#include "rudder_node.h"
#include "MessageScheduler.h"
#include "RudderNode.h"
#include "Node.h"
#include "Nmea2000.h"
#include "Nmea2000Encode.h"
#include "CanMessages.h"
#include "Types.h"
#include "DataStore.h"

// Stores all status data from a CANode.
// Invalid values for every field is the maximum positive value for that datatype.
struct NodeStatusData {
	int8_t   temp;
	uint8_t  voltage;
	uint8_t  load;
	uint16_t status;
	uint16_t errors;
};

struct NodeStatusData nodeStatusDataStore[NUM_NODES] = {
    {INT8_MAX, UINT8_MAX, UINT8_MAX, UINT16_MAX, UINT16_MAX},
    {INT8_MAX, UINT8_MAX, UINT8_MAX, UINT16_MAX, UINT16_MAX},
    {INT8_MAX, UINT8_MAX, UINT8_MAX, UINT16_MAX, UINT16_MAX},
    {INT8_MAX, UINT8_MAX, UINT8_MAX, UINT16_MAX, UINT16_MAX},
    {INT8_MAX, UINT8_MAX, UINT8_MAX, UINT16_MAX, UINT16_MAX},
    {INT8_MAX, UINT8_MAX, UINT8_MAX, UINT16_MAX, UINT16_MAX},
    {INT8_MAX, UINT8_MAX, UINT8_MAX, UINT16_MAX, UINT16_MAX}
};

// Add a helper for getting the NodeStatusData struct for a given node.
#define NODE_STATUS(node_id) (nodeStatusDataStore[(node_id) - 1])

// Set the timeout period for nodes (in units of centiseconds)
#define NODE_TIMEOUT 100

/**
 * This array stores the timeout counters for each node. Once they hit `NODE_TIMEOUT`, the nodes
 * are considered offline. This transition will reset the `nodeStatusDataStore` array to its default
 * values.
 */
uint8_t nodeStatusTimeoutCounters[NUM_NODES] = {
    NODE_TIMEOUT,
    NODE_TIMEOUT,
    NODE_TIMEOUT,
    NODE_TIMEOUT,
    NODE_TIMEOUT,
    NODE_TIMEOUT,
    NODE_TIMEOUT
};

// Declare some constants for use with the message scheduler
// (don't use PGN or message ID as it must be a uint8)
enum {
    SCHED_ID_RUDDER_ANGLE   = 1,
    SCHED_ID_CUSTOM_LIMITS  = 2,
    SCHED_ID_TEMPERATURE    = 3,
    SCHED_ID_STATUS         = 4
};

// Define some calibration setting constants
#define TO_PORT      1
#define TO_STARBOARD 0
enum {
	RUDDER_CAL_STATE_NULL,
	RUDDER_CAL_STATE_INIT,
	RUDDER_CAL_STATE_FIRST_TO_PORT,
	RUDDER_CAL_STATE_FIRST_TO_STARBOARD,
	RUDDER_CAL_STATE_SECOND_TO_PORT,
	RUDDER_CAL_STATE_SECOND_TO_STARBOARD,
	RUDDER_CAL_STATE_RECENTER
};

// Instantiate a struct to store calibration data.
struct RudderCalibrationData rudderCalData = {};

// The number of ticks (0.01s) that it takes until we consider the motor inoperable. This is set to
// a large value to prevent false positives.
// @see RudderCheckForMotorStall().
#define RUDDER_MOVEMENT_TIMEOUT 50

// Define the expected movement rate of the rudder in radians per second.
#define RUDDER_RATE 0.343

// Instantiate a struct to store rudder input data.
struct RudderSensorData rudderSensorData = {};

// Initialize the message scheduler

// Set up the message scheduler for MAVLink transmission
#define ECAN_MSGS_SIZE 4
static uint8_t ids[ECAN_MSGS_SIZE] = {
    SCHED_ID_RUDDER_ANGLE,
    SCHED_ID_CUSTOM_LIMITS,
    SCHED_ID_TEMPERATURE,
    SCHED_ID_STATUS
};
static uint16_t tsteps[ECAN_MSGS_SIZE][2][8] = {};
static uint8_t  mSizes[ECAN_MSGS_SIZE];
static MessageSchedule sched = {
	ECAN_MSGS_SIZE,
	ids,
	mSizes,
	0,
	tsteps
};

// Function prototypes
void UpdateSensorsAvailability(void);

void RudderNodeInit(void)
{
	nodeId = CAN_NODE_RUDDER_CONTROLLER;

	// Initialize our ECAN peripheral
	Ecan1Init(F_OSC, NODE_CAN_BAUD);

	// Initialize the EEPROM for storing the onboard parameters.
	enum DATASTORE_INIT x = DataStoreInit();
	if (x == DATASTORE_INIT_SUCCESS) {
		rudderCalData.RestoredCalibration = true;
		rudderCalData.Calibrated = true;
		LATAbits.LATA3 = 1;
	} else if (x == DATASTORE_INIT_FAIL) {
		FATAL_ERROR();
	}

	// Transmit the rudder angle at 10Hz
	if (!AddMessageRepeating(&sched, SCHED_ID_RUDDER_ANGLE, 10)) {
		while (1);
	}

	// Transmit status at 4Hz
	if (!AddMessageRepeating(&sched, SCHED_ID_CUSTOM_LIMITS, 4)) {
		while (1);
	}

	// Transmit temperature at 1Hz
	if (!AddMessageRepeating(&sched, SCHED_ID_TEMPERATURE, 1)) {
		while (1);
	}

	// Transmit error/status at 2Hz
	if (!AddMessageRepeating(&sched, SCHED_ID_STATUS, 2)) {
		while (1);
	}
}

void RudderCalibrate(void)
{
	if (rudderCalData.CalibrationState == RUDDER_CAL_STATE_INIT) {
		rudderCalData.Calibrated = false;
		rudderCalData.Calibrating = true;
		rudderCalData.CommandedRun = true;
		rudderCalData.PortLimitValue = 0;
		rudderCalData.StarLimitValue = 0;
		if (rudderSensorData.StarLimit) {
			rudderCalData.CalibrationState = RUDDER_CAL_STATE_FIRST_TO_PORT;
			rudderCalData.CommandedDirection = TO_PORT;
		} else {
			rudderCalData.CalibrationState = RUDDER_CAL_STATE_FIRST_TO_STARBOARD;
			rudderCalData.CommandedDirection = TO_STARBOARD;
		}
	} else if (rudderCalData.CalibrationState == RUDDER_CAL_STATE_FIRST_TO_PORT) {
		if (rudderSensorData.PortLimit) {
			rudderCalData.PortLimitValue = rudderSensorData.PotValue;
			rudderCalData.CalibrationState = RUDDER_CAL_STATE_SECOND_TO_STARBOARD;
			rudderCalData.CommandedDirection = TO_STARBOARD;
		}
	} else if (rudderCalData.CalibrationState == RUDDER_CAL_STATE_FIRST_TO_STARBOARD) {
		if (rudderSensorData.StarLimit) {
			rudderCalData.StarLimitValue = rudderSensorData.PotValue;
			rudderCalData.CalibrationState = RUDDER_CAL_STATE_SECOND_TO_PORT;
			rudderCalData.CommandedDirection = TO_PORT;
		}
	} else if (rudderCalData.CalibrationState == RUDDER_CAL_STATE_SECOND_TO_PORT) {
		if (rudderSensorData.PortLimit) {
			rudderCalData.PortLimitValue = rudderSensorData.PotValue;
			DataStoreSaveParameters();
			rudderCalData.CalibrationState = RUDDER_CAL_STATE_RECENTER;
			rudderCalData.CommandedDirection = TO_STARBOARD;
			rudderCalData.Calibrated = true;
		}
	} else if (rudderCalData.CalibrationState == RUDDER_CAL_STATE_SECOND_TO_STARBOARD) {
		if (rudderSensorData.StarLimit) {
			rudderCalData.StarLimitValue = rudderSensorData.PotValue;
			DataStoreSaveParameters();
			rudderCalData.CalibrationState = RUDDER_CAL_STATE_RECENTER;
			rudderCalData.CommandedDirection = TO_PORT;
			rudderCalData.Calibrated = true;
		}
	}else if (rudderCalData.CalibrationState == RUDDER_CAL_STATE_RECENTER) {
		if (fabs(rudderSensorData.RudderPositionAngle) < 0.1) {
			rudderCalData.CommandedRun = false;
			rudderCalData.Calibrating = false;
			rudderCalData.CalibrationState = RUDDER_CAL_STATE_NULL;
		}
	}
}

void RudderSendNmea(void)
{
	// Set CAN header information.
	CanMessage msg;
	PackagePgn127245(&msg, nodeId, 0xFF, 0xF, NAN, rudderSensorData.RudderPositionAngle);

	// And finally transmit it.
	Ecan1Transmit(&msg);
}

void RudderSendCustomLimit(void)
{
	CanMessage msg;
	CanMessagePackageRudderDetails(&msg, rudderSensorData.PotValue,
										 rudderCalData.PortLimitValue,
										 rudderCalData.StarLimitValue,
										 rudderSensorData.PortLimit,
										 rudderSensorData.StarLimit,
										 !nodeErrors,
										 rudderCalData.Calibrated,
										 rudderCalData.Calibrating
										 );

	Ecan1Transmit(&msg);
}

void RudderSendTemperature(void)
{
	// Specify a new CAN message w/ metadata
	CanMessage msg;
	
	// Send temp values:
	// * SID: invalid
	// * Temperature instance: Inside
	// * Humidity instance: Invalid
	// * Humidity: Invalid
	// * Pressure: Invalid
	PackagePgn130311(&msg,
                         nodeId,
                         PGN_SID_INVALID,
                         PGN_130311_TEMP_INST_INSIDE,
                         PGN_130311_HUMID_INST_INVALID,
                         rudderSensorData.Temperature,
                         NAN,
                         NAN);

	// And finally transmit it.
	Ecan1Transmit(&msg);
}

void SendAndReceiveEcan(void)
{
    // First update the sensor status
    UpdateSensorsAvailability();

    uint8_t messagesLeft = 0;
    CanMessage msg;
    uint32_t pgn;

    do {
        int foundOne = Ecan1Receive(&msg, &messagesLeft);
        if (foundOne) {
            // Process custom rudder messages. Anything not explicitly handled is assumed to be a NMEA2000 message.
            // If we receive a calibration message, start calibration if we aren't calibrating right now.
            if (msg.id == CAN_MSG_ID_RUDDER_SET_STATE) {
                bool calibrate;
                CanMessageDecodeRudderSetState(&msg, NULL, NULL, &calibrate);
                if (calibrate && rudderCalData.Calibrating == false) {
                    rudderCalData.CalibrationState = RUDDER_CAL_STATE_INIT;
                }
            // Update send message rates
            } else if (msg.id == CAN_MSG_ID_RUDDER_SET_TX_RATE) {
                uint16_t angleRate, statusRate;
                CanMessageDecodeRudderSetTxRate(&msg, &angleRate, &statusRate);
                UpdateMessageRate(angleRate, statusRate);
            } else if (msg.id == CAN_MSG_ID_STATUS) {
                uint8_t node, cpuLoad, voltage;
                int8_t temp;
                uint16_t status, errors;
                CanMessageDecodeStatus(&msg, &node, &cpuLoad, &temp, &voltage, &status, &errors);

                // If we've found a valid node, store the data for it.
                if (node > 0 && node <= NUM_NODES) {
                    // Update all of the data broadcast by this node.
                    nodeStatusDataStore[node - 1].load = cpuLoad;
                    nodeStatusDataStore[node - 1].temp = temp;
                    nodeStatusDataStore[node - 1].voltage = voltage;
                    nodeStatusDataStore[node - 1].status = status;
                    nodeStatusDataStore[node - 1].errors = errors;

                    // And reset the timeout counter for this node.
                    nodeStatusTimeoutCounters[node - 1] = 0;
                }
            } else {
                pgn = Iso11783Decode(msg.id, NULL, NULL, NULL);
                switch (pgn) {
                case PGN_RUDDER:
                    ParsePgn127245(msg.payload, NULL, NULL, &rudderSensorData.CommandedRudderAngle, NULL);
                break;
                }
            }
        }
    } while (messagesLeft > 0);

    // Sit in e-stop error if the Primary Node is broadcasting it and still exists on the network,
    // clearing the error when it stops showing e-stop or disappears off the network.
    if (nodeErrors & RUDDER_NODE_RESET_ESTOP_OR_ACS300_DISCON) {
        if (NODE_STATUS(CAN_NODE_PRIMARY_CONTROLLER).errors == UINT16_MAX ||
            (NODE_STATUS(CAN_NODE_PRIMARY_CONTROLLER).errors != UINT16_MAX &&
            !(NODE_STATUS(CAN_NODE_PRIMARY_CONTROLLER).errors & 0x0080))) {
            nodeErrors &= ~RUDDER_NODE_RESET_ESTOP_OR_ACS300_DISCON;
        }
    } else {
        if ((NODE_STATUS(CAN_NODE_PRIMARY_CONTROLLER).errors != UINT16_MAX &&
            (NODE_STATUS(CAN_NODE_PRIMARY_CONTROLLER).errors & 0x0080))) {
            nodeErrors |= RUDDER_NODE_RESET_ESTOP_OR_ACS300_DISCON;
        }
    }

    // And now transmit all messages for this timestep
    uint8_t msgs[ECAN_MSGS_SIZE];
    uint8_t count = GetMessagesForTimestep(&sched, msgs);
    int i;
    for (i = 0; i < count; ++i) {
        switch (msgs[i]) {
            case SCHED_ID_CUSTOM_LIMITS:
                RudderSendCustomLimit();
            break;

            case SCHED_ID_RUDDER_ANGLE:
                RudderSendNmea();
            break;

            case SCHED_ID_TEMPERATURE:
                RudderSendTemperature();
            break;

            case SCHED_ID_STATUS:
                NodeTransmitStatus();
            break;
        }
    }
}

void UpdateMessageRate(const uint8_t angleRate, const uint8_t statusRate)
{
    // Handle the angle message first
    if (angleRate != 0xFF) {
        if (angleRate == 0x00) {
            // TODO: write code for this
        } else if ((angleRate <= 100) && (angleRate >= 1)) {
            RemoveMessage(&sched, SCHED_ID_RUDDER_ANGLE);
            AddMessageRepeating(&sched, SCHED_ID_RUDDER_ANGLE, angleRate);
        }
    }

    // Handle the status message
    if (statusRate != 0xFF) {
        if (statusRate == 0x00) {
            // TODO: write code for this
        } else if ((statusRate <= 100) && (statusRate >= 1)) {
            RemoveMessage(&sched, SCHED_ID_CUSTOM_LIMITS);
            AddMessageRepeating(&sched, SCHED_ID_CUSTOM_LIMITS, statusRate);
        }
    }
}

uint8_t RudderCheckForMotorStall(uint8_t direction, uint16_t up)
{
    // Keep a counter for when to timeout on the motor being stalled or broken.
    static uint8_t counter = 0;

    // Track the last rudder angle so we can calculate the derivative. Also track the direction,
    // because if it changes during a sample then we should reset the internal state
    static float lastRudderAngle = NAN;
    static uint8_t lastDirection = 0;

    // If the system isn't calibrated, or is currently calibrating, don't do anything and make sure
    // the lastRudderAngle is held as a NAN.
    if (!(nodeStatus & RUDDER_NODE_STATUS_CALIBRATED) ||
         (nodeStatus & RUDDER_NODE_STATUS_CALIBRATING)) {
        lastRudderAngle = NAN;
    }
    // Once we're calibrated, engage the motor stall checking logic.
    else {
        // If the lastRudderAngle isn't set, that means we're entering a calibrated state. So
        // initialize the counter to 0 and the rudder angle to the current rudder angle. Note that
        // this will trigger the counter to increase, but only once, so it won't be a problem.
        if (!(lastRudderAngle == lastRudderAngle)) {
            lastRudderAngle = rudderSensorData.RudderPositionAngle;
            counter = 0;
        }

        // If we're entered a reset state, just output that we're in an error state
        if (counter > RUDDER_MOVEMENT_TIMEOUT) {
            return true;
        }
        // If the motor's no longer being driven, just reset everything for the next check. This is
        // done continously because it then accounts for the rudder being moved by external forces,
        // which shouldn't cause an error. This is also done when the direction value changes so that
        // we don't trigger false positives.
        else if (up == 0 || direction != lastDirection) {
            counter = 0;
            lastRudderAngle = rudderSensorData.RudderPositionAngle;
            lastDirection = direction;
        }
        // Now if the timeout counter is reached, time to check the rudder motion and make sure it's
        // doing the correct thing.
        else if (counter == RUDDER_MOVEMENT_TIMEOUT) {
            // Set the amount of movement expected during the elapsed period. This is the modeled
            // rudder rate for a RUDDER_MOVEMENT_TIMEOUT period of centiseconds, but halved, to give
            // use a little more noise tolerance.
            const float expectedMovement = RUDDER_RATE / (100 / RUDDER_MOVEMENT_TIMEOUT) / 2;

            // But if the rudder is being commanded and it's not moving in the right direction, increase
            // the error counter past RUDDER_MOVEMENT_TIMEOUT and indicate that we're now in reset.
            // Since the error counter is past the timeout, this function will now always return true.
            if ((direction && (rudderSensorData.RudderPositionAngle - lastRudderAngle) < expectedMovement) ||
                (!direction && (rudderSensorData.RudderPositionAngle - lastRudderAngle) > -expectedMovement)) {
                ++counter;
                return true;
            } else {
                counter = 0;
                lastRudderAngle = rudderSensorData.RudderPositionAngle;
            }
        }
        // Now until the next stall check occurs, just increment the counter if the rudder motor is
        // still being driven, otherwise reset.
        else {
            ++counter;
        }
    }

    return false;
}

/**
 * Calculate the rudder angle in radians from the sensor inputs. Also performs a 4Hz IIR exponential
 * average on the values to remove a lot of the noise from the potentiometer and analog sensor.
 */
void CalculateRudderAngle(void)
{
    rudderSensorData.RudderPositionAngle = PotToRads(rudderSensorData.PotValue, rudderCalData.StarLimitValue, rudderCalData.PortLimitValue);
}

float PotToRads(uint16_t input, uint16_t highSide, uint16_t lowSide)
{
    // This function converts the potentiometer reading from the boat's rudder sensors
    // to a value in degrees. It relies on each limit being hit to set the bounds on
    // the rudder sensor value and then maps that value from that range into the new
    // range of -.07854 to .07854 (+-45 deg). It is loosely based off of the integer
    // mapping function at: http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1289758376

    // Prepare our input. Here we subtract the baseline 'lowSide' value off of
    // the input so we start with a range from [0..highSide-lowSide] and map it
    // into the output range of 45 degrees.
    int32_t in_max = highSide - lowSide;
    float val = (float)((int32_t)input - (int32_t)lowSide);

    // Do the actual conversion reversing the range and mapping it into [.7854:-0.7854]
    float rads = (0.5 - val/((float)in_max))*2*0.7854;

    // Finally cap the value to +- our range to prevent odd errors later.
    if (rads > 0.7854) {
        rads = 0.7854;
    } else if (rads < -0.7854) {
        rads = -0.7854;
    }
	return rads;
}

/**
 * This function should be called at a constant rate (same units as NODE_TIMEOUT) and updates the
 * availability of any sensors and onboard nodes. This function is separated from the
 * `ProcessAllEcanMessages()` function because that function should be called as fast as possible,
 * while this one should be called at the base tick rate of the system.
 */
void UpdateSensorsAvailability(void)
{
    // Now if any nodes have timed out, reset their struct data since any data we have for them is
    // now invalid. Otherwise, keep incrementing their timeout counters. These are reset in
    // `ProcessAllEcanMessages()`.
    int i;
    for (i = 0; i < NUM_NODES; ++i) {
        // Be sure to not do this for the current node, as it won't ever receive CAN messages from
        // itself.
        if (i != nodeId - 1) {
            if (nodeStatusTimeoutCounters[i] < NODE_TIMEOUT) {
                ++nodeStatusTimeoutCounters[i];
            } else {
                nodeStatusDataStore[i].errors = UINT16_MAX;
                nodeStatusDataStore[i].load = UINT8_MAX;
                nodeStatusDataStore[i].status = UINT16_MAX;
                nodeStatusDataStore[i].temp = INT8_MAX;
                nodeStatusDataStore[i].voltage = UINT8_MAX;
            }
        }
    }
}
