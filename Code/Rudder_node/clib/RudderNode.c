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
										 true,
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
