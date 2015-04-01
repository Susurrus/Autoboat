#include <math.h>

#include "Ecan1.h"
#include "ballast_node.h"
#include "MessageScheduler.h"
#include "BallastNode.h"
#include "Node.h"
#include "Nmea2000.h"
#include "Nmea2000Encode.h"
#include "CanMessages.h"
#include "Types.h"
#include "DataStore.h"

// Declare some constants for use with the message scheduler
// (don't use PGN or message ID as it must be a uint8)
enum {
    SCHED_ID_BALLAST_ANGLE   = 1,
    SCHED_ID_CUSTOM_LIMITS  = 2,
    SCHED_ID_TEMPERATURE    = 3,
    SCHED_ID_STATUS         = 4
};

// Define some calibration setting constants
#define TO_PORT      1
#define TO_STARBOARD 0
enum {
	BALLAST_CAL_STATE_NULL,
	BALLAST_CAL_STATE_INIT,
	BALLAST_CAL_STATE_FIRST_TO_PORT,
	BALLAST_CAL_STATE_FIRST_TO_STARBOARD,
	BALLAST_CAL_STATE_SECOND_TO_PORT,
	BALLAST_CAL_STATE_SECOND_TO_STARBOARD,
	BALLAST_CAL_STATE_RECENTER
};

// Instantiate a struct to store calibration data.
struct BallastCalibrationData ballastCalData = {};

// Instantiate a struct to store ballast input data.
struct BallastSensorData ballastSensorData = {};

// Initialize the message scheduler

// Set up the message scheduler for MAVLink transmission
#define ECAN_MSGS_SIZE 4
static uint8_t ids[ECAN_MSGS_SIZE] = {
    SCHED_ID_BALLAST_ANGLE,
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

void BallastNodeInit(void)
{
	nodeId = CAN_NODE_RUDDER_CONTROLLER;

	// Initialize our ECAN peripheral
	Ecan1Init(F_OSC, NODE_CAN_BAUD);

	// Initialize the EEPROM for storing the onboard parameters.
	enum DATASTORE_INIT x = DataStoreInit();
	if (x == DATASTORE_INIT_SUCCESS) {
		ballastCalData.RestoredCalibration = true;
		ballastCalData.Calibrated = true;
		LATAbits.LATA3 = 1;
	} else if (x == DATASTORE_INIT_FAIL) {
		FATAL_ERROR();
	}

	// Transmit the ballast angle at 10Hz
	if (!AddMessageRepeating(&sched, SCHED_ID_BALLAST_ANGLE, 10)) {
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

void BallastCalibrate(void)
{
	if (ballastCalData.CalibrationState == BALLAST_CAL_STATE_INIT) {
		ballastCalData.Calibrated = false;
		ballastCalData.Calibrating = true;
		ballastCalData.CommandedRun = true;
		ballastCalData.PortLimitValue = 0;
		ballastCalData.StarLimitValue = 0;
		if (ballastSensorData.StarLimit) {
			ballastCalData.CalibrationState = BALLAST_CAL_STATE_FIRST_TO_PORT;
			ballastCalData.CommandedDirection = TO_PORT;
		} else {
			ballastCalData.CalibrationState = BALLAST_CAL_STATE_FIRST_TO_STARBOARD;
			ballastCalData.CommandedDirection = TO_STARBOARD;
		}
	} else if (ballastCalData.CalibrationState == BALLAST_CAL_STATE_FIRST_TO_PORT) {
		if (ballastSensorData.PortLimit) {
			ballastCalData.PortLimitValue = ballastSensorData.PotValue;
			ballastCalData.CalibrationState = BALLAST_CAL_STATE_SECOND_TO_STARBOARD;
			ballastCalData.CommandedDirection = TO_STARBOARD;
		}
	} else if (ballastCalData.CalibrationState == BALLAST_CAL_STATE_FIRST_TO_STARBOARD) {
		if (ballastSensorData.StarLimit) {
			ballastCalData.StarLimitValue = ballastSensorData.PotValue;
			ballastCalData.CalibrationState = BALLAST_CAL_STATE_SECOND_TO_PORT;
			ballastCalData.CommandedDirection = TO_PORT;
		}
	} else if (ballastCalData.CalibrationState == BALLAST_CAL_STATE_SECOND_TO_PORT) {
		if (ballastSensorData.PortLimit) {
			ballastCalData.PortLimitValue = ballastSensorData.PotValue;
			DataStoreSaveParameters();
			ballastCalData.CalibrationState = BALLAST_CAL_STATE_RECENTER;
			ballastCalData.CommandedDirection = TO_STARBOARD;
			ballastCalData.Calibrated = true;
		}
	} else if (ballastCalData.CalibrationState == BALLAST_CAL_STATE_SECOND_TO_STARBOARD) {
		if (ballastSensorData.StarLimit) {
			ballastCalData.StarLimitValue = ballastSensorData.PotValue;
			DataStoreSaveParameters();
			ballastCalData.CalibrationState = BALLAST_CAL_STATE_RECENTER;
			ballastCalData.CommandedDirection = TO_PORT;
			ballastCalData.Calibrated = true;
		}
	}else if (ballastCalData.CalibrationState == BALLAST_CAL_STATE_RECENTER) {
		if (fabsf(ballastSensorData.BallastPositionAngle) < 0.1) {
			ballastCalData.CommandedRun = false;
			ballastCalData.Calibrating = false;
			ballastCalData.CalibrationState = BALLAST_CAL_STATE_NULL;
		}
	}
}

void BallastSendNmea(void)
{
	// Set CAN header information.
	CanMessage msg;
	PackagePgn127245(&msg, nodeId, 0xFF, 0xF, NAN, ballastSensorData.BallastPositionAngle);

	// And finally transmit it.
	Ecan1Transmit(&msg);
}

void BallastSendCustomLimit(void)
{
	CanMessage msg;
	CanMessagePackageBallastDetails(&msg, ballastSensorData.PotValue,
										 ballastCalData.PortLimitValue,
										 ballastCalData.StarLimitValue,
										 ballastSensorData.PortLimit,
										 ballastSensorData.StarLimit,
										 true,
										 ballastCalData.Calibrated,
										 ballastCalData.Calibrating
										 );

	Ecan1Transmit(&msg);
}

void BallastSendTemperature(void)
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
                         PGN130311_TEMP_INST_INSIDE,
                         PGN130311_HUMID_INST_INVALID,
                         ballastSensorData.Temperature,
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
            // Process custom ballast messages. Anything not explicitly handled is assumed to be a NMEA2000 message.
            // If we receive a calibration message, start calibration if we aren't calibrating right now.
            if (msg.id == CAN_MSG_ID_RUDDER_SET_STATE) {
                bool calibrate;
                CanMessageDecodeBallastSetState(&msg, NULL, NULL, &calibrate);
                if (calibrate && ballastCalData.Calibrating == false) {
                    ballastCalData.CalibrationState = BALLAST_CAL_STATE_INIT;
                }
            }
            // TODO: Process CAN_MSG_ID_BALLAST_SET_ANGLE
        }
    } while (messagesLeft > 0);

    // And now transmit all messages for this timestep
    uint8_t msgs[ECAN_MSGS_SIZE];
    uint8_t count = GetMessagesForTimestep(&sched, msgs);
    int i;
    for (i = 0; i < count; ++i) {
        switch (msgs[i]) {
            case SCHED_ID_CUSTOM_LIMITS:
                BallastSendCustomLimit();
            break;

            case SCHED_ID_BALLAST_ANGLE:
                BallastSendNmea();
            break;

            case SCHED_ID_TEMPERATURE:
                BallastSendTemperature();
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
            RemoveMessage(&sched, SCHED_ID_BALLAST_ANGLE);
            AddMessageRepeating(&sched, SCHED_ID_BALLAST_ANGLE, angleRate);
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

void CalculateBallastAngle(void)
{
    ballastSensorData.BallastPositionAngle = PotToRads(ballastSensorData.PotValue, ballastCalData.StarLimitValue, ballastCalData.PortLimitValue);
}

float PotToRads(uint16_t input, uint16_t highSide, uint16_t lowSide)
{
    // This function converts the potentiometer reading from the boat's ballast sensors
    // to a value in degrees. It relies on each limit being hit to set the bounds on
    // the ballast sensor value and then maps that value from that range into the new
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
