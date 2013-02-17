#include "Ecan1.h"
#include "rudder_node.h"
#include "MessageScheduler.h"
#include "RudderNode.h"
#include "Node.h"
#include "Nmea2000.h"
#include "Nmea2000Encode.h"
#include "Nvram.h"
#include "CanMessages.h"
#include "Types.h"

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
struct RudderCalibrationData rudderCalData = {0};

// Instantiate a struct to store rudder input data.
struct RudderSensorData rudderSensorData = {0};

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
			SaveRudderCalibrationRange();
			rudderCalData.CalibrationState = RUDDER_CAL_STATE_RECENTER;
			rudderCalData.CommandedDirection = TO_STARBOARD;
			rudderCalData.Calibrated = true;
		}
	} else if (rudderCalData.CalibrationState == RUDDER_CAL_STATE_SECOND_TO_STARBOARD) {
		if (rudderSensorData.StarLimit) {
			rudderCalData.StarLimitValue = rudderSensorData.PotValue;
			SaveRudderCalibrationRange();
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
