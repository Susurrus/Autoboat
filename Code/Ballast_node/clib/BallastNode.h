#ifndef BALLAST_NODE_H
#define BALLAST_NODE_H

#include <stdint.h>
#include <stdbool.h>

// Keep track of the processor's operating frequency.
#define F_OSC 80000000L

/**
 * This enum declares the bitflags used for the nodeStatus variable in Node.h.
 */
enum BALLAST_NODE_STATUS {
	BALLAST_NODE_STATUS_CALIBRATED       = 0x0001, // Active when the ballast has been calibrated
	BALLAST_NODE_STATUS_CALIBRATING      = 0x0002, // Active while the ballast is performing the calibration procedure
	BALLAST_NODE_STATUS_STARBOARD_LIMIT  = 0x0004, // Active while the ballast is at the starboard limit
	BALLAST_NODE_STATUS_PORT_LIMIT       = 0x0008  // Active while the ballast is at the port limit
};

/**
 * This enum declares the bitflags used for the nodeErrors variable in Node.h.
 */
enum BALLAST_NODE_RESET {
    BALLAST_NODE_NOT_APPLICABLE = 0x0000
    // None exist
};

struct BallastCalibrationData {
	uint16_t PortLimitValue; // The lower limit on the ballast potentiometer.
	uint16_t StarLimitValue; // The upper limit on the ballast potentiometer.
	uint8_t CalibrationState;  // Tracks the internal state machine of the calibration FSM.
	uint8_t CommandedDirection; // Dictates the direction the ballast should be commanded.
	uint8_t CommandedRun; // Whether the ballast should be stepping now.
	bool RestoredCalibration; // If calibration data was restored after a reset.
	bool Calibrating;
	bool Calibrated;
};
extern struct BallastCalibrationData ballastCalData;

struct BallastSensorData {
	float CommandedBallastAngle; // The ballast angle in radians.
	float BallastPositionAngle; // The ballast angle in radians.
	float Temperature; // The temperature in Celsius.
	uint16_t PotValue; // The ballast potentiometer value.
	bool PortLimit; // Whether the port limit switch has been triggered.
	bool StarLimit; // Whether the starboard limit switch has been triggered.
};
extern struct BallastSensorData ballastSensorData;

/**
 * Initialize the ballast subsystem. Currently this means
 * scheduling repetitive transmission of CAN messages.
 */
void BallastSubsystemInit(void);

void BallastCalibrate(void);

/**
 * Transmit PGN 127245 (ballast angle) message via CAN.
 */
void BallastSendNmea(void);

/**
 * Transmit CUSTOM_LIMITS message via CAN.
 */
void BallastSendCustomLimit(void);

/**
 * Transmit PGN130311 (ambient temperature) message via CAN.
 */
void BallastSendTemperature(void);

/**
 * Manage the ECAN message system for a given timestamp. This
 * function reads in and processes all received timestamps. It
 * also checks which messages should be transmit for the current
 * timestep and sends those off.
 */
void SendAndReceiveEcan(void);

/**
 * Update the transmission rates of broadcast messages.
 * @param angleRate The desired transmission rate of the NMEA2000 PGN127245 message.
 * @param statusRate The desired transmission rate of the custom status message.
 */
void UpdateMessageRate(const uint8_t angleRate, const uint8_t statusRate);

/**
 * Helper function for retrieving whether a calibration should occur.
 * Used to pull this data into Simulink.
 * @returns A boolean for whether or not to calibrate.
 */
bool GetCalibrateMessage(void);

/**
 * Helper function for retrieving what the commanded angle should be.
 * Used to pull this data into Simulink.
 * @returns The desired angle in radians.
 */
float GetNewAngle(void);

/**
 * Convert the potentiometer readings to floating point radians. Tries to use integer math whenever
 * possible.
 */
float PotToRads(uint16_t input, uint16_t highSide, uint16_t lowSide);

#endif // BALLAST_NODE_H
