#ifndef _RUDDER_H_
#define _RUDDER_H_

#include <stdbool.h>

#include "types.h"

struct RudderData {
	tFloatToChar RudderAngle;
	tUnsignedShortToChar RudderPotValue;
	tUnsignedShortToChar RudderPotLimitStarboard;
	tUnsignedShortToChar RudderPotLimitPort;
	bool LimitHitStarboard;
	bool LimitHitPort;
	uint8_t RudderState; // Bitfield where 0th bit: disconnected, 1st bit: enabled/disabled, 2nd bit: calibrated, 3rd bit: calibrating, 
};
extern struct RudderData rudderSensorData;

/**
 * Returns the current vessel rudder angle. Units are in radians with positive values to port.
 */
float GetRudderAngle();

void GetRudderStatus(bool *calibrated, bool *calibrating);

/**
 * Stores a recorded value of the rudder angle.
 */
void SetRudderAngle(const uint8_t data[2]);

/**
 * Clears the stored rudder data to all zeros.
 */
void ClearRudderAngle();

void RudderStartCalibration(void);

void RudderSendAngleCommand(float angleCommand);

#endif // _RUDDER_H_
