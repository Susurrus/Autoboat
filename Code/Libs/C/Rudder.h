#ifndef RUDDER_H
#define RUDDER_H

#include <stdbool.h>
#include <stdint.h>

struct RudderData {
	float RudderAngle;
	uint16_t RudderPotValue;
	uint16_t RudderPotLimitStarboard;
	uint16_t RudderPotLimitPort;
	bool LimitHitStarboard;
	bool LimitHitPort;
	bool Enabled;
	bool Calibrated;
	bool Calibrating;
};
extern struct RudderData rudderSensorData;

/**
 * Returns the current vessel rudder angle. Units are in radians with positive values to port.
 */
float GetRudderAngle(void);

/**
 * Returns the rudder state as a 3-bit number following
 * from msg 0x8081 used with the rudder.
 */
uint8_t GetRudderStatus(void);

/**
 * Clears the stored rudder data to all zeros.
 */
void ClearRudderAngle(void);

void RudderStartCalibration(void);

void RudderSendAngleCommand(uint8_t sourceNode, float angleCommand);

#endif // RUDDER_H
