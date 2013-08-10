#include "Node.h"
#include "Rudder.h"
#include "Acs300.h"

/**
 * Note that this function utilizes the global "nodeId" value from Node.h.
 */
void ActuatorsTransmitCommands(float rudderCommand, int16_t throttleCommand)
{
	// Output a rudder angle command if the command has changed. This check is done for both previous commands so that each command is double-transmitted, ensuring delivery. These are high-priority messages.
	static float lastRudderCommand = 0.0;
	if (rudderCommand - lastRudderCommand != 0) {
		RudderSendAngleCommand(nodeId, rudderCommand);
		
		lastRudderCommand = rudderCommand;
	}

	// Output a throttle angle command if the command has changed. This check is done for both previous commands so that each command is double-transmitted, ensuring delivery. These are high-priority messages.
	static int16_t lastThrottleCommand = 0;
	if (throttleCommand - lastThrottleCommand != 0) {
		Acs300SendThrottleCommand(throttleCommand);
		
		lastThrottleCommand = throttleCommand;
	}
}