#ifndef HIL_NODE_H
#define HIL_NODE_H

/**
 * Declare flags for use with checking the `nodeStatus` variable declared in `Node.h`
 */
enum HIL_STATUS_FLAGS {
	// If the node is currently receiving UDP telemetry from the PC.
	NODE_STATUS_FLAG_HIL_ACTIVE    = 0x0001,
	// Set when the rudder subsystem is actively transmitting. This also indicates when sensor
	// feedback mode is engaged.
	NODE_STATUS_FLAG_RUDDER_ACTIVE = 0x0002,
	// This bit will be set if the ACS300 board is actively transmitting.
	NODE_STATUS_FLAG_PROP_ACTIVE   = 0x0004,
	// This bit will be set if the RC board is actively transmitting.
	NODE_STATUS_FLAG_RC_ACTIVE     = 0x0008
};

/**
 * Initialization function, configures everything the node needs.
 */
void HilNodeInit(void);

/**
 * Callback for Timer4 triggering every 250ms
 */
void HilNodeBlink(void);

/**
 * Callback for Timer2 triggering every 10ms.
 */
void HilNodeTimer100Hz(void);

/**
 * Process any received CAN messages coming through ECAN1. Store incoming
 * data into the hilDataToTransmit struct.
 */
uint8_t CanReceiveMessages(void);

#endif // HIL_NODE_H
