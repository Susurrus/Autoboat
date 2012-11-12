#ifndef _CAN_MESSAGES_H_
#define _CAN_MESSAGES_H_

/**
 * This module defines all of the custom CAN messages used within the SeaSlug project. These messages all use the standard CAN ID size (11-bits).
 * Usage of this code involves calling one of the `CanMessagePackage()` functions and then feeding
 * the resultant struct into the ECAN transmission library.
 */

//TODO: Add Status decoder
//TODO: Add RudderSetState decoder

#include "types.h"
#include "ecanDefinitions.h"

 // Define the standard (11-bit) IDs for all custom CAN messages.
 // We make sure not to trample on the 300/301/302/400/401/402 messages used by the ACS300.
enum {
	// Rudder messages
	CAN_MSG_ID_RUDDER_DETAILS      = 0x080,
	CAN_MSG_ID_RUDDER_SET_STATE    = 0x081,
	CAN_MSG_ID_RUDDER_SET_TX_RATE  = 0x082,
	
	// General messages
	CAN_MSG_ID_STATUS              = 0x090
};

// Define the length of all of the custom CAN messages.
enum {
	// Rudder messages
	CAN_MSG_SIZE_RUDDER_DETAILS      = 7,
	CAN_MSG_SIZE_RUDDER_SET_STATE    = 1,
	CAN_MSG_SIZE_RUDDER_SET_TX_RATE  = 2,
	
	// General messages
	CAN_MSG_SIZE_STATUS              = 5
};

// Give each CAN node a unique ID
enum {
    CAN_NODE_PRIMARY_CONTROLLER = 1,
    CAN_NODE_RUDDER_CONTROLLER  = 2,
    CAN_NODE_RC                 = 3,
    CAN_NODE_POWER_SENSOR       = 4
};

/**
 * Package the data that makes up a STATUS CAN message.
 */
void CanMessagePackageStatus(tCanMessage *msg, uint8_t nodeId, uint16_t statusBitfield, uint16_t errorBitfield);

/**
 * Package the data that makes up a RUDDER_SET_STATE message into a struct suitable for transmission.
 */
void CanMessagePackageRudderSetState(tCanMessage *msg, bool enable, bool reset, bool calibrate);

void CanMessagePackageRudderDetails(tCanMessage *msg, uint16_t potVal, uint16_t portLimitVal, uint16_t sbLimitVal, bool portLimitTrig, bool sbLimitTrig, bool enabled, bool calibrated, bool calibrating);

#endif // _CAN_MESSAGES_H_