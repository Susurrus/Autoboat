#include "Uart1.h"
#include "Ecan1.h"
#include "CanMessages.h"
#include "Node.h"
#include "RcNode.h"
#include "DataStore.h"

// Store some values for calibrating the RC transmitter.
uint16_t rcRudderRange[2] = {};
uint16_t rcThrottleRange[2] = {};
bool restoredCalibration = false;
bool estopActive = false;

void RcNodeInit(void)
{
	nodeId = CAN_NODE_RC;

	// Initialize our ECAN peripheral
	Ecan1Init(F_OSC, NODE_CAN_BAUD);
	
	// Initialize the EEPROM for storing the onboard parameters.
	enum DATASTORE_INIT x = DataStoreInit();
	if (x == DATASTORE_INIT_SUCCESS) {
		restoredCalibration = true;
	} else if (x == DATASTORE_INIT_FAIL) {
		FATAL_ERROR();
	}
}

bool GetEstopStatus(void)
{
    return estopActive;
}

uint8_t ProcessAllEcanMessages(void)
{
	uint8_t messagesLeft = 0;
	uint8_t messagesHandled = 0;
	CanMessage msg;

	do {
		int foundOne = Ecan1Receive(&msg, &messagesLeft);
		if (foundOne) {
			// Decode status messages for the primary controller node. If it's in estop, then we
			// should disable everything!
			if (msg.id == CAN_MSG_ID_STATUS) {
				uint8_t node;
				uint16_t status, errors;
				CanMessageDecodeStatus(&msg, &node, NULL, NULL, NULL, &status, &errors);
				if (node == CAN_NODE_PRIMARY_CONTROLLER) {
					// TODO: Move all *Node.h files into /Libs/C and use the proper FLAG constant
					// value here for the primary node.
					if (errors & 0x80) {
						estopActive = true;
					} else {
						estopActive = false;
					}
				}
            }

			++messagesHandled;
		}
	} while (messagesLeft > 0);

	return messagesHandled;
}