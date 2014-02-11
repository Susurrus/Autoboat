#include <pps.h>

#include "Uart1.h"
#include "Ecan1.h"
#include "CanMessages.h"
#include "Node.h"
#include "RcNode.h"
#include "RcCapture.h"
#include "DataStore.h"

// Store some values for calibrating the RC transmitter.
uint16_t rcRudderRange[2] = {};
uint16_t rcThrottleRange[2] = {};
bool restoredCalibration = false;
bool estopActive = false;

void RcNodeInit(void)
{
	nodeId = CAN_NODE_RC;
	
	// Initialize the EEPROM for storing the onboard parameters.
	enum DATASTORE_INIT x = DataStoreInit();
	if (x == DATASTORE_INIT_SUCCESS) {
		restoredCalibration = true;
	} else if (x == DATASTORE_INIT_FAIL) {
		FATAL_ERROR();
	}

	// Initialize RC capture stuff: IC1,2,7,8 and Timer2.
	RcCaptureInit();

	// Initialize ECAN1
	Ecan1Init(F_OSC);

	// And configure the Peripheral Pin Select pins:
	PPSUnLock;
#ifdef __dsPIC33FJ128MC802__
	// To enable ECAN1 pins: TX on 7, RX on 4
	PPSOutput(OUT_FN_PPS_C1TX, OUT_PIN_PPS_RP7);
	PPSInput(IN_FN_PPS_C1RX, IN_PIN_PPS_RP4);

	// To enable IC1 input on RP9 (RB9)
	PPSInput(IN_FN_PPS_IC1, IN_PIN_PPS_RP9);

	// To enable IC2 input on RP15 (RB15)
	PPSInput(IN_FN_PPS_IC2, IN_PIN_PPS_RP15);

	// To enable IC7 input on RP10 (RB10)
	PPSInput(IN_FN_PPS_IC7, IN_PIN_PPS_RP10);

	// To enable IC8 input on RP14 (RB14)
	PPSInput(IN_FN_PPS_IC8, IN_PIN_PPS_RP14);
#elif __dsPIC33EP256MC502__
	// To enable ECAN1 pins: TX on 39, RX on 36
	PPSOutput(OUT_FN_PPS_C1TX, OUT_PIN_PPS_RP39);
	PPSInput(IN_FN_PPS_C1RX, IN_PIN_PPS_RP36);

	// To enable IC1 input on RP41 (RB9)
	PPSInput(IN_FN_PPS_IC1, IN_PIN_PPS_RP41);

	// To enable IC2 input on RPI47 (RB15)
	PPSInput(IN_FN_PPS_IC2, IN_PIN_PPS_RPI47);

	// To enable IC7 input on RP42 (RB10)
	PPSInput(IN_FN_PPS_IC7, IN_PIN_PPS_RP42);

	// To enable IC8 input on RPI46 (RB14)
	PPSInput(IN_FN_PPS_IC8, IN_PIN_PPS_RPI46);
#endif
	PPSLock;
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