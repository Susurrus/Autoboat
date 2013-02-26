#include "Node.h"
#include "MavlinkGlue.h"
#include "Acs300.h"
#include "Uart1.h"
#include "Ecan1.h"
#include "PrimaryNode.h"
#include "DataStore.h"

void PrimaryNodeInit(void)
{
	nodeId = CAN_NODE_PRIMARY_CONTROLLER;

	// Initialize UART1 to 115200 for groundstation communications.
	Uart1Init(BAUD115200_BRG_REG);

	// Initialize the EEPROM for non-volatile data storage. DataStoreInit() also takes care of
	// initializing the onboard data store to the current parameter values so all subsequent calls
	// to DataStoreLoadAllParameters() should work.
	if (!DataStoreInit()) {
		FATAL_ERROR();
	}

	// Initialize the MAVLink communications channel
	MavLinkInit();
}