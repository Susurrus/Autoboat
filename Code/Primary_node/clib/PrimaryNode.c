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

	// Initialize the EEPROM for non-volatile data storage.
	if (!DataStoreInit()) {
		FATAL_ERROR();
	}

	// And load all stored parameters in the EEPROM. If this errors out, assume its because the
	// EEPROM is currently empty (like if the PIC was just flashed). So write the current parameters
	// and try reading them again and only error out if either of those fail.
	if (!DataStoreLoadAllParameters()) {
		if (DataStoreStoreAllParameters()) {
			if (!DataStoreLoadAllParameters()) {
				FATAL_ERROR();
			}
		} else {
			FATAL_ERROR();
		}
	}

	// Initialize the MAVLink communications channel
	MavLinkInit();
}