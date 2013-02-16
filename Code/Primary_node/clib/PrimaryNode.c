#include "Node.h"
#include "MavlinkGlue.h"
#include "DEE.h"
#include "Acs300.h"
#include "Uart1.h"
#include "Ecan1.h"
#include "PrimaryNode.h"

void PrimaryNodeInit(void)
{
	nodeId = CAN_NODE_PRIMARY_CONTROLLER;

	// Initialize UART1 to 115200 for groundstation communications.
	Uart1Init(BAUD115200_BRG_REG);

	// Initialize the EEPROM for non-volatile data storage
	DataEEInitAndClear();

	// Initialize the MAVLink communications channel
	MavLinkInit();
}