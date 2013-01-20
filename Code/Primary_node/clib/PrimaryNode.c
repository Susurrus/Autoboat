#include "Node.h"
#include "MavlinkGlue.h"
#include "DEE.h"
#include "Acs300.h"
#include "Uart1.h"
#include "Ecan1.h"

// Calculate the BRG register value necessary for 115200 baud with a 80MHz clock.
#define BAUD115200_BRG_REG 21

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

void SendThrottleCommand(int16_t command)
{
    CanMessage msg = {};
    Acs300PackageWriteParam(&msg, ACS300_PARAM_CC, (uint16_t)command);
    Ecan1Transmit(&msg);
}