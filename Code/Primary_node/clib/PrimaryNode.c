#include "Node.h"
#include "MavlinkGlue.h"
#include "DEE.h"
#include "Acs300.h"
#include "Uart1.h"
#include "Uart2.h"
#include "ecanFunctions.h"

// This is the value of the BRG register for configuring different baud
// rates. These BRG values have been calculated based on a 40MHz system clock.
#define BAUD57600_BRG_REG 42
#define BAUD115200_BRG_REG 21

void PrimaryNodeInit(void)
{
	nodeId = CAN_NODE_PRIMARY_CONTROLLER;

	// Initialize UART2 to 57600 for the Revolution GS.
	Uart2Init(BAUD57600_BRG_REG);

	// Initialize UART1 to 115200 for groundstation communications.
	Uart1Init(BAUD115200_BRG_REG);

	DataEEInitAndClear();
	MavLinkInit();
}

void SendThrottleCommand(int16_t command)
{
    tCanMessage msg = {};
    Acs300PackageWriteParam(&msg, ACS300_PARAM_CC, (uint16_t)command);
    ecan1_buffered_transmit(&msg);
}