#include "Node.h"
#include "MavlinkGlue.h"
#include "Acs300.h"
#include "Uart1.h"
#include "Ecan1.h"
#include "PrimaryNode.h"
#include "DataStore.h"
#include "EcanSensors.h"

#include <pps.h>
#include <adc.h>

void PrimaryNodeInit(void)
{
	// Set the ID for the primary node.
	nodeId = CAN_NODE_PRIMARY_CONTROLLER;

	// Initialize UART1 to 115200 for groundstation communications.
	Uart1Init(BAUD115200_BRG_REG);

	// Initialize the EEPROM for non-volatile data storage. DataStoreInit() also takes care of
	// initializing the onboard data store to the current parameter values so all subsequent calls
	// to DataStoreLoadAllParameters() should work.
	if (!DataStoreInit()) {
		FATAL_ERROR();
	}

	// Initialize ECAN1
	Ecan1Init();

	// Initialize the MAVLink communications channel
	MavLinkInit();

	// Set up the ADC
	PrimaryNodeAdcInit();

	// Finally perform the necessary pin mappings:
	PPSUnLock;
	// To enable ECAN1 pins: TX on 7, RX on 4
	PPSOutput(OUT_FN_PPS_C1TX, OUT_PIN_PPS_RP7);
	PPSInput(PPS_C1RX, PPS_RP4);

	// To enable UART1 pins: TX on 11, RX on 13
	PPSOutput(OUT_FN_PPS_U1TX, OUT_PIN_PPS_RP11);
	PPSInput(PPS_U1RX, PPS_RP13);

	// To enable UART2 pins: TX on 8, RX on 9
	PPSOutput(OUT_FN_PPS_U2TX, OUT_PIN_PPS_RP8);
	PPSInput(PPS_U2RX, PPS_RP9);
	PPSLock;
}

/**
 * Perform main timed loop at 100Hz.
 */
void PrimaryNode100HzLoop(void)
{
	// Keep an internal counter around so that other processes can occur at less than 100Hz.
	static uint8_t internalCounter = 0;

	// Process incoming ECAN messages.
	ProcessAllEcanMessages();

	// Check for new MaVLink messages.
	MavLinkReceive();

	// Send any necessary messages for this timestep.
	MavLinkTransmit();

	// At 2Hz transmit a NODE_STATUS ECAN message.
	if (internalCounter == 0 || internalCounter == 50) {
		NodeTransmitStatus();
	}

	// Update the onboard system time counter.
	++nodeSystemTime;

	// Update the internal counter.
	if (internalCounter == 99) {
		internalCounter = 0;
	} else {
		++internalCounter;
	}
}

void PrimaryNodeAdcInit(void)
{
	// FIXME: Add ADC initialization here.
}