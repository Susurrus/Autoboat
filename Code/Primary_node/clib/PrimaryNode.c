#include "Node.h"
#include "MavlinkGlue.h"
#include "Acs300.h"
#include "Uart1.h"
#include "Ecan1.h"
#include "PrimaryNode.h"
#include "DataStore.h"
#include "EcanSensors.h"
#include "Rudder.h"
#include "Actuators.h"

#include <stdlib.h>

#include <pps.h>
#include <adc.h>
// Clearing the TRIS bit for a pin specifies it as an output
#define OUTPUT 0

// Declare some macros for helping setting bit values
#define ON  1
#define OFF 0

// Specify how long after startup the node should stay in reset to let things stabilize. Units are
// centiseconds.
#define STARTUP_RESET_TIME 200

// Track a bunch of variables for use with transmission over MAVLink.
MavlinkData internalVariables;

void PrimaryNodeInit(void)
{
	// Set the ID for the primary node.
	nodeId = CAN_NODE_PRIMARY_CONTROLLER;

	// We aren't calculating CPU load for the primary controller
	nodeCpuLoad = UINT8_MAX;

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

	// And specify input/output settings for digital I/O ports:
	// A3 (output): Red LED on the CANode, blinks at 2Hz when the system is in reset, and is solid when the system hit a fatal error, otherwise off.
	_TRISA3 = OUTPUT;
	// A4 (output): Amber LED on the CANode, blinks at 1Hz for status.
	_TRISA4 = OUTPUT;
	// B12 (output): Amber automode LED on the CANode Primary Shield, on when system is autonomous, blinking at 4Hz when in manual override, and off otherwise.
	_TRISB12 = OUTPUT;
	// B15 (output): Amber GPS LED on the CANode Primary Shield, on when GPS is active & receiving good data.
	_TRISB15 = OUTPUT;

	// Now before we start everything, make sure we have our state correct given that we just started
	// up. Every sensor is assumed to be online, but just expiring on initialization, so here we call
	// the necessary code to trigger the timeout event for every sensor.
	UpdateSensorsAvailability();
}

/**
 * Perform main timed loop at 100Hz.
 */
void PrimaryNode100HzLoop(void)
{
	// Keep an internal counter around so that other processes can occur at less than 100Hz.
	static uint8_t internalCounter = 0;

	// Clear state on when errors
	ClearStateWhenErrors();

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

	// Set a reset signal for the first 2 seconds, allowing things to stabilize a bit before the
	// system responds. This is especially crucial because it can take up to a second for sensors to
	// timeout and appear as offline.
	if (nodeSystemTime == 0) {
		nodeErrors |= PRIMARY_NODE_RESET_STARTUP;
	} else if (nodeSystemTime == STARTUP_RESET_TIME) {
		nodeErrors &= ~PRIMARY_NODE_RESET_STARTUP;
	}

	// Update the shield LEDs
	SetResetModeLed();
	SetAutoModeLed();

	// And make sure the primary LED is blinking indicating that the node is operational
	SetStatusModeLed();

	// Update the onboard system time counter.
	++nodeSystemTime;

	// Update the internal counter.
	if (internalCounter == 99) {
		internalCounter = 0;
	} else {
		++internalCounter;
	}
}

/**
 * Clear the GPS and Rudder internal data structures when the system goes into reset mode. This is
 * useful primarily when testing the primary controller and the node is left on through multiple
 * test runs.
 */
void ClearStateWhenErrors(void)
{
	static uint16_t lastErrorState = 0;

	if (!lastErrorState && nodeErrors) {
		ClearGpsData();
		ClearRudderAngle();
	}
	lastErrorState = nodeErrors;
}

/**
 * Set the primary status indicator LED to always blink
 */
void SetStatusModeLed(void)
{
	static uint8_t statusModeBlinkCounter = 0;
	
	if (statusModeBlinkCounter == 0) {
		_LATA4 = ON;
		statusModeBlinkCounter = 1;
	} else if (statusModeBlinkCounter == 100) {
		_LATA4 = OFF;
		++statusModeBlinkCounter;
	} else if (statusModeBlinkCounter == 199) {
		statusModeBlinkCounter = 0;
	} else {
		++statusModeBlinkCounter;
	}
}

/**
 * Set the reset indicator LED dependent on whether we are in a reset state or not
 */
void SetResetModeLed(void)
{
	static uint8_t resetModeBlinkCounter = 0;
	if (nodeErrors) {
		if (resetModeBlinkCounter == 0) {
			_LATA3 = ON;
			resetModeBlinkCounter = 1;
		} else if (resetModeBlinkCounter == 50) {
			_LATA3 = OFF;
			++resetModeBlinkCounter;
		} else if (resetModeBlinkCounter == 99) {
			resetModeBlinkCounter = 0;
		} else {
			++resetModeBlinkCounter;
		}
	} else {
		_LATA3 = OFF;
		resetModeBlinkCounter = 0;
	}
}

/**
 * Set the autonomous mode LED dependent on whether we are in autonomous mode, manual override,
 * or regular manual mode. LED is solid for autonomous mode, flashing for manual override, and
 * off for regular manual control.
 */
void SetAutoModeLed(void)
{
	static uint8_t autoModeBlinkCounter = 0;
	
	if (nodeErrors & PRIMARY_NODE_RESET_MANUAL_OVERRIDE) {
		if (autoModeBlinkCounter == 0) {
			_LATB12 = ON;
			autoModeBlinkCounter = 1;
		} else if (autoModeBlinkCounter == 25) {
			_LATB12 = OFF;
			++autoModeBlinkCounter;
		} else if (autoModeBlinkCounter == 49) {
			autoModeBlinkCounter = 0;
		} else {
			++autoModeBlinkCounter;
		}
	} else if (nodeStatus & PRIMARY_NODE_STATUS_AUTOMODE) {
		_LATB12 = ON;
		autoModeBlinkCounter = 0;
	} else {
		_LATB12 = OFF;
		autoModeBlinkCounter = 0;
	}
}

void PrimaryNodeMuxAndOutputControllerCommands(float rudderCommand, int16_t throttleCommand)
{
	float rc;
	int16_t tc;

	// Select actuator commands based on vehicle mode.
	if (nodeStatus & PRIMARY_NODE_STATUS_AUTOMODE) {
		rc = rudderCommand;
		// Throttle command is not managed by the autonomous controller yet
		GetMavLinkManualControl(NULL, &tc);
		tc = ProcessManualThrottleCommand(tc);
	} else {
		GetMavLinkManualControl(&rc, &tc);
		
		// First process the rudder command adding filtering, a deadband, and unit conversion.
		// First convert to radians:
		rc = rc * 7.854e-4;

		// Now perform some binning w/ hysteresis.
		rc = ProcessManualRudderCommand(rc);

		// Process the throttle command as well.
		tc = ProcessManualThrottleCommand(tc);
	}

	// Track what the commanded values were
	internalVariables.RudderCommand = rc;
	internalVariables.ThrottleCommand = tc;

	// Only transmit these commands if there are no errors.
	if (!nodeErrors) {
		ActuatorsTransmitCommands(rc, tc);
	}
}

float ProcessManualRudderCommand(float rc)
{
	static const int numBins = 9;
	static const float transitions[] = {0.0, 6*M_PI/180, 11*M_PI/180, 16*M_PI/180, 21*M_PI/180, 26*M_PI/180, 31*M_PI/180, 36*M_PI/180, 40*M_PI/180};
	static const float binAngles[] = {0.0, 6*M_PI/180, 12*M_PI/180, 18*M_PI/180, 23*M_PI/180, 28*M_PI/180, 33*M_PI/180, 39*M_PI/180, 45*M_PI/180};
	static int bin = 0; // Track the current bin we're in.
	static float lastRc;

	// Cap the input to +- 45 degrees.
	if (rc > 0.7854) {
		rc = 0.7854;
	} else if (rc < -0.7854) {
		rc = -0.7854;
	}

	// Now average with the last sample
	rc = (rc + lastRc) / 2.0;
	lastRc = rc;

	// And finally add the ~8% deadband
	if (rc > -0.3 && rc < 0.3) {
		rc = 0.0;
	}

	// Attempt to move up a bin.
	if (bin + 1 < numBins)  {
		if (fabs(rc) > (transitions[bin + 1] + 0.0349)) {
			++bin;
		}
	}

	// Attempt to step down a bin.
	if (bin - 1 >= 0)  {
		if (fabs(rc) < (transitions[bin] - 0.0436)) {
			--bin;
		}
	}

	// Finally return the mapped value accounting for sign.
	if (rc < 0) {
		return -binAngles[bin];
	} else {
		return binAngles[bin];
	}
}

int16_t ProcessManualThrottleCommand(int16_t tc)
{
	// Add an 8% deadband
	if (tc > -40 && tc < 40) {
		tc = 0;
	}
	
	return tc;
}

void PrimaryNodeAdcInit(void)
{
	// FIXME: Add ADC initialization here.
}