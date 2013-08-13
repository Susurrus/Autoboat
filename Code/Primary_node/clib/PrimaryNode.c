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
#include <dma.h>
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

// Specify the maximum value of the ADC
#define ANmax ((1 << 12) - 1)

// Store analog sensors here
struct {
	float powerRailVoltage;
	float powerRailCurrent;
} analogSensors;

// Store actuator commmands here. Used by the MAVLink code
ActuatorCommands currentCommands;

// Set up DMA memory for the ADC. This is 16 words because the ADC is operated in scatter/gather
// mode where all ADC channels get their own word, so even though I'm only collecting data on 4 pins,
// every pin needs its own memory location.
static volatile uint16_t adcBuf[16] __attribute__((space(dma), aligned(256))) = {};

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

	// Check ADC inputs
	// Battery voltage = 1/.06369 * 3.3 / (1 << 12 - 1) (AN0)
	analogSensors.powerRailVoltage = 3.3 / ANmax / .06369 * (float)adcBuf[0];

	// Battery current = 1/.03660 * 3.3 / (1 << 12 - 1) (AN3)
	analogSensors.powerRailCurrent = 3.3 / ANmax / .03660 * (float)adcBuf[3];

	// Input voltage (AN5)
	nodeVoltage = (uint8_t)(3.3 / ANmax * (21.0 + 2.0) / 2.0 * 10.0 * (float)adcBuf[5]);
			
	// Onboard temperature (AN1)
	nodeTemp = (int8_t)((3.3 / ANmax * (float)adcBuf[1] - 0.5) * 100.0);

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

	// If we've switched to a new waypoint, announce to QGC that we have.
	CheckMissionStatus();

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
	float muxedRc;
	int16_t muxedTc;

	// Obtain and filter the manual control inputs
	float manRc;
	int16_t manTc;
	GetMavLinkManualControl(&manRc, &manTc);
	manRc = ProcessManualRudderCommand(manRc * 7.854e-4);
	manTc = ProcessManualThrottleCommand(manTc);
	currentCommands.primaryManualRudderCommand = manRc;
	currentCommands.primaryManualThrottleCommand = manTc;

	// Track autonomous actuator commands
	currentCommands.autonomousRudderCommand = rudderCommand;
	currentCommands.autonomousThrottleCommand = throttleCommand;

	// Select actuator commands based on vehicle mode.
	if (nodeStatus & PRIMARY_NODE_STATUS_AUTOMODE) {
		muxedRc = rudderCommand;
		// Throttle command is not managed by the autonomous controller yet
		muxedTc = manTc;
	} else {
		muxedRc = manRc;
		muxedTc = manTc;
	}

	// Only transmit these commands if there are no errors.
	if (!nodeErrors) {
		ActuatorsTransmitCommands(muxedRc, muxedTc);
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

void CheckMissionStatus(void)
{
	static int8_t lastMission = 0;

	int8_t currentMission;
	GetCurrentMission(&currentMission);
	if (currentMission != lastMission) {
		MavLinkSendCurrentMission();
		lastMission = currentMission;
	}
}

/**
 * Configure ADC1 for reading in 4 channels:
 * AN0 - Power rail voltage
 * AN1 - Analog temperature sensor (TC1047)
 * AN3 - Power rail current
 * AN5 - Input voltage (voltage divider)
 * Relies on DMA
 */
void PrimaryNodeAdcInit(void)
{
	/// ADC1 Configuration
	// Enable ADC interrupts at lowest priority
	ConfigIntADC1(ADC_INT_DISABLE & ADC_INT_PRI_7);

	// Configure ADC1
	uint16_t config1 = ADC_MODULE_ON & // Enable the ADC
	                   ADC_IDLE_CONTINUE & // Operate even when in sleep mode
	                   ADC_ADDMABM_SCATTR & // Put each pin's sample in their own location in memory
			ADC_AD12B_12BIT &   // Set into 12-bit mode
			ADC_FORMAT_INTG &   // Retrieve value as a standard integer
			ADC_CLK_AUTO &      // Start new sample immediately after last one
			ADC_AUTO_SAMPLING_ON & // Continually sample
			ADC_MULTIPLE &     // Sample all channels simultaneously
			ADC_SAMP_ON;           // Start sampling
	uint16_t config2 = ADC_VREF_AVDD_AVSS & // Use GND & 3.3V as voltage references
			ADC_SCAN_ON &                   // Scan all input pins for channel 0
			ADC_SELECT_CHAN_0 &             // Only use the 1st input channel (ADC is broken into 4)
			ADC_DMA_ADD_INC_4 &             // Trigger a DMA transfer every 4 samples (which should be equal to the number of inputs you have)
			ADC_ALT_BUF_OFF &               // Always fill the buffers from the start address
			ADC_ALT_INPUT_OFF;              // Don't use alternate sample modes
	uint16_t config3 = ADC_SAMPLE_TIME_15 &  // Set sampling time to 15*T_AD. 14 is the fastest that 12-bit sampling can go.
			ADC_CONV_CLK_SYSTEM &           // Base the sampling clock off the system clock
			ADC_CONV_CLK_32Tcy;              // Run the ADC using 1* the system clock
	uint16_t config4 = ADC_DMA_BUF_LOC_1;  // Store only one sample per analog input
	uint16_t configport_h = ENABLE_ALL_DIG_16_31;  // Disable analog inputs for ports 16-31
	uint16_t configport_l = ENABLE_AN0_ANA & // Sample AN0
	                       ENABLE_AN1_ANA & // Sample AN1
	                       ENABLE_AN3_ANA & // Sample AN3
	                       ENABLE_AN5_ANA;  // Sample AN5
	uint16_t configscan_h = SCAN_NONE_16_31;
	uint16_t configscan_l = SKIP_SCAN_AN2 & SKIP_SCAN_AN4 & SKIP_SCAN_AN6 & SKIP_SCAN_AN7 & SKIP_SCAN_AN8 &
	                        SKIP_SCAN_AN9 & SKIP_SCAN_AN10 & SKIP_SCAN_AN11 & SKIP_SCAN_AN12 & SKIP_SCAN_AN13 &
	                        SKIP_SCAN_AN14 & SKIP_SCAN_AN15;
	OpenADC1(config1, config2, config3, config4, configport_l, configport_h, configscan_h, configscan_l);

	/// DMA1 Configuration
	// Disable DMA1 interrupts and set them to the same priority as the ADC
	ConfigIntDMA1(DMA1_INT_DISABLE & DMA1_INT_PRI_7);

	// And configure DMA1 for use with the ADC.
	uint16_t config = DMA1_MODULE_ON & // Enable DMA1
	                  DMA1_SIZE_WORD & // Data is word-sized
	                  PERIPHERAL_TO_DMA1 & // Data is coming in from the peripheral
	                  DMA1_INTERRUPT_BLOCK & // Perform interrupt when all data has been moved
	                  DMA1_NORMAL & // Unsure, but normal operation sounds right
	                  DMA1_PERIPHERAL_INDIRECT & // Required for using the ADC in scatter/gather mode
	                  DMA1_CONTINUOUS; // Continually operate the DMA

    OpenDMA1(config, // Set a bunch of configuration values
	         DMA1_AUTOMATIC, // Transfer interrupts are automatic
	         __builtin_dmaoffset(adcBuf), // The stating address is where the adcBuf is.
	         NULL, // There's no STB address in this case
	         (volatile unsigned int)&ADC1BUF0, // Specify this is coming from the ADC1 sample buffer
	         0 // Only transfer a single word per request
	);
	DMA1REQbits.IRQSEL = 13; // Attach this DMA to the ADC1 conversion done event
}

float GetPowerRailVoltage(void)
{
	return analogSensors.powerRailVoltage;
}

float GetPowerRailCurrent(void)
{
	return analogSensors.powerRailCurrent;
}