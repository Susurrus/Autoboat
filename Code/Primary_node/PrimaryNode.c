// C standard library includes
#include <stdlib.h>

// Microchip standard library includes
#include <pps.h>
#include <adc.h>
#include <dma.h>

// Project includes
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
#include "MissionManager.h"
#include "Timer2.h"

// MATLAB-generate code includes
#include "controller.h"

// Clearing the TRIS bit for a pin specifies it as an output
#define OUTPUT 0

// Declare some macros for helping setting bit values
#define ON  1
#define OFF 0

// Specify how long after startup the node should stay in reset to let things stabilize. Units are
// centiseconds.
#define STARTUP_RESET_TIME 200

// Keep track of the processor's operating frequency.
#define F_OSC 80000000L

// Define the maximum value of the ADC input
#define ANmax 4095.0f

// Flag for triggering a run of the primary loop. Set by the timer interrupt.
bool runTasks = false;

// Store analog sensors here
struct {
	float powerRailVoltage;
	float powerRailCurrent;
} analogSensors;

// Store actuator commmands here. Used by the MAVLink code
ActuatorCommands currentCommands;

// Set up DMA memory for the ADC. But with the scatter- gather mode enabled on the ADC, we reserve
// an array for all 16 possible inputs, so we align to 32-byte boundaries instead.
#ifdef __dsPIC33FJ128MC802__
static volatile uint16_t adcDmaBuffer[16] __attribute__((space(dma),aligned(32)));
#elif __dsPIC33EP256MC502__
static volatile uint16_t adcDmaBuffer[16] __attribute__((aligned(32)));
#endif

// Declare some function prototypes.
void Adc1Init(void);
void PrimaryNode100HzLoop(void);
void PrimaryNodeMuxAndOutputControllerCommands(float rudderCommand, int16_t throttleCommand);
void SetTaskFlag(void);

// Set processor configuration settings
#ifdef __dsPIC33FJ128MC802__
	// Use internal RC to start; we then switch to PLL'd iRC.
	_FOSCSEL(FNOSC_FRC & IESO_OFF);
	// Clock Pragmas
	_FOSC(FCKSM_CSECMD & OSCIOFNC_ON & POSCMD_NONE);
	// Disable watchdog timer
	_FWDT(FWDTEN_OFF);
	// Disable JTAG and specify port 3 for ICD pins.
	_FICD(JTAGEN_OFF & ICS_PGD3);
#elif __dsPIC33EP256MC502__
	// Use internal RC to start; we then switch to PLL'd iRC.
	_FOSCSEL(FNOSC_FRC & IESO_OFF);
	// Clock Pragmas
	_FOSC(FCKSM_CSECMD & OSCIOFNC_ON & POSCMD_NONE);
	// Disable watchdog timer
	_FWDT(FWDTEN_OFF);
	// Disable JTAG and specify port 2 for ICD pins.
	_FICD(JTAGEN_OFF & ICS_PGD2);
#endif

int main(void)
{
	/// First step is to move over to the FRC w/ PLL clock from the default FRC clock.
	// Set the clock to 79.84MHz.
    PLLFBD = 63;            // M = 65
    CLKDIVbits.PLLPOST = 0; // N1 = 2
    CLKDIVbits.PLLPRE = 1;  // N2 = 3

	// Initiate Clock Switch to FRM oscillator with PLL.
    __builtin_write_OSCCONH(0x01);
    __builtin_write_OSCCONL(OSCCON | 0x01);

	// Wait for Clock switch to occur.
	while (OSCCONbits.COSC != 1);

	// And finally wait for the PLL to lock.
    while (OSCCONbits.LOCK != 1);
	
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
	Ecan1Init(F_OSC);

	// Initialize the MAVLink communications channel
	MavLinkInit();

	// Set up the ADC
	Adc1Init();

	// Set up a timer at 100.0320Hz, where F_timer = F_CY / 256 / prescalar.
	Timer2Init(PrimaryNode100HzLoop, F_OSC / 2 / 256 / 100);

	// Finally perform the necessary pin mappings:
	PPSUnLock;

#ifdef __dsPIC33FJ128MC802__
	// To enable ECAN1 pins: TX on 7, RX on 4
	PPSOutput(OUT_FN_PPS_C1TX, OUT_PIN_PPS_RP7);
	PPSInput(PPS_C1RX, PPS_RP4);

	// To enable UART1 pins: TX on 11, RX on 13
	PPSOutput(OUT_FN_PPS_U1TX, OUT_PIN_PPS_RP11);
	PPSInput(PPS_U1RX, PPS_RP13);

	// To enable UART2 pins: TX on 8, RX on 9
	PPSOutput(OUT_FN_PPS_U2TX, OUT_PIN_PPS_RP8);
	PPSInput(PPS_U2RX, PPS_RP9);
#elif __dsPIC33EP256MC502__
	// To enable ECAN1 pins: TX on 39, RX on 36
	PPSOutput(OUT_FN_PPS_C1TX, OUT_PIN_PPS_RP39);
	PPSInput(PPS_C1RX, PPS_RP36);

	// To enable UART1 pins: TX on 43, RX on 45
	PPSOutput(OUT_FN_PPS_U1TX, OUT_PIN_PPS_RP43);
	PPSInput(PPS_U1RX, PPS_RPI45);

	// To enable UART2 pins: TX on 40, RX on 41
	PPSOutput(OUT_FN_PPS_U2TX, OUT_PIN_PPS_RP40);
	PPSInput(PPS_U2RX, PPS_RP41);
#endif
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

	// Finally initialize the controller model
	controller_initialize();

	// Run system tasks when a timer interrupt has been triggered.
	while (true) {
		if (runTasks) {
			PrimaryNode100HzLoop();
			runTasks = false;
		}
	}
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
	// Battery voltage in Volts
	analogSensors.powerRailVoltage = (3.3 / ANmax) / .06369 * (float)adcDmaBuffer[0];

	// Battery current in Amps
	analogSensors.powerRailCurrent = (3.3 / ANmax) / .03660 * (float)adcDmaBuffer[3];

	// Input voltage in dV (AN5)
	nodeVoltage = (uint8_t)((3.3 / ANmax) * ((21.0 + 2.0) / 2.0) * 10.0 * (float)adcDmaBuffer[5]);
			
	// Onboard temperature in degrees C (AN1)
	nodeTemp = (int8_t)((3.3 / ANmax * (float)adcDmaBuffer[1] - 0.5) * 100.0);

	// Send any necessary messages for this timestep.
	MavLinkTransmit();

	// At 2Hz transmit a NODE_STATUS ECAN message.
	if (internalCounter == 0 || internalCounter == 50) {
		NodeTransmitStatus();
	}

	// Set a reset signal for the first 2 seconds, allowing things to stabilize a bit before the
	// system responds. This is especially crucial because it can take up to a second for sensors to
	// timeout and appear as offline.
	// FIXME: Note that this variable wrapping around will cause problems. This shouldn't be an issue
	// as it goes until 500 days before this will happen.
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

	// Run the next timestep of the controller.
	float rCommand;
	int16_t tCommand;
	bool reset = (nodeErrors != 0);
	controller_custom(&gpsDataStore, &throttleDataStore.rpm, &rudderSensorData.RudderAngle,
		&reset, &waterDataStore.speed, &rCommand, &tCommand);

	// And output the necessary control outputs.
	PrimaryNodeMuxAndOutputControllerCommands(rCommand, tCommand);

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
 * Relies on DMA1
 */
void Adc1Init(void)
{
	// Initialize ADC for reading temperature, 2x power rail voltage, and current draw.
	// Use standard V_ref+/V_ref- voltage references.
	SetChanADC1(
		ADC_CH123_NEG_SAMPLEA_VREFN & ADC_CH123_NEG_SAMPLEB_VREFN & ADC_CH123_POS_SAMPLEA_0_1_2 & ADC_CH123_POS_SAMPLEB_0_1_2,
		ADC_CH0_POS_SAMPLEA_AN0 & ADC_CH0_NEG_SAMPLEA_VREFN & ADC_CH0_POS_SAMPLEB_AN0 & ADC_CH0_NEG_SAMPLEB_VREFN
	);
	// Open AN1 (temperature) and AN5 (voltage) pins for 12-bit unsigned integer readings. Also note
	// that this will only store one sample per analog input.
#ifdef __dsPIC33FJ128MC802__
	OpenADC1(
		ADC_MODULE_ON & ADC_IDLE_CONTINUE & ADC_ADDMABM_SCATTR & ADC_AD12B_12BIT & ADC_FORMAT_INTG & ADC_CLK_AUTO & ADC_AUTO_SAMPLING_ON & ADC_SIMULTANEOUS & ADC_SAMP_ON,
		ADC_VREF_AVDD_AVSS & ADC_SCAN_ON & ADC_SELECT_CHAN_0 & ADC_DMA_ADD_INC_4 & ADC_ALT_BUF_OFF & ADC_ALT_INPUT_OFF,
		ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_INTERNAL_RC & ADC_CONV_CLK_32Tcy,
		ADC_DMA_BUF_LOC_1,
		ENABLE_AN0_ANA & ENABLE_AN1_ANA & ENABLE_AN3_ANA & ENABLE_AN5_ANA,
		ENABLE_ALL_DIG_16_31,
		SCAN_NONE_16_31,
		SKIP_SCAN_AN2 & SKIP_SCAN_AN4 & SKIP_SCAN_AN6 & SKIP_SCAN_AN7 &
		SKIP_SCAN_AN8 & SKIP_SCAN_AN9 & SKIP_SCAN_AN10 & SKIP_SCAN_AN11 & SKIP_SCAN_AN12 & SKIP_SCAN_AN13 &
		SKIP_SCAN_AN14 & SKIP_SCAN_AN15
	);
#elif __dsPIC33EP256MC502__
	OpenADC1(
		ADC_MODULE_ON & ADC_IDLE_CONTINUE & ADC_ADDMABM_SCATTR & ADC_AD12B_12BIT & ADC_FORMAT_INTG & ADC_SSRC_AUTO & ADC_AUTO_SAMPLING_ON & ADC_SIMULTANEOUS & ADC_SAMP_ON,
		ADC_VREF_AVDD_AVSS & ADC_SCAN_ON & ADC_SELECT_CHAN_0 & ADC_DMA_ADD_INC_4 & ADC_ALT_BUF_OFF & ADC_ALT_INPUT_OFF,
		ADC_SAMPLE_TIME_31 & ADC_CONV_CLK_INTERNAL_RC & ADC_CONV_CLK_32Tcy,
		ADC_DMA_BUF_LOC_1,
		0, // Don't read any pins in porta
		ENABLE_AN0_ANA & ENABLE_AN1_ANA & ENABLE_AN3_ANA & ENABLE_AN5_ANA, // Enable our specific pins
		0, // Don't read any pins in portc
		0, // Don't read any pins in portd
		0, // Don't read any pins in porte
		0, // Don't read any pins in portf
		0, // Don't read any pins in portg
		0, // Don't read any pins in porth
		0, // Don't read any pins in porti
		0, // Don't read any pins in portj
		0, // Don't read any pins in portk
		SCAN_NONE_16_31,
		SKIP_SCAN_AN2 & SKIP_SCAN_AN4 & SKIP_SCAN_AN6 & SKIP_SCAN_AN7 &
		SKIP_SCAN_AN8 & SKIP_SCAN_AN9 & SKIP_SCAN_AN10 & SKIP_SCAN_AN11 & SKIP_SCAN_AN12 & SKIP_SCAN_AN13 &
		SKIP_SCAN_AN14 & SKIP_SCAN_AN15
	);
#endif
	// Open DMA1 for receiving ADC values
	OpenDMA1(DMA1_MODULE_ON & DMA1_SIZE_WORD & PERIPHERAL_TO_DMA1 & DMA1_INTERRUPT_BLOCK & DMA1_NORMAL & DMA1_PERIPHERAL_INDIRECT & DMA1_CONTINUOUS,
		  DMA1_AUTOMATIC,
#ifdef __dsPIC33FJ128MC802__
		  __builtin_dmaoffset(adcDmaBuffer),
#elif __dsPIC33EP256MC502__
		  (unsigned long int)adcDmaBuffer,
#endif
		  NULL,
		  (uint16_t)&ADC1BUF0,
		  3); // Specify the number of pins being measured (n) as n-1 here. Must match ADC_DMA_ADD_INC_n setting.
	DMA1REQbits.IRQSEL = 0x0D; // Attach this DMA to the ADC1 conversion done event
}

float GetPowerRailVoltage(void)
{
	return analogSensors.powerRailVoltage;
}

float GetPowerRailCurrent(void)
{
	return analogSensors.powerRailCurrent;
}

/**
 * Timer interrupt callback. Sets a flag that the main execution loop waits on to do everything.
 */
void SetTaskFlag(void)
{
	runTasks = true;
}