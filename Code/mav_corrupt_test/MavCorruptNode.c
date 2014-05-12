// C standard library includes
#include <stdlib.h>
#include <stdio.h>

// Microchip standard library includes
#include <xc.h>
#include <pps.h>
#include <adc.h>
#include <uart.h>
#include <dma.h>
#include <timer.h>

// Project includes
#include "MavlinkGlue.h"
#include "Uart1.h"
#include "Uart2.h"
#include "MavCorruptNode.h"
#include "MissionManager.h"

// Define some macros for setting pins as inputs or outputs using the TRIS pins.
#define OUTPUT 0
#define INPUT 1

// Declare some macros for helping setting bit values
#define ON  1
#define OFF 0

// Store actuator commmands here. Used by the MAVLink code
ActuatorCommands currentCommands;

// Specify how long after startup the node should stay in reset to let things stabilize. Units are
// centiseconds.
#define STARTUP_RESET_TIME 200

// Keep track of the processor's operating frequency.
#define F_OSC 80000000L

// Declare some function prototypes.
void PrimaryNode100HzLoop(void);

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

	// Initialize UART1 to 115200 for groundstation communications.
	Uart1Init(BAUD115200_BRG_REG);

	// Initialize UART2 to 115200 for monitoring the groundstation communications.
	Uart2Init(BAUD115200_BRG_REG);

	// Initialize the MAVLink communications channel
	MavLinkInit();

        // Set up Timer 2 for 100Hz operation and no interrupts (polling interface used)
	OpenTimer2(T2_ON & T2_IDLE_CON & T2_GATE_OFF & T2_PS_1_256 & T2_32BIT_MODE_OFF & T2_SOURCE_INT, UINT16_MAX);
	ConfigIntTimer2(T2_INT_PRIOR_1 & T2_INT_OFF);

	// Finally perform the necessary pin mappings:
	PPSUnLock;

#ifdef __dsPIC33FJ128MC802__
	// To enable ECAN1 pins: TX on 7, RX on 4
	PPSOutput(OUT_FN_PPS_C1TX, OUT_PIN_PPS_RP7);
	PPSInput(IN_FN_PPS_C1RX, IN_PIN_PPS_RP4);

	// To enable UART1 pins: TX on 11, RX on 13
	PPSOutput(OUT_FN_PPS_U1TX, OUT_PIN_PPS_RP11);
	PPSInput(IN_FN_PPS_U1RX, IN_PIN_PPS_RP13);
#elif __dsPIC33EP256MC502__
	// To enable ECAN1 pins: TX on 39, RX on 36
	PPSOutput(OUT_FN_PPS_C1TX, OUT_PIN_PPS_RP39);
	PPSInput(IN_FN_PPS_C1RX, IN_PIN_PPS_RP36);

	// To enable UART1 pins: TX on 43 (B11), MAVLink input decode on 43 (B11), RX on 45 (B13)
	PPSOutput(OUT_FN_PPS_U1TX, OUT_PIN_PPS_RP43);
	PPSInput(IN_FN_PPS_U1RX, IN_PIN_PPS_RPI45);
	
	// Enable UART2 RX on the same pin as UART1 TX, so we can decode it's live output stream
	PPSInput(IN_FN_PPS_U2RX, IN_PIN_PPS_RP43);
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
	// B0 (output): Debugging output pin for when the MAVLink stream gets corrupted. High when it is.
	_LATB0 = OFF;
	ANSELBbits.ANSB0 = 0; // Also disable analog functionality on B0 so we can use it as a digital pin.
	_TRISB0 = OUTPUT;

        MissionInit();
        int i;
        mavlink_mission_item_t m = {};
        for (i = 0; i < 15; ++i) {
            m.x = i;
            m.y = 100.0;
            m.z = i;
            m.param1 = 1;
            m.param2 = 2;
            m.param3 = 3;
            m.param4 = 4;
            m.command = MAV_CMD_NAV_WAYPOINT;
            m.frame = MAV_FRAME_LOCAL_NED;
            m.autocontinue = true;
            MavLinkAppendMission(&m);
        }
        SetCurrentMission(0);
        Mission m2 = {};
        SetStartingPoint(&m2);

	// Report on system status now that initialization is complete.
	MavLinkSendStatusText(MAV_SEVERITY_INFO, "Finished initialization. MAV_CORRUPT_NODE");

	// Run system tasks when a timer interrupt has been triggered.
	uint8_t inData;
	mavlink_message_t msg = {0};
	mavlink_status_t status = {0};
	uint16_t noMessageBytes = 0; // Track how many bytes we've received and have not decoded a message. Once this hits 2*MAVLINK_MAX_PACKET_LEN, we assume we have corruption.
	bool uart1TxStateIsGood = true; // Keeps track of if UART1 is transmitting properly.
	uint32_t firstCorruptionTime = 0; // Track when the corruption first occurred.
	while (true) {

            // Check for new MAVLink messages. We may get multiple of the
            // same message between our 100Hz primary controller ticks, so data may be overridden,
            // but that doesn't really matter, as we were losing that data anyways when we were
            // calling it at 100Hz.
            MavLinkReceive();

		// We continuously process the input on UART2, which should be an exact copy of the data
		// output from UART1, which is a MAVLink stream in the SeaSlug dialect.
		if (Uart2ReadByte(&inData)) {
			// If we managed to process a MAVLink message successfully, reset our invalid char counter.
			// We use comm channel 1, because 0 is being used by MavlinkGlue.c.
			if (mavlink_parse_char(MAVLINK_COMM_1, inData, &msg, &status)) {
				noMessageBytes = 0;
				
				// If we were in a corrupted state, we leave it and send a STATUSTEXT MAVLink message
				// so the operator knows what happened.
				if (!uart1TxStateIsGood) {
					char statusText[50];
					snprintf(statusText, 50, "MAVLink corruption at %lu.", firstCorruptionTime);
					MavLinkSendStatusText(MAV_SEVERITY_NOTICE, statusText);

					// Now fix our state.
					uart1TxStateIsGood = true;

					// And clear the output pin.
					_LATB0 = OFF;
				}
			}

			// Now if we've reached our limit of MAVLINK_MAX_PACKET_LEN, then reset UART1.
			// We timeout after 75 bytes because that's larger than any message transmit by the SeaSlug.
			// The largest message is common:STATUSTEXT.
			if (uart1TxStateIsGood && ++noMessageBytes >= 75) {
				// We're now in a corrupted state.
				uart1TxStateIsGood = false;

				// So set our debugging pin high.
				_LATB0 = ON;

				// Save the time that this corruption happened at
				firstCorruptionTime = nodeSystemTime;

				// Reset our MAVLink decoder now. Make sure we're on comm channel 1 to match the
				// decoding call above.
				mavlink_reset_channel_status(MAVLINK_COMM_1);

				// And reset our UART1 hardware.
				Uart1Init(BAUD115200_BRG_REG);
			}
		}
		if (TMR2 > F_OSC / 2 / 256 / 100) {
                        _LATB15 = 1;
			PrimaryNode100HzLoop();
                        _LATB15 = 0;
                        TMR2 = 0;
		}
	}
}

/**
 * Perform main timed loop at 100Hz.
 */
void PrimaryNode100HzLoop(void)
{

    // Increment the counters for the mission and parameter protocols. They need to time some things
    // this way, as they're normally called as fast as possible, so this external counter method is
    // used.
    IncrementMissionCounter();

	// And make sure the primary LED is blinking indicating that the node is operational
	SetStatusModeLed();

	// Update the onboard system time counter. We make sure we don't overflow here as we can
        // run into issues with startup code being executed again.
        if (nodeSystemTime < UINT32_MAX) {
            ++nodeSystemTime;
        }

	// Send any necessary messages for this timestep.
	MavLinkTransmit();
}

/**
 * Set the primary status indicator LED to always blink at 1Hz.
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