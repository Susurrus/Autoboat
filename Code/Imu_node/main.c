#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include <xc.h>
#include <pps.h>

#include "ImuNode.h"
#include "Timer2.h"
#include "Uart1.h"

// Flag for triggering a run of the primary loop. Set by the timer interrupt.
static bool runTasks = false;

// Set some function prototypes.
void SetTaskFlag(void);

// Set processor configuration settings
#define F_OSC 80000000l
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

int main()
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

	// Initialize ADCs for reading voltage and temperature sensors
	ImuNodeInit(F_OSC);

	// Set up a timer at 100.0320Hz, where F_timer = F_CY / 256 / prescalar.
	Timer2Init(SetTaskFlag, F_OSC / 2 / 256 / 100);

	// Run system tasks when a timer interrupt has been triggered.
	while (true) {
		if (runTasks) {
			Run100HzTasks();
			runTasks = false;
		}
		RunContinuousTasks();
	}
}

/**
 * Timer interrupt callback. Sets a flag that the main execution loop waits on to do everything.
 */
void SetTaskFlag(void)
{
    runTasks = true;
}
