// Include Microchip headers
#include <xc.h>
#include <pps.h>

// Include standard library headers
#include <stdint.h>
#include <stddef.h>

// Include user headers
#include "mavlink.h"
#include "Timer2.h"

// Keep track of the processor's operating frequency.
#define F_OSC 80000000L

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

static uint32_t systemTime = 0; // Seconds since the system powered on.

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

    // Initialize everything
    HilNodeInit();

    // We don't have any main() code, everything's done in the UART and Timer interrupts.
    while (true);
}

void HilNodeInit(void)
{
    // Set a unique node ID for this node.
    nodeId = CAN_NODE_HIL;

	// And configure the Peripheral Pin Select pins:
	PPSUnLock;
#ifdef __dsPIC33FJ128MC802__
	// To enable UART1 pins: TX on 11, RX on 13
	PPSOutput(OUT_FN_PPS_U1TX, OUT_PIN_PPS_RP11);
	PPSInput(IN_FN_PPS_U1RX, IN_PIN_PPS_RP13);
#elif __dsPIC33EP256MC502__
	// To enable UART1 pins: TX on 43, RX on 45
	PPSOutput(OUT_FN_PPS_U1TX, OUT_PIN_PPS_RP43);
	PPSInput(IN_FN_PPS_U1RX, IN_PIN_PPS_RPI45);
#endif
	PPSLock;

    // Enable pin A4, the amber LED on the CAN node, as an output. We'll blink this at 1Hz. It'll
	// stay lit when in HIL mode with it turning off whenever packets are received.
    _TRISA4 = 0;

    // Set Timer2 to be a 1Hz timer. Used for blinking the amber status LED and incrementing a system clock.
    Timer2Init(MavlinkObserverTime1Hz, 39062);
}

void MavlinkObserverTime1Hz(void)
{
        // Keep a variable here for scaling the 4Hz timer to a 1Hz timer.
	static int timerCounter = 0;

	// Check if it's time to toggle the status LED. The limit is decided based on whether HIL is
	// active and if the rudder is detected.
	int countLimit = 6;
	if (++timerCounter >= countLimit) {
		_LATA4 ^= 1;
		timerCounter = 0;
	}
}
