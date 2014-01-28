// Include Microchip headers
#include <xc.h>
#include <pps.h>
#include <uart.h>

// Include standard library headers
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdio.h>

// Include user headers
#include "mavlink.h"
#include "Timer2.h"

void MavlinkObserverTime4Hz(void);

static const char startupString[] = "[%lu]: STARTUP - Monitoring UART1 for valid MAVLink data.\n";
static const char errorString[] = "[%lu]: ERROR - MAVLink corruption found in remote system; resetting it. (remote system time %lu)\n";
static const char goodUpdateString[] = "[%lu]: UPDATE - Data good. (remote system time:%lu)\n";
static const char corruptedUpdateString[] = "[%lu]: UPDATE - DATA CORRUPTED. (remote system time:%lu)\n";
static const char timeString[] = "[%lu]: UPDATE - Found first SYSTEM_TIME message. (remote system time:%lu)\n";
static const char msgString[] = "[%lu]: UPDATE - Found %u message. (remote system time:%lu)\n";

// Keep track of the processor's operating frequency.
#define F_OSC 80000000L

// Calculate the BRG register value necessary for 115200 baud with a 80MHz clock.
#define BAUD115200_BRG_REG 21

// Specify the update period in seconds, we use 15 minutes now.
#define UPDATE_PERIOD (5*60)

// Track the state of the MAVLink connection, whether it's corrupted or not.
#define CORRUPTED 0
#define GOOD      1

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
static uint32_t remoteSystemTime = 0; // Seconds since the remote (watched) system powered on.
static bool sendUpdate = false; // Event flag for triggering a UART transmission update.
static int state = GOOD;

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

    // Enable pin A3, the red LED on the CAN node, as an output. We'll blink this at 1Hz. It'll
	// stay lit when in HIL mode with it turning off whenever packets are received.
	_LATA3 = 0;
    _TRISA3 = 0;

    // Set Timer2 to be a 4Hz timer. Used for blinking the amber status LED and incrementing a system clock.
    Timer2Init(MavlinkObserverTime4Hz, 39062);

	// Set B14 to an output, used for triggering the reset on the watched hardware. This is an
	// active-low signal, so we set it high when it's not being used
	_LATB14 = 1;
	_TRISB14 = 0;

    // Configure and open the port.
	OpenUART1(
		UART_EN & UART_IDLE_CON & UART_IrDA_DISABLE & UART_MODE_FLOW & UART_UEN_00 & UART_EN_WAKE & UART_DIS_LOOPBACK & UART_DIS_ABAUD & UART_NO_PAR_8BIT & UART_UXRX_IDLE_ONE & UART_BRGH_SIXTEEN & UART_1STOPBIT,
		UART_INT_TX_LAST_CH & UART_IrDA_POL_INV_ZERO & UART_SYNC_BREAK_DISABLED & UART_TX_ENABLE & UART_INT_RX_CHAR & UART_ADR_DETECT_DIS & UART_RX_OVERRUN_CLEAR,
		BAUD115200_BRG_REG
	);

    // Disable all interrupts, we just read/write directly.
    ConfigIntUART1(UART_RX_INT_DIS & UART_RX_INT_PR6 &
                   UART_TX_INT_DIS & UART_TX_INT_PR6);

	// Let the user know we're running. Do this every hour as well
	char outString[100];
	snprintf(outString, sizeof(outString), startupString, systemTime);
	putsUART1((unsigned int*)outString);

	// Track if this is the first SYSTEM_TIME mesage received, as we only want to print the message
	// that we received a message when we receive the first one.
	bool firstTimestamp = true;

    // We don't have any main() code, everything's done in the UART and Timer interrupts.
	mavlink_message_t msg = {0};
	mavlink_status_t status = {0};
	uint16_t noMessageBytes = 0; // Track how many bytes we've received and have not decoded a message. Once this hits MAVLINK_MAX_PACKET_LEN, we assume the other chip has now been corrupted.
    while (true) {
		// The snprintf() function calls can take a while, so the UART RX FIFO can back up. When this
		// happens, no more data can be received. And evidently the FIFO just always thinks it has data.
		// Checking for this event and clearing this bit fixes all these issues.s
		if (U1STAbits.OERR) {
			U1STAbits.OERR = 0;
		}

		if (U1STAbits.URXDA) {
			uint8_t c = (uint8_t)U1RXREG;
			if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

				// If it's a system time message, log the remote system time
				if (msg.msgid == MAVLINK_MSG_ID_SYSTEM_TIME) {
					remoteSystemTime = mavlink_msg_system_time_get_time_boot_ms(&msg) / 1000;

					// And report finding the first timestamp by printing to UART1
					if (firstTimestamp) {
						char outString[120];
						snprintf(outString, sizeof(outString), timeString, systemTime, remoteSystemTime);
						putsUART1((unsigned int*)outString);
						firstTimestamp = false;
					}
				} else {
//					char outString[150];
//					snprintf(outString, sizeof(outString), msgString, systemTime, msg.msgid, remoteSystemTime);
//					putsUART1((unsigned int*)outString);
				}

				// Reset our count of unmatches bytes
				noMessageBytes = 0;
				state = GOOD;
			} else {
				if (++noMessageBytes == MAVLINK_MAX_PACKET_LEN && state == GOOD) {
					// We're now in a corrupted state.
					state = CORRUPTED;

					// Turn on the red LED to indicate an error
					_LATA3 = 1;

					// Now we reset the remote processor (using RB14 as a digital output), holding it low for a bit.
					_LATB14 = 0;

					// Now we report the error by printing to UART1
					char outString[150];
					snprintf(outString, sizeof(outString), errorString, systemTime, remoteSystemTime);
					putsUART1((unsigned int*)outString);

					// Reset our watcher now.
					remoteSystemTime = 0;
					mavlink_reset_channel_status(MAVLINK_COMM_0);
					firstTimestamp = true;

					// Stop holding the device in reset.
					_LATB14 = 1;

					// And wait for the systemclock to increment by a second, which is how much time we'll give the remote node to start up.
					uint32_t currentTime = systemTime;
					while (systemTime == currentTime);

					// Don't send an update until the system restarts.
					sendUpdate = false;

					// And turn off the red LED
					_LATA3 = 0;
				}
			}
		}

		// Output a status message every 15m. Let's us know the system's still active.
		if (sendUpdate) {
			char outString[120];
			if (state == CORRUPTED) {
				snprintf(outString, sizeof(outString), corruptedUpdateString, systemTime, remoteSystemTime);
			} else {
				snprintf(outString, sizeof(outString), goodUpdateString, systemTime, remoteSystemTime);
			}
			putsUART1((unsigned int*)outString);
			sendUpdate = false;
		}
	}
}

void MavlinkObserverTime4Hz(void)
{
    // Keep a variable here for scaling the 4Hz timer to a 1Hz timer.
	static int timerCounter = 0;

	// Check if it's time to toggle the status LED. The limit is decided based on whether HIL is
	// active and if the rudder is detected.
	if (++timerCounter >= 4) {

		// Increment the system time.
		++systemTime;

		// Flag that an update message should be sent out if it's been 15 minutes.
		if (systemTime % UPDATE_PERIOD == 0) {
			sendUpdate = true;
		}

		// Toggle the amber LED
		_LATA4 ^= 1;

		// Reset the counter
		timerCounter = 0;
	}
}
