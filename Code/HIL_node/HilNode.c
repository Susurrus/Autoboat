#include <xc.h>

#include <stdint.h>

#include "Hil.h"
#include "Acs300.h"
#include "MessageScheduler.h"
#include "Ecan1.h"
#include "Nmea2000.h"
#include "Rudder.h"
#include "Nmea2000Encode.h"
#include "Node.h"
#include "CanMessages.h"

//Use internal RC
_FOSCSEL(FNOSC_FRC & IESO_OFF);
//Clock Pragmas
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);
// Disable watchdog timer
_FWDT(FWDTEN_OFF);
//ICD Pragmas
_FICD(JTAGEN_OFF & ICS_PGD3);

// Store the timer callback used by timer2.
void (*timer2Callback)(void);

// Keep a variable here for counting the 100Hz timer so that it can be used to blink an LED
// at 1Hz.
int timerCounter = 0;

/**
 * Initializes Timer 2 for a 156,250 Hz clock. The prescalar can be used to modify this clock rate
 * to the desied ones. When the clock expires it triggers a call to `timerCallbackFcn` and resets.
 * The Timer is set up for a 256x multiplier on the prescalar, so your effective prescalar is really
 * 256*prescalar, with the final frequency being 156250/256/prescalar Hz.
 * @param timerCallbackFcn
 * @param prescalar
 */
void Timer2Init(void (*timerCallbackFcn)(void), uint16_t prescalar)
{
    T2CON = 0;
    IFS0bits.T2IF = 0;
    IPC1bits.T2IP2 = 1;
    IEC0bits.T2IE = 1;
    PR2 = prescalar;
    T2CON = 0x8030;

    timer2Callback = timerCallbackFcn;
}

void HilNodeInit(void)
{
    // Set a unique node ID for this node.
    nodeId = CAN_NODE_HIL;

    // Initialize communications for HIL.
    HilInit();

    // Set up Timer2 for a 100Hz timer.
    Timer2Init(HilTransmitData, 1562);

    // Set UART1 to be pins B11/B13 TX/RX
    // (see Section 30 of the dsPIC33f manual)
    __builtin_write_OSCCONL(OSCCON & ~(1<<6));

    RPOR5bits.RP11R = 3; // Set RP11 to be U1TX
    RPINR18bits.U1RXR = 13; // Set U1RX to pin RP13

    __builtin_write_OSCCONL(OSCCON | (1<<6));

    // Enable pins B10/A4 as an digital output
    TRISBbits.TRISB10 = 0;
    TRISAbits.TRISA4 = 0;
}

int main()
{
    PLLFBD = 63;            // M = 65
    CLKDIVbits.PLLPOST = 0; // N1 = 2
    CLKDIVbits.PLLPRE = 1;  // N2 = 3

    __builtin_write_OSCCONH(0x01); // Initiate Clock Switch to

    __builtin_write_OSCCONL(OSCCON | 0x01); // Start clock switching

    while (OSCCONbits.COSC != 1); // Wait for Clock switch to occur

    while (OSCCONbits.LOCK != 1);
	
	HilNodeInit();

    while (1) {
        HilReceiveData();
    }
}

// Declare some constants for use with the message scheduler
// (don't use PGN or message ID as it must be a uint8)
enum {
    // Rudder messages
    SCHED_ID_RUDDER_ANGLE = 1,
    SCHED_ID_RUDDER_LIMITS, // For status bits

    // Throttle messages
    SCHED_ID_THROTTLE_STATUS,

    // RC node status messages
    SCHED_ID_RC_STATUS,

    // GPS messages
    SCHED_ID_LAT_LON,
    SCHED_ID_COG_SOG,

    // IMU messages
    SCHED_ID_ATT_ANGLE,

    // DST800 messages
    SCHED_ID_WATER_SPD
};

// Set up the message scheduler's various data structures.
#define ECAN_MSGS_SIZE 8
static uint8_t ids[ECAN_MSGS_SIZE] = {
    SCHED_ID_RUDDER_ANGLE,
    SCHED_ID_RUDDER_LIMITS,
    SCHED_ID_THROTTLE_STATUS,
    SCHED_ID_RC_STATUS,
    SCHED_ID_LAT_LON,
    SCHED_ID_COG_SOG,
    SCHED_ID_ATT_ANGLE,
    SCHED_ID_WATER_SPD
};
static uint16_t tsteps[ECAN_MSGS_SIZE][2][8] = {};
static uint8_t  mSizes[ECAN_MSGS_SIZE];
static MessageSchedule sched = {
	ECAN_MSGS_SIZE,
	ids,
	mSizes,
	0,
	tsteps
};

void CanEnableMessages(void)
{
	// Transmit the rudder angle at 10Hz
	if (!AddMessageRepeating(&sched, SCHED_ID_RUDDER_ANGLE, 10)) {
		while (1);
	}

	// Transmit the rudder status at 10Hz
	if (!AddMessageRepeating(&sched, SCHED_ID_RUDDER_LIMITS, 10)) {
		while (1);
	}

	// Transmit the throttle status at 100Hz
	if (!AddMessageRepeating(&sched, SCHED_ID_THROTTLE_STATUS, 100)) {
		while (1);
	}

	// Transmit the throttle status at 2Hz
	if (!AddMessageRepeating(&sched, SCHED_ID_RC_STATUS, 2)) {
		while (1);
	}

	// Transmit latitude/longitude at 5Hz
	if (!AddMessageRepeating(&sched, SCHED_ID_LAT_LON, 5)) {
		while (1);
	}

	// Transmit heading & speed at 4Hz
	if (!AddMessageRepeating(&sched, SCHED_ID_COG_SOG, 4)) {
		while (1);
	}
}

void CanDisableMessages(void)
{
    ClearSchedule(&sched);
}

uint8_t CanReceiveMessages(void)
{
	uint8_t messagesLeft = 0;
	CanMessage msg;
	uint32_t pgn;

	uint8_t messagesHandled = 0;

	do {
		int foundOne = Ecan1Receive(&msg, &messagesLeft);
		if (foundOne) {
			// Process throttle messages here. Anything not explicitly handled is assumed to be a NMEA2000 message.
			if (msg.id == ACS300_CAN_ID_WR_PARAM) { // From the ACS300
                uint16_t address, data;
                Acs300DecodeWriteParam(msg.payload, &address, &data);
                if (address == ACS300_PARAM_CC) {
                    hilDataToTransmit.data.tCommandSpeed = (float)(int16_t)data;
                }
			} else {
				pgn = Iso11783Decode(msg.id, NULL, NULL, NULL);
				switch (pgn) {
                // Decode the commanded rudder angle from the PGN127245 messages.
				case PGN_RUDDER: {
					ParsePgn127245(msg.payload, NULL, NULL, NULL, &hilDataToTransmit.data.rCommandAngle, NULL);
				} break;
				}
			}

			++messagesHandled;
		}
	} while (messagesLeft > 0);

	return messagesHandled;
}

void CanTransmitMessages(void)
{
    // Track the last HIL status
    static bool hilStatus = false;

    // Enable or disable CAN messages based on whether HIL is active or not.
    if (!hilStatus && HilActive()) {
        CanEnableMessages();
        hilStatus = true;
    } else if (hilStatus && !HilActive()) {
        CanDisableMessages();
        hilStatus = false;
    }

    // Track the messages to be transmit for this timestep.
    static uint8_t msgs[ECAN_MSGS_SIZE];

    uint8_t messagesToSend = GetMessagesForTimestep(&sched, msgs);
    int i;
    CanMessage msg;
    for (i = 0; i < messagesToSend; ++i) {
        switch (msgs[i]) {
            case SCHED_ID_RC_STATUS:
                CanMessagePackageStatus(&msg, CAN_NODE_RC, 0, 0, 0);
                Ecan1BufferedTransmit(&msg);
            break;
            case SCHED_ID_RUDDER_ANGLE:
                PackagePgn127245(&msg, nodeId, 0xFF, 0xF, NAN, hilReceivedData.data.rAngle);
                Ecan1BufferedTransmit(&msg);
            break;
            case SCHED_ID_RUDDER_LIMITS:
                CanMessagePackageRudderDetails(&msg, 0, 0, 0, false, false, true, true, false);
                Ecan1BufferedTransmit(&msg);
            break;
            case SCHED_ID_THROTTLE_STATUS:
                Acs300PackageHeartbeat(&msg, (uint16_t)hilReceivedData.data.tSpeed, 0, 0, 0);
                Ecan1BufferedTransmit(&msg);
            break;
            case SCHED_ID_LAT_LON:
                PackagePgn129025(&msg, nodeId, hilReceivedData.data.gpsLatitude, hilReceivedData.data.gpsLongitude);
                Ecan1BufferedTransmit(&msg);
            break;
            case SCHED_ID_COG_SOG:
                PackagePgn129026(&msg, nodeId, 0xFF, 0x7, hilReceivedData.data.gpsCog, hilReceivedData.data.gpsSog);
                Ecan1BufferedTransmit(&msg);
            break;
        }
    }
}

/**
 * Timer 2 interrupt. Merely calls the timerCallback() specified in Timer2Init().
 */
void __attribute__((interrupt, auto_psv)) _T2Interrupt(void)
{
    timer2Callback();

    // Toggle the amber LED at 1Hz.
    if (++timerCounter == 100) {
        LATA ^= 0x0010;
        timerCounter = 0;
    }

    IFS0bits.T2IF = 0;
}