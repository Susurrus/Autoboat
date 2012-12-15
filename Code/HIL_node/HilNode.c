#include <xc.h>

#include <stdint.h>
#include <pps.h>

#include "Hil.h"
#include "Acs300.h"
#include "MessageScheduler.h"
#include "Ecan1.h"
#include "Nmea2000.h"
#include "Rudder.h"
#include "Nmea2000Encode.h"
#include "Node.h"
#include "CanMessages.h"
#include "Timer2.h"
#include "Timer4.h"
#include "HilNode.h"

//Use internal RC
_FOSCSEL(FNOSC_FRC & IESO_OFF);
//Clock Pragmas
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);
// Disable watchdog timer
_FWDT(FWDTEN_OFF);
//ICD Pragmas
_FICD(JTAGEN_OFF & ICS_PGD3);

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

int main()
{
    PLLFBD = 63;            // M = 65
    CLKDIVbits.PLLPOST = 0; // N1 = 2
    CLKDIVbits.PLLPRE = 1;  // N2 = 3

    __builtin_write_OSCCONH(0x01); // Initiate Clock Switch to

    __builtin_write_OSCCONL(OSCCON | 0x01); // Start clock switching

    while (OSCCONbits.COSC != 1); // Wait for Clock switch to occur

    while (OSCCONbits.LOCK != 1);

    // Initialize everything
    HilNodeInit();

    // We continually loop through processing serial data and CAN messages
    // Transmission of both are handled by interrupts.
    while (1) {
        HilReceiveData();
        CanReceiveMessages();
    }
}

void HilNodeInit(void)
{
    // Set a unique node ID for this node.
    nodeId = CAN_NODE_HIL;

	// And configure the Peripheral Pin Select pins:
	PPSUnLock;
	// To enable ECAN1 pins: TX on 7, RX on 4
	PPSOutput(OUT_FN_PPS_C1TX, OUT_PIN_PPS_RP7);
	PPSInput(PPS_C1RX, PPS_RP4);

	// To enable UART1 pins: TX on 11, RX on 13
	PPSOutput(OUT_FN_PPS_U1TX, OUT_PIN_PPS_RP11);
	PPSInput(PPS_U1RX, PPS_RP13);
	PPSLock;

    // Finally enable pins B10/A4 as an digital output
    TRISBbits.TRISB10 = 0;
    TRISAbits.TRISA4 = 0;

    // Initialize communications for HIL.
    HilInit();

    // Set up Timer2 for a 100Hz timer.
    Timer2Init(HilNodeTimer100Hz, 1562);

	// Set up Timer4 to manage the blinking amber LED.
	// Initially set it for 0.25s.
	Timer4Init(HilNodeBlink, 39062);

    // Initialize ECAN1
    Ecan1Init();

	// Set a schedule for outgoing CAN messages
    // Transmit the rudder angle at 10Hz
    if (!AddMessageRepeating(&sched, SCHED_ID_RUDDER_ANGLE, 10)) {
            while (1);
    }

    // Transmit the rudder status at 10Hz
    if (!AddMessageRepeating(&sched, SCHED_ID_RUDDER_LIMITS, 10)) {
            while (1);
    }

    // Transmit the throttle status at 100Hz
    if (!AddMessageRepeating(&sched, SCHED_ID_THROTTLE_STATUS, 10)) {
            while (1);
    }

    // Transmit the RC status at 2Hz
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

void HilNodeBlink(void)
{
    // Keep a variable here for counting the 4Hz timer so that it can be used to blink an LED
    // at 1Hz.
    static int timerCounter = 0;

    /// Toggle the amber LED at 1Hz when no in HIL and 4Hz otherwise.
    if (++timerCounter >= ((HIL_ACTIVE)?1:4)) {
        LATA ^= 0x0010;
		timerCounter = 0;
    }
}

void HilNodeTimer100Hz(void)
{

    // Track the messages to be transmit for this timestep.
	// Here we emulate the same transmission frequency of the messages actually transmit
	// by the onboard sensors.
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

	// Transmit the HIL data at 100Hz
	HilTransmitData();
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
