#include <xc.h>

#include <stdint.h>
#include <pps.h>
#include <stddef.h>

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
#include "Ethernet.h"

#ifndef NAN
#define NAN __builtin_nan("")
#endif

// Use internal RC to start; we then switch to PLL'd iRC.
_FOSCSEL(FNOSC_FRC & IESO_OFF);
// Clock Pragmas
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);
// Disable watchdog timer
_FWDT(FWDTEN_OFF);
// Disable JTAG and specify port 3 for ICD pins.
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
    SCHED_ID_GPS_FIX,

    // IMU messages
    SCHED_ID_ATT_ANGLE,

    // DST800 messages
    SCHED_ID_WATER_SPD
};

// Set up the message scheduler's various data structures.
#define ECAN_MSGS_SIZE 9
static uint8_t ids[ECAN_MSGS_SIZE] = {
    SCHED_ID_RUDDER_ANGLE,
    SCHED_ID_RUDDER_LIMITS,
    SCHED_ID_THROTTLE_STATUS,
    SCHED_ID_RC_STATUS,
    SCHED_ID_LAT_LON,
    SCHED_ID_COG_SOG,
    SCHED_ID_GPS_FIX,
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
	// Switch the clock over to 80MHz.
    PLLFBD = 63;            // M = 65
    CLKDIVbits.PLLPOST = 0; // N1 = 2
    CLKDIVbits.PLLPRE = 1;  // N2 = 3

    __builtin_write_OSCCONH(0x01); // Initiate Clock Switch to

    __builtin_write_OSCCONL(OSCCON | 0x01); // Start clock switching

    while (OSCCONbits.COSC != 1); // Wait for Clock switch to occur

    while (OSCCONbits.LOCK != 1);

    // Initialize everything
    HilNodeInit();

    // We continually loop through processing CAN messages and also HIL messages.
    while (true) {
        CanReceiveMessages();
		HilReceive();
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

	// Configure SPI1 so that:
	//  * (input) SPI1.SDI = B8
	PPSInput(PPS_SDI1, PPS_RP10);
	//  * SPI1.SCK is output on B9
	PPSOutput(OUT_FN_PPS_SCK1, OUT_PIN_PPS_RP9);
	//  * (output) SPI1.SDO = B10
	PPSOutput(OUT_FN_PPS_SDO1, OUT_PIN_PPS_RP8);
	PPSLock;

    // Enable pin A4, the amber LED on the CAN node, as an output. We'll blink this at 1Hz. It'll
	// stay lit when in HIL mode with it turning off whenever packets are received.
    _TRISA4 = 0;

    // Initialize communications for HIL.
    HilInit();

	// Set Timer4 to be a 4Hz timer. Used for blinking the amber status LED.
	Timer4Init(HilNodeBlink, 39062);

    // Set up Timer2 for a 100Hz timer. This triggers CAN message transmission at the same frequency
	// that the sensors actually do onboard the boat.
    Timer2Init(HilNodeTimer100Hz, 1562);

    // Initialize ECAN1
    Ecan1Init();

	// Set a schedule for outgoing CAN messages
    // Transmit the rudder angle at 10Hz
    if (!AddMessageRepeating(&sched, SCHED_ID_RUDDER_ANGLE, 10)) {
		FATAL_ERROR();
    }

    // Transmit the rudder status at 10Hz
    if (!AddMessageRepeating(&sched, SCHED_ID_RUDDER_LIMITS, 10)) {
		FATAL_ERROR();
    }

    // Transmit the throttle status at 100Hz
    if (!AddMessageRepeating(&sched, SCHED_ID_THROTTLE_STATUS, 10)) {
		FATAL_ERROR();
    }

    // Transmit the RC status at 2Hz
    if (!AddMessageRepeating(&sched, SCHED_ID_RC_STATUS, 2)) {
		FATAL_ERROR();
    }

    // Transmit latitude/longitude at 5Hz
    if (!AddMessageRepeating(&sched, SCHED_ID_LAT_LON, 5)) {
		FATAL_ERROR();
    }

    // Transmit heading & speed at 5Hz
    if (!AddMessageRepeating(&sched, SCHED_ID_COG_SOG, 5)) {
		FATAL_ERROR();
    }

    // Transmit heading & speed at 5Hz
    if (!AddMessageRepeating(&sched, SCHED_ID_GPS_FIX, 5)) {
		FATAL_ERROR();
    }
}

void HilNodeBlink(void)
{
	// Keep a variable here for scaling the 4Hz timer to a 1Hz timer.
	static int timerCounter = 0;

	// Check if it's time to toggle the status LED.
	if (++timerCounter >= (HIL_IS_ACTIVE()?1:4)) {
		_LATA4 ^= 1;
		timerCounter = 0;
	}

	// Update HIL status every .25s.
	if (HIL_IS_ACTIVE()) {
		nodeStatus |= NODE_STATUS_FLAG_HIL_ACTIVE;
	} else {
		nodeStatus &= ~NODE_STATUS_FLAG_HIL_ACTIVE;
	}
}

void HilNodeTimer100Hz(void)
{
    // Track the messages to be transmit for this timestep.
	// Here we emulate the same transmission frequency of the messages actually transmit
	// by the onboard sensors.
    static uint8_t msgs[ECAN_MSGS_SIZE];

	// We don't do any transmission of CAN messages if we're inactive.
	if (!HIL_IS_ACTIVE()) {
		return;
	}

    uint8_t messagesToSend = GetMessagesForTimestep(&sched, msgs);
    int i;
    CanMessage msg;
    for (i = 0; i < messagesToSend; ++i) {
        switch (msgs[i]) {
            case SCHED_ID_RC_STATUS:
                CanMessagePackageStatus(&msg, CAN_NODE_RC, 0, 0, 0);
                Ecan1Transmit(&msg);
            break;
            case SCHED_ID_RUDDER_ANGLE:
                PackagePgn127245(&msg, nodeId, 0xFF, 0xF, NAN, hilReceivedData.data.rAngle);
                Ecan1Transmit(&msg);
            break;
            case SCHED_ID_RUDDER_LIMITS:
                CanMessagePackageRudderDetails(&msg, 0, 0, 0, false, false, true, true, false);
                Ecan1Transmit(&msg);
            break;
            case SCHED_ID_THROTTLE_STATUS:
                Acs300PackageHeartbeat(&msg, (uint16_t)hilReceivedData.data.tSpeed, 0, 0, 0);
                Ecan1Transmit(&msg);
            break;
            case SCHED_ID_LAT_LON:
                PackagePgn129025(&msg, nodeId, hilReceivedData.data.gpsLatitude, hilReceivedData.data.gpsLongitude);
                Ecan1Transmit(&msg);
            break;
            case SCHED_ID_COG_SOG:
                PackagePgn129026(&msg, nodeId, 0xFF, 0x7, hilReceivedData.data.gpsCog, hilReceivedData.data.gpsSog);
                Ecan1Transmit(&msg);
            break;
            case SCHED_ID_GPS_FIX:
                PackagePgn129539(&msg, nodeId, 0xFF, PGN_129539_MODE_3D, PGN_129539_MODE_3D, 100, 100, 100);
                Ecan1Transmit(&msg);
            break;
        }
    }
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
                // Decode the commanded rudder angle from the PGN127245 messages. Either the actual
				// angle or the commanded angle are decoded as appropriate. This is in order to
				// support actual sensor mode where the real rudder is used in simulation.
				case PGN_RUDDER: {
					float angleCommand, angleActual;
					uint8_t tmp = ParsePgn127245(msg.payload, NULL, NULL, &angleCommand, &angleActual);
					// Record the commanded angle if it was decoded
					if (tmp & 0x4) {
						hilDataToTransmit.data.rCommandAngle = angleCommand;
					}
					// Record the actual angle if it was decoded
					if (tmp & 0x8) {
						hilDataToTransmit.data.rudderAngle = angleActual;
					}
				} break;
				}
			}

			++messagesHandled;
		}
	} while (messagesLeft > 0);

	return messagesHandled;
}
