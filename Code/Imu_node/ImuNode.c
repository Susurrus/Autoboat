/*
The MIT License

Copyright (c) 2010 UCSC Autonomous Systems Lab

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.

*/

#include "Node.h"
#include "Nmea0183.h"
#include "RevoGs.h"
#include "Uart1.h"
#include "MessageScheduler.h"
#include "Types.h"
#include "ecanFunctions.h"
#include "CanMessages.h"

typedef union {
    float flData;
    struct {
        unsigned long fraction: 23;
        unsigned exponent: 8;
        unsigned sign: 1;
    } data;
} ieee754_32;

// Calculate the BRG register value necessary for 57600 baud with a 80MHz clock.
#define BAUD57600_BRG_REG 42

// Declare some constants for use with the message scheduler
// (don't use PGN or message ID as it must be a uint8)
#define SCHED_ID_IMU_DATA 1

// Set up the message scheduler's various data structures.
#define ECAN_MSGS_SIZE 1
static uint8_t ids[ECAN_MSGS_SIZE] = {
    SCHED_ID_IMU_DATA
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

int16_t _convertTo3fp13(float x);
void ImuNodeEnableCanMessages(void);

void ImuNodeInit(void)
{
	nodeId = CAN_NODE_IMU_SENSOR;

    Uart1Init(BAUD57600_BRG_REG);

    // Set the IMU_DATA can message to transmit at 25Hz.
    ImuNodeEnableCanMessages();
}

void ImuNodeEnableCanMessages(void)
{
	// Transmit the rudder angle at 10Hz
	if (!AddMessageRepeating(&sched, SCHED_ID_IMU_DATA, 25)) {
		while (1);
	}
}

void ImuNodeDisableCanMessages(void)
{
    ClearSchedule(&sched);
}

/**
 * This function reads in new data from UART2 and feeds it into the NMEA0183 parser which then
 * calls the Revo GS library for parsing the data out.
 */
void ImuNodeProcessRevoData(void)
{
	// The following variables are all necessary for the NMEA0183 parsing of the RevoGS messages.
	static char sentence[127];
	static uint8_t sentenceIndex;
	static uint8_t checksum;
	static uint8_t sentenceState;

    uint8_t c;
	while (Uart1ReadByte(&c)) {
		buildAndCheckSentence((char)c, sentence, &sentenceIndex, &sentenceState, &checksum, RevoGsParseSentence);
	}
}

void ImuNodeTransmitCanMessages(void)
{
    // Track the messages to be transmit for this timestep.
    static uint8_t msgs[ECAN_MSGS_SIZE];

    uint8_t messagesToSend = GetMessagesForTimestep(&sched, msgs);
    int i;
    tCanMessage msg;
    for (i = 0; i < messagesToSend; ++i) {
        switch (msgs[i]) {
            case SCHED_ID_IMU_DATA: {
                CanMessagePackageImuData(&msg,
                                         revoGsDataStore.heading.flData,
                                         revoGsDataStore.pitch.flData,
                                         revoGsDataStore.roll.flData);
                ecan1_buffered_transmit(&msg);
            } break;
        }
    }
}