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

// ==============================================================
// This code provides a protocol decoder for the binary communications
// protocol used between the groundstation/HIL and the dsPIC in
// the Autoboat project. As most programming relies on Simulink and
// the Real-Time Workshop, retrieval functions here return arrays of
// data to be compatible (instead of much-nicer structs).
// A complete structure is passed byte-by-byte and assembled in an
// internal buffer. This is then verified by its checksum and the
//data pushed into the appropriate struct. This data can then be
// retrieved via an accessor function.
//
// While this code was written specifically for the Autoboat and its
// protocol, it has been kept as modular as possible to be useful
// in other situations with the most minimal alterations.
//
// Code by: Bryant W. Mairs
// First Revision: Aug 25 2010
// ==============================================================

#include "Hil.h"
#include "Uart1.h"
#include "Rudder.h"

#include <xc.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

// This is the value of the BRG register for configuring different baud
// rates. These BRG values have been calculated based on a 40MHz system clock.
#define BAUD115200_BRG_REG 21

// Here we declare a struct used to hold the HIL data that's ready for transmission.
// We declare it module-level to prevent reallocation every time and so that it can
// be initialized with some values.
typedef struct {
	uint8_t headerByte1;
	uint8_t headerByte2;
	uint8_t checksum;
	uint8_t messageSize;
	union HilDataToPc data;
	uint8_t footerByte1;
	uint8_t footerByte2;
} HilWrapper;
static HilWrapper wrapper = {
    '%',
    '&',
    0,
    sizeof(union HilDataToPc),
    {},
    '^',
    '&'
};

union HilDataToPc hilDataToTransmit = {};
union HilDataFromPc hilReceivedData = {};

// Keep track of how many messages were successfully received.
static uint32_t receivedMessageCount = 0;
// Keep track of how many fails we've run into
static uint32_t failedMessageCount = 0;
static uint8_t sameFailedMessageFlag = 0;

// Track whether HIL is currently active.
static bool hilStatus = false;

void HilInit(void)
{
	// Initialize UART1 to 115200 for HIL communications.
	Uart1Init(BAUD115200_BRG_REG);
	int i;
	for (i = 0; i < 100; ++i) {
		Uart1WriteByte('h');
	}
}

/**
 * This function builds a full message internally byte-by-byte,
 * verifies its checksum, and then pushes that data into the
 * appropriate struct.
 * sensorMode is a boolean that is true when generated actuator sensor data
 * should be overridden by real-world actuator sensor data.
 */
void HilBuildMessage(uint8_t data)
{
	static uint8_t message[64] = {};
	static uint8_t messageIndex = 0;
	static uint8_t messageState = 0;

	// This contains the function's state of whether
	// it is currently building a message.
	// 0 - Awaiting header byte 0 (%)
	// 1 - Awaiting header byte 1 (&)
	// 2 - Building message
	// 3 - Awaiting header byte 0 (^)
	// 4 - Awaiting header byte 1 (&)
	// 5 - Reading checksum character

	// We start recording a new message if we see the header
	if (messageState == 0) {
		if (data == '%') {
			message[0] = data;
			messageIndex++;
			messageState = 1;
		} else {
			messageIndex = 0;
			messageState = 0;

			// Here we've failed parsing a message so count another failure.
			if (!sameFailedMessageFlag) {
				failedMessageCount++;
				sameFailedMessageFlag = 1;
			}
		}
	} else if (messageState == 1) {
		// If we don't find the necessary ampersand we start over
		// waiting for a new sentence
		if (data == '&') {
			message[messageIndex] = data;
			messageIndex++;
			messageState = 2;

		} else if (data != '%'){
			messageIndex = 0;
			messageState = 0;

			// Here we've failed parsing a message so count another failure.
			if (!sameFailedMessageFlag) {
				failedMessageCount++;
				sameFailedMessageFlag = 1;
			}
		}
	} else if (messageState == 2) {
		// Record every character that comes in now that we're building a sentence.
		// Stop scanning once we've reached the message length of characters.
		message[messageIndex++] = data;
		if (messageIndex > 3 && messageIndex == message[3] + 5) {
			if (data == '^') {
				messageState = 3;
			} else {
				messageState = 0;
				messageIndex = 0;

				// Here we've failed parsing a message.
				failedMessageCount++;
				sameFailedMessageFlag = 1;
			}
		} else if (messageIndex == sizeof(message) - 3) {
			// If we've filled up the buffer, ignore the entire message as we can't store it all
			messageState = 0;
			messageIndex = 0;

			// Here we've failed parsing a message.
			failedMessageCount++;
			sameFailedMessageFlag = 1;
		}
	} else if (messageState == 3) {
            // If we don't find the necessary ampersand we continue
            // recording data as we haven't found the footer yet until
            // we've filled up the entire message (ends at 124 characters
            // as we need room for the 2 footer chars).
            message[messageIndex++] = data;
            if (data == '&') {
                message[messageIndex] = data;

                // The checksum is now verified and if successful the message
                // is stored in the appropriate struct.
                if (message[2] == HilCalculateChecksum(&message[4], sizeof(union HilDataFromPc))) {
                    // Trigger a debugging signal ona pin indicating that a message was succesfully
                    // decoded. This is cleared using the 1ms timer.
                    LATBbits.LATB10 = 1;
                    // We now memcpy all the data into our global data struct.
                    receivedMessageCount++;
                    memcpy(&hilReceivedData, &message[4], sizeof(union HilDataFromPc));

                    // Now that we've successfully parsed a message, clear the flag.
                    sameFailedMessageFlag = 0;
                } else {
                    // Here we've failed parsing a message.
                    failedMessageCount++;
                    sameFailedMessageFlag = 1;
                }

                // We clear all state variables here regardless of success.
                messageIndex = 0;
                messageState = 0;
                int b;
                for (b = 0; b < sizeof(message); b++) {
                    message[b] = 0;
                }
            } else {
                messageIndex = 0;
                messageState = 0;
            }
            LATBbits.LATB10 = 0;
	}
}

/**
 * This function should be called continously. Each timestep
 * it runs through the most recently received data, parsing
 * it for sensor data. Once a complete message has been parsed
 * the data inside will be returned through the message
 * array.
 * This function also sets/clears the `hilStatus` variable which specifies whether HIL is currently
 * active or not. This uses a 20-sample timeout, which equates to 0.2s if this function is called
 * every 1/100th of a second.
 * It's input is a boolean that is true when generated actuator sensor data
 * should be overridden by real-world actuator sensor data.
 */
void HilReceiveData(void)
{
    static int countDown = 0; // Keep a .2s timer going and disable after it expires.
    uint8_t c;
    if (countDown < 20) {
        ++countDown;
    } else {
        hilStatus = false;
    }
    while (Uart1ReadByte(&c)) {
        HilBuildMessage(c);
        countDown = 0;
    }
    if (!hilStatus && countDown == 0) {
        hilStatus = true;
    }
}

bool HilActive(void)
{
    return hilStatus;
}

void HilTransmitData(void)
{
    // First copy the timestamp from the last received HIL data packet to the new outgoing one.
    hilDataToTransmit.data.timestamp = hilReceivedData.data.timestamp;
	
    // Fill the wrapper with the latest data.
    memcpy(&wrapper.data, &hilDataToTransmit, sizeof(union HilDataToPc));

    // Calculate checksum over the messageType, messageSize, and data.
    wrapper.checksum = HilCalculateChecksum((const uint8_t *)&wrapper.data, sizeof(union HilDataToPc));

    // And then enqueue the new data for transmission.
    Uart1WriteData(&wrapper, sizeof(HilWrapper));
}

/**
 * This function calculates the checksum of some bytes in an
 * array by XORing all of them.
 */
uint8_t HilCalculateChecksum(const uint8_t *sentence, uint8_t size)
{

	uint8_t checkSum = 0;
	uint8_t i;
	for (i = 0; i < size; i++) {
		checkSum ^= sentence[i];
	}

	return checkSum;
}