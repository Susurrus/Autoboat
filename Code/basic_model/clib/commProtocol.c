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

#include "commProtocol.h"
#include "uart1.h"
#include "uart2.h"
#include "Rudder.h"
#include "types.h"
#include "ecanSensors.h"

#include <stdint.h>
#include <string.h>

// Declaration of the relevant message structs used.
static struct {
	uint8_t newData;
	tUnsignedShortToChar timestamp;
} tHilData;

// This is the value of the BRG register for configuring different baud
// rates. These BRG values have been calculated based on a 40MHz system clock.
#define BAUD57600_BRG_REG 42
#define BAUD115200_BRG_REG 21

// Keep track of how many messages were successfully received.
static uint32_t receivedMessageCount = 0;
// Keep track of how many fails we've run into
static uint32_t failedMessageCount = 0;
static uint8_t sameFailedMessageFlag = 0;

void cpInitCommunications(void) {
        // Initialize UART2 to 57600 for the Revolution GS.
        // It is also used for HIL data transmission.
	initUart2(BAUD57600_BRG_REG);

        // Initialize UART1 to 115200 for groundstation communications.
	initUart1(BAUD115200_BRG_REG);
}

/**
 * This function builds a full message internally byte-by-byte,
 * verifies its checksum, and then pushes that data into the
 * appropriate struct.
 * sensorMode is a boolean that is true when generated actuator sensor data
 * should be overridden by real-world actuator sensor data.
 */
void buildAndCheckMessage(uint8_t characterIn, uint8_t sensorMode) {
	static uint8_t message[64];
	static uint8_t messageIndex;
	static uint8_t messageState;

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
		if (characterIn == '%') {
			message[messageIndex] = characterIn;
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
		if (characterIn == '&') {
			message[messageIndex] = characterIn;
			messageIndex++;
			messageState = 2;

		} else if (characterIn != '%'){
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
		message[messageIndex++] = characterIn;
		if (messageIndex > 3 && messageIndex == message[3] + 5) {
			if (characterIn == '^') {
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
		message[messageIndex++] = characterIn;
		if (characterIn == '&') {
			messageState = 4;
		} else if (messageIndex == sizeof(message) - 2) {
			messageState = 0;
			messageIndex = 0;
		}
	} else if (messageState == 4) {
		// Record the checksum byte.
		message[messageIndex] = characterIn;

		// The checksum is now verified and if successful the message
		// is stored in the appropriate struct.
		if (message[messageIndex] == calculateChecksum(&message[2], messageIndex - 4)) {
			// We now memcpy all the data into our global data structs.
			receivedMessageCount++;
			if (message[2] == 1) {
				// NOTE: We skip data 4 & 5 as it's unnecessary throttle data.
				UpdateGpsDataFromHil(&message[6]);

				// Only update the rudder data if we're not in a sensor-override mode. This mode
				// specifies that real-world actuator sensor data will be used instead of generated.
				if (!sensorMode) {
					SetRudderAngle(&message[23]);
				}
				SetHilData(&message[31]);
			}

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
	}
}

/**
 * This function should be called continously. Each timestep
 * it runs through the most recently received data, parsing
 * it for sensor data. Once a complete message has been parsed
 * the data inside will be returned through the message
 * array.
 * It's input is a boolean that is true when generated actuator sensor data
 * should be overridden by real-world actuator sensor data.
 */
void processNewCommData(uint8_t sensorMode)
{
	while (uart2RxBuffer.dataSize > 0) {
		uint8_t c;
		CB_ReadByte(&uart2RxBuffer, &c);
		buildAndCheckMessage(c, sensorMode);
	}
}

/**
 * This function sets the proper UART2
 * baud rate depending on the HIL mode.
 * A mode value of 0 will set the baud
 * rate back to the default 57600. A value
 * of 1 will set it to 115200.
 */
void setHilMode(uint8_t mode) {
	static uint8_t oldMode = 0;

	// Detect a change to HIL
	if (!oldMode && mode) {
		changeUart2BaudRate(BAUD115200_BRG_REG);
		oldMode = mode;
	} else if (oldMode && !mode) {
		changeUart2BaudRate(BAUD57600_BRG_REG);
		oldMode = mode;
	}
}

inline void enableHil(void) {
	setHilMode(1);
}

inline void disableHil(void) {
	setHilMode(0);
}

/**
 * This function calculates the checksum of some bytes in an
 * array by XORing all of them.
 */
uint8_t calculateChecksum(uint8_t *sentence, uint8_t size) {

	uint8_t checkSum = 0;
	uint8_t i;
	for (i = 0; i < size; i++) {
		checkSum ^= sentence[i];
	}

	return checkSum;
}

void UpdateGpsDataFromHil(uint8_t *data) {
	if (data[16]) {
		gpsDataStore.lat.chData[0] = data[0];
		gpsDataStore.lat.chData[1] = data[1];
		gpsDataStore.lat.chData[2] = data[2];
		gpsDataStore.lat.chData[3] = data[3];

		gpsDataStore.lon.chData[0] = data[4];
		gpsDataStore.lon.chData[1] = data[5];
		gpsDataStore.lon.chData[2] = data[6];
		gpsDataStore.lon.chData[3] = data[7];

		gpsDataStore.alt.chData[0] = data[8];
		gpsDataStore.alt.chData[1] = data[9];
		gpsDataStore.alt.chData[2] = data[10];
		gpsDataStore.alt.chData[3] = data[11];

		gpsDataStore.cog.chData[0] = data[12];
		gpsDataStore.cog.chData[1] = data[13];
		gpsDataStore.sog.chData[0] = data[14];
		gpsDataStore.sog.chData[1] = data[15];

		gpsDataStore.newData = 1;

        // Finally reset the timeout counter for the GPS if we're receiving HIL data.
		sensorAvailability.gps.enabled_counter = 0;
		sensorAvailability.gps.active_counter = 0;
		UpdateSensorsAvailability();
	}
}

void SetHilData(uint8_t *data)
{
	tHilData.timestamp.chData[0] = data[0];
	tHilData.timestamp.chData[1] = data[1];
	tHilData.newData = 1;
}

uint16_t GetCurrentTimestamp(void)
{
	return tHilData.timestamp.usData;
}

uint8_t IsNewHilData(void)
{
	return tHilData.newData;
}

/**
 * Add all 27 data + 7 header/footer bytes of the actuator struct to UART2's transmission queue.
 */
inline void uart2EnqueueActuatorData(uint8_t data[32])
{
	uart2EnqueueData(data, 32);
}
