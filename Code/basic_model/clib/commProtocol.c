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
#include <string.h>
#include "uart1.h"
#include "uart2.h"

// This is the value of the BRG to set the baud rate
// to 115200 for running HIL.
#define BAUD115200_BRG_REG 21

// This is the value of the BRG
#define BAUD4800_BRG_REG 520

// These are local declarations of each of the message structs.
// They're populated with relevant data by buildAndcheckMessage().
tSensorData sensorDataMessage;
tActuatorData actuatorDataMessage;

unsigned long receivedMessageCount; // Keep track of how many messages were successfully received.

void cpInitCommunications() {
	initUart1(BAUD115200_BRG_REG);
	initUart2(BAUD4800_BRG_REG);
}

/**
 * This function builds a full message internally byte-by-byte,
 * verifies its checksum, and then pushes that data into the
 * appropriate struct.
 */
void buildAndCheckMessage(unsigned char characterIn) {
	static unsigned char message[64];
	static unsigned char messageIndex;
	static unsigned char messageState;

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
		}
	} else if (messageState == 1) {
		// If we don't find the necessary ampersand we start over
		// waiting for a new sentence
		if (characterIn == '&') {
			message[messageIndex] = characterIn;
			messageIndex++;
			messageState = 2;
		} else {
			messageIndex = 0;
			messageState = 0;
		}
	} else if (messageState == 2) {
		// Record every character that comes in now that we're building a sentence.
		// Only stop if we run out of room or an asterisk is found.
		message[messageIndex] = characterIn;
		messageIndex++;
		if (characterIn == '^') {
			messageState = 3;
		} else if (messageIndex == 62) {
			// If we've filled up the buffer, ignore the entire message as we can't store it all
			messageState = 0;
			messageIndex = 0;
		}
	} else if (messageState == 3) {
		// If we don't find the necessary ampersand we continue
		// recording data as we haven't found the footer yet until
		// we've filled up the entire message (ends at 124 characters
		// as we need room for the 2 footer chars).
		message[messageIndex] = characterIn;
		messageIndex++;
		if (characterIn == '&') {
			messageState = 4;
		} else if (messageIndex == 63) {
			messageState = 0;
			messageIndex = 0;
		}
	} else if (messageState == 4) {
		// Record the second ASCII-hex character of the checksum byte.
		message[messageIndex] = characterIn;

		// The checksum is now verified and if successful the message
		// is stored in the appropriate struct.
		if (message[messageIndex] == calculateChecksum(&message[2], messageIndex - 4)) {
			// We now memcpy all the data into our global data structs.
			// NOTE: message[3] is used to skip the header & message ID info
			receivedMessageCount++;
			if (message[2] == 1) {
				setSensorData(&message[3]);
			}
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
 */
void processNewCommData(unsigned char* message) {
	while (getLength(&uart2RxBuffer) > 0) {
		buildAndCheckMessage(readFront(&uart2RxBuffer));
	}
}

/**
 * This function sets the proper UART2
 * baud rate depending on the HIL mode.
 * A mode value of 0 will set the baud
 * rate back to the default 1200. A value
 * of 1 will set it to 115200.
 */
void setHilMode(unsigned char mode) {
	static unsigned char oldMode = 0;

	// Detect a change to HIL
	if (!oldMode && mode) {
		changeUart2BaudRate(BAUD115200_BRG_REG);
		oldMode = mode;
	} else if (oldMode && !mode) {
		changeUart2BaudRate(BAUD4800_BRG_REG);
		oldMode = mode;
	}
}

inline void enableHil() {
	setHilMode(1);
}

inline void disableHil() {
	setHilMode(0);
}

/**
 * This function calculates the checksum of some bytes in an
 * array by XORing all of them.
 */
unsigned char calculateChecksum(unsigned char* sentence, unsigned char size) {

	unsigned char checkSum = 0;
	unsigned char i;
	for (i = 0; i < size; i++) {
		checkSum ^= sentence[i];
	}

	return checkSum;
}

void setSensorData(unsigned char* data) {
	sensorDataMessage.speed.chData[0] = data[0];
	sensorDataMessage.speed.chData[1] = data[1];
	sensorDataMessage.lat.chData[0] = data[2];
	sensorDataMessage.lat.chData[1] = data[3];
	sensorDataMessage.lat.chData[2] = data[4];
	sensorDataMessage.lat.chData[3] = data[5];
	sensorDataMessage.lon.chData[0] = data[6];
	sensorDataMessage.lon.chData[1] = data[7];
	sensorDataMessage.lon.chData[2] = data[8];
	sensorDataMessage.lon.chData[3] = data[9];
	sensorDataMessage.alt.chData[0] = data[10];
	sensorDataMessage.alt.chData[1] = data[11];
	sensorDataMessage.alt.chData[2] = data[12];
	sensorDataMessage.alt.chData[3] = data[13];
	sensorDataMessage.year = data[14];
	sensorDataMessage.month = data[15];
	sensorDataMessage.day = data[16];
	sensorDataMessage.hour = data[17];
	sensorDataMessage.minute = data[18];
	sensorDataMessage.second = data[19];
	sensorDataMessage.cog.chData[0] = data[20];
	sensorDataMessage.cog.chData[1] = data[21];
	sensorDataMessage.cog.chData[2] = data[22];
	sensorDataMessage.cog.chData[3] = data[23];
	sensorDataMessage.sog.chData[0] = data[24];
	sensorDataMessage.sog.chData[1] = data[25];
	sensorDataMessage.sog.chData[2] = data[26];
	sensorDataMessage.sog.chData[3] = data[27];
	sensorDataMessage.newGpsData = data[28];
	sensorDataMessage.r_Position.chData[0] = data[29];
	sensorDataMessage.r_Position.chData[1] = data[30];
	sensorDataMessage.r_SBLimit = data[31];
	sensorDataMessage.r_PortLimit = data[32];
	sensorDataMessage.b_Position.chData[0] = data[33];
	sensorDataMessage.b_Position.chData[1] = data[34];
	sensorDataMessage.b_SBLimit = data[35];
	sensorDataMessage.b_PortLimit = data[36];
	sensorDataMessage.timestamp.chData[0] = data[37];
	sensorDataMessage.timestamp.chData[1] = data[38];
	sensorDataMessage.newData = 1;
}

void getSensorData(unsigned char* data) {
	data[0] = sensorDataMessage.speed.chData[0];
	data[1] = sensorDataMessage.speed.chData[1];
	data[2] = sensorDataMessage.lat.chData[0];
	data[3] = sensorDataMessage.lat.chData[1];
	data[4] = sensorDataMessage.lat.chData[2];
	data[5] = sensorDataMessage.lat.chData[3];
	data[6] = sensorDataMessage.lon.chData[0];
	data[7] = sensorDataMessage.lon.chData[1];
	data[8] = sensorDataMessage.lon.chData[2];
	data[9] = sensorDataMessage.lon.chData[3];
	data[10] = sensorDataMessage.alt.chData[0];
	data[11] = sensorDataMessage.alt.chData[1];
	data[12] = sensorDataMessage.alt.chData[2];
	data[13] = sensorDataMessage.alt.chData[3];
	data[14] = sensorDataMessage.year;
	data[15] = sensorDataMessage.month;
	data[16] = sensorDataMessage.day;
	data[17] = sensorDataMessage.hour;
	data[18] = sensorDataMessage.minute;
	data[19] = sensorDataMessage.second;
	data[20] = sensorDataMessage.cog.chData[0];
	data[21] = sensorDataMessage.cog.chData[1];
	data[22] = sensorDataMessage.cog.chData[2];
	data[23] = sensorDataMessage.cog.chData[3];
	data[24] = sensorDataMessage.sog.chData[0];
	data[25] = sensorDataMessage.sog.chData[1];
	data[26] = sensorDataMessage.sog.chData[2];
	data[27] = sensorDataMessage.sog.chData[3];
	data[28] = sensorDataMessage.newGpsData;
	data[29] = sensorDataMessage.r_Position.chData[0];
	data[30] = sensorDataMessage.r_Position.chData[1];
	data[31] = sensorDataMessage.r_SBLimit;
	data[32] = sensorDataMessage.r_PortLimit;
	data[33] = sensorDataMessage.b_Position.chData[0];
	data[34] = sensorDataMessage.b_Position.chData[1];
	data[35] = sensorDataMessage.b_SBLimit;
	data[36] = sensorDataMessage.b_PortLimit;
	data[37] = sensorDataMessage.timestamp.chData[0];
	data[38] = sensorDataMessage.timestamp.chData[1];
	data[39] = sensorDataMessage.newData;
	sensorDataMessage.newData = 0;
}

void clearSensorData() {
	sensorDataMessage.speed.shData = 0;
	sensorDataMessage.lat.flData = 0.0;
	sensorDataMessage.lon.flData = 0.0;
	sensorDataMessage.alt.flData = 0.0;
	sensorDataMessage.year = 0;
	sensorDataMessage.month = 0;
	sensorDataMessage.day = 0;
	sensorDataMessage.hour = 0;
	sensorDataMessage.minute = 0;
	sensorDataMessage.second = 0;
	sensorDataMessage.cog.flData = 0.0;
	sensorDataMessage.sog.flData = 0.0;
	sensorDataMessage.newGpsData = 0;
	sensorDataMessage.r_Position.usData = 0;
	sensorDataMessage.r_SBLimit = 0;
	sensorDataMessage.r_PortLimit = 0;
	sensorDataMessage.b_Position.usData = 0;
	sensorDataMessage.b_SBLimit = 0;
	sensorDataMessage.b_PortLimit = 0;
	sensorDataMessage.timestamp.shData = 0;
	sensorDataMessage.newData = 0;
}

void setActuatorData(unsigned char* data) {
	actuatorDataMessage.r_enable = data[0];
	actuatorDataMessage.r_direction = data[1];
	actuatorDataMessage.r_up.chData[0] = data[2];
	actuatorDataMessage.r_up.chData[1] = data[3];
	actuatorDataMessage.r_period.chData[0] = data[4];
	actuatorDataMessage.r_period.chData[1] = data[5];
	actuatorDataMessage.b_enable = data[6];
	actuatorDataMessage.b_direction = data[7];
	actuatorDataMessage.t_identifier.chData[0] = data[8];
	actuatorDataMessage.t_identifier.chData[1] = data[9];
	actuatorDataMessage.t_identifier.chData[2] = data[10];
	actuatorDataMessage.t_identifier.chData[3] = data[11];
	actuatorDataMessage.data[0] = data[12];
	actuatorDataMessage.data[1] = data[13];
	actuatorDataMessage.data[2] = data[14];
	actuatorDataMessage.data[3] = data[15];
	actuatorDataMessage.data[4] = data[16];
	actuatorDataMessage.data[5] = data[17];
	actuatorDataMessage.size = data[18];
	actuatorDataMessage.trigger = data[19];
	actuatorDataMessage.timestamp.chData[0] = data[20];
	actuatorDataMessage.timestamp.chData[1] = data[21];
}

void getActuatorData(unsigned char* data) {
	data[0] = actuatorDataMessage.r_enable;
	data[1] = actuatorDataMessage.r_direction;
	data[2] = actuatorDataMessage.r_up.chData[0];
	data[3] = actuatorDataMessage.r_up.chData[1];
	data[4] = actuatorDataMessage.r_period.chData[0];
	data[5] = actuatorDataMessage.r_period.chData[1];
	data[6] = actuatorDataMessage.b_enable;
	data[7] = actuatorDataMessage.b_direction;
	data[8] = actuatorDataMessage.t_identifier.chData[0];
	data[9] = actuatorDataMessage.t_identifier.chData[1];
	data[10] = actuatorDataMessage.t_identifier.chData[2];
	data[11] = actuatorDataMessage.t_identifier.chData[3];
	data[12] = actuatorDataMessage.data[0];
	data[13] = actuatorDataMessage.data[1];
	data[14] = actuatorDataMessage.data[2];
	data[15] = actuatorDataMessage.data[3];
	data[16] = actuatorDataMessage.data[4];
	data[17] = actuatorDataMessage.data[5];
	data[18] = actuatorDataMessage.size;
	data[19] = actuatorDataMessage.trigger;
	actuatorDataMessage.timestamp.chData[0] = data[20];
	actuatorDataMessage.timestamp.chData[1] = data[21];
}

/**
 * Add all 22 data + 7 header/footer bytes of the actuator struct to UART2's transmission queue.
 */
inline void uart2EnqueueActuatorData(unsigned char *data) {
	uart2EnqueueData(data, 29);
}

/**
 * Add all 90 data + 7 header/footer bytes of the actuator struct to UART2's transmission queue.
 */
inline void uart1EnqueueStateData(unsigned char *data) {
	uart1EnqueueData(data, 97);
}
