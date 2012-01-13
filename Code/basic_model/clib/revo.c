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
// revo.c
// This code implements a driver for the Revolution GS IMU. It is
// based off of gps.c as that device talks NMEA0183 as well.
// ==============================================================

#include <string.h>
#include <stdlib.h>

#include "revo.h"
#include "uart1.h"
#include "types.h"
#include "conversions.h"

static RevoData receivedData;
static char sentence[127];
static unsigned char sentenceIndex;
static unsigned char checksum;
static unsigned char sentenceState;

void processRevoSentence(char sentence[]) {
	if (sentence[5] == 'H' && sentence[6] == 'T' && sentence[7] == 'M') {
		parseHTM(sentence);
	}
}

void processNewRevoData() {
	//while (GetLength(&uart1RxBuffer) > 0) {
	//	unsigned char c;
	//	Read(&uart1RxBuffer, &c);
	//	buildAndCheckSentence((char)c, sentence, &sentenceIndex, &sentenceState, &checksum, processRevoSentence);
	//}
}

void getRevoData(unsigned char* data) {
	data[0] = receivedData.heading.chData[0];
	data[1] = receivedData.heading.chData[1];
	data[2] = receivedData.heading.chData[2];
	data[3] = receivedData.heading.chData[3];
	data[4] = receivedData.magStatus;
	
	data[5] = receivedData.pitch.chData[0];
	data[6] = receivedData.pitch.chData[1];
	data[7] = receivedData.pitch.chData[2];
	data[8] = receivedData.pitch.chData[3];
	data[9] = receivedData.pitchStatus;
	
	data[10] = receivedData.roll.chData[0];
	data[11] = receivedData.roll.chData[1];
	data[12] = receivedData.roll.chData[2];
	data[13] = receivedData.roll.chData[3];
	data[14] = receivedData.rollStatus;
	
	data[15] = receivedData.dip.chData[0];
	data[16] = receivedData.dip.chData[1];
	data[17] = receivedData.dip.chData[2];
	data[18] = receivedData.dip.chData[3];
	
	data[19] = receivedData.magneticMagnitude.chData[0];
	data[20] = receivedData.magneticMagnitude.chData[1];
	
	data[21] = receivedData.newData;
	
	// Mark this data as old now
	receivedData.newData = 0;
}

void clearRevoData() {
	receivedData.heading.flData = 0.0;
	receivedData.magStatus = 0;
	receivedData.pitch.flData = 0.0;
	receivedData.pitchStatus = 0;
	receivedData.roll.flData = 0.0;
	receivedData.rollStatus = 0;
	receivedData.dip.flData = 0.0;
	receivedData.magneticMagnitude.usData = 0;
	receivedData.newData = 0;
}

void parseHTM(char* stream) {
	char token[15]; // Tokens set to 15 characters in length
	
	
	// Initialize tokenizer with the data stream. This first token is ignored
	// as it just contains "$PTNTHTM".
	myTokenizer(stream, ',', token);
	
	// 1.- True heading (x.x)
	myTokenizer(NULL, ',', token);
	if (strlen(token) > 0) {
		receivedData.heading.flData = atof(token);	
	}
	
	// 2.- Magnetometer status (C,L,M,N,O,P,H)
	myTokenizer(NULL, ',', token);
	if (strlen(token) == 1) {
		receivedData.magStatus = token[0];
	}
	
	// 3.- Pitch angle (x.x)
	myTokenizer(NULL, ',', token);
	if (strlen(token) > 0) {
		receivedData.pitch.flData = atof(token);	
	}
	
	// 4.- Pitch status (N,O,P)
	myTokenizer(NULL, ',', token);
	if (strlen(token) == 1) {
		receivedData.pitchStatus = token[0];
	}
	
	// 5.- Roll angle (x.x)
	myTokenizer(NULL, ',', token);	
	if (strlen(token) > 0) {
		receivedData.roll.flData = atof(token);	
	}
	
	// 6.- Roll status (N,O,P)
	myTokenizer(NULL, ',', token);	
	if (strlen(token) == 1) {
		receivedData.rollStatus = token[0];
	}
	
	// 7.- Dip angle (x.x)
	myTokenizer(NULL, ',', token);	
	if (strlen(token) > 0) {
		receivedData.dip.flData = atof(token);	
	}
	
	// 7.- Relative magnitude horizontal component of Earth's magnetic field
	myTokenizer(NULL, ',', token);
	if (strlen(token) > 0) {
		receivedData.magneticMagnitude.usData = atoi(token);
	}
	
	// Turn the flag on of new data
	receivedData.newData = 1;
}
