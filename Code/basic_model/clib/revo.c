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
#include "uart2.h"
#include "types.h"
#include "conversions.h"
#include "nmea0183.h"

struct RevoData revoDataStore;
static char sentence[127];
static unsigned char sentenceIndex;
static unsigned char checksum;
static unsigned char sentenceState;

void processRevoSentence(char sentence[]) {
	if (sentence[4] == 'H' && sentence[5] == 'T' && sentence[6] == 'M') {
		parseHTM(sentence);
	}
}

void processNewRevoData(void) {
	while (GetLength(&uart2RxBuffer) > 0) {
		unsigned char c;
		Read(&uart2RxBuffer, &c);
		buildAndCheckSentence((char)c, sentence, &sentenceIndex, &sentenceState, &checksum, processRevoSentence);
	}
}

void getRevoData(unsigned char* data) {
	data[0] = revoDataStore.heading.chData[0];
	data[1] = revoDataStore.heading.chData[1];
	data[2] = revoDataStore.heading.chData[2];
	data[3] = revoDataStore.heading.chData[3];
	data[4] = revoDataStore.magStatus;
	
	data[5] = revoDataStore.pitch.chData[0];
	data[6] = revoDataStore.pitch.chData[1];
	data[7] = revoDataStore.pitch.chData[2];
	data[8] = revoDataStore.pitch.chData[3];
	data[9] = revoDataStore.pitchStatus;
	
	data[10] = revoDataStore.roll.chData[0];
	data[11] = revoDataStore.roll.chData[1];
	data[12] = revoDataStore.roll.chData[2];
	data[13] = revoDataStore.roll.chData[3];
	data[14] = revoDataStore.rollStatus;
	
	data[15] = revoDataStore.dip.chData[0];
	data[16] = revoDataStore.dip.chData[1];
	data[17] = revoDataStore.dip.chData[2];
	data[18] = revoDataStore.dip.chData[3];
	
	data[19] = revoDataStore.magneticMagnitude.chData[0];
	data[20] = revoDataStore.magneticMagnitude.chData[1];
	
	data[21] = revoDataStore.newData;
	
	// Mark this data as old now
	revoDataStore.newData = 0;
}

void clearRevoData(void) {
	revoDataStore.heading.flData = 0.0;
	revoDataStore.magStatus = 0;
	revoDataStore.pitch.flData = 0.0;
	revoDataStore.pitchStatus = 0;
	revoDataStore.roll.flData = 0.0;
	revoDataStore.rollStatus = 0;
	revoDataStore.dip.flData = 0.0;
	revoDataStore.magneticMagnitude.usData = 0;
	revoDataStore.newData = 0;
}

void parseHTM(char *stream) {
	char token[15]; // Tokens set to 15 characters in length
	
	// Initialize tokenizer with the data stream. This first token is ignored
	// as it just contains "$PTNTHTM".
	myTokenizer(stream, ',', token);
	
	// 1.- True heading (x.x)
	myTokenizer(NULL, ',', token);
	if (strlen(token) > 0) {
		revoDataStore.heading.flData = atof(token);	
	}
	
	// 2.- Magnetometer status (C,L,M,N,O,P,H)
	myTokenizer(NULL, ',', token);
	if (strlen(token) == 1) {
		revoDataStore.magStatus = token[0];
	}
	
	// 3.- Pitch angle (x.x)
	myTokenizer(NULL, ',', token);
	if (strlen(token) > 0) {
		revoDataStore.pitch.flData = atof(token);	
	}
	
	// 4.- Pitch status (N,O,P)
	myTokenizer(NULL, ',', token);
	if (strlen(token) == 1) {
		revoDataStore.pitchStatus = token[0];
	}
	
	// 5.- Roll angle (x.x)
	myTokenizer(NULL, ',', token);	
	if (strlen(token) > 0) {
		revoDataStore.roll.flData = atof(token);	
	}
	
	// 6.- Roll status (N,O,P)
	myTokenizer(NULL, ',', token);	
	if (strlen(token) == 1) {
		revoDataStore.rollStatus = token[0];
	}
	
	// 7.- Dip angle (x.x)
	myTokenizer(NULL, ',', token);	
	if (strlen(token) > 0) {
		revoDataStore.dip.flData = atof(token);	
	}
	
	// 7.- Relative magnitude horizontal component of Earth's magnetic field
	myTokenizer(NULL, ',', token);
	if (strlen(token) > 0) {
		revoDataStore.magneticMagnitude.usData = atoi(token);
	}
	
	// Turn the flag on of new data
	revoDataStore.newData = 1;
}
