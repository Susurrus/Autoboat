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
// This code implements a driver for the Revolution GS IMU. It relies
// on the Nmea0183 library for parsing.
// ==============================================================

#include <string.h>
#include <stdlib.h>
#include <stdint.h>

#include "RevoGs.h"
#include "Types.h"
#include "Nmea0183.h"

struct RevoGsData revoGsDataStore;

void RevoGsParseSentence(const char *sentence)
{
	if (sentence[4] == 'H' && sentence[5] == 'T' && sentence[6] == 'M') {
		RevoGsParseHtm(sentence);
	}
}

void RevoGsParseHtm(const char *stream)
{
	char token[15]; // Tokens set to 15 characters in length
	
	// Initialize tokenizer with the data stream. This first token is ignored
	// as it just contains "$PTNTHTM".
	myTokenizer(stream, ',', token);
	
	// 1.- True heading (x.x)
    // We convert heading to be between -pi and pi so that it's consistent with the roll and pitch
	myTokenizer(NULL, ',', token);
	if (strlen(token) > 0) {
		float tmp = atof(token);
		if (tmp > 180.0) {
			tmp -= 360;
		}
		revoGsDataStore.heading.flData = tmp * M_PI / 180;
	}
	
	// 2.- Magnetometer status (C,L,M,N,O,P,H)
	myTokenizer(NULL, ',', token);
	if (strlen(token) == 1) {
		revoGsDataStore.magStatus = token[0];
	}
	
	// 3.- Pitch angle (x.x)
	myTokenizer(NULL, ',', token);
	if (strlen(token) > 0) {
		revoGsDataStore.pitch.flData = atof(token) * M_PI / 180;
	}
	
	// 4.- Pitch status (N,O,P)
	myTokenizer(NULL, ',', token);
	if (strlen(token) == 1) {
		revoGsDataStore.pitchStatus = token[0];
	}
	
	// 5.- Roll angle (x.x)
	myTokenizer(NULL, ',', token);	
	if (strlen(token) > 0) {
		revoGsDataStore.roll.flData = atof(token) * M_PI / 180;
	}
	
	// 6.- Roll status (N,O,P)
	myTokenizer(NULL, ',', token);	
	if (strlen(token) == 1) {
		revoGsDataStore.rollStatus = token[0];
	}
	
	// 7.- Dip angle (x.x)
	myTokenizer(NULL, ',', token);	
	if (strlen(token) > 0) {
		revoGsDataStore.dip.flData = atof(token) * M_PI / 180;
	}
	
	// 7.- Relative magnitude horizontal component of Earth's magnetic field
	myTokenizer(NULL, ',', token);
	if (strlen(token) > 0) {
		revoGsDataStore.magneticMagnitude.usData = atoi(token);
	}
}

void RevoGsClearData(void)
{
	revoGsDataStore.heading.flData = 0.0;
	revoGsDataStore.magStatus = 0;
	revoGsDataStore.pitch.flData = 0.0;
	revoGsDataStore.pitchStatus = 0;
	revoGsDataStore.roll.flData = 0.0;
	revoGsDataStore.rollStatus = 0;
	revoGsDataStore.dip.flData = 0.0;
	revoGsDataStore.magneticMagnitude.usData = 0;
}
