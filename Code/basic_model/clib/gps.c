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
// gps.c
// This is code implements a NMEA0138 parse for use in the autoboat
// project. It makes use of the circular buffer data structure cBuffer.c.
// It has been 
// written to be implemented in Simulink through the use of C-Function
// Call blocks. buildAndCheckSentence will append bytes from a data stream
// onto an internally managed buffer. Once a complete sentence has been
// constructed & verified by its checksum, it's passed off to the parser.
// This code is utilized in Simulink by using a UART or datastream block
// and feeding its output into a C-Function block calling buildAndCheckSentence.
// The required C files for compilation are: cBuffer.c, sbrk.c, gps.c.
// 
// Code by: Mariano I. Lizarraga
// First Revision: Aug 21 2008 @ 21:15
// Modified by: Bryant W. Mairs
// First Revision Aug 25 2010
// ==============================================================

#include <string.h>
#include <stdlib.h>

#include "gps.h"
#include "uart2.h"
#include "nmea0183.h"
#include "types.h"
#include "conversions.h"

static tGpsData gpsSensorData;
static char sentence[127];
static unsigned char sentenceIndex;
static unsigned char checksum;
static unsigned char sentenceState;

/**
 * This function initializes the GPS by reconfiguring it to NOT output
 * GSA or GSV messages. This should leave just GGA and RMC messages,
 * which are the only two we care about.
 * It assumes that it is using UART2 and relies on the circular buffer
 * initialized for UART2 for transmission.
 */
void initGps(void) {
	// Configure GPS by:
	// - Disabling GSA
	// - Disabling GSV
	unsigned char disableGSASentence[] = "$PSRF103,2,0,0,1*26\r\n";
	unsigned char disableGSVSentence[] = "$PSRF103,3,0,0,1*27\r\n";

	// Enqueue the size of the sentences - 1 to remove string-terminating null character
	uart2EnqueueData(disableGSASentence, sizeof(disableGSASentence) - 1);
	uart2EnqueueData(disableGSVSentence, sizeof(disableGSVSentence) - 1);
}

void processGpsSentence(char *sentence) {
	if (sentence[3] == 'R' &&
		sentence[4] == 'M' &&
		sentence[5] == 'C') {
		parseRMC(sentence);
	} else if(sentence[3] == 'G' &&
		sentence[4] == 'G' &&
		sentence[5] == 'A') {
		parseGGA(sentence);
	}
}

void processNewGpsData(void) {
	while (GetLength(&uart2RxBuffer) > 0) {
		unsigned char c;
		Read(&uart2RxBuffer, &c);
		buildAndCheckSentence((char)c, sentence, &sentenceIndex, &sentenceState, &checksum, processGpsSentence);
	}
}

void GetGpsData(tGpsData *gpsData) {
	memcpy(gpsData, &gpsSensorData, sizeof(tGpsData));
}

void GetGpsDataMatlab(unsigned char* data) {
	data[0] = gpsSensorData.lat.chData[0];
	data[1] = gpsSensorData.lat.chData[1];
	data[2] = gpsSensorData.lat.chData[2];
	data[3] = gpsSensorData.lat.chData[3];
	data[4] = gpsSensorData.lon.chData[0];
	data[5] = gpsSensorData.lon.chData[1];
	data[6] = gpsSensorData.lon.chData[2];
	data[7] = gpsSensorData.lon.chData[3];
	data[8] = gpsSensorData.alt.chData[0];
	data[9] = gpsSensorData.alt.chData[1];
	data[10] = gpsSensorData.alt.chData[2];
	data[11] = gpsSensorData.alt.chData[3];
	
	// Add date info
	data[12] = gpsSensorData.year;
	data[13] = gpsSensorData.month;
	data[14] = gpsSensorData.day;
	
	// Add time info
	data[15] = gpsSensorData.hour;
	data[16] = gpsSensorData.min;
	data[17] = gpsSensorData.sec;
	
	data[18] = gpsSensorData.cog.chData[0];
	data[19] = gpsSensorData.cog.chData[1];
	data[20] = gpsSensorData.cog.chData[2];
	data[21] = gpsSensorData.cog.chData[3];
	data[22] = gpsSensorData.sog.chData[0];
	data[23] = gpsSensorData.sog.chData[1];
	data[24] = gpsSensorData.sog.chData[2];
	data[25] = gpsSensorData.sog.chData[3];
	data[26] = gpsSensorData.hdop.chData[0];
	data[27] = gpsSensorData.hdop.chData[1];
	data[28] = gpsSensorData.hdop.chData[2];
	data[29] = gpsSensorData.hdop.chData[3];
	
	data[30] = gpsSensorData.fix;
	data[31] = gpsSensorData.sats;
	data[32] = gpsSensorData.newData;
	
	// Mark this data as old now
	gpsSensorData.newData = 0;
}

void SetGpsData(tGpsData *gpsData) {
	memcpy(&gpsSensorData, gpsData, sizeof(tGpsData));
}

void clearGpsData(void) {
	gpsSensorData.year = 0;
	gpsSensorData.month = 0;
	gpsSensorData.day = 0;
	gpsSensorData.hour = 0;
	gpsSensorData.min = 0;
	gpsSensorData.sec = 0;
	gpsSensorData.lat.flData = 0.0;
	gpsSensorData.lon.flData = 0.0;
	gpsSensorData.alt.flData = 0.0;
	gpsSensorData.cog.flData = 0.0;
	gpsSensorData.sog.flData = 0.0;
	gpsSensorData.hdop.flData = 0.0;
	gpsSensorData.fix = 0;
	gpsSensorData.sats = 0;
	gpsSensorData.newData = 0;
}

void parseRMC(char* stream) {
	// declare the local vars
	char token[15]; // Tokens set to 15 characters in length
	char tmp [3] ={0,0,'\0'}, tmp3[4]={0,0,0,'\0'};
	unsigned char chTmp = 0;
	
	
	// initialize tokenizer, let go first token which holds the msg type
	// token = strtok(stream, ",");
	myTokenizer(stream, ',', token);
	
	// 1.- hhmmss.ssss
	myTokenizer(NULL, ',', token);
	if (strlen(token)>5) {
		tmp[0] = token[0]; tmp[1] = token[1];
		gpsSensorData.hour = (unsigned char) atoi(tmp);
		tmp[0] = token[2]; tmp[1] = token[3];
		gpsSensorData.min = (unsigned char) atoi(tmp);
		tmp[0] = token[4]; tmp[1] = token[5];
		gpsSensorData.sec = (unsigned char) atoi(tmp);		
	}
	
	// 2.- Status of position Fix
	myTokenizer(NULL, ',', token);
	if (strlen(token)== 1) {
		if (token[0] == 'A' || token[0] == 'D') {
			gpsSensorData.fix = 1;
		} else {
			gpsSensorData.fix = 0;
		}
	}
	
	// 3.- Latitude
	// ddmm.mmmmmm
	myTokenizer(NULL, ',', token);
	if (strlen(token)>0) {
		// get the first two values
		tmp[0] = token[0]; tmp[1] = token[1];
		// get the degrees
		chTmp = (unsigned char)atoi(tmp);
		// make the degrees zero for minutes conversion
		token[0]='0'; token[1]='0';
		// get the float
		gpsSensorData.lat.flData = degMinToDeg(chTmp,atof(token));
		
		// 4.- Latitude Sector
		myTokenizer(NULL, ',', token);
		if (strlen(token)==1) {
			// set the sign of the float value
			if (token[0] == 'S' || token[0] == 'W') {
				gpsSensorData.lat.flData = -gpsSensorData.lat.flData;
			}
		}
	}
	
	// 5.- Longitude
	// dddmm.mmmmmm
	myTokenizer(NULL, ',', token);
	if (strlen(token)>0) {
		// get the first two values
		tmp3[0] = token[0]; tmp3[1] = token[1]; tmp3[2] = token[2];
		// get the degrees
		chTmp = (unsigned char)atoi(tmp3);
		// make the degrees zero for minutes conversion
		token[0]='0'; token[1]='0'; token [2] = '0';
		// get the float
		gpsSensorData.lon.flData = degMinToDeg(chTmp,atof(token));
		
		// 6.- Longitude Sector
		myTokenizer(NULL, ',', token);
		if (strlen(token) == 1) {
			// set the sign of the float value
			if (token[0] == 'S' || token[0] == 'W') {
				gpsSensorData.lon.flData = -gpsSensorData.lon.flData;
			}
		}
	}
	
	// 7.- Speed over ground in meters (converted from knots)
	// xx.xx
	myTokenizer(NULL, ',', token);	
	if (strlen(token) > 0) {
		gpsSensorData.sog.flData = .5144444*atof(token);
	}
	
	// 8.- Course over ground in degrees
	// xxx.xxx
	myTokenizer(NULL, ',', token);	
	if (strlen(token) > 0) {
		gpsSensorData.cog.flData = atof(token);	
	}
	
	// 9.- UTC Date
	// ddmmyy
	myTokenizer(NULL, ',', token);	
	if (strlen(token) > 5) {
		// get day
		tmp[0]= token[0]; tmp[1]=token[1];
		gpsSensorData.day = (unsigned char) atoi(tmp);	
		// get month
		tmp[0]= token[2]; tmp[1]=token[3];
		gpsSensorData.month = (unsigned char) atoi(tmp);	
		// get year
		tmp[0]= token[4]; tmp[1]=token[5];
		gpsSensorData.year = (unsigned char) atoi(tmp);	
	}
	
	// turn the flag on of new data
	gpsSensorData.newData = 1;
}

void parseGGA(char* stream) {
	// declare the local vars
	char token[15]; // Declare a token to be 15 characters in length
	char tmp [3] ={0,0,'\0'}, tmp3[4]={0,0,0,'\0'};
	unsigned char chTmp = 0;
	
	// initialize tokenizer, let go first token which holds the msg type
	myTokenizer(stream, ',', token);
	
	// 1.- hhmmss.ssss
	myTokenizer(NULL, ',', token);
	// if (strlen(token)>5) {
		// tmp[0] = token[0]; tmp[1] = token[1];
		// gpsSensorData.hour = (unsigned char) atoi(tmp);
		// tmp[0] = token[2]; tmp[1] = token[3];
		// gpsSensorData.min = (unsigned char) atoi(tmp);
		// tmp[0] = token[4]; tmp[1] = token[5];
		// gpsSensorData.sec = (unsigned char) atoi(tmp);		
	// }
	
	// 2.- Latitude
	// ddmm.mmmmmm
	myTokenizer(NULL, ',', token);
	if (strlen(token)>0) {
		// // get the first two values
		// tmp[0] = token[0]; tmp[1] = token[1];
		// // get the degrees
		// chTmp = (unsigned char)atoi(tmp);
		// // make the degrees zero for minutes conversion
		// token[0]='0'; token[1]='0';
		// // get the float
		// gpsSensorData.lat.flData = degMinToDeg(chTmp,atof(token));		
		// // 3.- Latitude Sector
		myTokenizer(NULL, ',', token);
		// if (strlen(token)==1) {
			// // Set the sign of the float value.
			// // South & west are negative, so we invert the sign in
			// // those cases. North/East don't change the value so no
			// // need to check those.
			// if (token[0] == 'S' || token[1] == 'W') {
				// gpsSensorData.lat.flData = -gpsSensorData.lat.flData;
			// }
		// }
	}
	
	// 4.- Longitude
	// ddmm.mmmmmm
	myTokenizer(NULL, ',', token);	
	if (strlen(token)>0) {
		// // get the first two values
		// tmp3[0] = token[0]; tmp3[1] = token[1]; tmp3[2] = token[2];
		// // get the degrees
		// chTmp = (unsigned char)atoi(tmp3);
		// // make the degrees zero for minutes conversion
		// token[0]='0'; token[1]='0'; token [2] = '0';
		// // get the float
		// gpsSensorData.lon.flData = degMinToDeg(chTmp,atof(token));
		
		// // 5.- Longitude Sector
		myTokenizer(NULL, ',', token);
		
		// if (strlen(token)>0) {
			// // set the sign of the float value
			// if (token[0] == 'S' || token[0] == 'W') {
				// gpsSensorData.lon.flData = -gpsSensorData.lon.flData;
			// }
		// }
	}
	
	// 6.- Quality Indicator
	myTokenizer(NULL, ',', token);
	// if (strlen(token) == 1) {
		// gpsSensorData.fix = (char)atoi(token);
	// }

	// 7.- Sats used in solution
	// xx
	myTokenizer(NULL, ',', token);	
	if (strlen(token) > 0) {
		gpsSensorData.sats = (unsigned char) atoi(token);	
	}
	
	// 8.- Horizontal dilution of solution in meters
	// xx.xx
	myTokenizer(NULL, ',', token);	
	if (strlen(token) > 0) {
		gpsSensorData.hdop.flData = atof(token);	
	}
	
	// 9.- Altitude above mean sea level given in meters
	// xxx.xxx
	myTokenizer(NULL, ',', token);	
	if (strlen(token) > 0) {
		gpsSensorData.alt.flData = atof(token);	
	}
	
	// Here we don't enable the newData flag as this GGA message only
	// gives us status information, not position information. If we 
	// set the newData flag the controller thinks there's new position
	// data when this is really just a status update.
	// gpsSensorData.newData = 1;
}
