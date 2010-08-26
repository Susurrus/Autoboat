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

#include "gps.h"

tGpsData gpsControlData;
char sentence[127];
unsigned char sentenceIndex;
unsigned char checksum;
unsigned char sentenceState;

/**
 * This function converts one hex ASCII character to numeric
 * hex. It's used for the checksum comparison.
 */
char hex2char(char halfhex) {
	if ((halfhex - 48) < 9) {
		return (halfhex - 48);
	}
	return (halfhex - 55);
}

float degMinToDeg(unsigned char degrees, float minutes) {
	return ((float)degrees + minutes/60.0);
}

void buildAndCheckSentence(unsigned char characterIn) {
	// Full specification for NMEA0138 specifies a maximum sentence length
	// of 255 characters. We're going to ignore this for half the length as
	// we shouldn't get anything that big.

	// This contains the function's state of whether
	// it is currently building a sentence.
	// 0 - Awaiting start character ($)
	// 1 - Building sentence
	// 2 - Building first checksum character
	// 3 - Building second checksum character
	
	// We start recording a new sentence if we see a dollarsign.
	// The sentenceIndex is hard-set to 1 so that multiple dollar-signs
	// keep you at the beginning.
	if (characterIn == '$') {
		sentence[0] = characterIn;
		sentenceIndex = 1;
		sentenceState = 1;
	} else if (sentenceState == 1) {
		// Record every character that comes in now that we're building a sentence.
		// Only stop if we run out of room or an asterisk is found.
		sentence[sentenceIndex++] = characterIn;
		if (characterIn == '*') {
			sentenceState = 2;
		} else if (sentenceIndex > 127) {
			// If we've filled up the buffer, ignore the entire message as we can't store it all
			sentenceState = 0;
			sentenceIndex = 0;
		}
	} else if (sentenceState == 2) {
		// Record the first ASCII-hex character of the checksum byte.
		checksum = hex2char(characterIn) << 4;
		sentenceState = 3;
	} else if (sentenceState == 3) {
		// Record the second ASCII-hex character of the checksum byte.
		checksum |= hex2char(characterIn);

		// Now that we've compiled a complete GPS sentence, let's check the checksum and parse it.
		// This code currently only supports RMC and GGA messages.
		if (checksum == getChecksum(sentence, sentenceIndex)) {
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
		
		// We clear all state variables here regardless of success.
		sentenceIndex = 0;
		sentenceState = 0;
	}
}

// GPS checksum code based on 
// http://www.codeproject.com/KB/mobile/WritingGPSApplications2.aspx
// original code in C# written by Jon Person, author of "GPS.NET" (www.gpsdotnet.com)
unsigned char getChecksum(char* sentence, unsigned char size) {

    // Loop through all chars to get a checksum
    unsigned char checkSum = 0;
	unsigned char i;
	for (i = 0; i < size; i++) {
		if (sentence[i] == '$') {
			// Ignore the dollar sign
			continue;
		} else if (sentence[i] == '*') {
			// Stop processing before the asterisk
			break;
		} else {
			checkSum ^= sentence[i];
		}
    }
    // Return the checksum 
    return checkSum;
}

void getGpsMainData(float* data) {
	data[0] = gpsControlData.lat.flData;
	data[1] = gpsControlData.lon.flData;
	data[2] = gpsControlData.height.flData;
	
	// Add date info
	tFloatToChar tmp;
	tmp.chData[0] = gpsControlData.day;
	tmp.chData[1] = gpsControlData.month;
	tmp.chData[2] = gpsControlData.year;
	tmp.chData[3] = 0;
	data[3] = tmp.flData;
	
	// Add time info
	tmp.chData[0] = gpsControlData.sec;
	tmp.chData[1] = gpsControlData.min;
	tmp.chData[2] = gpsControlData.hour;
	tmp.chData[3] = 0;
	data[4] = tmp.flData;
	
	data[5] = (float)gpsControlData.cog.usData;
	data[6] = (float)gpsControlData.sog.usData/100.0;
	
	// Mark this data as old now
	gpsControlData.newData = 0;
}

// a return value of 1 means the string is done. No more tokens
// This function is stateful, call it once with the String and then with NULL
// similar to strtok but this will support succesive tokens like
// "find,,,the,,commas"
unsigned char myTokenizer(char* stringToTokenize, char token, char * returnToken) {
	static char * pch;
	static char * prevPch;
	static char * lastByte;
	
	// Make sure the return token is "empty"
	// Tokens set to max-length of 15 bytes
	memset(returnToken, 0, 15);
	
	// get the pointer to next token if it exists 
	// and the stringToTokenize is null
	// Bahavior similar to strtok
	if (stringToTokenize == NULL) {
		pch = strchr(prevPch, token);
	} else {
		pch = strchr(stringToTokenize, token);
		prevPch = stringToTokenize;
		lastByte = stringToTokenize + strlen(stringToTokenize);
	} 
	
	if (pch != NULL) {
		memcpy(returnToken, prevPch, pch-prevPch );
		prevPch = pch+1;
	} else {
		memcpy(returnToken, prevPch, lastByte-prevPch );
	}

	return pch == NULL;
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
		gpsControlData.hour = (unsigned char) atoi(tmp);
		tmp[0] = token[2]; tmp[1] = token[3];
		gpsControlData.min = (unsigned char) atoi(tmp);
		tmp[0] = token[4]; tmp[1] = token[5];
		gpsControlData.sec = (unsigned char) atoi(tmp);		
	}
	
	// 2.- Status of position Fix
	myTokenizer(NULL, ',', token);
	if (strlen(token)== 1) {
		if (token[0] == 'A' || token[0] == 'D') {
			gpsControlData.fix = 1;
		} else {
			gpsControlData.fix = 0;
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
		gpsControlData.lat.flData = degMinToDeg(chTmp,atof(token));
		
		// 4.- Latitude Sector
		myTokenizer(NULL, ',', token);
		if (strlen(token)==1) {
			// set the sign of the float value
			if (token[0] == 'S' || token[0] == 'W') {
				gpsControlData.lat.flData = -gpsControlData.lat.flData;
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
		gpsControlData.lon.flData = degMinToDeg(chTmp,atof(token));
		
		// 6.- Longitude Sector
		myTokenizer(NULL, ',', token);
		if (strlen(token) == 1) {
			// set the sign of the float value
			if (token[0] == 'S' || token[0] == 'W') {
				gpsControlData.lon.flData = -gpsControlData.lon.flData;
			}
		}
	}
	
	// 7.- SOG in knots but gets stored in cm/s CAUTION
	// xx.xx
	myTokenizer(NULL, ',', token);	
	if (strlen(token) > 0) {
		gpsControlData.sog.usData = (unsigned short) (atof(token)*KTS2MPS*100.0);	
	}
	
	// 8.- COG in degrees
	// xxx.xxx
	myTokenizer(NULL, ',', token);	
	if (strlen(token) > 0) {
		gpsControlData.cog.usData = (unsigned short) atof(token);	
	}
	
	// 9.- UTC Date
	// ddmmyy
	myTokenizer(NULL, ',', token);	
	if (strlen(token) > 5) {
		// get day
		tmp[0]= token[0]; tmp[1]=token[1];
		gpsControlData.day = (unsigned char) atoi(tmp);	
		// get month
		tmp[0]= token[2]; tmp[1]=token[3];
		gpsControlData.month = (unsigned char) atoi(tmp);	
		// get year
		tmp[0]= token[4]; tmp[1]=token[5];
		gpsControlData.year = (unsigned char) atoi(tmp);	
	}
	
	// turn the flag on of new data
	gpsControlData.newData = 1;
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
	if (strlen(token)>5) {
		tmp[0] = token[0]; tmp[1] = token[1];
		gpsControlData.hour = (unsigned char) atoi(tmp);
		tmp[0] = token[2]; tmp[1] = token[3];
		gpsControlData.min = (unsigned char) atoi(tmp);
		tmp[0] = token[4]; tmp[1] = token[5];
		gpsControlData.sec = (unsigned char) atoi(tmp);		
	}
	
	// 2.- Latitude
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
		gpsControlData.lat.flData = degMinToDeg(chTmp,atof(token));		
		// 3.- Latitude Sector
		myTokenizer(NULL, ',', token);
		if (strlen(token)==1) {
			// Set the sign of the float value.
			// South & west are negative, so we invert the sign in
			// those cases. North/East don't change the value so no
			// need to check those.
			if (token[0] == 'S' || token[1] == 'W') {
				gpsControlData.lat.flData = -gpsControlData.lat.flData;
			}
		}
	}
	
	// 4.- Longitude
	// ddmm.mmmmmm
	myTokenizer(NULL, ',', token);	
	if (strlen(token)>0) {
		// get the first two values
		tmp3[0] = token[0]; tmp3[1] = token[1]; tmp3[2] = token[2];
		// get the degrees
		chTmp = (unsigned char)atoi(tmp3);
		// make the degrees zero for minutes conversion
		token[0]='0'; token[1]='0'; token [2] = '0';
		// get the float
		gpsControlData.lon.flData = degMinToDeg(chTmp,atof(token));
		
		// 5.- Longitude Sector
		myTokenizer(NULL, ',', token);
		
		if (strlen(token)>0) {
			// set the sign of the float value
			if (token[0] == 'S' || token[0] == 'W') {
				gpsControlData.lon.flData = -gpsControlData.lon.flData;
			}
		}
	}
	
	// 6.- Quality Indicator
	myTokenizer(NULL, ',', token);
	if (strlen(token)== 1) {
		gpsControlData.fix = (char)atoi(token);
	}

	// 7.- Sats used in solution
	// xx
	myTokenizer(NULL, ',', token);	
	if (strlen(token)>0) {
		gpsControlData.sats = (unsigned char) atoi(token);	
	}
	
	// 8.- Horizontal dilution of solution given from 0 to 99.99 but 
	// stored from 0 - 990 
	//in integers, i.e HDOP = HDOP_stored/100 CAUTION
	// xx.xx
	myTokenizer(NULL, ',', token);	
	if (strlen(token)>0) {
		gpsControlData.hdop.usData = (unsigned short) (atof(token)*10.0);	
	}
	
	// 9.- Altitude above mean sea level given in meters
	// xxx.xxx
	myTokenizer(NULL, ',', token);	
	if (strlen(token)>0) {
		gpsControlData.height.flData = atof(token);	
	}
	
	// turn the flag on of new data
	gpsControlData.newData = 1;
}
