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

#ifndef REVO_H
#define REVO_H

#include "Types.h"

struct RevoData {
	tFloatToChar           heading;
	char                   magStatus;
	tFloatToChar           pitch;
	char                   pitchStatus;
	tFloatToChar           roll;
	char                   rollStatus;
	tFloatToChar           dip;
	tUnsignedShortToChar   magneticMagnitude;
	unsigned char          newData; // Flag for whether this struct stores new data
};
extern struct RevoData revoDataStore;

void processRevoSentence(char *sentence);

/**
 * Pull new bytes from the UART2 receive buffer and
 * calls buildAndCheckSentence on each of them.
 */
void processNewRevoData(void);

/**
 * Computes the checksum for a given GPS sentence.
 */
unsigned char getChecksum(char *sentence, unsigned char size);

/**
 * This is a Matlab helper function that returns the most recent 
 * GPS data in a large array that Matlab can handle.
 * @param data A pointer to a float array for storing the GPS data that was requested.
 */
void getRevoData(unsigned char *data);

/**
 * This function resets the entire revo data struct to zeros.
 */
void clearRevoData(void);

/**
 * A simple tokenizer. Similar to strtok(), but supports
 * multiple tokens in a row.
 */
unsigned char myTokenizer(char *stringToTokenize, char token, char *returnToken);

/**
 * Parses proprietary NMEA0183 HTM sentences. Results are stored in the
 * globally-declared revoData struct.
 */
void parseHTM(char* stream);
       
#endif // REVO_H
