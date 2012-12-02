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

#ifndef REVO_GS_H
#define REVO_GS_H

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
};
extern struct RevoData revoDataStore;

/**
 * Pull new bytes from the UART2 receive buffer and calls buildAndCheckSentence on each of them.
 * This function should be called repeatedly for receiving and processing received data.
 */
void RevoGsProcessData(void);

/**
 * Parse an NMEA0183-style sentence from the Revolution GS. Checks the message type and
 * pases off to the appropriate helper parsing function.
 */
void RevoGsParseSentence(const char *sentence);

/**
 * Parses proprietary NMEA0183 HTM sentences. Results are stored in the
 * globally-declared revoData struct.
 */
void RevoGsParseHtm(const char *stream);

/**
 * This function resets the entire revo data struct to zeros.
 */
void RevoGsClearData(void);
       
#endif // REVO_GS_H
