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

#include "Node.h"
#include "Nmea0183.h"
#include "RevoGs.h"
#include "Uart1.h"

void ImuNodeInit(void)
{
	nodeId = CAN_NODE_IMU_SENSOR;
}

/**
 * This function reads in new data from UART2 and feeds it into the NMEA0183 parser which then
 * calls the Revo GS library for parsing the data out.
 */
void ImuNodeProcessRevoData(void)
{
	// The following variables are all necessary for the NMEA0183 parsing of the RevoGS messages.
	static char sentence[127];
	static uint8_t sentenceIndex;
	static uint8_t checksum;
	static uint8_t sentenceState;

    uint8_t c;
	while (Uart1ReadByte(&c)) {
		buildAndCheckSentence((char)c, sentence, &sentenceIndex, &sentenceState, &checksum, RevoGsParseSentence);
	}
}