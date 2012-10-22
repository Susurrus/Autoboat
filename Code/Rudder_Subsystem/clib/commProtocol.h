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

#ifndef _COMMPROTOCOL_H_
#define _COMMPROTOCOL_H_

#include <stdint.h>

/**
 * This function initializes all onboard UART communications
 */
void cpInitCommunications();

/**
 * This function builds a full message internally byte-by-byte,
 * verifies its checksum, and then pushes that data into the
 * appropriate struct.
 */
void buildAndCheckMessage(uint8_t characterIn, uint8_t sensorMode);

void processNewCommData(uint8_t sensorMode);

/**
 * The following functions change the UART2 baud rate to allow
 * for HIL mode (running at 115200baud) and back to the old baud rate.
 */
void setHilMode(uint8_t mode);

inline void enableHil();

inline void disableHil();

/**
 * This function calculates the checksum of some bytes in an
 * array by XORing all of them.
 */
uint8_t calculateChecksum(uint8_t* sentence, uint8_t size);

/**
 * This function takes the byte array of GPS data received during HIL and
 * writes it into the GPS storage struct. It does some necessary conversions.
 */
void UpdateGpsDataFromHil(uint8_t* data);

void SetHilData(uint8_t *data);

uint16_t GetCurrentTimestamp();

uint8_t IsNewHilData();

inline void uart2EnqueueActuatorData(uint8_t *data);

#endif /* _COMMPROTOCOL_H_ */
