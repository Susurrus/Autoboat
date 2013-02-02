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

#ifndef HIL_H
#define HIL_H

#include <stdint.h>
#include "Node.h"
#include "HilNode.h"

// Define some helper macros for use with the `nodeStatus` variable.
#define HIL_ACTIVE (nodeStatus & NODE_STATUS_FLAG_HIL_ACTIVE)

/**
 * Declare the struct that holds all the data transmit to the PC for HIL.
 */
union HilDataToPc {
    struct x {
        float rCommandAngle;
        float tCommandSpeed;
        float rudderAngle;
        uint16_t sensorOverride;
        uint16_t timestamp;
    } data;
    uint8_t bytes[sizeof(struct x)];
};
extern union HilDataToPc hilDataToTransmit;

/**
 * Declare the struct that holds all the data received from the PC for HIL.
 */
union HilDataFromPc {
    struct y {
        float tSpeed;
        float rAngle;
        int32_t gpsLatitude;
        int32_t gpsLongitude;
        uint16_t gpsCog;
        uint16_t gpsSog;
        uint16_t timestamp;
    } data;
    uint8_t bytes[sizeof(struct y)];
};
extern union HilDataFromPc hilReceivedData;

/**
 * This function initializes all onboard UART communications
 */
void HilInit(void);

/**
 * This function builds a full message internally byte-by-byte,
 * verifies its checksum, and then pushes that data into the
 * appropriate struct.
 */
void HilBuildMessage(uint8_t data);

void HilReceive(void);

void HilProcessData(uint8_t *data, unsigned short dataLen);

void HilTransmitData(void);

void HilSetActive(void);

void HilSetInactive(void);

/**
 * This function calculates the checksum of some bytes in an
 * array by XORing all of them.
 */
uint8_t HilCalculateChecksum(const uint8_t *sentence, uint8_t size);

#endif // HIL_H
