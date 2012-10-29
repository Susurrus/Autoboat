#ifndef _NMEA2000_ENCODE_H_
#define _NMEA2000_ENCODE_H_

#include "ecanDefinitions.h"

// Define a constant for specifying when a field is invalid. Use instead of all 1s or all 0s.
#define CAN_INV_DATA 0xFFFFFFFF

void PackagePgn127245(tCanMessage *msg, uint8_t sourceDevice, uint8_t instance, uint8_t dirOrder, float angleOrder, float position);

#endif // _NMEA2000_ENCODE_H_