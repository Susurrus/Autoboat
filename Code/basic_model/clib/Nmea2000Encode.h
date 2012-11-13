#ifndef _NMEA2000_ENCODE_H_
#define _NMEA2000_ENCODE_H_

#include "ecanDefinitions.h"

// Define a constant for specifying when a field is invalid. Use instead of all 1s or all 0s.
#define CAN_INV_DATA 0xFFFFFFFF

/**
 *
 * @param msg
 * @param sourceDevice An ID representing the transmitting device.
 * @param instance Which rudder instance this is. Used for when multiple rudder controller exist. If unknown set to 0xFF.
 * @param dirOrder UNKNOWN. Set to 3 to specify invalid.
 * @param angleOrder Commanded rudder angle in radians. Set to >90 or <-90 to specify as invalid
 * @param position Sensed rudder angle in radians. Set to >90 or <-90 to specify as invalid.
 */
void PackagePgn127245(tCanMessage *msg, uint8_t sourceDevice, uint8_t instance, uint8_t dirOrder, float angleOrder, float position);

#endif // _NMEA2000_ENCODE_H_