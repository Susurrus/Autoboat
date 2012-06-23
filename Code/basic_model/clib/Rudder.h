#ifndef _RUDDER_H_
#define _RUDDER_H_

#include <stdbool.h>

#include "types.h"

// Declaration of the relevant message structs used.
struct RudderData {
	tUnsignedShortToChar Position;
	bool PortLimit;
	bool StarboardLimit;
};
extern struct RudderData rudderDataStore;

/**
 * Retrieves the stored rudder data from the struct into
 * a packed uint8 array. Suitable for using in Matlab.
 */
void GetRudderData(uint8_t *data);

/**
 * Records the provided rudder data in a uint8 array
 * into a local tRudderData struct.
 */
void SetRudderData(uint8_t *data);

/**
 * Clears the stored rudder data to all zeros.
 */
void ClearRudderData();

#endif // _RUDDER_H_
