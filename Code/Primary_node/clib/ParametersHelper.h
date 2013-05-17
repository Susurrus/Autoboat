#ifndef PARAMETERS_HELPER_H
#define PARAMETERS_HELPER_H

#include <stdint.h>

/**
 * Provides a helper function for retrieving the automode boolean value from the nodeStatus bitfield.
 * @returns A boolean of whether the vehicle is in autonomous mode or not.
 */
uint8_t GetAutoMode(void);

/**
 * Provides a helper function for updating the autonomous mode of the vehicle. When the vehicle is
 * switched into autonomous mode, a new starting location is stored.
 * @param newMode True to put the vehicle into autonomous mode, False otherwise.
 */
void SetAutoMode(uint8_t newMode);

/**
 * Helper function for toggling the autonomous mode on the node.
 */
void ToggleAutoMode(void);

#endif // PARAMETERS_HELPER_H