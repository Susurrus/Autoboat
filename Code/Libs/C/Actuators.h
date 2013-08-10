#ifndef ACTUATORS_H
#define ACTUATORS_H

/**
 * This library provides functionality related to the actuators: the rudder and the propeller.
 */

void ActuatorsTransmitCommands(float rudderCommand, int16_t throttleCommand);

#endif // ACTUATORS_H