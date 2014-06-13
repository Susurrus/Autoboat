#ifndef ACTUATORS_H
#define ACTUATORS_H

/**
 * This library provides functionality related to the actuators: the rudder and the propeller.
 */

/**
 * Transmits the given actuator commands to all appropriate actuators. By default, this function will
 * only transmit commands if they are different from the last time this function was called. Setting
 * the forceTransmission parameter to true overrides this behavior.
 * @param rudderCommand The rudder command, in radians
 * @param throttleCommand The throttle command, [-1000,1000]
 * @param forceTransmission Forces the commands to be transmit even if they haven't changed
 */
void ActuatorsTransmitCommands(float rudderCommand, int16_t throttleCommand, bool forceTransmission);

#endif // ACTUATORS_H