#ifndef CAN_H
#define CAN_H

#include <stdint.h>

/**
 * This function should be called every timestep to process any received ECAN messages.
 */
uint8_t CanReceiveMessages(void);

void CanTransmitMessages(void);

#endif // CAN_H
