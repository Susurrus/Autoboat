#ifndef HIL_NODE_H
#define HIL_NODE_H

// Declare flags for use with checking the `nodeStatus` variable declared in `Node.h`
#define NODE_STATUS_FLAG_HIL_ACTIVE 0x0001

/**
 * Initialization function, configures everything the node needs.
 */
void HilNodeInit(void);

/**
 * Callback for Timer4 triggering every 250ms
 */
void HilNodeBlink(void);

/**
 * Callback for Timer2 triggering every 10ms.
 */
void HilNodeTimer100Hz(void);

/**
 * Process any received CAN messages coming through ECAN1. Store incoming
 * data into the hilDataToTransmit struct.
 */
uint8_t CanReceiveMessages(void);

#endif // HIL_NODE_H
