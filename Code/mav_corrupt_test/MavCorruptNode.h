#ifndef MAV_CORRUPT_NODE_H
#define MAV_CORRUPT_NODE_H

// Calculate the BRG register value necessary for 115200 baud with a 80MHz clock.
#define BAUD115200_BRG_REG 21

#define FATAL_ERROR() while(1)

/**
 * Set the primary status indicator LED to always blink
 */
void SetStatusModeLed(void);

#endif // MAV_CORRUPT_NODE_H
