#ifndef _UART2_H_
#define _UART2_H_

// USAGE:
// Add initUart2() to an initialization sequence called once on startup.
// Use uart2Enqueue*Data() to push appropriately-sized data chunks into the queue and begin transmission.

#include "circBuffer.h"

extern CircBuffer uart2RxBuffer;
extern CircBuffer uart2TxBuffer;

void initUart2(unsigned int brgRegister);
void changeUart2BaudRate(unsigned short brgRegister);

/**
 * This function starts a transmission sequence after enqueuing a single byte into
 * the buffer.
 */
void uart2EnqueueByte(unsigned char datum);

/**
 * This function augments the uart2EnqueueByte() function by providing an interface
 * that enqueues multiple bytes.
 */
void uart2EnqueueData(unsigned char *data, unsigned char length);

#endif /* _UART2_H_ */
