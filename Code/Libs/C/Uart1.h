#ifndef UART1_H
#define UART1_H

// USAGE:
// Add initUart1() to an initialization sequence called once on startup.
// Use uart1Enqueue*Data() to push appropriately-sized data chunks into the queue and begin transmission.

#include "CircularBuffer.h"

extern CircularBuffer uart1RxBuffer;
extern CircularBuffer uart1TxBuffer;

void initUart1(unsigned int brgRegister);
void changeUart1BaudRate(unsigned short brgRegister);

/**
 * This function starts a transmission sequence after enqueuing a single byte into
 * the buffer.
 */
void uart1EnqueueByte(unsigned char datum);

/**
 * This function augments the uart1EnqueueByte() function by providing an interface
 * that enqueues multiple bytes.
 */
void uart1EnqueueData(unsigned char *data, unsigned char length);

#endif // UART1_H
