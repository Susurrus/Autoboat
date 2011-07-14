#ifndef _UART1_H_
#define _UART1_H_

// USAGE:
// Add initUart1() to an initialization sequence called once on startup.
// Use uart1Enqueue*Data() to push appropriately-sized data chunks into the queue. 

#include "circBuffer.h"

extern CircBuffer uart1RxBuffer;
extern CircBuffer uart1TxBuffer;

void initUart1(unsigned int brgRegister);
void changeUart1BaudRate(unsigned short brgRegister);

/**
 * This starts a transmission sequence. The transmission sent interrupt is called which then
 * continues this transmission sequence.
 */
void uart1EnqueueData(unsigned char *data, unsigned char length);

#endif /* _UART1_H_ */
