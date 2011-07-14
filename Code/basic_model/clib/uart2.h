#ifndef _UART2_H_
#define _UART2_H_

// USAGE:
// Add initUart2() to an initialization sequence called once on startup.
// Use uart2Enqueue*Data() to push appropriately-sized data chunks into the queue.
// Call startUart2Transmission() every so often to initiate transmission of the data.

#include "circBuffer.h"

extern CircBuffer uart2RxBuffer;
extern CircBuffer uart2TxBuffer;

void initUart2(unsigned int brgRegister);
void changeUart2BaudRate(unsigned short brgRegister);

/**
 * This starts a transmission sequence. The transmission sent interrupt is called which then
 * continues this transmission sequence.
 */
void uart2EnqueueData(unsigned char *data, unsigned char length);

#endif /* _UART2_H_ */
