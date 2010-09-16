
#ifndef _UART2_H_
#define _UART2_H_

// USAGE:
// Add initUart2Transmission() to an initialization sequence called once on startup.
// Use uart2Transmit*Data() to push appropriately-sized data chunks into the queue.
// Call startUart2Transmission() every so often to initiate transmission of the data.

void startUart2Transmission();
void uart2EnqueueActuatorData(unsigned char *data);
void uart2EnqueueStateData(unsigned char *data);

// UART2 Reception code
void parseNewRxData(unsigned char* message);
void initUart2();

#endif /* _UART2_H_ */
