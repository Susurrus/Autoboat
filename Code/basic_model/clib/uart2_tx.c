
// Includes:
// Circular buffer code
#include "uart2_tx.h"
#include "circBuffer.h"
#include <p33Fxxxx.h>

// Circular buffer declaration
CircBuffer uart2TxBuffer;

/**
 * This is the interrupt handler for UART2 transmission.
 * It is called after at least one byte is transmitted (
 * depends on UTXISEL<1:0> as to specifics). This function
 * therefore keeps adding bytes to transmit if there're more
 * in the queue.
 */
void __attribute__((__interrupt__, no_auto_psv_)) _U2TXInterrupt(void) {
	IFS1bits.U2TXIF = 0;
	startUart2Transmission();
}

/**
 * This function actually initiates transmission. It
 * attempts to start transmission with the first element
 * in the queue if transmission isn't already proceeding.
 * Once transmission starts the interrupt handler will
 * keep things moving from there.
 */
void startUart2Transmission() {
	if (getLength(&uart2TxBuffer) > 0 ) {
		U2TXREG = readFront(&uart2TxBuffer);
	}
}

/**
 * This function transmits a complete actuator struct
 * through UART2 using a circular buffer.
 * It works by just feeding each byte into the circular
 * buffer. Only reason it's specific to the actuator data
 * is because of the length of the array passed.
 */
void uart2EnqueueActuatorData(unsigned char *data) {
	unsigned char g;
	// Add all 22+6 bytes of the actuator struct to the queue.
	for (g = 0; g < 28;g++) {
		writeBack(&uart2TxBuffer,data[g]);
	}
}

/**
 * This function transmits a complete state struct
 * through UART2 using a circular buffer.
 * It works by just feeding each byte into the circular
 * buffer. Only reason it's specific to the actuator data
 * is because of the length of the array passed.
 */
void uart2EnqueueStateData(unsigned char *data) {
	unsigned char g;
	// Add all 23+6 bytes of the state struct to the queue.
	for (g = 0; g < 29;g++) {
		writeBack(&uart2TxBuffer,data[g]);
	}
}

/**
 * Initialization function for the circular buffer.
 * Should be called in initialization code for the
 * model.
 */
void initUart2Transmission() {
	U2STAbits.UTXEN = 1; // Enable transmission
	IEC1bits.U2TXIE = 1; // Enable transmission interrupt
	newCircBuffer(&uart2TxBuffer);
}
