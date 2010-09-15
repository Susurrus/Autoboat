
#include "commProtocol.h"
#include "circBuffer.h"
#include "uart2_rx.h"
#include <p33Fxxxx.h>

CircBuffer uart2RxBuffer;

void __attribute__((__interrupt__, no_auto_psv_)) _U2RXInterrupt(void) {

	while (U2STAbits.URXDA == 1) {
		writeBack(&uart2RxBuffer, (unsigned char)U2RXREG);
	}
	
	// Clear buffer overflow bit if triggered
	if (U2STAbits.OERR == 1) {
		U2STAbits.OERR = 0;
	}

	IFS1bits.U2RXIF = 0;
}

/**
 * This function should be called continously. Each timestep
 * it runs through the most recently received data, parsing
 * it for sensor data. Once a complete message has been parsed
 * the data inside will be returned through the message
 * array.
 */
void parseNewRxData(unsigned char* message) {
	while (getLength(&uart2RxBuffer) > 0) {
		buildAndCheckMessage(readFront(&uart2RxBuffer));
	}
}

/**
 * Initialization function for the circular buffer.
 * Should be called in initialization code for the
 * model.
 */
void initUart2Reception() {
	IEC1bits.U2RXIE = 1; // Enable reception interrupt
	newCircBuffer(&uart2RxBuffer);
}
