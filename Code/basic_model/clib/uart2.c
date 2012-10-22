#include "CircularBuffer.h"
#include "uart2.h"
#include <p33Fxxxx.h>

CircularBuffer uart2RxBuffer;
uint8_t u2RxBuf[64];
CircularBuffer uart2TxBuffer;
uint8_t u2TxBuf[64];

/*
 * Private functions.
 */
void startUart2Transmission(void);

/**
 * Initialization function for the UART2 peripheral.
 * Should be called in initialization code for the
 * model. This function configures the UART
 * for whatever baud rate is specified. It also configures two circular buffers
 * for transmission and reception.
 */
void initUart2(unsigned int brgRegister) {

    // First initialize the necessary circular buffers.
    CB_Init(&uart2RxBuffer, u2RxBuf, sizeof(u2RxBuf));
    CB_Init(&uart2TxBuffer, u2TxBuf, sizeof(u2TxBuf));

    // Configure and open the port;
    // U2MODE Register
    // ==============
    U2MODEbits.UARTEN	= 0;		// Disable the port
    U2MODEbits.USIDL 	= 0;		// Stop on idle
    U2MODEbits.IREN		= 0;		// No IR decoder
    U2MODEbits.RTSMD	= 0;		// Ready to send mode (irrelevant)
    U2MODEbits.UEN		= 0;		// Only RX and TX
    U2MODEbits.WAKE		= 1;		// Enable at startup
    U2MODEbits.LPBACK	= 0;		// Disable loopback
    U2MODEbits.ABAUD	= 0;		// Disable autobaud
    U2MODEbits.URXINV	= 0;		// Normal operation (high is idle)
    U2MODEbits.PDSEL	= 0;		// No parity 8 bit
    U2MODEbits.STSEL	= 0;		// 1 stop bit
    U2MODEbits.BRGH 	= 0;		// Low speed mode

    // U2STA Register
    // ==============
    U2STAbits.URXISEL	= 2;		// RX interrupt when 3 chars are in
    U2STAbits.OERR		= 0;		// clear overun error

    U2BRG = brgRegister;			// Set the baud rate register

    // Finally setup interrupts for proper UART communication.
    IPC7bits.U2TXIP = 6;    		// Interrupt priority 6
    IPC7bits.U2RXIP = 6;    		// Interrupt priority 6
    IEC1bits.U2TXIE = 1; 			// Enable transmission interrupt
    IEC1bits.U2RXIE = 1; 			// Enable reception interrupt

    // Enable the port;
    U2MODEbits.UARTEN	= 1;		// Enable the port
    U2STAbits.UTXEN		= 1;		// Enable TX

}

void changeUart2BaudRate(unsigned short brgRegister) {

    unsigned char utxen = U2STAbits.UTXEN;

    // Disable the port;
    U2MODEbits.UARTEN = 0;

    // Change the BRG register to set the new baud rate
    U2BRG = brgRegister;

    // Enable the port restoring the previous transmission settings
    U2MODEbits.UARTEN	= 1;
    U2STAbits.UTXEN		= utxen;
}

/**
 * This function actually initiates transmission. It
 * attempts to start transmission with the first element
 * in the queue if transmission isn't already proceeding.
 * Once transmission starts the interrupt handler will
 * keep things moving from there. The buffer is checked
 * for new data and the transmission buffer is checked that
 * it has room for new data before attempting to transmit.
 */
void startUart2Transmission() {
    if (uart2TxBuffer.dataSize > 0 && !U2STAbits.UTXBF) {
        // A temporary variable is used here because writing directly into U2TXREG causes some weird issues.
        unsigned char c;
        CB_ReadByte(&uart2TxBuffer, &c);
        U2TXREG = c;
    }
}

/**
 * This function supplements the uart2EnqueueData() function by also
 * providing an interface that only enqueues a single byte.
 */
void uart2EnqueueByte(unsigned char datum) {
    CB_WriteByte(&uart2TxBuffer, datum);
    startUart2Transmission();
}

/**
 * This function enqueues all bytes in the passed data character array according to the passed
 * length.
 */
void uart2EnqueueData(unsigned char *data, unsigned char length) {
    unsigned char g;

    for (g = 0; g < length; g++) {
        CB_WriteByte(&uart2TxBuffer, data[g]);
    }

    startUart2Transmission();
}

void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void) {

    // Keep receiving new bytes while the buffer has data.
    while (U2STAbits.URXDA == 1) {
        CB_WriteByte(&uart2RxBuffer, (unsigned char)U2RXREG);
    }

    // Clear buffer overflow bit if triggered
    if (U2STAbits.OERR == 1) {
        U2STAbits.OERR = 0;
    }

    // Clear the interrupt flag
    IFS1bits.U2RXIF = 0;
}

/**
 * This is the interrupt handler for UART2 transmission.
 * It is called after at least one byte is transmitted (
 * depends on UTXISEL<1:0> as to specifics). This function
 * therefore keeps adding bytes to transmit if there're more
 * in the queue.
 */
void __attribute__((__interrupt__, no_auto_psv)) _U2TXInterrupt(void) {
    startUart2Transmission();

    // Clear the interrupt flag
    IFS1bits.U2TXIF = 0;
}
