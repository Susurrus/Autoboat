#include "CircularBuffer.h"
#include "Uart2.h"
#include <xc.h>

static CircularBuffer uart2RxBuffer;
static uint8_t u2RxBuf[1024];
static CircularBuffer uart2TxBuffer;
static uint8_t u2TxBuf[1024];

/*
 * Private functions.
 */
void Uart2StartTransmission(void);

/**
 * Initialization function for the UART1 peripheral.
 * Should be called in initialization code for the
 * model. This function configures the UART
 * for whatever baud rate is specified. It also configures two circular buffers
 * for transmission and reception.
 */
void Uart2Init(uint16_t brgRegister)
{

    // First initialize the necessary circular buffers.
    CB_Init(&uart2RxBuffer, u2RxBuf, sizeof(u2RxBuf));
    CB_Init(&uart2TxBuffer, u2TxBuf, sizeof(u2TxBuf));

    // Configure and open the port;
    // U1MODE Register
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

    // U1STA Register
    // ==============
    U2STAbits.URXISEL	= 2;		// RX interrupt when 3 chars are in
    U1STAbits.UTXISEL0	= 1;
    U1STAbits.UTXISEL1	= 0;		// TX interrupt when FIFO buffer is empty. There's no reason to
	                                // interrupt after every byte, so this reduces the number of
	                                // interrupts.
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

void Uart2ChangeBaudRate(uint16_t brgRegister)
{

    uint8_t utxen = U2STAbits.UTXEN;

    // Disable the port;
    U2MODEbits.UARTEN = 0;

    // Change the BRG register to set the new baud rate
    U1BRG = brgRegister;

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
void Uart2StartTransmission(void)
{
    while (uart2TxBuffer.dataSize > 0 && !U2STAbits.UTXBF) {
        // A temporary variable is used here because writing directly into U1TXREG causes some weird issues.
        uint8_t c;
        CB_ReadByte(&uart2TxBuffer, &c);
        U2TXREG = c;
    }
}

int Uart2ReadByte(uint8_t *datum)
{
    return CB_ReadByte(&uart2RxBuffer, datum);
}

/**
 * This function supplements the uart1EnqueueData() function by also
 * providing an interface that only enqueues a single byte.
 */
void Uart2WriteByte(uint8_t datum)
{
    CB_WriteByte(&uart2TxBuffer, datum);
    Uart2StartTransmission();
}

/**
 * This function enqueues all bytes in the passed data character array according to the passed
 * length.
 */
int Uart2WriteData(const void *data, size_t length)
{
    int success = CB_WriteMany(&uart2TxBuffer, data, length, false);

    Uart2StartTransmission();

    return success;
}

void __attribute__((__interrupt__, no_auto_psv)) _U2RXInterrupt(void)
{

    // Keep receiving new bytes while the buffer has data.
    while (U2STAbits.URXDA == 1) {
            CB_WriteByte(&uart2RxBuffer, (uint8_t)U2RXREG);
    }

    // Clear buffer overflow bit if triggered
    if (U2STAbits.OERR == 1) {
            U2STAbits.OERR = 0;
    }

    // Clear the interrupt flag
    IFS1bits.U2RXIF = 0;
}

/**
 * This is the interrupt handler for UART1 transmission.
 * It is called after at least one byte is transmitted (
 * depends on UTXISEL<1:0> as to specifics). This function
 * therefore keeps adding bytes to transmit if there're more
 * in the queue.
 */
void __attribute__((__interrupt__, no_auto_psv)) _U2TXInterrupt(void)
{
    Uart2StartTransmission();

    // Clear the interrupt flag
    IFS1bits.U2TXIF = 0;
}
