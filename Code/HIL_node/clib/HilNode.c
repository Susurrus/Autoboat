#include <xc.h>

#include <stdint.h>

#include "Hil.h"

//Use internal RC
_FOSCSEL(FNOSC_FRC & IESO_OFF);
//Clock Pragmas
_FOSC(FCKSM_CSECMD & OSCIOFNC_OFF & POSCMD_XT);
// Disable watchdog timer
_FWDT(FWDTEN_OFF);
//ICD Pragmas
_FICD(JTAGEN_OFF & ICS_PGD3);

// Store the timer callback used by timer2.
void (*timer2Callback)(void);

// Keep a variable here for counting the 100Hz timer so that it can be used to blink an LED
// at 1Hz.
int timerCounter = 0;

/**
 * Initializes Timer 2 for a 156,250 Hz clock. The prescalar can be used to modify this clock rate
 * to the desied ones. When the clock expires it triggers a call to `timerCallbackFcn` and resets.
 * The Timer is set up for a 256x multiplier on the prescalar, so your effective prescalar is really
 * 256*prescalar, with the final frequency being 156250/256/prescalar Hz.
 * @param timerCallbackFcn
 * @param prescalar
 */
void Timer2Init(void (*timerCallbackFcn)(void), uint16_t prescalar)
{
    // 156,250 Hz clock.
    T2CON = 0;
    IFS0bits.T2IF = 0;
    IPC1bits.T2IP2 = 1;
    IEC0bits.T2IE = 1;
    PR2 = prescalar;
    T2CON = 0x8030;

    timer2Callback = timerCallbackFcn;
}

void HilNodeInit(void)
{
    // Set a unique node ID for this node.
    //nodeId = CAN_NODE_HIL;

    // Initialize communications for HIL.
    HilInit();

    // Set up Timer2 for a 100Hz timer.
    Timer2Init(HilTransmitData, 1562);

    // Set UART1 to be pins B11/B13 TX/RX
    // (see Section 30 of the dsPIC33f manual)
    __builtin_write_OSCCONL(OSCCON & ~(1<<6));

    RPOR5bits.RP11R = 3; // Set RP11 to be U1TX
    RPINR18bits.U1RXR = 13; // Set U1RX to pin RP13

    __builtin_write_OSCCONL(OSCCON | (1<<6));

    // Enable pins B10/A4 as an digital output
    TRISBbits.TRISB10 = 0;
    TRISAbits.TRISA4 = 0;
}

int main()
{
    PLLFBD = 63;            // M = 65
    CLKDIVbits.PLLPOST = 0; // N1 = 2
    CLKDIVbits.PLLPRE = 1;  // N2 = 3

    __builtin_write_OSCCONH(0x01); // Initiate Clock Switch to

    __builtin_write_OSCCONL(OSCCON | 0x01); // Start clock switching

    while (OSCCONbits.COSC != 1); // Wait for Clock switch to occur

    while (OSCCONbits.LOCK != 1);
	
	HilNodeInit();

    while (1) {
        HilReceiveData();
    }
}

/**
 * Timer 2 interrupt. Merely calls the timerCallback() specified in Timer2Init().
 */
void __attribute__((interrupt, auto_psv)) _T2Interrupt(void)
{
    timer2Callback();

    // Toggle the amber LED at 1Hz.
    if (++timerCounter == 100) {
        LATA ^= 0x0010;
        timerCounter = 0;
    }

    IFS0bits.T2IF = 0;
}