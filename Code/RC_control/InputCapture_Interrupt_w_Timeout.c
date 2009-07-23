#include "p33fxxxx.h"
#include "InCap.h" // Includes code for initializing input capture.

// Create 16-bit unsigned data type
typedef unsigned int uint16_T;

volatile uint16_T ic2up;
volatile uint16_T ic1up;
void __attribute__((__interrupt__)) _IC1Interrupt(void)
{
  static uint16_T IC1TMR;
  uint16_T tmp;
  uint16_T calcul;                     /* intermediate value */
  while (IC1CONbits.ICBNE == 1)
    tmp = IC1BUF;                      /* take the last value */
  if ((IC1CON & 1)==1)                 /* rising edge */
  {
    IC1CON = (IC1CON & 0xfffe);        /* detect next falling edge */
  } else                               /* falling edge */
  {
    IC1CON = (IC1CON | 0x0001);        /* detect next rising edge */
    calcul = tmp - IC1TMR;
    ic1up = calcul;
  }
  
  // Reset our 50ms timer to detect DC signals
  TMR5HLD = 0x00; //Write msw to the Type C timer holding register
  TMR4 = 0x00; //Write lsw to the Type B timer register

  IC1TMR = tmp;
  _IC1IF = 0;
}

void __attribute__((__interrupt__)) _IC2Interrupt(void)
{
  static uint16_T IC2TMR;
  uint16_T tmp;
  uint16_T calcul;                     /* intermediate value */
  while (IC2CONbits.ICBNE == 1)
    tmp = IC2BUF;                      /* take the last value */
  if ((IC2CON & 1)==1)                 /* rising edge */
  {
    IC2CON = (IC2CON & 0xfffe);        /* detect next falling edge */
  } else                               /* falling edge */
  {
    IC2CON = (IC2CON | 0x0001);        /* detect next rising edge */
    calcul = tmp - IC2TMR;
    ic2up = calcul;
  }
  
  // Reset our 50ms timer to detect DC signals
  TMR5HLD = 0x00; //Write msw to the Type C timer holding register
  TMR4 = 0x00; //Write lsw to the Type B timer register
  
  IC2TMR = tmp;
  _IC2IF = 0;
}

// Sets up timer 4 & 5 in 32bit operation to detect DC values on IC inputs.
void customInit() {
    T5CONbits.TON = 0; // Stop any 16-bit Timer5 operation
    T4CONbits.TON = 0; // Stop any 16/32-bit Timer4 operation
    T4CONbits.T32 = 1; // Enable 32-bit Timer mode
    T4CONbits.TCS = 0; // Select internal instruction cycle clock
    T4CONbits.TGATE = 0; // Disable Gated Timer mode
    T4CONbits.TCKPS = 0b00;// Select 1:1 Prescaler
    TMR5 = 0x00; // Clear 32-bit Timer (msw)
    TMR4 = 0x00; // Clear 32-bit Timer (lsw)
    PR5 = 0x0010; // Load 32-bit period value (msw)
    PR4 = 0x0000; // Load 32-bit period value (lsw)
    IPC7bits.T5IP = 0x01; // Set Timer5 Interrupt Priority Level
    IFS1bits.T5IF = 0; // Clear Timer5 Interrupt Flag
    IEC1bits.T5IE = 1; // Enable Timer5 interrupt
    T4CONbits.TON = 1; // Start 32-bit Timer
    
    // Configure our input captures
    OpenCapture1(IC_IDLE_CON & IC_TIMER2_SRC & IC_INT_1CAPTURE & IC_EVERY_RISE_EDGE);
    ConfigIntCapture1(IC_INT_ON & IC_INT_PRIOR_7);
    OpenCapture2(IC_IDLE_CON & IC_TIMER2_SRC & IC_INT_1CAPTURE & IC_EVERY_RISE_EDGE);
    ConfigIntCapture2(IC_INT_ON & IC_INT_PRIOR_7);
}

// Interrupt handler for timer 5.
// After 50ms timeout for input capture 1 & 2, clear the high-time for the
// signal to 0.
void __attribute__((__interrupt__, __shadow__)) _T5Interrupt(void)
{
    ic1up = 0;
    ic2up = 0;
    IFS1bits.T5IF = 0; //Clear Timer5 interrupt flag
}

extern uint16_T get_IC1_Hightime() {
    if (ic2up == 0) {
        return 0;
    }
    else {
        return ic1up;
    }
}

extern uint16_T get_IC2_Hightime() {
    return ic2up;
}
