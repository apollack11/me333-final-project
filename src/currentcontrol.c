#include "NU32.h"          // constants, functions for startup and UART

void PWM_init(void) {
  TRISDbits.TRISD5 = 0;

  // TIMER 3
  T3CONbits.TCKPS = 0b000;     // Timer3 prescaler N=1 (1:1)
  PR3 = 799;              // period = (PR3+1) * N * 12.5 ns = 50 us, 20 kHz
  TMR3 = 0;                // initial TMR3 count is 0
  OC1CONbits.OCTSEL = 1;
  OC1CONbits.OCM = 0b110;  // PWM mode without fault pin; other OC1CON bits are defaults
  OC1RS = 800;             // duty cycle = OC1RS/(PR3+1) = 75%
  OC1R = 800;              // initialize before turning OC1 on; afterward it is read-only
  T3CONbits.ON = 1;        // turn on Timer3
  OC1CONbits.ON = 1;       // turn on OC1

  // TIMER 2
  T2CONbits.TCKPS = 2;            // Timer2 prescaler N=4 (1:4)
  PR2 = 1999;
  TMR2 = 0;
  T2CONbits.ON = 1;
  IPC2bits.T2IP = 5;            // step 4: interrupt priority 2
  IPC2bits.T2IS = 0;            // step 4: interrupt priority 1
  IFS0bits.T2IF = 0;            // step 5: clear the flag
  IFS0bits.T2IF = 1;            // step 5: set the flag
  IEC0bits.T2IE = 1;            // step 6: enable T2
}
