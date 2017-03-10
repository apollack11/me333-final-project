#include "NU32.h"          // constants, functions for startup and UART
#include "isense.h"
#include "utilities.h"
#include "positioncontrol.h"

void Position_control_init(void) {
  TRISDbits.TRISD6 = 0;

  // TIMER 4
  T4CONbits.TCKPS = 0b110;            // Timer4 prescaler N=64 (1:64)
  PR4 = 6249;                  // period = (PR4+1) * N * 12.5 ns = 5 us, 200 Hz
  TMR4 = 0;
  T4CONbits.ON = 1;
  IPC4bits.T4IP = 5;            // step 4: interrupt priority 5
  IPC4bits.T4IS = 0;            // step 4: interrupt subpriority 0
  IFS0bits.T4IF = 0;            // step 5: clear the flag
  IFS0bits.T4IF = 1;            // step 5: set the flag
  IEC0bits.T4IE = 1;            // step 6: enable T4
}
