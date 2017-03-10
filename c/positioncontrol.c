#include "NU32.h"          // constants, functions for startup and UART
#include "isense.h"
#include "utilities.h"
#include "positioncontrol.h"

// // static volatile int Speed;
// static volatile float Desired_Angle;
// static volatile float Kp_position = 150, Ki_position = 0, Kd_position = 5000; // set position control gains to 0
// static volatile int Eint = 0;
// static volatile int Error_prev = 0;
// static volatile int Reference_current = 0;
// // static volatile int Reference_position_array[ITEST_SIZE];
// // static volatile int Actual_position_array[ITEST_SIZE];
// // static volatile int StoringData = 0;    // if this flag = 1, currently storing plot data
//
// void __ISR(_TIMER_4_VECTOR, IPL5SOFT) Controller4(void) { // _TIMER_4_VECTOR = 8
//     static int actual_position = 0;
//
//     switch (current_mode) {
//       case HOLD: {
//         // measure the angle value (deg)
//         actual_position = encoder_angle();
//
//         // PI Controller
//         float error = Desired_Angle - actual_position;
//         float edot = error - Error_prev;
//         Eint = Eint + error;
//         float u = Kp_position * error + Ki_position * Eint + Kd_position * edot;
//
//         // not sure what to put here
//         if (u > 300) {
//           Reference_current = 300;
//         } else if (u < -300) {
//           Reference_current = -300;
//         } else {
//           Reference_current = u;
//         }
//
//         Error_prev = error;
//         break;
//       }
//       // case TRACK: {
//       //
//       //   break;
//       // }
//     }
//     IFS0bits.T4IF = 0;
// }

// float get_position_control_Kp(void) {
//   return Kp_position;
// }
//
// float get_position_control_Ki(void) {
//   return Ki_position;
// }
//
// float get_position_control_Kd(void) {
//   return Kd_position;
// }
//
// int get_reference_current(void) {
//   return Reference_current;
// }
//
// void set_desired_angle(float angle) {
//   Desired_Angle = angle;
// }
//
// void clear_error_variables(void) {
//   Eint = 0;
//   Error_prev = 0;
//   Reference_current = 0;
// }
//
// void set_position_control_gains(float Kp, float Ki, float Kd) {
//   Kp_position = Kp;
//   Ki_position = Ki;
//   Kd_position = Kd;
// }

void Position_control_init(void) {
  TRISDbits.TRISD6 = 0;

  // TIMER 4
  T4CONbits.TCKPS = 0b110;            // Timer4 prescaler N=64 (1:64)
  PR4 = 6249;                  // period = (PR4+1) * N * 12.5 ns = 5 us, 200 Hz
  TMR4 = 0;
  T4CONbits.ON = 1;
  IPC4bits.T4IP = 5;            // step 4: interrupt priority 2
  IPC4bits.T4IS = 0;            // step 4: interrupt subpriority 1
  IFS0bits.T4IF = 0;            // step 5: clear the flag
  IFS0bits.T4IF = 1;            // step 5: set the flag
  IEC0bits.T4IE = 1;            // step 6: enable T4
}
