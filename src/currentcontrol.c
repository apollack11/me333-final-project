#include "NU32.h"          // constants, functions for startup and UART
#include "isense.h"
#include "currentcontrol.h"

// #define BUF_SIZE 200
// #define NUMSAMPS 1000      // number of points in the waveform
// #define PLOTPTS 200        // number of data points to plot
// #define DECIMATION 10      // plot every 10th point
// #define SAMPLE_TIME 10     // 10 core timer ticks = 250ns

// #define ITEST_SIZE 100
//
// static volatile int Speed;
// // static volatile float Angle;
// static volatile float Kp_current = 20, Ki_current = 1; // set current control gains to 0
// // static volatile float Kp_position = 4.76, Ki_position = 0.32, Kd_position = 10.63; // set position control gains to 0
// static volatile int Eint = 0;
// static volatile int Reference_current_array[ITEST_SIZE];
// static volatile int Actual_current_array[ITEST_SIZE];
// static volatile int StoringData = 0;    // if this flag = 1, currently storing plot data
//
// void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Controller(void) { // _TIMER_2_VECTOR = 8
//     static int current_count = 0;
//     static int actual_current = 0;
//     static int reference_current = 200;
//
//     switch (current_mode) {
//       case IDLE: {
//         LATDbits.LATD5 = 0;
//         OC1RS = 0;
//         break;
//       }
//       case PWM: {
//         if (Speed <= 0) {
//           LATDbits.LATD5 = 0;
//           OC1RS = (unsigned int) ((-Speed / 100.0) * PR3);
//         } else {
//           LATDbits.LATD5 = 1;
//           OC1RS = (unsigned int) ((Speed / 100.0) * PR3);
//         }
//         break;
//       }
//       case ITEST: {
//         if (current_count == 0) {
//           Eint = 0;
//         }
//
//         // current counter changes sign of reference current every 25 iterations
//         if (current_count != 0 && current_count % 25 == 0) { // check how many iterations through the ISR
//           reference_current *= -1; // set reference current to opposite sign
//         }
//         if (current_count < 100) {
//           // measure the current value
//           actual_current = (int)ADC_milliAmps();
//           // store the value of reference and actual current in the reference current array
//           Reference_current_array[current_count] = reference_current;
//           Actual_current_array[current_count] = actual_current;
//
//           // PI Controller
//           float error = reference_current - actual_current;
//           Eint = Eint + error;
//           float u = Kp_current * error + Ki_current * Eint;
//
//           float unew = ((u / 1000) / 3.3) * 100;
//           if (unew > 100.0) {
//             unew = 100.0;
//           } else if (unew < -100.0) {
//             unew = -100.0;
//           }
//
//           // set new PWM value
//           if (unew < 0) {
//             LATDbits.LATD5 = 1;
//             OC1RS = (unsigned int) ((-unew / 100.0) * PR3);
//           } else {
//             LATDbits.LATD5 = 0;
//             OC1RS = (unsigned int) ((unew / 100.0) * PR3);
//           }
//
//           current_count++; // increment current count
//         }
//         else {
//           StoringData = 0;
//           current_count = 0;
//           reference_current = 200;
//           current_mode = IDLE;
//         }
//
//         break;
//       }
//       // case HOLD: {
//       //
//       //   break;
//       // }
//       // case TRACK: {
//       //
//       //   break;
//       // }
//     }
//     IFS0bits.T2IF = 0;
// }

// float get_current_control_Kp(void) {
//   return Kp_current;
// }
//
// float get_current_control_Ki(void) {
//   return Ki_current;
// }
//
// int get_current_StoringData(void) {
//   return StoringData;
// }
//
// int* get_reference_array(void) {
//   return Reference_current_array;
// }
//
// int* get_actual_array(void) {
//   return Actual_current_array;
// }
//
// void set_current_control_gains(float Kp, float Ki) {
//   Kp_current = Kp;
//   Ki_current = Ki;
// }
//
// void set_current_Eint(int val) {
//   Eint = val;
// }
//
// void set_Speed(int val) {
//   Speed = val;
// }
//
// void set_current_StoringData(int val) {
//   StoringData = val;
// }

void Current_control_init(void) {
  TRISDbits.TRISD5 = 0;

  // TIMER 3
  T3CONbits.TCKPS = 0b000;     // Timer3 prescaler N=1 (1:1)
  PR3 = 3999;              // period = (PR3+1) * N * 12.5 ns = 50 us, 20 kHz
  TMR3 = 0;                // initial TMR3 count is 0
  OC1CONbits.OCTSEL = 1;
  OC1CONbits.OCM = 0b110;  // PWM mode without fault pin; other OC1CON bits are defaults
  OC1RS = 4000;             // duty cycle = OC1RS/(PR3+1) = 75%
  OC1R = 4000;              // initialize before turning OC1 on; afterward it is read-only
  T3CONbits.ON = 1;        // turn on Timer3
  OC1CONbits.ON = 1;       // turn on OC1

  // TIMER 2
  T2CONbits.TCKPS = 0b000;            // Timer2 prescaler N=1 (1:1)
  PR2 = 15999;            // period = (PR2+1) * N * 12.5 ns = 20 us, 5 kHz
  TMR2 = 0;
  T2CONbits.ON = 1;
  IPC2bits.T2IP = 5;            // step 4: interrupt priority 2
  IPC2bits.T2IS = 0;            // step 4: interrupt subpriority 1
  IFS0bits.T2IF = 0;            // step 5: clear the flag
  IFS0bits.T2IF = 1;            // step 5: set the flag
  IEC0bits.T2IE = 1;            // step 6: enable T2
}
