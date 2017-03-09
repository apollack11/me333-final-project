#include "NU32.h"          // config bits, constants, funcs for startup and UART
#include "encoder.h"
#include "isense.h"
#include "utilities.h"
#include "currentcontrol.h"
#include "positioncontrol.h"
#include <stdio.h>

#define BUF_SIZE 200
#define ITEST_SIZE 100

static volatile int Speed;
static volatile float Desired_angle;
static volatile float Kp_current = 20, Ki_current = 1; // set current control gains to 0
//static volatile float Kp_current = 0.27, Ki_current = 0.033; // set current control gains to 0
// static volatile float Kp_position = 150, Ki_position = 0, Kd_position = 5000; // set position control gains to 0
static volatile float Kp_position = 150, Ki_position = 0, Kd_position = 5000; // set position control gains to 0
static volatile float Desired_current = 0;
static volatile int Eint_current = 0;
static volatile int Eint_position = 0;
static volatile float Error_prev = 0;
static volatile int Reference_current_array[ITEST_SIZE];
static volatile int Actual_current_array[ITEST_SIZE];
static volatile int StoringData = 0;    // if this flag = 1, currently storing plot data

static volatile int Eint_check;
static volatile float Error_prev_check;
static volatile float Error_check;
static volatile float U_check;

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) CurrentController(void) { // _TIMER_2_VECTOR = 8
    static int current_count = 0;
    static float actual_current = 0;
    static float reference_current = 200;

    switch (get_mode()) {
      case IDLE: {
        LATDbits.LATD5 = 0;
        OC1RS = 0;
        break;
      }
      case PWM: {
        if (Speed <= 0) {
          LATDbits.LATD5 = 0;
          OC1RS = (unsigned int) ((-Speed / 100.0) * PR3);
        } else {
          LATDbits.LATD5 = 1;
          OC1RS = (unsigned int) ((Speed / 100.0) * PR3);
        }
        break;
      }
      case ITEST: {
        // current counter changes sign of reference current every 25 iterations
        if (current_count != 0 && current_count % 25 == 0) { // check how many iterations through the ISR
          reference_current *= -1; // set reference current to opposite sign
        }
        if (current_count < 100) {
          // measure the current value
          actual_current = ADC_milliAmps();
          // store the value of reference and actual current in the reference current array
          Reference_current_array[current_count] = (int)reference_current;
          Actual_current_array[current_count] = (int)actual_current;

          // PI Controller
          float error = reference_current - actual_current;
          Eint_current = Eint_current + error;
          float u = Kp_current * error + Ki_current * Eint_current;

          float unew = ((u / 1000) / 3.3) * 100;
          if (unew > 100.0) {
            unew = 100.0;
          } else if (unew < -100.0) {
            unew = -100.0;
          }

          // set new PWM value
          if (unew < 0) {
            LATDbits.LATD5 = 1;
            OC1RS = (unsigned int) ((-unew / 100.0) * PR3);
          } else {
            LATDbits.LATD5 = 0;
            OC1RS = (unsigned int) ((unew / 100.0) * PR3);
          }

          current_count++; // increment current count
        }
        else {
          StoringData = 0;
          current_count = 0;
          reference_current = 200;
          set_mode(IDLE);
        }

        break;
      }
      case HOLD: {
        // measure the current value
        actual_current = ADC_milliAmps();
        reference_current = Desired_current;

        // PI Controller
        float error = reference_current - actual_current;
        Eint_current = Eint_current + error;
        float u = Kp_current * error + Ki_current * Eint_current;

        float unew = ((u / 1000) / 3.3) * 100;
        if (unew > 100.0) {
          unew = 100.0;
        } else if (unew < -100.0) {
          unew = -100.0;
        }

        // set new PWM value
        if (unew < 0) {
          LATDbits.LATD5 = 1;
          OC1RS = (unsigned int) ((-unew / 100.0) * PR3);
        } else {
          LATDbits.LATD5 = 0;
          OC1RS = (unsigned int) ((unew / 100.0) * PR3);
        }

        break;
      }
      // case TRACK: {
      //
      //   break;
      // }
    }
    IFS0bits.T2IF = 0;
}

void __ISR(_TIMER_4_VECTOR, IPL5SOFT) PositionController(void) {
    static float actual_position = 0;

    switch (get_mode()) {
      case IDLE: {

        break;
      }
      case PWM: {

        break;
      }
      case ITEST: {

        break;
      }
      case HOLD: {
        // measure the angle value (deg)
        actual_position = encoder_angle();

        // PI Controller
        float error = Desired_angle - actual_position;
        float edot = error - Error_prev;
        Eint_position = Eint_position + error;
        float u = -(Kp_position * error + Ki_position * Eint_position + Kd_position * edot);

        // // if (flag == 0) {
        //   Eint_check = Eint_position;
        //   Error_prev_check = Error_prev;
        //   Error_check = Desired_angle - actual_position;
        //   U_check = -(Kp_position * (Desired_angle - actual_position));
        //   // flag = 1;
        // // }

        // not sure what to put here
        if (u > 300) {
          Desired_current = 300;
        } else if (u < -300) {
          Desired_current = -300;
        } else {
          Desired_current = u;
        }

        Error_prev = error;
        break;
      }
      // case TRACK: {
      //
      //   break;
      // }
    }
    IFS0bits.T4IF = 0;
}

int main()
{
  char buffer[BUF_SIZE];
  NU32_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
  NU32_LED1 = 1;  // turn off the LEDs
  NU32_LED2 = 1;

  ADC_init();

  __builtin_disable_interrupts();
  encoder_init();
  Current_control_init();
  Position_control_init();
  __builtin_enable_interrupts();

  encoder_reset(); // might need this? encoder reading high value at startup

  set_mode(IDLE);

  float kptemp = 0, kitemp = 0, kdtemp = 0;
  float angle = 0;

  while(1)
  {
    NU32_ReadUART3(buffer,BUF_SIZE); // we expect the next character to be a menu command
    NU32_LED2 = 1;                   // clear the error LED
    switch (buffer[0]) {
      case 'a':                      // read ADC (counts)
      {
        sprintf(buffer,"%d\r\n", ADC_count()); // return current sensor ADC count
        NU32_WriteUART3(buffer);
        break;
      }
      case 'b':                      // read ADC (mA)
      {
        sprintf(buffer,"%f\r\n", ADC_milliAmps()); // return current sensor mA
        NU32_WriteUART3(buffer);
        break;
      }
      case 'c':                      // read encoder (counts)
      {
        sprintf(buffer,"%d\r\n", encoder_ticks()); // return encoder ticks
        NU32_WriteUART3(buffer);
        break;
      }
      case 'd':                      // read encoder (degrees)
      {
        sprintf(buffer,"%d\r\n", encoder_angle()); // return encoder angle
        NU32_WriteUART3(buffer);
        break;
      }
      case 'e':                      // reset encoder
      {
        encoder_reset();
        sprintf(buffer,"%d\r\n", encoder_angle()); // return encoder angle (should be 0.0)
        NU32_WriteUART3(buffer);
        break;
      }
      case 'f':                      // send PWM value to motor
      {
        set_mode(PWM);
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%d", &Speed);
        break;
      }
      case 'g':                      // set current gains
      {
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%f", &kptemp);
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%f", &kitemp);
        __builtin_disable_interrupts(); // keep ISR disabled as briefly as possible
        Kp_current = kptemp;
        Ki_current = kitemp;
        __builtin_enable_interrupts();  // only 2 simple C commands while ISRs disabled
        break;
      }
      case 'h':                      // get current gains
      {
        sprintf(buffer,"%f\r\n", Kp_current); // return Kp_current
        NU32_WriteUART3(buffer);
        sprintf(buffer,"%f\r\n", Ki_current); // return Ki_current
        NU32_WriteUART3(buffer);
        break;
      }
      case 'i':                      // set position gains
      {
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%f", &kptemp);
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%f", &kitemp);
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%f", &kdtemp);
        __builtin_disable_interrupts(); // keep ISR disabled as briefly as possible
        Kp_position = kptemp;
        Ki_position = kitemp;
        Kd_position = kdtemp;
        __builtin_enable_interrupts();  // only 2 simple C commands while ISRs disabled
        break;
      }
      case 'j':                      // get position gains
      {
        sprintf(buffer,"%f\r\n", Kp_position); // return Kp_position
        NU32_WriteUART3(buffer);
        sprintf(buffer,"%f\r\n", Ki_position); // return Ki_position
        NU32_WriteUART3(buffer);
        sprintf(buffer,"%f\r\n", Kd_position); // return Kd_position
        NU32_WriteUART3(buffer);
        break;
      }
      case 'k':                      // current test mode
      {
        Eint_current = 0;
        StoringData = 1;
        set_mode(ITEST);
        while (StoringData) {
          ;
        }
        sprintf(buffer,"%d\r\n", ITEST_SIZE); // send number of values in the arrays
        NU32_WriteUART3(buffer);
        int i;
        for (i = 0; i < ITEST_SIZE; i++) {
          __builtin_disable_interrupts();
          sprintf(buffer,"%d %d\r\n", Reference_current_array[i], Actual_current_array[i]); // return Ki_current
          NU32_WriteUART3(buffer);
          __builtin_enable_interrupts();
        }
        break;
      }
      case 'l':                      // go to motor angle
      {
        Eint_current = 0;
        Eint_position = 0;
        Error_prev = 0;
        Desired_current = 0;
        NU32_ReadUART3(buffer,BUF_SIZE);
        sscanf(buffer, "%f", &angle);
        __builtin_disable_interrupts(); // keep ISR disabled as briefly as possible
        Desired_angle = angle;
        __builtin_enable_interrupts();  // only 2 simple C commands while ISRs disabled
        set_mode(HOLD);
        break;
      }
      case 'p':
      {
        set_mode(IDLE);
        Speed = 0;
        break;
      }
      case 'q':
      {
        // handle q for quit. Later you may want to return to IDLE mode here.
        set_mode(IDLE);
        break;
      }
      case 'r':
      {
        sprintf(buffer,"%s\r\n", get_string_mode());
        NU32_WriteUART3(buffer);
        break;
      }
      case 'z':
      {
        sprintf(buffer,"%f\r\n", Desired_current);
        NU32_WriteUART3(buffer);
        sprintf(buffer,"%d\r\n", Eint_check);
        NU32_WriteUART3(buffer);
        sprintf(buffer,"%d\r\n", Eint_position);
        NU32_WriteUART3(buffer);
        sprintf(buffer,"%f\r\n", Error_prev_check);
        NU32_WriteUART3(buffer);
        sprintf(buffer,"%f\r\n", Error_check);
        NU32_WriteUART3(buffer);
        sprintf(buffer,"%f\r\n", U_check);
        NU32_WriteUART3(buffer);
        sprintf(buffer,"%f\r\n", Desired_angle);
        NU32_WriteUART3(buffer);
        break;
      }
      default:
      {
        NU32_LED2 = 0;  // turn on LED2 to indicate an error
        break;
      }
    }
  }
  return 0;
}
