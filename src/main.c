#include "NU32.h"          // config bits, constants, funcs for startup and UART
#include "encoder.h"
#include "isense.h"
#include "utilities.h"
#include "currentcontrol.h"
#include <stdio.h>

#define BUF_SIZE 200
#define NUMSAMPS 1000      // number of points in the waveform
#define PLOTPTS 200        // number of data points to plot
#define DECIMATION 10      // plot every 10th point
#define SAMPLE_TIME 10     // 10 core timer ticks = 250ns

#define ITEST_SIZE 100

static volatile int Speed;
static volatile float Kp_current = 40, Ki_current = 0.32; // set current control gains to 0
static volatile int Eint = 0;
static volatile int Reference_current_array[ITEST_SIZE];
static volatile int Actual_current_array[ITEST_SIZE];
static volatile int StoringData = 0;    // if this flag = 1, currently storing plot data

static volatile float U_check;
static volatile int Eint_check;
static volatile int Eint_check2;
static volatile float Error_check;
static volatile int Ref_check;
static volatile int Actual_check;
static volatile float Unew_check;

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Controller(void) { // _TIMER_2_VECTOR = 8
    static int current_count = 0;
    static int actual_current = 0;
    static int reference_current = 200;

    switch (current_mode) {
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
        if (current_count == 0) {
          Eint = 0;
        }

        // current counter changes sign of reference current every 25 iterations
        if (current_count != 0 && current_count % 25 == 0) { // check how many iterations through the ISR
          reference_current *= -1; // set reference current to opposite sign
        }
        if (current_count < 100) {
          // measure the current value
          actual_current = (int)ADC_milliAmps();
          // store the value of reference and actual current in the reference current array
          Reference_current_array[current_count] = reference_current;
          Actual_current_array[current_count] = actual_current;

          // PI Controller
          float error = reference_current - actual_current;
          Eint = Eint + error;
          float u = Kp_current * error + Ki_current * Eint;

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
          current_mode = IDLE;
        }

        break;
      }
    }
    IFS0bits.T2IF = 0;
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
  PWM_init();
  __builtin_enable_interrupts();

  current_mode = IDLE;

  float kptemp = 0, kitemp = 0;

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
        current_mode = PWM;
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
        Kp_current = kptemp;                    // copy local variables to globals used by ISR
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
      case 'k':                      // current test mode
      {
        Eint = 0;
        StoringData = 1;
        current_mode = ITEST;
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
      case 'p':
      {
        current_mode = IDLE;
        Speed = 0;
        break;
      }
      case 'q':
      {
        // handle q for quit. Later you may want to return to IDLE mode here.
        current_mode = IDLE;
        break;
      }
      case 'r':
      {
        sprintf(buffer,"%s\r\n", get_current_mode());
        NU32_WriteUART3(buffer);
        break;
      }
      case 'z':
      {
        sprintf(buffer,"%f\r\n", U_check);
        NU32_WriteUART3(buffer);
        // sprintf(buffer,"%d\r\n", Eint_check2);
        // NU32_WriteUART3(buffer);
        sprintf(buffer,"%d\r\n", Eint_check);
        NU32_WriteUART3(buffer);
        sprintf(buffer,"%f\r\n", Error_check);
        NU32_WriteUART3(buffer);
        sprintf(buffer,"%d\r\n", Ref_check);
        NU32_WriteUART3(buffer);
        sprintf(buffer,"%d\r\n", Actual_check);
        NU32_WriteUART3(buffer);
        sprintf(buffer,"%f\r\n", Unew_check);
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
