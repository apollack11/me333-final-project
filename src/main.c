#include "NU32.h"          // config bits, constants, funcs for startup and UART
#include "encoder.h"
#include "isense.h"
#include "utilities.h"
#include "currentcontrol.h"
#include <stdio.h>
// include other header files here

#define BUF_SIZE 200
#define NUMSAMPS 1000      // number of points in the waveform
#define PLOTPTS 200        // number of data points to plot
#define DECIMATION 10      // plot every 10th point
#define SAMPLE_TIME 10     // 10 core timer ticks = 250ns

static volatile int Speed;

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Controller(void) { // _TIMER_2_VECTOR = 8
    switch (current_mode) {
      case IDLE: {
        LATDbits.LATD5 = 0;
        OC1RS = 0;
        break;
      }
      case PWM: {
        if (Speed <= 0) {
          LATDbits.LATD5 = 0;
          OC1RS = (unsigned int) ((Speed / 100.0) * PR3);
        } else {
          LATDbits.LATD5 = 1;
          OC1RS = (unsigned int) ((Speed / 100.0) * PR3);
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
        sprintf(buffer,"%d\r\n", Speed); // return encoder angle (should be 0.0)
        NU32_WriteUART3(buffer);
        break;
      }
      case 'p':
      {
        current_mode = IDLE;
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
      default:
      {
        NU32_LED2 = 0;  // turn on LED2 to indicate an error
        break;
      }
    }
  }
  return 0;
}
