#include "NU32.h"          // config bits, constants, funcs for startup and UART
#include "encoder.h"
#include "isense.h"
#include "utilities.h"
#include <stdio.h>
// include other header files here

#define BUF_SIZE 200

int main()
{
  char buffer[BUF_SIZE];
  NU32_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
  NU32_LED1 = 1;  // turn off the LEDs
  NU32_LED2 = 1;
  __builtin_disable_interrupts();
  // in future, initialize modules or peripherals here
  encoder_init();
  ADC_init();
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
