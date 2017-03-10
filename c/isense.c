#include "isense.h"
#include "NU32.h"          // constants, functions for startup and UART

#define VOLTS_PER_COUNT (3.3/1024)
#define CORE_TICK_TIME 25    // nanoseconds between core ticks
#define SAMPLE_TIME 10       // 10 core timer ticks = 250 ns

unsigned int ADC_count(void) { // sample & convert the value on the given
                                           // adc pin the pin should be configured as an
                                           // analog input in AD1PCFG
    unsigned int elapsed = 0, finish_time = 0;
    AD1CHSbits.CH0SA = 0;                // connect chosen pin to MUXA for sampling
    AD1CON1bits.SAMP = 1;                  // start sampling
    elapsed = _CP0_GET_COUNT();
    finish_time = elapsed + SAMPLE_TIME;
    while (_CP0_GET_COUNT() < finish_time) {
      ;                                   // sample for more than 250 ns
    }
    AD1CON1bits.SAMP = 0;                 // stop sampling and start converting
    while (!AD1CON1bits.DONE) {
      ;                                   // wait for the conversion process to finish
    }
    return ADC1BUF0;                      // read the buffer with the result
}

float ADC_milliAmps(void) {
  return (1.9186 * ADC_count() - 913.18);
}

void ADC_init() {
  AD1PCFGbits.PCFG0 = 0;                 // AN0 is an adc pin
  AD1CON3bits.ADCS = 2;                   // ADC clock period is Tad = 2*(ADCS+1)*Tpb =
                                          //                           2*3*12.5ns = 75ns
  AD1CON1bits.ADON = 1;                   // turn on A/D converter
}
