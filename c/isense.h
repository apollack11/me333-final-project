#ifndef ISENSE__H__
#define ISENSE__H__

void ADC_init();          // initialize the encoder module

unsigned int ADC_count();          // read the ADC, in counts

float ADC_milliAmps();          // read the ADC current in mA

#endif
