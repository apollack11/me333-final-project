#include <stdio.h>
#include "NU32.h"          // constants, functions for startup and UART
#include "LCD.h"
#define NUMSAMPS 1000      // number of points in the waveform
#define PLOTPTS 200        // number of data points to plot
#define DECIMATION 10      // plot every 10th point
#define SAMPLE_TIME 10     // 10 core timer ticks = 250ns

void printGainsToLCD(void);

static volatile int Waveform[NUMSAMPS]; // waveform
static volatile int ADCarray[PLOTPTS];  // measured values to plot
static volatile int REFarray[PLOTPTS];  // reference values to plot
static volatile int StoringData = 0;    // if this flag = 1, currently storing plot data
static volatile float Kp = 0, Ki = 0;   // control gains
static volatile int Eint;

void printGainsToLCD() {
  LCD_Clear();                              // clear LCD screen
  LCD_Move(0,0);
  char msg[100];
  sprintf(msg, "Kp = %f", Kp);
  LCD_WriteString(msg);                     // write msg at row 0 col 0
  LCD_Move(1,0);
  sprintf(msg, "Ki = %f", Ki);
  LCD_WriteString(msg);                     // write msg at row 0 col 0
}

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Controller(void) { // _TIMER_2_VECTOR = 8
    static int counter = 0;    // initialize the counter once
    static int plotind = 0;    // index for data arrays; counts up to PLOTPTS
    static int decctr = 0;     // counts to store data once every DECIMATION
    static int adcval = 0;

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
    adcval = ADC1BUF0;                    // read the buffer with the result

    // PI Controller
    float error = Waveform[counter] - adcval;
    Eint = Eint + error;
    float u = Kp * error + Ki * Eint;

    float unew = u + 50.0;
    if (unew > 100.0) {
      unew = 100.0;
    } else if (unew < 0.0) {
      unew = 0.0;
    }

    OC1RS = (unsigned int) ((unew / 100.0) * 3999);

    if (StoringData) {
      decctr++;
      if (decctr == DECIMATION) {       // after DECIMATION control loops
        decctr = 0;                     // reset decimation counter
        ADCarray[plotind] = adcval;     // store data in global arrays
        REFarray[plotind] = Waveform[counter];
        plotind++;                      // increment plot data index
      }
      if (plotind == PLOTPTS) {         // if max number of plot points is reached,
        plotind = 0;                    // reset the plot index
        StoringData = 0;                // tell main data is ready to be sent to matlab
      }
    }
    counter++;                  // add one to the counter every time ISR is entered
    if (counter == NUMSAMPS) {
      counter = 0;              // roll the counter over when needed
    }
    IFS0bits.T2IF = 0;
}

void makeWaveform() {
  int i = 0, center = 500, A = 300; // square wave
  for (i = 0; i < NUMSAMPS; ++i) {
    if (i < NUMSAMPS/2) {
      Waveform[i] = center + A;
    } else {
      Waveform[i] = center - A;
    }
  }
}

int main(void) {
  NU32_Startup();          // cache on, interrupts on, LED/button init, UART init
  LCD_Setup();

  AD1PCFGbits.PCFG0 = 0;
  AD1CON3bits.ADCS = 2;
  AD1CON1bits.ADON = 1;

  char message[100];             // message to and from matlab
  float kptemp = 0, kitemp = 0;  // temporary local gains
  int i = 0;                     // plot data loop counter

  makeWaveform();
  T3CONbits.TCKPS = 0b000;     // Timer3 prescaler N=1 (1:4)
  PR3 = 3999;              // period = (PR2+1) * N * 12.5 ns = 50 us, 20 kHz
  TMR3 = 0;                // initial TMR2 count is 0
  OC1CONbits.OCTSEL = 1;
  OC1CONbits.OCM = 0b110;  // PWM mode without fault pin; other OC1CON bits are defaults
  OC1RS = 3000;             // duty cycle = OC1RS/(PR2+1) = 75%
  OC1R = 500;              // initialize before turning OC1 on; afterward it is read-only
  T3CONbits.ON = 1;        // turn on Timer3
  OC1CONbits.ON = 1;       // turn on OC1

  __builtin_disable_interrupts(); // step 2: disable interrupts
  T2CONbits.TCKPS = 2;            // Timer2 prescaler N=4 (1:4)
  PR2 = 19999;
  TMR2 = 0;
  T2CONbits.ON = 1;
  IPC2bits.T2IP = 5;            // step 4: interrupt priority 2
  IPC2bits.T2IS = 0;            // step 4: interrupt priority 1
  IFS0bits.T2IF = 0;            // step 5: clear the flag
  IFS0bits.T2IF = 1;            // step 5: set the flag
  IEC0bits.T2IE = 1;            // step 6: enable T2
  __builtin_enable_interrupts();  // step 7: enable interrupts

  while(1) {
    NU32_ReadUART3(message, sizeof(message)); // wait for message from matlab
    sscanf(message, "%f %f", &kptemp, &kitemp);
    __builtin_disable_interrupts(); // keep ISR disabled as briefly as possible
    Kp = kptemp;                    // copy local variables to globals used by ISR
    Ki = kitemp;
    __builtin_enable_interrupts();  // only 2 simple C commands while ISRs disabled
    Eint = 0;
    StoringData = 1;                // message to ISR to start storing data
    printGainsToLCD();
    while (StoringData) {
      ; // do nothing
    }
    for (i = 0; i < PLOTPTS; i++) { // send plot data to matlab
      sprintf(message, "%d %d %d\r\n", PLOTPTS-i, ADCarray[i], REFarray[i]);
      NU32_WriteUART3(message);
    }
  }
  return 0;
}
