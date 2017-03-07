#ifndef UTILITIES__H__
#define UTILITIES__H__

typedef enum {
  IDLE,
  PWM,
  ITEST,
  HOLD,
  TRACK
} Mode;

Mode current_mode;

char * get_current_mode();

#endif
