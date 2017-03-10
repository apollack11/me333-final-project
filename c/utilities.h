#ifndef UTILITIES__H__
#define UTILITIES__H__

typedef enum {
  IDLE,
  PWM,
  ITEST,
  HOLD,
  TRACK
} Mode;

// Mode current_mode;

int get_mode();

char * get_string_mode();

void set_mode(Mode m);

#endif
