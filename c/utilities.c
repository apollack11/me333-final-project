#include "utilities.h"

static volatile Mode mode = IDLE;

int get_mode(void) {
  return mode;
}

char * get_string_mode(void) {
  switch(mode) {
    case IDLE: {
      return "IDLE";
      break;
    }
    case PWM: {
      return "PWM";
      break;
    }
    case ITEST: {
      return "ITEST";
      break;
    }
    case HOLD: {
      return "HOLD";
      break;
    }
    case TRACK: {
      return "TRACK";
      break;
    }
  }
}

void set_mode(Mode m) {
  __builtin_disable_interrupts();
  mode = m;
  __builtin_enable_interrupts();
}
