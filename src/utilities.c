#include "utilities.h"

char * get_current_mode(void) {
  switch(current_mode) {
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
