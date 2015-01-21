#ifndef KILL_MOTORS_H_
#define KILL_MOTORS_H_

#include "routine.h"

class Kill_Motors : public Routine {
public:
  Kill_Motors(Flight_Control* flight_control, int routine_code)
              : Routine(flight_control, routine_code) { }
  ~Kill_Motors() { }

  bool ExecuteCycle() {
    if(flight_control()->is_armed()) {
      flight_control()->arm(false);
      return false;
    }
    return false;
  }
};

#endif
