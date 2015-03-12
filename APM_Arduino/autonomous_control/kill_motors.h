#ifndef KILL_MOTORS_H_
#define KILL_MOTORS_H_

#include "routine.h"

class Kill_Motors : public Routine {
public:
  Kill_Motors(Flight_Control* flight_control, int routine_code)
              : Routine(flight_control, routine_code, "KILL_MOTORS"), cntrl_up_(0, 0, 1) { }
  ~Kill_Motors() { }

  bool ExecuteCycle() {
    if(flight_control()->armed()) {
      flight_control()->set_armed(false);
      flight_control()->execute(cntrl_up(), 0, 0);
      return false;
    }
    return false;
  }

  Vector3f& cntrl_up() { return cntrl_up_; }
private:
  Vector3f cntrl_up_;
};
#endif
