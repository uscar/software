#ifndef TAKEOFF_H_
#define TAKEOFF_H_

#include "routine.h"

class Takeoff : public Routine {
public:
  Takeoff(Flight_Control* flight_control, int routine_code)
                                          : Routine(flight_control, routine_code, "TAKEOFF"),
                                            curr_throttle_(MOTOR_MIN), up_cntrl_(0, 0, 1) { };
  ~Takeoff() { };

  bool ExecuteCycle();

  float CalculateThrottle();
  float curr_throttle() { return curr_throttle_; }
  void set_curr_throttle(float curr_throttle) { curr_throttle_ = curr_throttle; }

  Vector3f& up_cntrl() { return up_cntrl_; }
private:
  static const float END_HEIGHT = 1.0;
  float curr_throttle_;
  Vector3f up_cntrl_;
};

#endif
