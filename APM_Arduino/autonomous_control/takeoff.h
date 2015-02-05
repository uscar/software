#ifndef TAKEOFF_H_
#define TAKEOFF_H_

#include "routine.h"

class Takeoff : public Routine {
public:
  Takeoff(Flight_Control* flight_control, int routine_code)
                                          : Routine(flight_control, routine_code, "TAKEOFF"),
                                            up_cntrl_(0, 0, 1) { };
  ~Takeoff() { };
  
  bool ExecuteCycle();

  int CalculateThrust();
 
  Vector3f& up_cntrl() { return up_cntrl_; }
private:
  static constexpr float END_HEIGHT = 1.0; 
  Vector3f up_cntrl_;
};

#endif
