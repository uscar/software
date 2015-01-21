#ifndef TAKEOFF_H_
#define TAKEOFF_H_

#include "routine.h"
#include <AP_Baro.h>

extern AP_Baro_MS5611 baro;

class Takeoff : public Routine {
public:
  Takeoff(Flight_Control* flight_control, int routine_code)
                                          : Routine(flight_control, routine_code),
                                            curr_height_(0),
                                            up_cntrl_(0, 0, 1) { };
  ~Takeoff() { };
  
  bool ExecuteCycle();

  int CalculateThrust();

  float curr_height() { return curr_height_; }
  void set_curr_height(float curr_height) { curr_height_ = curr_height; }
  Vector3f& up_cntrl() { return up_cntrl_; }

private:
  static const float END_HEIGHT = 1; 
  float curr_height_;
  Vector3f up_cntrl_;
};

#endif
