#ifndef TAKEOFF_H_
#define TAKEOFF_H_

#define END_HEIGHT 1

#include "routine.h"

class Takeoff : public Routine {
public:
  Takeoff(Flight_Control* flight_control) : Routine(flight_control),
                                            curr_height_(0),
                                            up_cmd_(0, 0, 1) { };
  ~Takeoff() { };
  
  bool ExecuteCycle();

  float curr_height() { return curr_height_; }
  void set_curr_height(float curr_height) { curr_height_ = curr_height; }
  Vector3f& up_cmd() { return up_cmd_; }

private:
  float curr_height_;
  Vector3f up_cmd_;
};

#endif