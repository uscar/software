#ifndef ALTITUDE_HOLD_H_
#define ALTITUDE_HOLD_H_

#include "flight_control.h"
#include "routine.h"

class Altitude_Hold : public Routine {
public:
  Altitude_Hold(Flight_Control* flight_control, int routine_code)
               : Routine(flight_control, routine_code) { }
  ~Altitude_Hold() { }
  bool ExecuteCycle();

  float curr_height() { return curr_height_; }
  void set_curr_height(float curr_height) { curr_height_ = curr_height; }
private:
  float curr_height_;
};

#endif