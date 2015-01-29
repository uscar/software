#ifndef ALTITUDE_HOLD_H_
#define ALTITUDE_HOLD_H_

#include "flight_control.h"
#include "routine.h"

class Altitude_Hold : public Routine {
public:
  Altitude_Hold(Flight_Control* flight_control, int routine_code)
               : Routine(flight_control, routine_code, "ALT_HOLD") { }
  ~Altitude_Hold() { }
  bool ExecuteCycle();
};

#endif
