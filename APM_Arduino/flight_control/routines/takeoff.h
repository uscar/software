#ifndef TAKEOFF_H_
#define TAKEOFF_H_

#include "routine.h"

class Takeoff : public Routine {
public:
  Takeoff(Flight_Control* flight_control) : Routine(flight_control) { }
  ~Takeoff();
};

#endif