#ifndef ROUTINE_H_
#define ROUTINE_H_
#include "../flight_control.h"

class Routine {
public:
  Routine(Flight_Control* flight_control) : flight_control_(flight_control), finished_(false) { }
  virtual ~Routine();
  virtual bool ExecuteFull() = 0;
  virtual bool ExecuteCycle() = 0;

  Flight_Control* flight_control() { return flight_control_; }
  
  void set_finished(bool finished) { finished_ = finished; }
  bool finished() { return finished_; }
protected:
  Flight_Control* flight_control_;
  bool finished_;
};

#endif