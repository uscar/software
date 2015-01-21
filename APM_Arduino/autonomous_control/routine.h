#ifndef ROUTINE_H_
#define ROUTINE_H_

#include "flight_control.h"

class Routine {
public:
  Routine(Flight_Control* flight_control, int routine_code) :
                          flight_control_(flight_control),
                          finished_(false), routine_code_(routine_code) { }
  Routine() : finished_(false), routine_code_(-1) { }

  virtual ~Routine() { }
  
  /** Executes cycles until it returns true. Not recommended b/c uninterruptable **/
  bool ExecuteFull() {
    while(!ExecuteCycle());
    set_finished(true);
    return true;
  }

  /** Returns true if finished with execution, false otherwise **/
  virtual bool ExecuteCycle() { return false; };
  
  void set_finished(bool finished) { finished_ = finished; }
  bool finished() { return finished_; }

  Flight_Control* flight_control() const { return flight_control_; }
  
  int routine_code() const { return routine_code_; }
protected:
  bool finished_;
  Flight_Control* flight_control_;
  int routine_code_;
};

#endif
