#ifndef ROUTINE_H_
#define ROUTINE_H_

#include <thread>
#include "flight_control.h"

class Routine {
public:
  Routine(Flight_Control* flight_control) : flight_control_(flight_control),
                          finished_(false), interrupted_(false) { }
  virtual ~Routine();
  
  void ExecuteRoutine() {
    
  }

  Flight_Control* flight_control() { return flight_control_; }
  
  void interrupt_routine() {
    interrupted_ = true;
  }

  void set_finished(bool finished) { finished_ = finished; }
  bool finished() { return finished_; }
  bool interrupted() { return interrupted_; }

protected:
  virtual bool ExecuteCycle() = 0;
  
  bool ExecuteFull() {
    while(!interrupted() && !ExecuteCycle());
    set_finished(true);
    return true;
  }

  Flight_Control* flight_control_;
  bool finished_;
  bool interrupted_;
};

#endif
