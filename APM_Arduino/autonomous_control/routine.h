#ifndef ROUTINE_H_
#define ROUTINE_H_

#include <AP_Baro.h>
#include "flight_control.h"

extern const AP_HAL::HAL& hal;
extern AP_Baro_MS5611 baro; 

class Routine {
public:
  Routine(Flight_Control* flight_control, const int routine_code, const char* name) :
                          flight_control_(flight_control),
                          routine_code_(routine_code),
                          name_(name) { }
  virtual ~Routine() {
    delete name_;
  }
  

  /** Returns true if finished with execution 
    * (i.e should set to default routine), false otherwise **/
  virtual bool ExecuteCycle() = 0;

  /** Executes cycles until it returns true. Not recommended b/c uninterruptable **/
  bool ExecuteFull() {
    while(!ExecuteCycle());
    return true;
  }

  /** Simplistic update to current height.
    * Might want to move to flight_control class for consistency between routines. **/
  void update_curr_height() {
    baro.read();
    curr_height_ = baro.get_altitude();
  }

  float curr_height() { return curr_height_; }

  Flight_Control* flight_control() const { return flight_control_; }
  const char* name() const { return name_; };
  const int routine_code() const { return routine_code_; }
protected:
  float curr_height_;
  Flight_Control* flight_control_;
  const int routine_code_;
  const char* name_;
};

#endif
