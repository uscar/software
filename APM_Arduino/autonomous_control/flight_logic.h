#ifndef FLIGHT_LOGIC_H_
#define FLIGHT_LOGIC_H_

#include "altitude_hold.h"
#include "flight_control.h"
#include "kill_motors.h"
#include "landing.h"
#include "routine.h"
#include "takeoff.h"

extern const AP_HAL::HAL& hal;

enum Routine_Codes {
  kTAKEOFF, kALT_HOLD, kLANDING, kKILL_MOTORS,
  kDEFAULT_ROUTINE, kROUTINES_SIZE
};

class Flight_Logic {
public:
  Flight_Logic(Flight_Control* flight_control) : flight_control_(flight_control) {
    routine_list = new Routine*[kROUTINES_SIZE];
    routine_list[kTAKEOFF] = new Takeoff(flight_control, kTAKEOFF);
    routine_list[kALT_HOLD] = new Altitude_Hold(flight_control, kALT_HOLD);
    routine_list[kLANDING] = new Landing(flight_control, kLANDING);
    routine_list[kKILL_MOTORS] = new Kill_Motors(flight_control, kKILL_MOTORS);
    routine_list[kDEFAULT_ROUTINE] = routine_list[kALT_HOLD];
    set_curr_routine(kKILL_MOTORS);
  }

  ~Flight_Logic() {
    for(int i = 0; i < kROUTINES_SIZE; ++i) {
      delete routine_list[i];
    } 
    delete [] routine_list;
  }

  void ExecuteCurrRoutine();

  void set_curr_routine(int routine_code);

  Flight_Control* flight_control() const { return flight_control_; }
  Routine* curr_routine() const { return curr_routine_; }
  const char* get_curr_routine_name() const { return curr_routine()->name(); }
private:
  Flight_Control* flight_control_;
  Routine* curr_routine_;
  Routine** routine_list;
};

#endif
