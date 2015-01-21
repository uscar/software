#ifndef FLIGHT_LOGIC_H_
#define FLIGHT_LOGIC_H_
#define stringify( str ) # str

#include "altitude_hold.h"
#include "flight_control.h"
#include "kill_motors.h"
#include "landing.h"
#include "routine.h"
#include "takeoff.h"

enum Routine_Codes {
  kTAKEOFF, kALT_HOLD, kLANDING, kKILL_MOTORS,
  kDEFAULT_ROUTINE, kROUTINES_SIZE
};

class Flight_Logic {
public:
  Flight_Logic(Flight_Control* flight_control) : flight_control_(flight_control) {
    routine_list = new Routine[kROUTINES_SIZE];
    routine_list[kTAKEOFF] = Takeoff(flight_control, kTAKEOFF);
    routine_list[kALT_HOLD] = Altitude_Hold(flight_control, kALT_HOLD);
    routine_list[kLANDING] = Landing(flight_control, kLANDING);
    routine_list[kKILL_MOTORS] = Kill_Motors(flight_control, kKILL_MOTORS);
    routine_list[kDEFAULT_ROUTINE] = routine_list[kALT_HOLD];
    curr_routine_ = routine_list[kKILL_MOTORS];
  }

  ~Flight_Logic() { delete [] routine_list; }

  void ExecuteCurrRoutine();

  void set_curr_routine(int routine_code);

  Flight_Control* flight_control() { return flight_control_; }
  Routine& curr_routine() { return curr_routine_; }
private:
  Flight_Control* flight_control_;
  Routine curr_routine_;
  Routine* routine_list;
};

#endif
