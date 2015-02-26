#include "flight_logic.h"

void Flight_Logic::set_curr_routine(int routine_code) {
  if(routine_code >= kROUTINES_SIZE || routine_code < 0) {
    curr_routine_ = routine_list[kDEFAULT_ROUTINE];
  }
  else {
    curr_routine_ = routine_list[routine_code];
  }
  hal.console->print("Switching to routine: "); hal.console->println(curr_routine()->name());
}

void Flight_Logic::ExecuteCurrRoutine() {
  if(curr_routine()->ExecuteCycle()) {
    if(curr_routine()->routine_code() == kLANDING) {
      set_curr_routine(kKILL_MOTORS); 
    }
    else {
      set_curr_routine(kDEFAULT_ROUTINE);
    }
  }
}
