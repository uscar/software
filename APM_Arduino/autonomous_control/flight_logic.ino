#include "flight_logic.h"

void Flight_Logic::set_curr_routine(int routine_code) {
  if(routine_code >= kROUTINES_SIZE || routine_code < 0) {
    set_curr_routine(kDEFAULT_ROUTINE);
  }
  else {
    curr_routine_ = routine_list[routine_code];
  }
}

void Flight_Logic::ExecuteCurrRoutine() {
  if(curr_routine().ExecuteCycle()) {
    set_curr_routine(kDEFAULT_ROUTINE);
  }
}