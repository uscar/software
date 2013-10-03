/*

#include "PID.h"

PID::PID () {

  // Set up constants.
  roll.P =
  roll.I =
  roll.D =
  roll.thres_low = ;
  roll.thres_high = ;
  pitch.P =
  pitch.I =
  pitch.D =
  roll.thres_low = ;
  roll.thres_high = ;
  yaw.P =
  yaw.I =
  yaw.D =
  yaw.thres_low = ;
  yaw.thres_high = ;
}
void PID::process (float dt) {
  for (size_t i = 0; i < 6; i++) {
    float tmp_P = vars[i].pos_desire - vars[i].pos_current;
    vars[i].D = (tmp_P - vars[i].P) / dt;
    vars[i].P = tmp_P;
    tmp_P = constrain(tmp_P, - vars[i].thres_P, vars[i].thres_P);
    vars[i].I = constrain(vars[i].I + vars[i].P * dt,
                          - vars[i].thres_I, vars[i].thres_I);

    vars[i].cmd_result = vars[i].P * vars[i].kP + vars[i].I * vars[i].kI
                         + vars[i].D * vars[i]*kD;
}

*/
