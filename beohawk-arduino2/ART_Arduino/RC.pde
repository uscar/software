#include "RC.h"

void RC::init () {
  apm.Init(&base.isr_registry);
}

void RC::get_pilot () {
  
  // Read raw inputs and filter, so that they won't go crazy.
  for (size_t i = 0; i < 8; i++) {
    uint32_t input_new = apm.InputCh(i);
    uint32_t diff_input = input_new - inputs_raw[i];
    if (diff_input == input_new)
      break;
    else if (diff_input > 40) {
      inputs_raw[i] += 40; return;
    } else if (diff_input < -40) {
      inputs_raw[i] -= 40; return;
    } inputs_raw[i] = input_new;
  }

  // Calc pilot commands.
  pilot.roll = 0.15 * (inputs_raw[0] - RC_MIDCHANNEL);
  pilot.pitch = -0.15 * (inputs_raw[1] - RC_MIDCHANNEL);
  pilot.yaw = (inputs_raw[3] - RC_MIDCHANNEL);
  pilot.throttle = inputs_raw[2];
}

void RC::set_motor (uint32_t cmd_ch1, uint32_t cmd_ch2,
                    uint32_t cmd_ch3, uint32_t cmd_ch4) {
  apm.OutputCh(0, cmd_ch1);
  apm.OutputCh(1, cmd_ch2);
  apm.OutputCh(2, cmd_ch3);
  apm.OutputCh(3, cmd_ch4);
}
