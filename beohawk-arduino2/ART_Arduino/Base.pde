#include "Base.h"

void Base::init () {
  isr_registry.init();
  adc_scheduler.init(&isr_registry);
}
