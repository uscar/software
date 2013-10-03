#ifndef _BASE_H_
#define _BASE_H_

class Base;

#include "IMU.h"

class Base {
 public:

  Arduino_Mega_ISR_Registry isr_registry;
  AP_TimerProcess  adc_scheduler;

  boolean motors_armed;

  void init (); 
} base;

#endif
