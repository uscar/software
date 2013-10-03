#ifndef _RC_H_
#define _RC_H_

class RC;

#define RC_MINTHROTTLE 1100
#define RC_MIDCHANNEL 1500

class RC {
 public:
   
  APM_RC_APM1 apm;

  uint32_t inputs_raw[8];
  struct {
    uint32_t roll, pitch, yaw, throttle;
  } pilot;

  void init ();

  void get_pilot ();

  void set_motor (uint32_t cmd_ch1, uint32_t cmd_ch2,
                  uint32_t cmd_ch3, uint32_t cmd_ch4);

} rc;

#endif
