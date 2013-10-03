/*

#ifndef _PID_H_
#define _PID_H_

class PID;

class PID {
 public:

  PID ();

  union {
    union TYPE_PID_ELEM {
      struct {
        float P, I, D, kP, kI, kD,
              pos_current, pos_desire, cmd_result,
              thres_P, thres_I;
      }; float data[11];
    } vars[6];
    struct { union TYPE_PID_ELEM roll, pitch, yaw, z, x, y; };
  };

  void process () {


  }
} pid;

#endif
*/
