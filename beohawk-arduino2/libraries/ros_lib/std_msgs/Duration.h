#ifndef ros_Duration_h
#define ros_Duration_h

#include "WProgram.h"
#include "ros.h"
#include "ros/duration.h"

namespace std_msgs
{

  class Duration : public ros::Msg
  {
    public:
      ros::Duration data;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      union {
        unsigned long real;
        unsigned long base;
      } u_sec;
      u_sec.real = this->data.sec;
      *(outbuffer + offset + 0) = (u_sec.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sec.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sec.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sec.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data.sec);
      union {
        unsigned long real;
        unsigned long base;
      } u_nsec;
      u_nsec.real = this->data.nsec;
      *(outbuffer + offset + 0) = (u_nsec.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nsec.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nsec.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nsec.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->data.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        unsigned long real;
        unsigned long base;
      } u_sec;
      u_sec.base = 0;
      u_sec.base |= ((typeof(u_sec.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sec.base |= ((typeof(u_sec.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sec.base |= ((typeof(u_sec.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sec.base |= ((typeof(u_sec.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data.sec = u_sec.real;
      offset += sizeof(this->data.sec);
      union {
        unsigned long real;
        unsigned long base;
      } u_nsec;
      u_nsec.base = 0;
      u_nsec.base |= ((typeof(u_nsec.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_nsec.base |= ((typeof(u_nsec.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_nsec.base |= ((typeof(u_nsec.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_nsec.base |= ((typeof(u_nsec.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->data.nsec = u_nsec.real;
      offset += sizeof(this->data.nsec);
     return offset;
    }

    const char * getType(){ return "std_msgs/Duration"; };

  };

}
#endif