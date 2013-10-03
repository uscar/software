#ifndef ros_Header_h
#define ros_Header_h

#include "WProgram.h"
#include "ros.h"
#include "ros/time.h"

namespace std_msgs
{

  class Header : public ros::Msg
  {
    public:
      unsigned long seq;
      ros::Time stamp;
      unsigned char * frame_id;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      union {
        unsigned long real;
        unsigned long base;
      } u_seq;
      u_seq.real = this->seq;
      *(outbuffer + offset + 0) = (u_seq.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_seq.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_seq.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_seq.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->seq);
      union {
        unsigned long real;
        unsigned long base;
      } u_sec;
      u_sec.real = this->stamp.sec;
      *(outbuffer + offset + 0) = (u_sec.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_sec.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_sec.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_sec.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      union {
        unsigned long real;
        unsigned long base;
      } u_nsec;
      u_nsec.real = this->stamp.nsec;
      *(outbuffer + offset + 0) = (u_nsec.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nsec.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nsec.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nsec.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      long * length_frame_id = (long *)(outbuffer + offset);
      *length_frame_id = strlen( (const char*) this->frame_id);
      offset += 4;
      memcpy(outbuffer + offset, this->frame_id, *length_frame_id);
      offset += *length_frame_id;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        unsigned long real;
        unsigned long base;
      } u_seq;
      u_seq.base = 0;
      u_seq.base |= ((typeof(u_seq.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_seq.base |= ((typeof(u_seq.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_seq.base |= ((typeof(u_seq.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_seq.base |= ((typeof(u_seq.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->seq = u_seq.real;
      offset += sizeof(this->seq);
      union {
        unsigned long real;
        unsigned long base;
      } u_sec;
      u_sec.base = 0;
      u_sec.base |= ((typeof(u_sec.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_sec.base |= ((typeof(u_sec.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_sec.base |= ((typeof(u_sec.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_sec.base |= ((typeof(u_sec.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->stamp.sec = u_sec.real;
      offset += sizeof(this->stamp.sec);
      union {
        unsigned long real;
        unsigned long base;
      } u_nsec;
      u_nsec.base = 0;
      u_nsec.base |= ((typeof(u_nsec.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_nsec.base |= ((typeof(u_nsec.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_nsec.base |= ((typeof(u_nsec.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_nsec.base |= ((typeof(u_nsec.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->stamp.nsec = u_nsec.real;
      offset += sizeof(this->stamp.nsec);
      long * length_frame_id = (long *)(inbuffer + offset);
      offset += 4;
      this->frame_id = (inbuffer + offset);
      offset += *length_frame_id;
     return offset;
    }

    const char * getType(){ return "std_msgs/Header"; };

  };

}
#endif