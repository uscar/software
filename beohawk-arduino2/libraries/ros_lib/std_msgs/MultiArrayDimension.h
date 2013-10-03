#ifndef ros_MultiArrayDimension_h
#define ros_MultiArrayDimension_h

#include "WProgram.h"
#include "ros.h"

namespace std_msgs
{

  class MultiArrayDimension : public ros::Msg
  {
    public:
      unsigned char * label;
      unsigned long size;
      unsigned long stride;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      long * length_label = (long *)(outbuffer + offset);
      *length_label = strlen( (const char*) this->label);
      offset += 4;
      memcpy(outbuffer + offset, this->label, *length_label);
      offset += *length_label;
      union {
        unsigned long real;
        unsigned long base;
      } u_size;
      u_size.real = this->size;
      *(outbuffer + offset + 0) = (u_size.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_size.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_size.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_size.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->size);
      union {
        unsigned long real;
        unsigned long base;
      } u_stride;
      u_stride.real = this->stride;
      *(outbuffer + offset + 0) = (u_stride.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_stride.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_stride.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_stride.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stride);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      long * length_label = (long *)(inbuffer + offset);
      offset += 4;
      this->label = (inbuffer + offset);
      offset += *length_label;
      union {
        unsigned long real;
        unsigned long base;
      } u_size;
      u_size.base = 0;
      u_size.base |= ((typeof(u_size.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_size.base |= ((typeof(u_size.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_size.base |= ((typeof(u_size.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_size.base |= ((typeof(u_size.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->size = u_size.real;
      offset += sizeof(this->size);
      union {
        unsigned long real;
        unsigned long base;
      } u_stride;
      u_stride.base = 0;
      u_stride.base |= ((typeof(u_stride.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_stride.base |= ((typeof(u_stride.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_stride.base |= ((typeof(u_stride.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_stride.base |= ((typeof(u_stride.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->stride = u_stride.real;
      offset += sizeof(this->stride);
     return offset;
    }

    const char * getType(){ return "std_msgs/MultiArrayDimension"; };

  };

}
#endif