#ifndef ros_PointField_h
#define ros_PointField_h

#include "WProgram.h"
#include "ros.h"

namespace sensor_msgs
{

  class PointField : public ros::Msg
  {
    public:
      unsigned char * name;
      unsigned long offset;
      unsigned char datatype;
      unsigned long count;
      enum { INT8 = 1 };
      enum { UINT8 = 2 };
      enum { INT16 = 3 };
      enum { UINT16 = 4 };
      enum { INT32 = 5 };
      enum { UINT32 = 6 };
      enum { FLOAT32 = 7 };
      enum { FLOAT64 = 8 };

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      long * length_name = (long *)(outbuffer + offset);
      *length_name = strlen( (const char*) this->name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, *length_name);
      offset += *length_name;
      union {
        unsigned long real;
        unsigned long base;
      } u_offset;
      u_offset.real = this->offset;
      *(outbuffer + offset + 0) = (u_offset.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_offset.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_offset.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_offset.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->offset);
      union {
        unsigned char real;
        unsigned char base;
      } u_datatype;
      u_datatype.real = this->datatype;
      *(outbuffer + offset + 0) = (u_datatype.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->datatype);
      union {
        unsigned long real;
        unsigned long base;
      } u_count;
      u_count.real = this->count;
      *(outbuffer + offset + 0) = (u_count.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_count.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_count.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_count.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->count);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      long * length_name = (long *)(inbuffer + offset);
      offset += 4;
      this->name = (inbuffer + offset);
      offset += *length_name;
      union {
        unsigned long real;
        unsigned long base;
      } u_offset;
      u_offset.base = 0;
      u_offset.base |= ((typeof(u_offset.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_offset.base |= ((typeof(u_offset.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_offset.base |= ((typeof(u_offset.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_offset.base |= ((typeof(u_offset.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->offset = u_offset.real;
      offset += sizeof(this->offset);
      union {
        unsigned char real;
        unsigned char base;
      } u_datatype;
      u_datatype.base = 0;
      u_datatype.base |= ((typeof(u_datatype.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      this->datatype = u_datatype.real;
      offset += sizeof(this->datatype);
      union {
        unsigned long real;
        unsigned long base;
      } u_count;
      u_count.base = 0;
      u_count.base |= ((typeof(u_count.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_count.base |= ((typeof(u_count.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_count.base |= ((typeof(u_count.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_count.base |= ((typeof(u_count.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->count = u_count.real;
      offset += sizeof(this->count);
     return offset;
    }

    const char * getType(){ return "sensor_msgs/PointField"; };

  };

}
#endif