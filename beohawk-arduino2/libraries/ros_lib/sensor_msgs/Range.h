#ifndef ros_Range_h
#define ros_Range_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/Header.h"

namespace sensor_msgs
{

  class Range : public ros::Msg
  {
    public:
      std_msgs::Header header;
      unsigned char radiation_type;
      float field_of_view;
      float min_range;
      float max_range;
      float range;
      enum { ULTRASOUND = 0 };
      enum { INFRARED = 1 };

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        unsigned char real;
        unsigned char base;
      } u_radiation_type;
      u_radiation_type.real = this->radiation_type;
      *(outbuffer + offset + 0) = (u_radiation_type.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->radiation_type);
      union {
        float real;
        unsigned long base;
      } u_field_of_view;
      u_field_of_view.real = this->field_of_view;
      *(outbuffer + offset + 0) = (u_field_of_view.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_field_of_view.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_field_of_view.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_field_of_view.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->field_of_view);
      union {
        float real;
        unsigned long base;
      } u_min_range;
      u_min_range.real = this->min_range;
      *(outbuffer + offset + 0) = (u_min_range.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_min_range.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_min_range.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_min_range.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->min_range);
      union {
        float real;
        unsigned long base;
      } u_max_range;
      u_max_range.real = this->max_range;
      *(outbuffer + offset + 0) = (u_max_range.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_max_range.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_max_range.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_max_range.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->max_range);
      union {
        float real;
        unsigned long base;
      } u_range;
      u_range.real = this->range;
      *(outbuffer + offset + 0) = (u_range.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_range.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_range.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_range.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->range);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        unsigned char real;
        unsigned char base;
      } u_radiation_type;
      u_radiation_type.base = 0;
      u_radiation_type.base |= ((typeof(u_radiation_type.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      this->radiation_type = u_radiation_type.real;
      offset += sizeof(this->radiation_type);
      union {
        float real;
        unsigned long base;
      } u_field_of_view;
      u_field_of_view.base = 0;
      u_field_of_view.base |= ((typeof(u_field_of_view.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_field_of_view.base |= ((typeof(u_field_of_view.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_field_of_view.base |= ((typeof(u_field_of_view.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_field_of_view.base |= ((typeof(u_field_of_view.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->field_of_view = u_field_of_view.real;
      offset += sizeof(this->field_of_view);
      union {
        float real;
        unsigned long base;
      } u_min_range;
      u_min_range.base = 0;
      u_min_range.base |= ((typeof(u_min_range.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_min_range.base |= ((typeof(u_min_range.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_min_range.base |= ((typeof(u_min_range.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_min_range.base |= ((typeof(u_min_range.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->min_range = u_min_range.real;
      offset += sizeof(this->min_range);
      union {
        float real;
        unsigned long base;
      } u_max_range;
      u_max_range.base = 0;
      u_max_range.base |= ((typeof(u_max_range.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_max_range.base |= ((typeof(u_max_range.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_max_range.base |= ((typeof(u_max_range.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_max_range.base |= ((typeof(u_max_range.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->max_range = u_max_range.real;
      offset += sizeof(this->max_range);
      union {
        float real;
        unsigned long base;
      } u_range;
      u_range.base = 0;
      u_range.base |= ((typeof(u_range.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_range.base |= ((typeof(u_range.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_range.base |= ((typeof(u_range.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_range.base |= ((typeof(u_range.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->range = u_range.real;
      offset += sizeof(this->range);
     return offset;
    }

    const char * getType(){ return "sensor_msgs/Range"; };

  };

}
#endif