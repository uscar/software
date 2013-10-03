#ifndef ros_RegionOfInterest_h
#define ros_RegionOfInterest_h

#include "WProgram.h"
#include "ros.h"

namespace sensor_msgs
{

  class RegionOfInterest : public ros::Msg
  {
    public:
      unsigned long x_offset;
      unsigned long y_offset;
      unsigned long height;
      unsigned long width;
      bool do_rectify;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      union {
        unsigned long real;
        unsigned long base;
      } u_x_offset;
      u_x_offset.real = this->x_offset;
      *(outbuffer + offset + 0) = (u_x_offset.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x_offset.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x_offset.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x_offset.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x_offset);
      union {
        unsigned long real;
        unsigned long base;
      } u_y_offset;
      u_y_offset.real = this->y_offset;
      *(outbuffer + offset + 0) = (u_y_offset.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y_offset.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y_offset.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y_offset.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y_offset);
      union {
        unsigned long real;
        unsigned long base;
      } u_height;
      u_height.real = this->height;
      *(outbuffer + offset + 0) = (u_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->height);
      union {
        unsigned long real;
        unsigned long base;
      } u_width;
      u_width.real = this->width;
      *(outbuffer + offset + 0) = (u_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_width.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->width);
      union {
        bool real;
        unsigned char base;
      } u_do_rectify;
      u_do_rectify.real = this->do_rectify;
      *(outbuffer + offset + 0) = (u_do_rectify.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->do_rectify);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        unsigned long real;
        unsigned long base;
      } u_x_offset;
      u_x_offset.base = 0;
      u_x_offset.base |= ((typeof(u_x_offset.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x_offset.base |= ((typeof(u_x_offset.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x_offset.base |= ((typeof(u_x_offset.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x_offset.base |= ((typeof(u_x_offset.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x_offset = u_x_offset.real;
      offset += sizeof(this->x_offset);
      union {
        unsigned long real;
        unsigned long base;
      } u_y_offset;
      u_y_offset.base = 0;
      u_y_offset.base |= ((typeof(u_y_offset.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y_offset.base |= ((typeof(u_y_offset.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y_offset.base |= ((typeof(u_y_offset.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y_offset.base |= ((typeof(u_y_offset.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y_offset = u_y_offset.real;
      offset += sizeof(this->y_offset);
      union {
        unsigned long real;
        unsigned long base;
      } u_height;
      u_height.base = 0;
      u_height.base |= ((typeof(u_height.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_height.base |= ((typeof(u_height.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_height.base |= ((typeof(u_height.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_height.base |= ((typeof(u_height.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->height = u_height.real;
      offset += sizeof(this->height);
      union {
        unsigned long real;
        unsigned long base;
      } u_width;
      u_width.base = 0;
      u_width.base |= ((typeof(u_width.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_width.base |= ((typeof(u_width.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_width.base |= ((typeof(u_width.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_width.base |= ((typeof(u_width.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->width = u_width.real;
      offset += sizeof(this->width);
      union {
        bool real;
        unsigned char base;
      } u_do_rectify;
      u_do_rectify.base = 0;
      u_do_rectify.base |= ((typeof(u_do_rectify.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      this->do_rectify = u_do_rectify.real;
      offset += sizeof(this->do_rectify);
     return offset;
    }

    const char * getType(){ return "sensor_msgs/RegionOfInterest"; };

  };

}
#endif