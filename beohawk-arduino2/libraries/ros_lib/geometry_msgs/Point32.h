#ifndef ros_Point32_h
#define ros_Point32_h

#include "WProgram.h"
#include "ros.h"

namespace geometry_msgs
{

  class Point32 : public ros::Msg
  {
    public:
      float x;
      float y;
      float z;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      union {
        float real;
        unsigned long base;
      } u_x;
      u_x.real = this->x;
      *(outbuffer + offset + 0) = (u_x.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_x.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_x.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_x.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->x);
      union {
        float real;
        unsigned long base;
      } u_y;
      u_y.real = this->y;
      *(outbuffer + offset + 0) = (u_y.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_y.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_y.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_y.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->y);
      union {
        float real;
        unsigned long base;
      } u_z;
      u_z.real = this->z;
      *(outbuffer + offset + 0) = (u_z.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_z.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_z.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_z.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        unsigned long base;
      } u_x;
      u_x.base = 0;
      u_x.base |= ((typeof(u_x.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_x.base |= ((typeof(u_x.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_x.base |= ((typeof(u_x.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_x.base |= ((typeof(u_x.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->x = u_x.real;
      offset += sizeof(this->x);
      union {
        float real;
        unsigned long base;
      } u_y;
      u_y.base = 0;
      u_y.base |= ((typeof(u_y.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_y.base |= ((typeof(u_y.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_y.base |= ((typeof(u_y.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_y.base |= ((typeof(u_y.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->y = u_y.real;
      offset += sizeof(this->y);
      union {
        float real;
        unsigned long base;
      } u_z;
      u_z.base = 0;
      u_z.base |= ((typeof(u_z.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_z.base |= ((typeof(u_z.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_z.base |= ((typeof(u_z.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_z.base |= ((typeof(u_z.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->z = u_z.real;
      offset += sizeof(this->z);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/Point32"; };

  };

}
#endif