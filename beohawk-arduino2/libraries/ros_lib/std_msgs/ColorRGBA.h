#ifndef ros_ColorRGBA_h
#define ros_ColorRGBA_h

#include "WProgram.h"
#include "ros.h"

namespace std_msgs
{

  class ColorRGBA : public ros::Msg
  {
    public:
      float r;
      float g;
      float b;
      float a;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      union {
        float real;
        unsigned long base;
      } u_r;
      u_r.real = this->r;
      *(outbuffer + offset + 0) = (u_r.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_r.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_r.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_r.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->r);
      union {
        float real;
        unsigned long base;
      } u_g;
      u_g.real = this->g;
      *(outbuffer + offset + 0) = (u_g.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_g.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_g.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_g.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->g);
      union {
        float real;
        unsigned long base;
      } u_b;
      u_b.real = this->b;
      *(outbuffer + offset + 0) = (u_b.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_b.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_b.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_b.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->b);
      union {
        float real;
        unsigned long base;
      } u_a;
      u_a.real = this->a;
      *(outbuffer + offset + 0) = (u_a.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_a.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_a.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_a.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->a);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        float real;
        unsigned long base;
      } u_r;
      u_r.base = 0;
      u_r.base |= ((typeof(u_r.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_r.base |= ((typeof(u_r.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_r.base |= ((typeof(u_r.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_r.base |= ((typeof(u_r.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->r = u_r.real;
      offset += sizeof(this->r);
      union {
        float real;
        unsigned long base;
      } u_g;
      u_g.base = 0;
      u_g.base |= ((typeof(u_g.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_g.base |= ((typeof(u_g.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_g.base |= ((typeof(u_g.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_g.base |= ((typeof(u_g.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->g = u_g.real;
      offset += sizeof(this->g);
      union {
        float real;
        unsigned long base;
      } u_b;
      u_b.base = 0;
      u_b.base |= ((typeof(u_b.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_b.base |= ((typeof(u_b.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_b.base |= ((typeof(u_b.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_b.base |= ((typeof(u_b.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->b = u_b.real;
      offset += sizeof(this->b);
      union {
        float real;
        unsigned long base;
      } u_a;
      u_a.base = 0;
      u_a.base |= ((typeof(u_a.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_a.base |= ((typeof(u_a.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_a.base |= ((typeof(u_a.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_a.base |= ((typeof(u_a.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->a = u_a.real;
      offset += sizeof(this->a);
     return offset;
    }

    const char * getType(){ return "std_msgs/ColorRGBA"; };

  };

}
#endif