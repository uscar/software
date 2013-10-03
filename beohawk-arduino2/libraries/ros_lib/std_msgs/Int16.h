#ifndef ros_Int16_h
#define ros_Int16_h

#include "WProgram.h"
#include "ros.h"

namespace std_msgs
{

  class Int16 : public ros::Msg
  {
    public:
      int data;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      union {
        int real;
        unsigned int base;
      } u_data;
      u_data.real = this->data;
      *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_data.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int real;
        unsigned int base;
      } u_data;
      u_data.base = 0;
      u_data.base |= ((typeof(u_data.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_data.base |= ((typeof(u_data.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      this->data = u_data.real;
      offset += sizeof(this->data);
     return offset;
    }

    const char * getType(){ return "std_msgs/Int16"; };

  };

}
#endif