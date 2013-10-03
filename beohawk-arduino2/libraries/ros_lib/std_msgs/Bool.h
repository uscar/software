#ifndef ros_Bool_h
#define ros_Bool_h

#include "WProgram.h"
#include "ros.h"

namespace std_msgs
{

  class Bool : public ros::Msg
  {
    public:
      bool data;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      union {
        bool real;
        unsigned char base;
      } u_data;
      u_data.real = this->data;
      *(outbuffer + offset + 0) = (u_data.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->data);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        unsigned char base;
      } u_data;
      u_data.base = 0;
      u_data.base |= ((typeof(u_data.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      this->data = u_data.real;
      offset += sizeof(this->data);
     return offset;
    }

    const char * getType(){ return "std_msgs/Bool"; };

  };

}
#endif