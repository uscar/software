#ifndef ros_UInt64_h
#define ros_UInt64_h

#include "WProgram.h"
#include "ros.h"

namespace std_msgs
{

  class UInt64 : public ros::Msg
  {
    public:
      long data;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      *(outbuffer + offset++) = (data >> (8 * 0)) & 0xFF;
      *(outbuffer + offset++) = (data >> (8 * 1)) & 0xFF;
      *(outbuffer + offset++) = (data >> (8 * 2)) & 0xFF;
      *(outbuffer + offset++) = (data >> (8 * 3)) & 0xFF;
      *(outbuffer + offset++) = (data > 0) ? 0: 255;
      *(outbuffer + offset++) = (data > 0) ? 0: 255;
      *(outbuffer + offset++) = (data > 0) ? 0: 255;
      *(outbuffer + offset++) = (data > 0) ? 0: 255;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      data = 0;
      data += ((long)(*(inbuffer + offset++))) >> (8 * 0);
      data += ((long)(*(inbuffer + offset++))) >> (8 * 1);
      data += ((long)(*(inbuffer + offset++))) >> (8 * 2);
      data += ((long)(*(inbuffer + offset++))) >> (8 * 3);
      offset += 4;
     return offset;
    }

    const char * getType(){ return "std_msgs/UInt64"; };

  };

}
#endif