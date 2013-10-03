#ifndef ros_Empty_h
#define ros_Empty_h

#include "WProgram.h"
#include "ros.h"

namespace std_msgs
{

  class Empty : public ros::Msg
  {
    public:

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return "std_msgs/Empty"; };

  };

}
#endif