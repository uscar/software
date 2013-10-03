#ifndef ros_TwistStamped_h
#define ros_TwistStamped_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Twist.h"

namespace geometry_msgs
{

  class TwistStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::Twist twist;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->twist.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->twist.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/TwistStamped"; };

  };

}
#endif