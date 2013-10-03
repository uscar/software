#ifndef ros_Twist_h
#define ros_Twist_h

#include "WProgram.h"
#include "ros.h"
#include "geometry_msgs/Vector3.h"

namespace geometry_msgs
{

  class Twist : public ros::Msg
  {
    public:
      geometry_msgs::Vector3 linear;
      geometry_msgs::Vector3 angular;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->linear.serialize(outbuffer + offset);
      offset += this->angular.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->linear.deserialize(inbuffer + offset);
      offset += this->angular.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/Twist"; };

  };

}
#endif