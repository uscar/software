#ifndef ros_Pose_h
#define ros_Pose_h

#include "WProgram.h"
#include "ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Quaternion.h"

namespace geometry_msgs
{

  class Pose : public ros::Msg
  {
    public:
      geometry_msgs::Point position;
      geometry_msgs::Quaternion orientation;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->position.serialize(outbuffer + offset);
      offset += this->orientation.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->position.deserialize(inbuffer + offset);
      offset += this->orientation.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/Pose"; };

  };

}
#endif