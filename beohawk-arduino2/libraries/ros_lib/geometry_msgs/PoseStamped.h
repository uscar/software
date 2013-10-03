#ifndef ros_PoseStamped_h
#define ros_PoseStamped_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Pose.h"

namespace geometry_msgs
{

  class PoseStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::Pose pose;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/PoseStamped"; };

  };

}
#endif