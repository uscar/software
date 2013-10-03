#ifndef ros_PoseWithCovarianceStamped_h
#define ros_PoseWithCovarianceStamped_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseWithCovariance.h"

namespace geometry_msgs
{

  class PoseWithCovarianceStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::PoseWithCovariance pose;

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

    const char * getType(){ return "geometry_msgs/PoseWithCovarianceStamped"; };

  };

}
#endif