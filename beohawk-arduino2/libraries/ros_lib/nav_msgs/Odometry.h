#ifndef ros_Odometry_h
#define ros_Odometry_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/TwistWithCovariance.h"

namespace nav_msgs
{

  class Odometry : public ros::Msg
  {
    public:
      std_msgs::Header header;
      unsigned char * child_frame_id;
      geometry_msgs::PoseWithCovariance pose;
      geometry_msgs::TwistWithCovariance twist;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      long * length_child_frame_id = (long *)(outbuffer + offset);
      *length_child_frame_id = strlen( (const char*) this->child_frame_id);
      offset += 4;
      memcpy(outbuffer + offset, this->child_frame_id, *length_child_frame_id);
      offset += *length_child_frame_id;
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->twist.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      long * length_child_frame_id = (long *)(inbuffer + offset);
      offset += 4;
      this->child_frame_id = (inbuffer + offset);
      offset += *length_child_frame_id;
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->twist.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "nav_msgs/Odometry"; };

  };

}
#endif