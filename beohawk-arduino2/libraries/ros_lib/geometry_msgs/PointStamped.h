#ifndef ros_PointStamped_h
#define ros_PointStamped_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"

namespace geometry_msgs
{

  class PointStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::Point point;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->point.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->point.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/PointStamped"; };

  };

}
#endif