#ifndef ros_QuaternionStamped_h
#define ros_QuaternionStamped_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Quaternion.h"

namespace geometry_msgs
{

  class QuaternionStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::Quaternion quaternion;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->quaternion.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->quaternion.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/QuaternionStamped"; };

  };

}
#endif