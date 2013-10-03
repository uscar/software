#ifndef ros_Vector3Stamped_h
#define ros_Vector3Stamped_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Vector3.h"

namespace geometry_msgs
{

  class Vector3Stamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::Vector3 vector;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->vector.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->vector.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/Vector3Stamped"; };

  };

}
#endif