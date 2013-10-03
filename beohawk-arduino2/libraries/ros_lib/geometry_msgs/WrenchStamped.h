#ifndef ros_WrenchStamped_h
#define ros_WrenchStamped_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Wrench.h"

namespace geometry_msgs
{

  class WrenchStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geometry_msgs::Wrench wrench;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->wrench.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->wrench.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/WrenchStamped"; };

  };

}
#endif