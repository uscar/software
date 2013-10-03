#ifndef ros_Transform_h
#define ros_Transform_h

#include "WProgram.h"
#include "ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"

namespace geometry_msgs
{

  class Transform : public ros::Msg
  {
    public:
      geometry_msgs::Vector3 translation;
      geometry_msgs::Quaternion rotation;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->translation.serialize(outbuffer + offset);
      offset += this->rotation.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->translation.deserialize(inbuffer + offset);
      offset += this->rotation.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geometry_msgs/Transform"; };

  };

}
#endif