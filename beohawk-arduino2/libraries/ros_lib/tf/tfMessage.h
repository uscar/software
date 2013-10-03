#ifndef ros_tfMessage_h
#define ros_tfMessage_h

#include "WProgram.h"
#include "ros.h"
#include "geometry_msgs/TransformStamped.h"

namespace tf
{

  class tfMessage : public ros::Msg
  {
    public:
      unsigned char transforms_length;
      geometry_msgs::TransformStamped st_transforms;
      geometry_msgs::TransformStamped * transforms;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      *(outbuffer + offset++) = transforms_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( unsigned char i = 0; i < transforms_length; i++){
      offset += this->transforms[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      unsigned char transforms_lengthT = *(inbuffer + offset++);
      if(transforms_lengthT > transforms_length)
        this->transforms = (geometry_msgs::TransformStamped*)realloc(this->transforms, transforms_lengthT * sizeof(geometry_msgs::TransformStamped));
      offset += 3;
      transforms_length = transforms_lengthT;
      for( unsigned char i = 0; i < transforms_length; i++){
      offset += this->st_transforms.deserialize(inbuffer + offset);
        memcpy( &(this->transforms[i]), &(this->st_transforms), sizeof(geometry_msgs::TransformStamped));
      }
     return offset;
    }

    const char * getType(){ return "tf/tfMessage"; };

  };

}
#endif