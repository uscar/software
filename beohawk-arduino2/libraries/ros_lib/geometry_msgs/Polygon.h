#ifndef ros_Polygon_h
#define ros_Polygon_h

#include "WProgram.h"
#include "ros.h"
#include "geometry_msgs/Point32.h"

namespace geometry_msgs
{

  class Polygon : public ros::Msg
  {
    public:
      unsigned char points_length;
      geometry_msgs::Point32 st_points;
      geometry_msgs::Point32 * points;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      *(outbuffer + offset++) = points_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( unsigned char i = 0; i < points_length; i++){
      offset += this->points[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      unsigned char points_lengthT = *(inbuffer + offset++);
      if(points_lengthT > points_length)
        this->points = (geometry_msgs::Point32*)realloc(this->points, points_lengthT * sizeof(geometry_msgs::Point32));
      offset += 3;
      points_length = points_lengthT;
      for( unsigned char i = 0; i < points_length; i++){
      offset += this->st_points.deserialize(inbuffer + offset);
        memcpy( &(this->points[i]), &(this->st_points), sizeof(geometry_msgs::Point32));
      }
     return offset;
    }

    const char * getType(){ return "geometry_msgs/Polygon"; };

  };

}
#endif