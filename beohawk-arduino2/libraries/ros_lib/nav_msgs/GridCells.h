#ifndef ros_GridCells_h
#define ros_GridCells_h

#include "WProgram.h"
#include "ros.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/Point.h"

namespace nav_msgs
{

  class GridCells : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float cell_width;
      float cell_height;
      unsigned char cells_length;
      geometry_msgs::Point st_cells;
      geometry_msgs::Point * cells;

    virtual int serialize(unsigned char *outbuffer)
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      union {
        float real;
        unsigned long base;
      } u_cell_width;
      u_cell_width.real = this->cell_width;
      *(outbuffer + offset + 0) = (u_cell_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cell_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cell_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cell_width.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cell_width);
      union {
        float real;
        unsigned long base;
      } u_cell_height;
      u_cell_height.real = this->cell_height;
      *(outbuffer + offset + 0) = (u_cell_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_cell_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_cell_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_cell_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->cell_height);
      *(outbuffer + offset++) = cells_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( unsigned char i = 0; i < cells_length; i++){
      offset += this->cells[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      union {
        float real;
        unsigned long base;
      } u_cell_width;
      u_cell_width.base = 0;
      u_cell_width.base |= ((typeof(u_cell_width.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cell_width.base |= ((typeof(u_cell_width.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cell_width.base |= ((typeof(u_cell_width.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cell_width.base |= ((typeof(u_cell_width.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cell_width = u_cell_width.real;
      offset += sizeof(this->cell_width);
      union {
        float real;
        unsigned long base;
      } u_cell_height;
      u_cell_height.base = 0;
      u_cell_height.base |= ((typeof(u_cell_height.base)) (*(inbuffer + offset + 0))) << (8 * 0);
      u_cell_height.base |= ((typeof(u_cell_height.base)) (*(inbuffer + offset + 1))) << (8 * 1);
      u_cell_height.base |= ((typeof(u_cell_height.base)) (*(inbuffer + offset + 2))) << (8 * 2);
      u_cell_height.base |= ((typeof(u_cell_height.base)) (*(inbuffer + offset + 3))) << (8 * 3);
      this->cell_height = u_cell_height.real;
      offset += sizeof(this->cell_height);
      unsigned char cells_lengthT = *(inbuffer + offset++);
      if(cells_lengthT > cells_length)
        this->cells = (geometry_msgs::Point*)realloc(this->cells, cells_lengthT * sizeof(geometry_msgs::Point));
      offset += 3;
      cells_length = cells_lengthT;
      for( unsigned char i = 0; i < cells_length; i++){
      offset += this->st_cells.deserialize(inbuffer + offset);
        memcpy( &(this->cells[i]), &(this->st_cells), sizeof(geometry_msgs::Point));
      }
     return offset;
    }

    const char * getType(){ return "nav_msgs/GridCells"; };

  };

}
#endif