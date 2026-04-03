#ifndef _ROS_xbot_msgs_MapOverlayPolygon_h
#define _ROS_xbot_msgs_MapOverlayPolygon_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Polygon.h"

namespace xbot_msgs
{

  class MapOverlayPolygon : public ros::Msg
  {
    public:
      typedef geometry_msgs::Polygon _polygon_type;
      _polygon_type polygon;
      typedef const char* _color_type;
      _color_type color;
      typedef bool _closed_type;
      _closed_type closed;
      typedef float _line_width_type;
      _line_width_type line_width;

    MapOverlayPolygon():
      polygon(),
      color(""),
      closed(0),
      line_width(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->polygon.serialize(outbuffer + offset);
      uint32_t length_color = strlen(this->color);
      varToArr(outbuffer + offset, length_color);
      offset += 4;
      memcpy(outbuffer + offset, this->color, length_color);
      offset += length_color;
      union {
        bool real;
        uint8_t base;
      } u_closed;
      u_closed.real = this->closed;
      *(outbuffer + offset + 0) = (u_closed.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->closed);
      union {
        float real;
        uint32_t base;
      } u_line_width;
      u_line_width.real = this->line_width;
      *(outbuffer + offset + 0) = (u_line_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_line_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_line_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_line_width.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->line_width);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->polygon.deserialize(inbuffer + offset);
      uint32_t length_color;
      arrToVar(length_color, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_color; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_color-1]=0;
      this->color = (char *)(inbuffer + offset-1);
      offset += length_color;
      union {
        bool real;
        uint8_t base;
      } u_closed;
      u_closed.base = 0;
      u_closed.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->closed = u_closed.real;
      offset += sizeof(this->closed);
      union {
        float real;
        uint32_t base;
      } u_line_width;
      u_line_width.base = 0;
      u_line_width.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_line_width.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_line_width.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_line_width.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->line_width = u_line_width.real;
      offset += sizeof(this->line_width);
     return offset;
    }

    virtual const char * getType() override { return "xbot_msgs/MapOverlayPolygon"; };
    virtual const char * getMD5() override { return "120b1776b75dc17ea7f8c81f08a7abe3"; };

  };

}
#endif
