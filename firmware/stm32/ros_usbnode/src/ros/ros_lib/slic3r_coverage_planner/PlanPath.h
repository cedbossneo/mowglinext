#ifndef _ROS_SERVICE_PlanPath_h
#define _ROS_SERVICE_PlanPath_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Polygon.h"
#include "slic3r_coverage_planner/Path.h"

namespace slic3r_coverage_planner
{

static const char PLANPATH[] = "slic3r_coverage_planner/PlanPath";

  class PlanPathRequest : public ros::Msg
  {
    public:
      typedef uint8_t _fill_type_type;
      _fill_type_type fill_type;
      typedef float _angle_type;
      _angle_type angle;
      typedef float _distance_type;
      _distance_type distance;
      typedef float _outer_offset_type;
      _outer_offset_type outer_offset;
      typedef uint8_t _outline_count_type;
      _outline_count_type outline_count;
      typedef uint8_t _outline_overlap_count_type;
      _outline_overlap_count_type outline_overlap_count;
      typedef bool _skip_area_outline_type;
      _skip_area_outline_type skip_area_outline;
      typedef bool _skip_obstacle_outlines_type;
      _skip_obstacle_outlines_type skip_obstacle_outlines;
      typedef bool _skip_fill_type;
      _skip_fill_type skip_fill;
      typedef geometry_msgs::Polygon _outline_type;
      _outline_type outline;
      uint32_t holes_length;
      typedef geometry_msgs::Polygon _holes_type;
      _holes_type st_holes;
      _holes_type * holes;
      enum { FILL_LINEAR = 0 };
      enum { FILL_CONCENTRIC = 1 };

    PlanPathRequest():
      fill_type(0),
      angle(0),
      distance(0),
      outer_offset(0),
      outline_count(0),
      outline_overlap_count(0),
      skip_area_outline(0),
      skip_obstacle_outlines(0),
      skip_fill(0),
      outline(),
      holes_length(0), st_holes(), holes(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->fill_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->fill_type);
      offset += serializeAvrFloat64(outbuffer + offset, this->angle);
      offset += serializeAvrFloat64(outbuffer + offset, this->distance);
      offset += serializeAvrFloat64(outbuffer + offset, this->outer_offset);
      *(outbuffer + offset + 0) = (this->outline_count >> (8 * 0)) & 0xFF;
      offset += sizeof(this->outline_count);
      *(outbuffer + offset + 0) = (this->outline_overlap_count >> (8 * 0)) & 0xFF;
      offset += sizeof(this->outline_overlap_count);
      union {
        bool real;
        uint8_t base;
      } u_skip_area_outline;
      u_skip_area_outline.real = this->skip_area_outline;
      *(outbuffer + offset + 0) = (u_skip_area_outline.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->skip_area_outline);
      union {
        bool real;
        uint8_t base;
      } u_skip_obstacle_outlines;
      u_skip_obstacle_outlines.real = this->skip_obstacle_outlines;
      *(outbuffer + offset + 0) = (u_skip_obstacle_outlines.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->skip_obstacle_outlines);
      union {
        bool real;
        uint8_t base;
      } u_skip_fill;
      u_skip_fill.real = this->skip_fill;
      *(outbuffer + offset + 0) = (u_skip_fill.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->skip_fill);
      offset += this->outline.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->holes_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->holes_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->holes_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->holes_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->holes_length);
      for( uint32_t i = 0; i < holes_length; i++){
      offset += this->holes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->fill_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->fill_type);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angle));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->distance));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->outer_offset));
      this->outline_count =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->outline_count);
      this->outline_overlap_count =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->outline_overlap_count);
      union {
        bool real;
        uint8_t base;
      } u_skip_area_outline;
      u_skip_area_outline.base = 0;
      u_skip_area_outline.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->skip_area_outline = u_skip_area_outline.real;
      offset += sizeof(this->skip_area_outline);
      union {
        bool real;
        uint8_t base;
      } u_skip_obstacle_outlines;
      u_skip_obstacle_outlines.base = 0;
      u_skip_obstacle_outlines.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->skip_obstacle_outlines = u_skip_obstacle_outlines.real;
      offset += sizeof(this->skip_obstacle_outlines);
      union {
        bool real;
        uint8_t base;
      } u_skip_fill;
      u_skip_fill.base = 0;
      u_skip_fill.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->skip_fill = u_skip_fill.real;
      offset += sizeof(this->skip_fill);
      offset += this->outline.deserialize(inbuffer + offset);
      uint32_t holes_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      holes_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      holes_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      holes_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->holes_length);
      if(holes_lengthT > holes_length)
        this->holes = (geometry_msgs::Polygon*)realloc(this->holes, holes_lengthT * sizeof(geometry_msgs::Polygon));
      holes_length = holes_lengthT;
      for( uint32_t i = 0; i < holes_length; i++){
      offset += this->st_holes.deserialize(inbuffer + offset);
        memcpy( &(this->holes[i]), &(this->st_holes), sizeof(geometry_msgs::Polygon));
      }
     return offset;
    }

    virtual const char * getType() override { return PLANPATH; };
    virtual const char * getMD5() override { return "99bef52c4c2e1629fd84fcb3c35d0a15"; };

  };

  class PlanPathResponse : public ros::Msg
  {
    public:
      uint32_t paths_length;
      typedef slic3r_coverage_planner::Path _paths_type;
      _paths_type st_paths;
      _paths_type * paths;

    PlanPathResponse():
      paths_length(0), st_paths(), paths(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->paths_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->paths_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->paths_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->paths_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->paths_length);
      for( uint32_t i = 0; i < paths_length; i++){
      offset += this->paths[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t paths_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      paths_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      paths_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      paths_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->paths_length);
      if(paths_lengthT > paths_length)
        this->paths = (slic3r_coverage_planner::Path*)realloc(this->paths, paths_lengthT * sizeof(slic3r_coverage_planner::Path));
      paths_length = paths_lengthT;
      for( uint32_t i = 0; i < paths_length; i++){
      offset += this->st_paths.deserialize(inbuffer + offset);
        memcpy( &(this->paths[i]), &(this->st_paths), sizeof(slic3r_coverage_planner::Path));
      }
     return offset;
    }

    virtual const char * getType() override { return PLANPATH; };
    virtual const char * getMD5() override { return "581fb87ba33bdeb36f0d801b4d1d1525"; };

  };

  class PlanPath {
    public:
    typedef PlanPathRequest Request;
    typedef PlanPathResponse Response;
  };

}
#endif
