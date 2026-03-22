#ifndef _ROS_slic3r_coverage_planner_Path_h
#define _ROS_slic3r_coverage_planner_Path_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nav_msgs/Path.h"

namespace slic3r_coverage_planner
{

  class Path : public ros::Msg
  {
    public:
      typedef uint8_t _is_outline_type;
      _is_outline_type is_outline;
      typedef nav_msgs::Path _path_type;
      _path_type path;

    Path():
      is_outline(0),
      path()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->is_outline >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_outline);
      offset += this->path.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->is_outline =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->is_outline);
      offset += this->path.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "slic3r_coverage_planner/Path"; };
    virtual const char * getMD5() override { return "60d462a6fa4e8393828cd1e66919384e"; };

  };

}
#endif
