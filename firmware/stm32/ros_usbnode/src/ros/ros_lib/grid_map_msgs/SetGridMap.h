#ifndef _ROS_SERVICE_SetGridMap_h
#define _ROS_SERVICE_SetGridMap_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "grid_map_msgs/GridMap.h"

namespace grid_map_msgs
{

static const char SETGRIDMAP[] = "grid_map_msgs/SetGridMap";

  class SetGridMapRequest : public ros::Msg
  {
    public:
      typedef grid_map_msgs::GridMap _map_type;
      _map_type map;

    SetGridMapRequest():
      map()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->map.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->map.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SETGRIDMAP; };
    virtual const char * getMD5() override { return "4f8e24cfd42bc1470fe765b7516ff7e5"; };

  };

  class SetGridMapResponse : public ros::Msg
  {
    public:

    SetGridMapResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return SETGRIDMAP; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetGridMap {
    public:
    typedef SetGridMapRequest Request;
    typedef SetGridMapResponse Response;
  };

}
#endif
