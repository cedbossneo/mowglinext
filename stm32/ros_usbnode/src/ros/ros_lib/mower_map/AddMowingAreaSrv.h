#ifndef _ROS_SERVICE_AddMowingAreaSrv_h
#define _ROS_SERVICE_AddMowingAreaSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mower_map/MapArea.h"

namespace mower_map
{

static const char ADDMOWINGAREASRV[] = "mower_map/AddMowingAreaSrv";

  class AddMowingAreaSrvRequest : public ros::Msg
  {
    public:
      typedef mower_map::MapArea _area_type;
      _area_type area;
      typedef bool _isNavigationArea_type;
      _isNavigationArea_type isNavigationArea;

    AddMowingAreaSrvRequest():
      area(),
      isNavigationArea(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->area.serialize(outbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_isNavigationArea;
      u_isNavigationArea.real = this->isNavigationArea;
      *(outbuffer + offset + 0) = (u_isNavigationArea.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->isNavigationArea);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->area.deserialize(inbuffer + offset);
      union {
        bool real;
        uint8_t base;
      } u_isNavigationArea;
      u_isNavigationArea.base = 0;
      u_isNavigationArea.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->isNavigationArea = u_isNavigationArea.real;
      offset += sizeof(this->isNavigationArea);
     return offset;
    }

    virtual const char * getType() override { return ADDMOWINGAREASRV; };
    virtual const char * getMD5() override { return "b3ef56ffad622c37e65aec1dd5ae39a7"; };

  };

  class AddMowingAreaSrvResponse : public ros::Msg
  {
    public:

    AddMowingAreaSrvResponse()
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

    virtual const char * getType() override { return ADDMOWINGAREASRV; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class AddMowingAreaSrv {
    public:
    typedef AddMowingAreaSrvRequest Request;
    typedef AddMowingAreaSrvResponse Response;
  };

}
#endif
