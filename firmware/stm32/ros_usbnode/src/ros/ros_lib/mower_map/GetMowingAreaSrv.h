#ifndef _ROS_SERVICE_GetMowingAreaSrv_h
#define _ROS_SERVICE_GetMowingAreaSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "mower_map/MapArea.h"

namespace mower_map
{

static const char GETMOWINGAREASRV[] = "mower_map/GetMowingAreaSrv";

  class GetMowingAreaSrvRequest : public ros::Msg
  {
    public:
      typedef uint32_t _index_type;
      _index_type index;

    GetMowingAreaSrvRequest():
      index(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->index >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->index >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->index >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->index >> (8 * 3)) & 0xFF;
      offset += sizeof(this->index);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->index =  ((uint32_t) (*(inbuffer + offset)));
      this->index |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->index |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->index |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->index);
     return offset;
    }

    virtual const char * getType() override { return GETMOWINGAREASRV; };
    virtual const char * getMD5() override { return "ad7b979103dbd563a352ef5270716728"; };

  };

  class GetMowingAreaSrvResponse : public ros::Msg
  {
    public:
      typedef mower_map::MapArea _area_type;
      _area_type area;

    GetMowingAreaSrvResponse():
      area()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->area.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->area.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return GETMOWINGAREASRV; };
    virtual const char * getMD5() override { return "5c805694b8e8efd628f1a2c5e57d287b"; };

  };

  class GetMowingAreaSrv {
    public:
    typedef GetMowingAreaSrvRequest Request;
    typedef GetMowingAreaSrvResponse Response;
  };

}
#endif
