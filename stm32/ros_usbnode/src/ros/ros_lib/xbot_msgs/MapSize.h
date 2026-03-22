#ifndef _ROS_xbot_msgs_MapSize_h
#define _ROS_xbot_msgs_MapSize_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace xbot_msgs
{

  class MapSize : public ros::Msg
  {
    public:
      typedef float _mapWidth_type;
      _mapWidth_type mapWidth;
      typedef float _mapHeight_type;
      _mapHeight_type mapHeight;
      typedef float _mapCenterX_type;
      _mapCenterX_type mapCenterX;
      typedef float _mapCenterY_type;
      _mapCenterY_type mapCenterY;

    MapSize():
      mapWidth(0),
      mapHeight(0),
      mapCenterX(0),
      mapCenterY(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->mapWidth);
      offset += serializeAvrFloat64(outbuffer + offset, this->mapHeight);
      offset += serializeAvrFloat64(outbuffer + offset, this->mapCenterX);
      offset += serializeAvrFloat64(outbuffer + offset, this->mapCenterY);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mapWidth));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mapHeight));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mapCenterX));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mapCenterY));
     return offset;
    }

    virtual const char * getType() override { return "xbot_msgs/MapSize"; };
    virtual const char * getMD5() override { return "ef26e12ec75aba87aec8d6f1e7ef16ba"; };

  };

}
#endif
