#ifndef _ROS_SERVICE_GPSControlSrv_h
#define _ROS_SERVICE_GPSControlSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mower_msgs
{

static const char GPSCONTROLSRV[] = "mower_msgs/GPSControlSrv";

  class GPSControlSrvRequest : public ros::Msg
  {
    public:
      typedef uint8_t _gps_enabled_type;
      _gps_enabled_type gps_enabled;

    GPSControlSrvRequest():
      gps_enabled(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->gps_enabled >> (8 * 0)) & 0xFF;
      offset += sizeof(this->gps_enabled);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->gps_enabled =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->gps_enabled);
     return offset;
    }

    virtual const char * getType() override { return GPSCONTROLSRV; };
    virtual const char * getMD5() override { return "d9982cf964c37d45435cd4a8d34d8df4"; };

  };

  class GPSControlSrvResponse : public ros::Msg
  {
    public:

    GPSControlSrvResponse()
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

    virtual const char * getType() override { return GPSCONTROLSRV; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GPSControlSrv {
    public:
    typedef GPSControlSrvRequest Request;
    typedef GPSControlSrvResponse Response;
  };

}
#endif
