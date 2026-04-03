#ifndef _ROS_SERVICE_SetNavPointSrv_h
#define _ROS_SERVICE_SetNavPointSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace mower_map
{

static const char SETNAVPOINTSRV[] = "mower_map/SetNavPointSrv";

  class SetNavPointSrvRequest : public ros::Msg
  {
    public:
      typedef geometry_msgs::Pose _nav_pose_type;
      _nav_pose_type nav_pose;

    SetNavPointSrvRequest():
      nav_pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->nav_pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->nav_pose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SETNAVPOINTSRV; };
    virtual const char * getMD5() override { return "57762a01338710861d6dcc49f7745df2"; };

  };

  class SetNavPointSrvResponse : public ros::Msg
  {
    public:

    SetNavPointSrvResponse()
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

    virtual const char * getType() override { return SETNAVPOINTSRV; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetNavPointSrv {
    public:
    typedef SetNavPointSrvRequest Request;
    typedef SetNavPointSrvResponse Response;
  };

}
#endif
