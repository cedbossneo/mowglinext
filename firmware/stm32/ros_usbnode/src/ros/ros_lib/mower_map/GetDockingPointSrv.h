#ifndef _ROS_SERVICE_GetDockingPointSrv_h
#define _ROS_SERVICE_GetDockingPointSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace mower_map
{

static const char GETDOCKINGPOINTSRV[] = "mower_map/GetDockingPointSrv";

  class GetDockingPointSrvRequest : public ros::Msg
  {
    public:

    GetDockingPointSrvRequest()
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

    virtual const char * getType() override { return GETDOCKINGPOINTSRV; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetDockingPointSrvResponse : public ros::Msg
  {
    public:
      typedef geometry_msgs::Pose _docking_pose_type;
      _docking_pose_type docking_pose;

    GetDockingPointSrvResponse():
      docking_pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->docking_pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->docking_pose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return GETDOCKINGPOINTSRV; };
    virtual const char * getMD5() override { return "082f2ff1dbfba9861dd424f93f126a35"; };

  };

  class GetDockingPointSrv {
    public:
    typedef GetDockingPointSrvRequest Request;
    typedef GetDockingPointSrvResponse Response;
  };

}
#endif
