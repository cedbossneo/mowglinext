#ifndef _ROS_SERVICE_SetPoseSrv_h
#define _ROS_SERVICE_SetPoseSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Pose.h"

namespace xbot_positioning
{

static const char SETPOSESRV[] = "xbot_positioning/SetPoseSrv";

  class SetPoseSrvRequest : public ros::Msg
  {
    public:
      typedef geometry_msgs::Pose _robot_pose_type;
      _robot_pose_type robot_pose;

    SetPoseSrvRequest():
      robot_pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->robot_pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->robot_pose.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return SETPOSESRV; };
    virtual const char * getMD5() override { return "a1e8e42f88161a7bc7869c4cb33b8e1e"; };

  };

  class SetPoseSrvResponse : public ros::Msg
  {
    public:

    SetPoseSrvResponse()
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

    virtual const char * getType() override { return SETPOSESRV; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetPoseSrv {
    public:
    typedef SetPoseSrvRequest Request;
    typedef SetPoseSrvResponse Response;
  };

}
#endif
