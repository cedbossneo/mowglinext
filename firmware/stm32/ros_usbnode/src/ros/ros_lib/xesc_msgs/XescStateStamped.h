#ifndef _ROS_xesc_msgs_XescStateStamped_h
#define _ROS_xesc_msgs_XescStateStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "xesc_msgs/XescState.h"

namespace xesc_msgs
{

  class XescStateStamped : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef xesc_msgs::XescState _state_type;
      _state_type state;

    XescStateStamped():
      header(),
      state()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->state.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->state.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "xesc_msgs/XescStateStamped"; };
    virtual const char * getMD5() override { return "036068e7c8146c770f850f86e593ffd6"; };

  };

}
#endif
