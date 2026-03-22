#ifndef _ROS_rosbag_snapshot_msgs_SnapshotStatus_h
#define _ROS_rosbag_snapshot_msgs_SnapshotStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "rosgraph_msgs/TopicStatistics.h"

namespace rosbag_snapshot_msgs
{

  class SnapshotStatus : public ros::Msg
  {
    public:
      uint32_t topics_length;
      typedef rosgraph_msgs::TopicStatistics _topics_type;
      _topics_type st_topics;
      _topics_type * topics;
      typedef bool _enabled_type;
      _enabled_type enabled;

    SnapshotStatus():
      topics_length(0), st_topics(), topics(nullptr),
      enabled(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->topics_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->topics_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->topics_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->topics_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->topics_length);
      for( uint32_t i = 0; i < topics_length; i++){
      offset += this->topics[i].serialize(outbuffer + offset);
      }
      union {
        bool real;
        uint8_t base;
      } u_enabled;
      u_enabled.real = this->enabled;
      *(outbuffer + offset + 0) = (u_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->enabled);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t topics_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      topics_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      topics_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      topics_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->topics_length);
      if(topics_lengthT > topics_length)
        this->topics = (rosgraph_msgs::TopicStatistics*)realloc(this->topics, topics_lengthT * sizeof(rosgraph_msgs::TopicStatistics));
      topics_length = topics_lengthT;
      for( uint32_t i = 0; i < topics_length; i++){
      offset += this->st_topics.deserialize(inbuffer + offset);
        memcpy( &(this->topics[i]), &(this->st_topics), sizeof(rosgraph_msgs::TopicStatistics));
      }
      union {
        bool real;
        uint8_t base;
      } u_enabled;
      u_enabled.base = 0;
      u_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->enabled = u_enabled.real;
      offset += sizeof(this->enabled);
     return offset;
    }

    virtual const char * getType() override { return "rosbag_snapshot_msgs/SnapshotStatus"; };
    virtual const char * getMD5() override { return "da7b0d829f1bb54c3d787d81f6befe0f"; };

  };

}
#endif
