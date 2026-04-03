#ifndef _ROS_SERVICE_TriggerSnapshot_h
#define _ROS_SERVICE_TriggerSnapshot_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace rosbag_snapshot_msgs
{

static const char TRIGGERSNAPSHOT[] = "rosbag_snapshot_msgs/TriggerSnapshot";

  class TriggerSnapshotRequest : public ros::Msg
  {
    public:
      typedef const char* _filename_type;
      _filename_type filename;
      uint32_t topics_length;
      typedef char* _topics_type;
      _topics_type st_topics;
      _topics_type * topics;
      typedef ros::Time _start_time_type;
      _start_time_type start_time;
      typedef ros::Time _stop_time_type;
      _stop_time_type stop_time;

    TriggerSnapshotRequest():
      filename(""),
      topics_length(0), st_topics(), topics(nullptr),
      start_time(),
      stop_time()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_filename = strlen(this->filename);
      varToArr(outbuffer + offset, length_filename);
      offset += 4;
      memcpy(outbuffer + offset, this->filename, length_filename);
      offset += length_filename;
      *(outbuffer + offset + 0) = (this->topics_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->topics_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->topics_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->topics_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->topics_length);
      for( uint32_t i = 0; i < topics_length; i++){
      uint32_t length_topicsi = strlen(this->topics[i]);
      varToArr(outbuffer + offset, length_topicsi);
      offset += 4;
      memcpy(outbuffer + offset, this->topics[i], length_topicsi);
      offset += length_topicsi;
      }
      *(outbuffer + offset + 0) = (this->start_time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->start_time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->start_time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->start_time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->start_time.sec);
      *(outbuffer + offset + 0) = (this->start_time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->start_time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->start_time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->start_time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->start_time.nsec);
      *(outbuffer + offset + 0) = (this->stop_time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stop_time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stop_time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stop_time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stop_time.sec);
      *(outbuffer + offset + 0) = (this->stop_time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stop_time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stop_time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stop_time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stop_time.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_filename;
      arrToVar(length_filename, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_filename; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_filename-1]=0;
      this->filename = (char *)(inbuffer + offset-1);
      offset += length_filename;
      uint32_t topics_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      topics_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      topics_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      topics_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->topics_length);
      if(topics_lengthT > topics_length)
        this->topics = (char**)realloc(this->topics, topics_lengthT * sizeof(char*));
      topics_length = topics_lengthT;
      for( uint32_t i = 0; i < topics_length; i++){
      uint32_t length_st_topics;
      arrToVar(length_st_topics, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_topics; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_topics-1]=0;
      this->st_topics = (char *)(inbuffer + offset-1);
      offset += length_st_topics;
        memcpy( &(this->topics[i]), &(this->st_topics), sizeof(char*));
      }
      this->start_time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->start_time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->start_time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->start_time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->start_time.sec);
      this->start_time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->start_time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->start_time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->start_time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->start_time.nsec);
      this->stop_time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stop_time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stop_time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stop_time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stop_time.sec);
      this->stop_time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stop_time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stop_time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stop_time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stop_time.nsec);
     return offset;
    }

    virtual const char * getType() override { return TRIGGERSNAPSHOT; };
    virtual const char * getMD5() override { return "09f36a11eeb5a8ba983b3be50c4e7cc7"; };

  };

  class TriggerSnapshotResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _message_type;
      _message_type message;
      enum { NO_DATA_MESSAGE = no messages buffered on selected topics };

    TriggerSnapshotResponse():
      success(0),
      message("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
     return offset;
    }

    virtual const char * getType() override { return TRIGGERSNAPSHOT; };
    virtual const char * getMD5() override { return "ef7a6261f820dd9de10824939dd524d6"; };

  };

  class TriggerSnapshot {
    public:
    typedef TriggerSnapshotRequest Request;
    typedef TriggerSnapshotResponse Response;
  };

}
#endif
