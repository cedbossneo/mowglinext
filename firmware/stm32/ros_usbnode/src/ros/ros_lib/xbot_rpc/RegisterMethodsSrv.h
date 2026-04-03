#ifndef _ROS_SERVICE_RegisterMethodsSrv_h
#define _ROS_SERVICE_RegisterMethodsSrv_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace xbot_rpc
{

static const char REGISTERMETHODSSRV[] = "xbot_rpc/RegisterMethodsSrv";

  class RegisterMethodsSrvRequest : public ros::Msg
  {
    public:
      typedef const char* _node_id_type;
      _node_id_type node_id;
      uint32_t methods_length;
      typedef char* _methods_type;
      _methods_type st_methods;
      _methods_type * methods;

    RegisterMethodsSrvRequest():
      node_id(""),
      methods_length(0), st_methods(), methods(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_node_id = strlen(this->node_id);
      varToArr(outbuffer + offset, length_node_id);
      offset += 4;
      memcpy(outbuffer + offset, this->node_id, length_node_id);
      offset += length_node_id;
      *(outbuffer + offset + 0) = (this->methods_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->methods_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->methods_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->methods_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->methods_length);
      for( uint32_t i = 0; i < methods_length; i++){
      uint32_t length_methodsi = strlen(this->methods[i]);
      varToArr(outbuffer + offset, length_methodsi);
      offset += 4;
      memcpy(outbuffer + offset, this->methods[i], length_methodsi);
      offset += length_methodsi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_node_id;
      arrToVar(length_node_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_node_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_node_id-1]=0;
      this->node_id = (char *)(inbuffer + offset-1);
      offset += length_node_id;
      uint32_t methods_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      methods_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      methods_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      methods_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->methods_length);
      if(methods_lengthT > methods_length)
        this->methods = (char**)realloc(this->methods, methods_lengthT * sizeof(char*));
      methods_length = methods_lengthT;
      for( uint32_t i = 0; i < methods_length; i++){
      uint32_t length_st_methods;
      arrToVar(length_st_methods, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_methods; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_methods-1]=0;
      this->st_methods = (char *)(inbuffer + offset-1);
      offset += length_st_methods;
        memcpy( &(this->methods[i]), &(this->st_methods), sizeof(char*));
      }
     return offset;
    }

    virtual const char * getType() override { return REGISTERMETHODSSRV; };
    virtual const char * getMD5() override { return "90604c3748b17d356cbe0452dbc968b3"; };

  };

  class RegisterMethodsSrvResponse : public ros::Msg
  {
    public:

    RegisterMethodsSrvResponse()
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

    virtual const char * getType() override { return REGISTERMETHODSSRV; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class RegisterMethodsSrv {
    public:
    typedef RegisterMethodsSrvRequest Request;
    typedef RegisterMethodsSrvResponse Response;
  };

}
#endif
