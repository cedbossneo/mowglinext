#ifndef _ROS_xbot_rpc_RpcRequest_h
#define _ROS_xbot_rpc_RpcRequest_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace xbot_rpc
{

  class RpcRequest : public ros::Msg
  {
    public:
      typedef const char* _method_type;
      _method_type method;
      typedef const char* _params_type;
      _params_type params;
      typedef const char* _id_type;
      _id_type id;

    RpcRequest():
      method(""),
      params(""),
      id("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_method = strlen(this->method);
      varToArr(outbuffer + offset, length_method);
      offset += 4;
      memcpy(outbuffer + offset, this->method, length_method);
      offset += length_method;
      uint32_t length_params = strlen(this->params);
      varToArr(outbuffer + offset, length_params);
      offset += 4;
      memcpy(outbuffer + offset, this->params, length_params);
      offset += length_params;
      uint32_t length_id = strlen(this->id);
      varToArr(outbuffer + offset, length_id);
      offset += 4;
      memcpy(outbuffer + offset, this->id, length_id);
      offset += length_id;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_method;
      arrToVar(length_method, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_method; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_method-1]=0;
      this->method = (char *)(inbuffer + offset-1);
      offset += length_method;
      uint32_t length_params;
      arrToVar(length_params, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_params; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_params-1]=0;
      this->params = (char *)(inbuffer + offset-1);
      offset += length_params;
      uint32_t length_id;
      arrToVar(length_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_id-1]=0;
      this->id = (char *)(inbuffer + offset-1);
      offset += length_id;
     return offset;
    }

    virtual const char * getType() override { return "xbot_rpc/RpcRequest"; };
    virtual const char * getMD5() override { return "dbc38f22211a9f7c3ae542f63a9157c4"; };

  };

}
#endif
