#ifndef _ROS_xbot_rpc_RpcError_h
#define _ROS_xbot_rpc_RpcError_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace xbot_rpc
{

  class RpcError : public ros::Msg
  {
    public:
      typedef int16_t _code_type;
      _code_type code;
      typedef const char* _message_type;
      _message_type message;
      typedef const char* _id_type;
      _id_type id;
      enum { ERROR_INVALID_JSON =  -32700    };
      enum { ERROR_INVALID_REQUEST =  -32600    };
      enum { ERROR_METHOD_NOT_FOUND =  -32601    };
      enum { ERROR_INVALID_PARAMS =  -32602    };
      enum { ERROR_INTERNAL =  -32603    };
      enum { ERROR_SERVER_FIRST =  -32000    };
      enum { ERROR_SERVER_LAST =  -32099    };

    RpcError():
      code(0),
      message(""),
      id("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_code;
      u_code.real = this->code;
      *(outbuffer + offset + 0) = (u_code.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_code.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->code);
      uint32_t length_message = strlen(this->message);
      varToArr(outbuffer + offset, length_message);
      offset += 4;
      memcpy(outbuffer + offset, this->message, length_message);
      offset += length_message;
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
      union {
        int16_t real;
        uint16_t base;
      } u_code;
      u_code.base = 0;
      u_code.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_code.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->code = u_code.real;
      offset += sizeof(this->code);
      uint32_t length_message;
      arrToVar(length_message, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_message-1]=0;
      this->message = (char *)(inbuffer + offset-1);
      offset += length_message;
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

    virtual const char * getType() override { return "xbot_rpc/RpcError"; };
    virtual const char * getMD5() override { return "57f6bcc937ca3e1d8d2870babb2db645"; };

  };

}
#endif
