#ifndef _ROS_mower_msgs_Perimeter_h
#define _ROS_mower_msgs_Perimeter_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mower_msgs
{

  class Perimeter : public ros::Msg
  {
    public:
      typedef float _left_type;
      _left_type left;
      typedef float _center_type;
      _center_type center;
      typedef float _right_type;
      _right_type right;

    Perimeter():
      left(0),
      center(0),
      right(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left;
      u_left.real = this->left;
      *(outbuffer + offset + 0) = (u_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left);
      union {
        float real;
        uint32_t base;
      } u_center;
      u_center.real = this->center;
      *(outbuffer + offset + 0) = (u_center.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_center.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_center.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_center.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->center);
      union {
        float real;
        uint32_t base;
      } u_right;
      u_right.real = this->right;
      *(outbuffer + offset + 0) = (u_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_left;
      u_left.base = 0;
      u_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->left = u_left.real;
      offset += sizeof(this->left);
      union {
        float real;
        uint32_t base;
      } u_center;
      u_center.base = 0;
      u_center.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_center.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_center.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_center.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->center = u_center.real;
      offset += sizeof(this->center);
      union {
        float real;
        uint32_t base;
      } u_right;
      u_right.base = 0;
      u_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->right = u_right.real;
      offset += sizeof(this->right);
     return offset;
    }

    virtual const char * getType() override { return "mower_msgs/Perimeter"; };
    virtual const char * getMD5() override { return "1eef3cb68ced22453a42748a6794a55e"; };

  };

}
#endif
