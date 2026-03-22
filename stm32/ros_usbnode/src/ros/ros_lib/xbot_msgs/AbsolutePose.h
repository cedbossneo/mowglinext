#ifndef _ROS_xbot_msgs_AbsolutePose_h
#define _ROS_xbot_msgs_AbsolutePose_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geometry_msgs/PoseWithCovariance.h"
#include "geometry_msgs/Vector3.h"

namespace xbot_msgs
{

  class AbsolutePose : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      typedef uint32_t _sensor_stamp_type;
      _sensor_stamp_type sensor_stamp;
      typedef uint32_t _received_stamp_type;
      _received_stamp_type received_stamp;
      typedef uint8_t _source_type;
      _source_type source;
      typedef uint16_t _flags_type;
      _flags_type flags;
      typedef uint8_t _orientation_valid_type;
      _orientation_valid_type orientation_valid;
      typedef uint8_t _motion_vector_valid_type;
      _motion_vector_valid_type motion_vector_valid;
      typedef float _position_accuracy_type;
      _position_accuracy_type position_accuracy;
      typedef float _orientation_accuracy_type;
      _orientation_accuracy_type orientation_accuracy;
      typedef geometry_msgs::PoseWithCovariance _pose_type;
      _pose_type pose;
      typedef geometry_msgs::Vector3 _motion_vector_type;
      _motion_vector_type motion_vector;
      typedef float _vehicle_heading_type;
      _vehicle_heading_type vehicle_heading;
      typedef float _motion_heading_type;
      _motion_heading_type motion_heading;
      enum { SOURCE_GPS = 1 };
      enum { SOURCE_LIGHTHOUSE = 2 };
      enum { SOURCE_SENSOR_FUSION = 100 };
      enum { FLAG_GPS_RTK = 1                    };
      enum { FLAG_GPS_RTK_FIXED = 2              };
      enum { FLAG_GPS_RTK_FLOAT = 4              };
      enum { FLAG_GPS_DEAD_RECKONING = 8         };
      enum { FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE = 1     };
      enum { FLAG_SENSOR_FUSION_DEAD_RECKONING = 8           };

    AbsolutePose():
      header(),
      sensor_stamp(0),
      received_stamp(0),
      source(0),
      flags(0),
      orientation_valid(0),
      motion_vector_valid(0),
      position_accuracy(0),
      orientation_accuracy(0),
      pose(),
      motion_vector(),
      vehicle_heading(0),
      motion_heading(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->sensor_stamp >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->sensor_stamp >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->sensor_stamp >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->sensor_stamp >> (8 * 3)) & 0xFF;
      offset += sizeof(this->sensor_stamp);
      *(outbuffer + offset + 0) = (this->received_stamp >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->received_stamp >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->received_stamp >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->received_stamp >> (8 * 3)) & 0xFF;
      offset += sizeof(this->received_stamp);
      *(outbuffer + offset + 0) = (this->source >> (8 * 0)) & 0xFF;
      offset += sizeof(this->source);
      *(outbuffer + offset + 0) = (this->flags >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->flags >> (8 * 1)) & 0xFF;
      offset += sizeof(this->flags);
      *(outbuffer + offset + 0) = (this->orientation_valid >> (8 * 0)) & 0xFF;
      offset += sizeof(this->orientation_valid);
      *(outbuffer + offset + 0) = (this->motion_vector_valid >> (8 * 0)) & 0xFF;
      offset += sizeof(this->motion_vector_valid);
      union {
        float real;
        uint32_t base;
      } u_position_accuracy;
      u_position_accuracy.real = this->position_accuracy;
      *(outbuffer + offset + 0) = (u_position_accuracy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_position_accuracy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_position_accuracy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_position_accuracy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_accuracy);
      union {
        float real;
        uint32_t base;
      } u_orientation_accuracy;
      u_orientation_accuracy.real = this->orientation_accuracy;
      *(outbuffer + offset + 0) = (u_orientation_accuracy.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_orientation_accuracy.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_orientation_accuracy.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_orientation_accuracy.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->orientation_accuracy);
      offset += this->pose.serialize(outbuffer + offset);
      offset += this->motion_vector.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->vehicle_heading);
      offset += serializeAvrFloat64(outbuffer + offset, this->motion_heading);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      this->sensor_stamp =  ((uint32_t) (*(inbuffer + offset)));
      this->sensor_stamp |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->sensor_stamp |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->sensor_stamp |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->sensor_stamp);
      this->received_stamp =  ((uint32_t) (*(inbuffer + offset)));
      this->received_stamp |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->received_stamp |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->received_stamp |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->received_stamp);
      this->source =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->source);
      this->flags =  ((uint16_t) (*(inbuffer + offset)));
      this->flags |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->flags);
      this->orientation_valid =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->orientation_valid);
      this->motion_vector_valid =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->motion_vector_valid);
      union {
        float real;
        uint32_t base;
      } u_position_accuracy;
      u_position_accuracy.base = 0;
      u_position_accuracy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_position_accuracy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_position_accuracy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_position_accuracy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->position_accuracy = u_position_accuracy.real;
      offset += sizeof(this->position_accuracy);
      union {
        float real;
        uint32_t base;
      } u_orientation_accuracy;
      u_orientation_accuracy.base = 0;
      u_orientation_accuracy.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_orientation_accuracy.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_orientation_accuracy.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_orientation_accuracy.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->orientation_accuracy = u_orientation_accuracy.real;
      offset += sizeof(this->orientation_accuracy);
      offset += this->pose.deserialize(inbuffer + offset);
      offset += this->motion_vector.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->vehicle_heading));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->motion_heading));
     return offset;
    }

    virtual const char * getType() override { return "xbot_msgs/AbsolutePose"; };
    virtual const char * getMD5() override { return "ca30a78465d73c61712ddbbfef2442b7"; };

  };

}
#endif
