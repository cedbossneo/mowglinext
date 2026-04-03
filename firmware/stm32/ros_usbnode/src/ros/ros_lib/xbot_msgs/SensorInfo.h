#ifndef _ROS_xbot_msgs_SensorInfo_h
#define _ROS_xbot_msgs_SensorInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace xbot_msgs
{

  class SensorInfo : public ros::Msg
  {
    public:
      typedef const char* _sensor_id_type;
      _sensor_id_type sensor_id;
      typedef const char* _sensor_name_type;
      _sensor_name_type sensor_name;
      typedef uint8_t _value_type_type;
      _value_type_type value_type;
      typedef uint8_t _value_description_type;
      _value_description_type value_description;
      typedef const char* _unit_type;
      _unit_type unit;
      typedef bool _has_min_max_type;
      _has_min_max_type has_min_max;
      typedef float _min_value_type;
      _min_value_type min_value;
      typedef float _max_value_type;
      _max_value_type max_value;
      typedef bool _has_critical_low_type;
      _has_critical_low_type has_critical_low;
      typedef float _lower_critical_value_type;
      _lower_critical_value_type lower_critical_value;
      typedef bool _has_critical_high_type;
      _has_critical_high_type has_critical_high;
      typedef float _upper_critical_value_type;
      _upper_critical_value_type upper_critical_value;
      enum { TYPE_STRING = 1 };
      enum { TYPE_DOUBLE = 2 };
      enum { VALUE_DESCRIPTION_UNKNOWN = 0 };
      enum { VALUE_DESCRIPTION_TEMPERATURE = 1 };
      enum { VALUE_DESCRIPTION_VELOCITY = 2 };
      enum { VALUE_DESCRIPTION_ACCELERATION = 3 };
      enum { VALUE_DESCRIPTION_VOLTAGE = 4 };
      enum { VALUE_DESCRIPTION_CURRENT = 5 };
      enum { VALUE_DESCRIPTION_PERCENT = 6 };
      enum { VALUE_DESCRIPTION_DISTANCE = 7 };
      enum { VALUE_DESCRIPTION_RPM = 8 };
      enum { FLAG_GPS_RTK = 1                    };
      enum { FLAG_GPS_RTK_FIXED = 2              };
      enum { FLAG_GPS_RTK_FLOAT = 4              };
      enum { FLAG_GPS_DEAD_RECKONING = 8         };
      enum { FLAG_SENSOR_FUSION_RECENT_ABSOLUTE_POSE = 1     };
      enum { FLAG_SENSOR_FUSION_DEAD_RECKONING = 8           };

    SensorInfo():
      sensor_id(""),
      sensor_name(""),
      value_type(0),
      value_description(0),
      unit(""),
      has_min_max(0),
      min_value(0),
      max_value(0),
      has_critical_low(0),
      lower_critical_value(0),
      has_critical_high(0),
      upper_critical_value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_sensor_id = strlen(this->sensor_id);
      varToArr(outbuffer + offset, length_sensor_id);
      offset += 4;
      memcpy(outbuffer + offset, this->sensor_id, length_sensor_id);
      offset += length_sensor_id;
      uint32_t length_sensor_name = strlen(this->sensor_name);
      varToArr(outbuffer + offset, length_sensor_name);
      offset += 4;
      memcpy(outbuffer + offset, this->sensor_name, length_sensor_name);
      offset += length_sensor_name;
      *(outbuffer + offset + 0) = (this->value_type >> (8 * 0)) & 0xFF;
      offset += sizeof(this->value_type);
      *(outbuffer + offset + 0) = (this->value_description >> (8 * 0)) & 0xFF;
      offset += sizeof(this->value_description);
      uint32_t length_unit = strlen(this->unit);
      varToArr(outbuffer + offset, length_unit);
      offset += 4;
      memcpy(outbuffer + offset, this->unit, length_unit);
      offset += length_unit;
      union {
        bool real;
        uint8_t base;
      } u_has_min_max;
      u_has_min_max.real = this->has_min_max;
      *(outbuffer + offset + 0) = (u_has_min_max.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->has_min_max);
      offset += serializeAvrFloat64(outbuffer + offset, this->min_value);
      offset += serializeAvrFloat64(outbuffer + offset, this->max_value);
      union {
        bool real;
        uint8_t base;
      } u_has_critical_low;
      u_has_critical_low.real = this->has_critical_low;
      *(outbuffer + offset + 0) = (u_has_critical_low.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->has_critical_low);
      offset += serializeAvrFloat64(outbuffer + offset, this->lower_critical_value);
      union {
        bool real;
        uint8_t base;
      } u_has_critical_high;
      u_has_critical_high.real = this->has_critical_high;
      *(outbuffer + offset + 0) = (u_has_critical_high.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->has_critical_high);
      offset += serializeAvrFloat64(outbuffer + offset, this->upper_critical_value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_sensor_id;
      arrToVar(length_sensor_id, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_sensor_id; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_sensor_id-1]=0;
      this->sensor_id = (char *)(inbuffer + offset-1);
      offset += length_sensor_id;
      uint32_t length_sensor_name;
      arrToVar(length_sensor_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_sensor_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_sensor_name-1]=0;
      this->sensor_name = (char *)(inbuffer + offset-1);
      offset += length_sensor_name;
      this->value_type =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->value_type);
      this->value_description =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->value_description);
      uint32_t length_unit;
      arrToVar(length_unit, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_unit; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_unit-1]=0;
      this->unit = (char *)(inbuffer + offset-1);
      offset += length_unit;
      union {
        bool real;
        uint8_t base;
      } u_has_min_max;
      u_has_min_max.base = 0;
      u_has_min_max.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->has_min_max = u_has_min_max.real;
      offset += sizeof(this->has_min_max);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->min_value));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->max_value));
      union {
        bool real;
        uint8_t base;
      } u_has_critical_low;
      u_has_critical_low.base = 0;
      u_has_critical_low.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->has_critical_low = u_has_critical_low.real;
      offset += sizeof(this->has_critical_low);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->lower_critical_value));
      union {
        bool real;
        uint8_t base;
      } u_has_critical_high;
      u_has_critical_high.base = 0;
      u_has_critical_high.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->has_critical_high = u_has_critical_high.real;
      offset += sizeof(this->has_critical_high);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->upper_critical_value));
     return offset;
    }

    virtual const char * getType() override { return "xbot_msgs/SensorInfo"; };
    virtual const char * getMD5() override { return "eddc577c6e0d0709fafe9f34191b23ae"; };

  };

}
#endif
