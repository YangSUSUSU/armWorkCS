// Generated by gencpp from file llm_msgs/get_angle_act.msg
// DO NOT EDIT!


#ifndef LLM_MSGS_MESSAGE_GET_ANGLE_ACT_H
#define LLM_MSGS_MESSAGE_GET_ANGLE_ACT_H

#include <ros/service_traits.h>


#include <llm_msgs/get_angle_actRequest.h>
#include <llm_msgs/get_angle_actResponse.h>


namespace llm_msgs
{

struct get_angle_act
{

typedef get_angle_actRequest Request;
typedef get_angle_actResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct get_angle_act
} // namespace llm_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::llm_msgs::get_angle_act > {
  static const char* value()
  {
    return "5a40d1cc41b6533f906a6a90344676b6";
  }

  static const char* value(const ::llm_msgs::get_angle_act&) { return value(); }
};

template<>
struct DataType< ::llm_msgs::get_angle_act > {
  static const char* value()
  {
    return "llm_msgs/get_angle_act";
  }

  static const char* value(const ::llm_msgs::get_angle_act&) { return value(); }
};


// service_traits::MD5Sum< ::llm_msgs::get_angle_actRequest> should match
// service_traits::MD5Sum< ::llm_msgs::get_angle_act >
template<>
struct MD5Sum< ::llm_msgs::get_angle_actRequest>
{
  static const char* value()
  {
    return MD5Sum< ::llm_msgs::get_angle_act >::value();
  }
  static const char* value(const ::llm_msgs::get_angle_actRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::llm_msgs::get_angle_actRequest> should match
// service_traits::DataType< ::llm_msgs::get_angle_act >
template<>
struct DataType< ::llm_msgs::get_angle_actRequest>
{
  static const char* value()
  {
    return DataType< ::llm_msgs::get_angle_act >::value();
  }
  static const char* value(const ::llm_msgs::get_angle_actRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::llm_msgs::get_angle_actResponse> should match
// service_traits::MD5Sum< ::llm_msgs::get_angle_act >
template<>
struct MD5Sum< ::llm_msgs::get_angle_actResponse>
{
  static const char* value()
  {
    return MD5Sum< ::llm_msgs::get_angle_act >::value();
  }
  static const char* value(const ::llm_msgs::get_angle_actResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::llm_msgs::get_angle_actResponse> should match
// service_traits::DataType< ::llm_msgs::get_angle_act >
template<>
struct DataType< ::llm_msgs::get_angle_actResponse>
{
  static const char* value()
  {
    return DataType< ::llm_msgs::get_angle_act >::value();
  }
  static const char* value(const ::llm_msgs::get_angle_actResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // LLM_MSGS_MESSAGE_GET_ANGLE_ACT_H
