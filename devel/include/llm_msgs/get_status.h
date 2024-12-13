// Generated by gencpp from file llm_msgs/get_status.msg
// DO NOT EDIT!


#ifndef LLM_MSGS_MESSAGE_GET_STATUS_H
#define LLM_MSGS_MESSAGE_GET_STATUS_H

#include <ros/service_traits.h>


#include <llm_msgs/get_statusRequest.h>
#include <llm_msgs/get_statusResponse.h>


namespace llm_msgs
{

struct get_status
{

typedef get_statusRequest Request;
typedef get_statusResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct get_status
} // namespace llm_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::llm_msgs::get_status > {
  static const char* value()
  {
    return "7f7cdabc561edd46a5ab52db3c6d6e89";
  }

  static const char* value(const ::llm_msgs::get_status&) { return value(); }
};

template<>
struct DataType< ::llm_msgs::get_status > {
  static const char* value()
  {
    return "llm_msgs/get_status";
  }

  static const char* value(const ::llm_msgs::get_status&) { return value(); }
};


// service_traits::MD5Sum< ::llm_msgs::get_statusRequest> should match
// service_traits::MD5Sum< ::llm_msgs::get_status >
template<>
struct MD5Sum< ::llm_msgs::get_statusRequest>
{
  static const char* value()
  {
    return MD5Sum< ::llm_msgs::get_status >::value();
  }
  static const char* value(const ::llm_msgs::get_statusRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::llm_msgs::get_statusRequest> should match
// service_traits::DataType< ::llm_msgs::get_status >
template<>
struct DataType< ::llm_msgs::get_statusRequest>
{
  static const char* value()
  {
    return DataType< ::llm_msgs::get_status >::value();
  }
  static const char* value(const ::llm_msgs::get_statusRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::llm_msgs::get_statusResponse> should match
// service_traits::MD5Sum< ::llm_msgs::get_status >
template<>
struct MD5Sum< ::llm_msgs::get_statusResponse>
{
  static const char* value()
  {
    return MD5Sum< ::llm_msgs::get_status >::value();
  }
  static const char* value(const ::llm_msgs::get_statusResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::llm_msgs::get_statusResponse> should match
// service_traits::DataType< ::llm_msgs::get_status >
template<>
struct DataType< ::llm_msgs::get_statusResponse>
{
  static const char* value()
  {
    return DataType< ::llm_msgs::get_status >::value();
  }
  static const char* value(const ::llm_msgs::get_statusResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // LLM_MSGS_MESSAGE_GET_STATUS_H