// Generated by gencpp from file llm_msgs/set_force.msg
// DO NOT EDIT!


#ifndef LLM_MSGS_MESSAGE_SET_FORCE_H
#define LLM_MSGS_MESSAGE_SET_FORCE_H

#include <ros/service_traits.h>


#include <llm_msgs/set_forceRequest.h>
#include <llm_msgs/set_forceResponse.h>


namespace llm_msgs
{

struct set_force
{

typedef set_forceRequest Request;
typedef set_forceResponse Response;
Request request;
Response response;

typedef Request RequestType;
typedef Response ResponseType;

}; // struct set_force
} // namespace llm_msgs


namespace ros
{
namespace service_traits
{


template<>
struct MD5Sum< ::llm_msgs::set_force > {
  static const char* value()
  {
    return "6b0fdeb4ed7ee4c97030abfd78488ebb";
  }

  static const char* value(const ::llm_msgs::set_force&) { return value(); }
};

template<>
struct DataType< ::llm_msgs::set_force > {
  static const char* value()
  {
    return "llm_msgs/set_force";
  }

  static const char* value(const ::llm_msgs::set_force&) { return value(); }
};


// service_traits::MD5Sum< ::llm_msgs::set_forceRequest> should match
// service_traits::MD5Sum< ::llm_msgs::set_force >
template<>
struct MD5Sum< ::llm_msgs::set_forceRequest>
{
  static const char* value()
  {
    return MD5Sum< ::llm_msgs::set_force >::value();
  }
  static const char* value(const ::llm_msgs::set_forceRequest&)
  {
    return value();
  }
};

// service_traits::DataType< ::llm_msgs::set_forceRequest> should match
// service_traits::DataType< ::llm_msgs::set_force >
template<>
struct DataType< ::llm_msgs::set_forceRequest>
{
  static const char* value()
  {
    return DataType< ::llm_msgs::set_force >::value();
  }
  static const char* value(const ::llm_msgs::set_forceRequest&)
  {
    return value();
  }
};

// service_traits::MD5Sum< ::llm_msgs::set_forceResponse> should match
// service_traits::MD5Sum< ::llm_msgs::set_force >
template<>
struct MD5Sum< ::llm_msgs::set_forceResponse>
{
  static const char* value()
  {
    return MD5Sum< ::llm_msgs::set_force >::value();
  }
  static const char* value(const ::llm_msgs::set_forceResponse&)
  {
    return value();
  }
};

// service_traits::DataType< ::llm_msgs::set_forceResponse> should match
// service_traits::DataType< ::llm_msgs::set_force >
template<>
struct DataType< ::llm_msgs::set_forceResponse>
{
  static const char* value()
  {
    return DataType< ::llm_msgs::set_force >::value();
  }
  static const char* value(const ::llm_msgs::set_forceResponse&)
  {
    return value();
  }
};

} // namespace service_traits
} // namespace ros

#endif // LLM_MSGS_MESSAGE_SET_FORCE_H
