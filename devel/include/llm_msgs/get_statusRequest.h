// Generated by gencpp from file llm_msgs/get_statusRequest.msg
// DO NOT EDIT!


#ifndef LLM_MSGS_MESSAGE_GET_STATUSREQUEST_H
#define LLM_MSGS_MESSAGE_GET_STATUSREQUEST_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace llm_msgs
{
template <class ContainerAllocator>
struct get_statusRequest_
{
  typedef get_statusRequest_<ContainerAllocator> Type;

  get_statusRequest_()
    {
    }
  get_statusRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::llm_msgs::get_statusRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::llm_msgs::get_statusRequest_<ContainerAllocator> const> ConstPtr;

}; // struct get_statusRequest_

typedef ::llm_msgs::get_statusRequest_<std::allocator<void> > get_statusRequest;

typedef boost::shared_ptr< ::llm_msgs::get_statusRequest > get_statusRequestPtr;
typedef boost::shared_ptr< ::llm_msgs::get_statusRequest const> get_statusRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::llm_msgs::get_statusRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::llm_msgs::get_statusRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace llm_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::llm_msgs::get_statusRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::llm_msgs::get_statusRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::llm_msgs::get_statusRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::llm_msgs::get_statusRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::llm_msgs::get_statusRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::llm_msgs::get_statusRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::llm_msgs::get_statusRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::llm_msgs::get_statusRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::llm_msgs::get_statusRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "llm_msgs/get_statusRequest";
  }

  static const char* value(const ::llm_msgs::get_statusRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::llm_msgs::get_statusRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::llm_msgs::get_statusRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::llm_msgs::get_statusRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct get_statusRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::llm_msgs::get_statusRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::llm_msgs::get_statusRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // LLM_MSGS_MESSAGE_GET_STATUSREQUEST_H
