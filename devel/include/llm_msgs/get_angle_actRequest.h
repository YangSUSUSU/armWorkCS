// Generated by gencpp from file llm_msgs/get_angle_actRequest.msg
// DO NOT EDIT!


#ifndef LLM_MSGS_MESSAGE_GET_ANGLE_ACTREQUEST_H
#define LLM_MSGS_MESSAGE_GET_ANGLE_ACTREQUEST_H


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
struct get_angle_actRequest_
{
  typedef get_angle_actRequest_<ContainerAllocator> Type;

  get_angle_actRequest_()
    {
    }
  get_angle_actRequest_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::llm_msgs::get_angle_actRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::llm_msgs::get_angle_actRequest_<ContainerAllocator> const> ConstPtr;

}; // struct get_angle_actRequest_

typedef ::llm_msgs::get_angle_actRequest_<std::allocator<void> > get_angle_actRequest;

typedef boost::shared_ptr< ::llm_msgs::get_angle_actRequest > get_angle_actRequestPtr;
typedef boost::shared_ptr< ::llm_msgs::get_angle_actRequest const> get_angle_actRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::llm_msgs::get_angle_actRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::llm_msgs::get_angle_actRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace llm_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::llm_msgs::get_angle_actRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::llm_msgs::get_angle_actRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::llm_msgs::get_angle_actRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::llm_msgs::get_angle_actRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::llm_msgs::get_angle_actRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::llm_msgs::get_angle_actRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::llm_msgs::get_angle_actRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::llm_msgs::get_angle_actRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::llm_msgs::get_angle_actRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "llm_msgs/get_angle_actRequest";
  }

  static const char* value(const ::llm_msgs::get_angle_actRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::llm_msgs::get_angle_actRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::llm_msgs::get_angle_actRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::llm_msgs::get_angle_actRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct get_angle_actRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::llm_msgs::get_angle_actRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::llm_msgs::get_angle_actRequest_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // LLM_MSGS_MESSAGE_GET_ANGLE_ACTREQUEST_H
