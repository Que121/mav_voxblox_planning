// Generated by gencpp from file voxblox_msgs/FilePathResponse.msg
// DO NOT EDIT!


#ifndef VOXBLOX_MSGS_MESSAGE_FILEPATHRESPONSE_H
#define VOXBLOX_MSGS_MESSAGE_FILEPATHRESPONSE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace voxblox_msgs
{
template <class ContainerAllocator>
struct FilePathResponse_
{
  typedef FilePathResponse_<ContainerAllocator> Type;

  FilePathResponse_()
    {
    }
  FilePathResponse_(const ContainerAllocator& _alloc)
    {
  (void)_alloc;
    }







  typedef boost::shared_ptr< ::voxblox_msgs::FilePathResponse_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::voxblox_msgs::FilePathResponse_<ContainerAllocator> const> ConstPtr;

}; // struct FilePathResponse_

typedef ::voxblox_msgs::FilePathResponse_<std::allocator<void> > FilePathResponse;

typedef boost::shared_ptr< ::voxblox_msgs::FilePathResponse > FilePathResponsePtr;
typedef boost::shared_ptr< ::voxblox_msgs::FilePathResponse const> FilePathResponseConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::voxblox_msgs::FilePathResponse_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::voxblox_msgs::FilePathResponse_<ContainerAllocator> >::stream(s, "", v);
return s;
}


} // namespace voxblox_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::voxblox_msgs::FilePathResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::voxblox_msgs::FilePathResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::voxblox_msgs::FilePathResponse_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::voxblox_msgs::FilePathResponse_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::voxblox_msgs::FilePathResponse_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::voxblox_msgs::FilePathResponse_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::voxblox_msgs::FilePathResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "d41d8cd98f00b204e9800998ecf8427e";
  }

  static const char* value(const ::voxblox_msgs::FilePathResponse_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xd41d8cd98f00b204ULL;
  static const uint64_t static_value2 = 0xe9800998ecf8427eULL;
};

template<class ContainerAllocator>
struct DataType< ::voxblox_msgs::FilePathResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "voxblox_msgs/FilePathResponse";
  }

  static const char* value(const ::voxblox_msgs::FilePathResponse_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::voxblox_msgs::FilePathResponse_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
;
  }

  static const char* value(const ::voxblox_msgs::FilePathResponse_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::voxblox_msgs::FilePathResponse_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream&, T)
    {}

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct FilePathResponse_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::voxblox_msgs::FilePathResponse_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream&, const std::string&, const ::voxblox_msgs::FilePathResponse_<ContainerAllocator>&)
  {}
};

} // namespace message_operations
} // namespace ros

#endif // VOXBLOX_MSGS_MESSAGE_FILEPATHRESPONSE_H
