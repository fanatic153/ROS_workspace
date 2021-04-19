// Generated by gencpp from file beginner_tutorials/my_srvRequest.msg
// DO NOT EDIT!


#ifndef BEGINNER_TUTORIALS_MESSAGE_MY_SRVREQUEST_H
#define BEGINNER_TUTORIALS_MESSAGE_MY_SRVREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace beginner_tutorials
{
template <class ContainerAllocator>
struct my_srvRequest_
{
  typedef my_srvRequest_<ContainerAllocator> Type;

  my_srvRequest_()
    : id(0)  {
    }
  my_srvRequest_(const ContainerAllocator& _alloc)
    : id(0)  {
  (void)_alloc;
    }



   typedef int64_t _id_type;
  _id_type id;





  typedef boost::shared_ptr< ::beginner_tutorials::my_srvRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::beginner_tutorials::my_srvRequest_<ContainerAllocator> const> ConstPtr;

}; // struct my_srvRequest_

typedef ::beginner_tutorials::my_srvRequest_<std::allocator<void> > my_srvRequest;

typedef boost::shared_ptr< ::beginner_tutorials::my_srvRequest > my_srvRequestPtr;
typedef boost::shared_ptr< ::beginner_tutorials::my_srvRequest const> my_srvRequestConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::beginner_tutorials::my_srvRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::beginner_tutorials::my_srvRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::beginner_tutorials::my_srvRequest_<ContainerAllocator1> & lhs, const ::beginner_tutorials::my_srvRequest_<ContainerAllocator2> & rhs)
{
  return lhs.id == rhs.id;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::beginner_tutorials::my_srvRequest_<ContainerAllocator1> & lhs, const ::beginner_tutorials::my_srvRequest_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace beginner_tutorials

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::beginner_tutorials::my_srvRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::beginner_tutorials::my_srvRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::beginner_tutorials::my_srvRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::beginner_tutorials::my_srvRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::beginner_tutorials::my_srvRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::beginner_tutorials::my_srvRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::beginner_tutorials::my_srvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "ef7df1d34137d3879d089ad803388efa";
  }

  static const char* value(const ::beginner_tutorials::my_srvRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0xef7df1d34137d387ULL;
  static const uint64_t static_value2 = 0x9d089ad803388efaULL;
};

template<class ContainerAllocator>
struct DataType< ::beginner_tutorials::my_srvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "beginner_tutorials/my_srvRequest";
  }

  static const char* value(const ::beginner_tutorials::my_srvRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::beginner_tutorials::my_srvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int64 id\n"
;
  }

  static const char* value(const ::beginner_tutorials::my_srvRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::beginner_tutorials::my_srvRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.id);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct my_srvRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::beginner_tutorials::my_srvRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::beginner_tutorials::my_srvRequest_<ContainerAllocator>& v)
  {
    s << indent << "id: ";
    Printer<int64_t>::stream(s, indent + "  ", v.id);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BEGINNER_TUTORIALS_MESSAGE_MY_SRVREQUEST_H
