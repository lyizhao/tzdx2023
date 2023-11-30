// Generated by gencpp from file plan_env_msgs/ChunkStamps.msg
// DO NOT EDIT!


#ifndef PLAN_ENV_MSGS_MESSAGE_CHUNKSTAMPS_H
#define PLAN_ENV_MSGS_MESSAGE_CHUNKSTAMPS_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <exploration_fame/plan_env_msgs/IdxList.h>

namespace plan_env_msgs
{
template <class ContainerAllocator>
struct ChunkStamps_
{
  typedef ChunkStamps_<ContainerAllocator> Type;

  ChunkStamps_()
    : from_drone_id(0)
    , idx_lists()
    , time(0.0)  {
    }
  ChunkStamps_(const ContainerAllocator& _alloc)
    : from_drone_id(0)
    , idx_lists(_alloc)
    , time(0.0)  {
  (void)_alloc;
    }



   typedef int32_t _from_drone_id_type;
  _from_drone_id_type from_drone_id;

   typedef std::vector< ::plan_env_msgs::IdxList_<ContainerAllocator> , typename std::allocator_traits<ContainerAllocator>::template rebind_alloc< ::plan_env_msgs::IdxList_<ContainerAllocator> >> _idx_lists_type;
  _idx_lists_type idx_lists;

   typedef double _time_type;
  _time_type time;





  typedef boost::shared_ptr< ::plan_env_msgs::ChunkStamps_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::plan_env_msgs::ChunkStamps_<ContainerAllocator> const> ConstPtr;

}; // struct ChunkStamps_

typedef ::plan_env_msgs::ChunkStamps_<std::allocator<void> > ChunkStamps;

typedef boost::shared_ptr< ::plan_env_msgs::ChunkStamps > ChunkStampsPtr;
typedef boost::shared_ptr< ::plan_env_msgs::ChunkStamps const> ChunkStampsConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::plan_env_msgs::ChunkStamps_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::plan_env_msgs::ChunkStamps_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::plan_env_msgs::ChunkStamps_<ContainerAllocator1> & lhs, const ::plan_env_msgs::ChunkStamps_<ContainerAllocator2> & rhs)
{
  return lhs.from_drone_id == rhs.from_drone_id &&
    lhs.idx_lists == rhs.idx_lists &&
    lhs.time == rhs.time;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::plan_env_msgs::ChunkStamps_<ContainerAllocator1> & lhs, const ::plan_env_msgs::ChunkStamps_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace plan_env_msgs

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::plan_env_msgs::ChunkStamps_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::plan_env_msgs::ChunkStamps_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::plan_env_msgs::ChunkStamps_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::plan_env_msgs::ChunkStamps_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::plan_env_msgs::ChunkStamps_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::plan_env_msgs::ChunkStamps_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::plan_env_msgs::ChunkStamps_<ContainerAllocator> >
{
  static const char* value()
  {
    return "4649dd6de5a06d0faa920a6802067e3f";
  }

  static const char* value(const ::plan_env_msgs::ChunkStamps_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x4649dd6de5a06d0fULL;
  static const uint64_t static_value2 = 0xaa920a6802067e3fULL;
};

template<class ContainerAllocator>
struct DataType< ::plan_env_msgs::ChunkStamps_<ContainerAllocator> >
{
  static const char* value()
  {
    return "plan_env_msgs/ChunkStamps";
  }

  static const char* value(const ::plan_env_msgs::ChunkStamps_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::plan_env_msgs::ChunkStamps_<ContainerAllocator> >
{
  static const char* value()
  {
    return "\n"
"int32 from_drone_id\n"
"IdxList[] idx_lists\n"
"float64 time\n"
"================================================================================\n"
"MSG: plan_env_msgs/IdxList\n"
"int32[] ids\n"
;
  }

  static const char* value(const ::plan_env_msgs::ChunkStamps_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::plan_env_msgs::ChunkStamps_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.from_drone_id);
      stream.next(m.idx_lists);
      stream.next(m.time);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct ChunkStamps_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::plan_env_msgs::ChunkStamps_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::plan_env_msgs::ChunkStamps_<ContainerAllocator>& v)
  {
    s << indent << "from_drone_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.from_drone_id);
    s << indent << "idx_lists[]" << std::endl;
    for (size_t i = 0; i < v.idx_lists.size(); ++i)
    {
      s << indent << "  idx_lists[" << i << "]: ";
      s << std::endl;
      s << indent;
      Printer< ::plan_env_msgs::IdxList_<ContainerAllocator> >::stream(s, indent + "    ", v.idx_lists[i]);
    }
    s << indent << "time: ";
    Printer<double>::stream(s, indent + "  ", v.time);
  }
};

} // namespace message_operations
} // namespace ros

#endif // PLAN_ENV_MSGS_MESSAGE_CHUNKSTAMPS_H
