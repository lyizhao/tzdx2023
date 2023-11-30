// Generated by gencpp from file exploration_manager/DroneState.msg
// DO NOT EDIT!


#ifndef EXPLORATION_MANAGER_MESSAGE_DRONESTATE_H
#define EXPLORATION_MANAGER_MESSAGE_DRONESTATE_H


#include <string>
#include <vector>
#include <memory>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <geometry_msgs/Point.h>

namespace exploration_manager
{
template <class ContainerAllocator>
struct DroneState_
{
  typedef DroneState_<ContainerAllocator> Type;

  DroneState_()
    : drone_id(0)
    , grid_ids()
    , recent_attempt_time(0.0)
    , stamp(0.0)
    , pos()
    , vel()
    , yaw(0.0)
    , goal_posit()
    , role(0)  {
    }
  DroneState_(const ContainerAllocator& _alloc)
    : drone_id(0)
    , grid_ids(_alloc)
    , recent_attempt_time(0.0)
    , stamp(0.0)
    , pos(_alloc)
    , vel(_alloc)
    , yaw(0.0)
    , goal_posit(_alloc)
    , role(0)  {
  (void)_alloc;
    }



   typedef int32_t _drone_id_type;
  _drone_id_type drone_id;

   typedef std::vector<int8_t, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<int8_t>> _grid_ids_type;
  _grid_ids_type grid_ids;

   typedef double _recent_attempt_time_type;
  _recent_attempt_time_type recent_attempt_time;

   typedef double _stamp_type;
  _stamp_type stamp;

   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _pos_type;
  _pos_type pos;

   typedef std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> _vel_type;
  _vel_type vel;

   typedef float _yaw_type;
  _yaw_type yaw;

   typedef  ::geometry_msgs::Point_<ContainerAllocator>  _goal_posit_type;
  _goal_posit_type goal_posit;

   typedef int8_t _role_type;
  _role_type role;





  typedef boost::shared_ptr< ::exploration_manager::DroneState_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::exploration_manager::DroneState_<ContainerAllocator> const> ConstPtr;

}; // struct DroneState_

typedef ::exploration_manager::DroneState_<std::allocator<void> > DroneState;

typedef boost::shared_ptr< ::exploration_manager::DroneState > DroneStatePtr;
typedef boost::shared_ptr< ::exploration_manager::DroneState const> DroneStateConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::exploration_manager::DroneState_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::exploration_manager::DroneState_<ContainerAllocator> >::stream(s, "", v);
return s;
}


template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator==(const ::exploration_manager::DroneState_<ContainerAllocator1> & lhs, const ::exploration_manager::DroneState_<ContainerAllocator2> & rhs)
{
  return lhs.drone_id == rhs.drone_id &&
    lhs.grid_ids == rhs.grid_ids &&
    lhs.recent_attempt_time == rhs.recent_attempt_time &&
    lhs.stamp == rhs.stamp &&
    lhs.pos == rhs.pos &&
    lhs.vel == rhs.vel &&
    lhs.yaw == rhs.yaw &&
    lhs.goal_posit == rhs.goal_posit &&
    lhs.role == rhs.role;
}

template<typename ContainerAllocator1, typename ContainerAllocator2>
bool operator!=(const ::exploration_manager::DroneState_<ContainerAllocator1> & lhs, const ::exploration_manager::DroneState_<ContainerAllocator2> & rhs)
{
  return !(lhs == rhs);
}


} // namespace exploration_manager

namespace ros
{
namespace message_traits
{





template <class ContainerAllocator>
struct IsMessage< ::exploration_manager::DroneState_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::exploration_manager::DroneState_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::exploration_manager::DroneState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::exploration_manager::DroneState_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::exploration_manager::DroneState_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::exploration_manager::DroneState_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::exploration_manager::DroneState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "44423b0a2449be0e1b2083c475a58ca5";
  }

  static const char* value(const ::exploration_manager::DroneState_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x44423b0a2449be0eULL;
  static const uint64_t static_value2 = 0x1b2083c475a58ca5ULL;
};

template<class ContainerAllocator>
struct DataType< ::exploration_manager::DroneState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "exploration_manager/DroneState";
  }

  static const char* value(const ::exploration_manager::DroneState_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::exploration_manager::DroneState_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 drone_id\n"
"\n"
"int8[] grid_ids\n"
"float64 recent_attempt_time\n"
"float64 stamp\n"
"\n"
"# only used for simulation\n"
"float32[] pos\n"
"float32[] vel\n"
"float32 yaw\n"
"\n"
"# FAME\n"
"geometry_msgs/Point goal_posit\n"
"int8 role\n"
"\n"
"================================================================================\n"
"MSG: geometry_msgs/Point\n"
"# This contains the position of a point in free space\n"
"float64 x\n"
"float64 y\n"
"float64 z\n"
;
  }

  static const char* value(const ::exploration_manager::DroneState_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::exploration_manager::DroneState_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.drone_id);
      stream.next(m.grid_ids);
      stream.next(m.recent_attempt_time);
      stream.next(m.stamp);
      stream.next(m.pos);
      stream.next(m.vel);
      stream.next(m.yaw);
      stream.next(m.goal_posit);
      stream.next(m.role);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct DroneState_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::exploration_manager::DroneState_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::exploration_manager::DroneState_<ContainerAllocator>& v)
  {
    s << indent << "drone_id: ";
    Printer<int32_t>::stream(s, indent + "  ", v.drone_id);
    s << indent << "grid_ids[]" << std::endl;
    for (size_t i = 0; i < v.grid_ids.size(); ++i)
    {
      s << indent << "  grid_ids[" << i << "]: ";
      Printer<int8_t>::stream(s, indent + "  ", v.grid_ids[i]);
    }
    s << indent << "recent_attempt_time: ";
    Printer<double>::stream(s, indent + "  ", v.recent_attempt_time);
    s << indent << "stamp: ";
    Printer<double>::stream(s, indent + "  ", v.stamp);
    s << indent << "pos[]" << std::endl;
    for (size_t i = 0; i < v.pos.size(); ++i)
    {
      s << indent << "  pos[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.pos[i]);
    }
    s << indent << "vel[]" << std::endl;
    for (size_t i = 0; i < v.vel.size(); ++i)
    {
      s << indent << "  vel[" << i << "]: ";
      Printer<float>::stream(s, indent + "  ", v.vel[i]);
    }
    s << indent << "yaw: ";
    Printer<float>::stream(s, indent + "  ", v.yaw);
    s << indent << "goal_posit: ";
    s << std::endl;
    Printer< ::geometry_msgs::Point_<ContainerAllocator> >::stream(s, indent + "  ", v.goal_posit);
    s << indent << "role: ";
    Printer<int8_t>::stream(s, indent + "  ", v.role);
  }
};

} // namespace message_operations
} // namespace ros

#endif // EXPLORATION_MANAGER_MESSAGE_DRONESTATE_H
