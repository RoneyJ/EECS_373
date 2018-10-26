// Generated by gencpp from file baxter_playfile_nodes/playfileSrvRequest.msg
// DO NOT EDIT!


#ifndef BAXTER_PLAYFILE_NODES_MESSAGE_PLAYFILESRVREQUEST_H
#define BAXTER_PLAYFILE_NODES_MESSAGE_PLAYFILESRVREQUEST_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>


namespace baxter_playfile_nodes
{
template <class ContainerAllocator>
struct playfileSrvRequest_
{
  typedef playfileSrvRequest_<ContainerAllocator> Type;

  playfileSrvRequest_()
    : playfile_code(0)  {
    }
  playfileSrvRequest_(const ContainerAllocator& _alloc)
    : playfile_code(0)  {
  (void)_alloc;
    }



   typedef int32_t _playfile_code_type;
  _playfile_code_type playfile_code;



  enum {
    PRE_POSE = 0,
    DEMO_TRAJ = 1,
    SHY = 2,
    HUG = 3,
    SHAKE = 4,
    STICK_EM_UP = 5,
    WAVE = 6,
  };


  typedef boost::shared_ptr< ::baxter_playfile_nodes::playfileSrvRequest_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::baxter_playfile_nodes::playfileSrvRequest_<ContainerAllocator> const> ConstPtr;

}; // struct playfileSrvRequest_

typedef ::baxter_playfile_nodes::playfileSrvRequest_<std::allocator<void> > playfileSrvRequest;

typedef boost::shared_ptr< ::baxter_playfile_nodes::playfileSrvRequest > playfileSrvRequestPtr;
typedef boost::shared_ptr< ::baxter_playfile_nodes::playfileSrvRequest const> playfileSrvRequestConstPtr;

// constants requiring out of line definition

   

   

   

   

   

   

   



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::baxter_playfile_nodes::playfileSrvRequest_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::baxter_playfile_nodes::playfileSrvRequest_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace baxter_playfile_nodes

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': True, 'IsMessage': True, 'HasHeader': False}
// {'roscpp': ['/opt/ros/kinetic/share/roscpp/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'actionlib': ['/opt/ros/kinetic/share/actionlib/cmake/../msg'], 'trajectory_msgs': ['/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'baxter_core_msgs': ['/home/jproney/ros_ws/src/learning_ros_external_pkgs_kinetic/baxter_common/baxter_core_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'baxter_trajectory_streamer': ['/home/jproney/ros_ws/devel/share/baxter_trajectory_streamer/msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::baxter_playfile_nodes::playfileSrvRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::baxter_playfile_nodes::playfileSrvRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::baxter_playfile_nodes::playfileSrvRequest_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::baxter_playfile_nodes::playfileSrvRequest_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::baxter_playfile_nodes::playfileSrvRequest_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::baxter_playfile_nodes::playfileSrvRequest_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::baxter_playfile_nodes::playfileSrvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "74e1052863ffa71232dcc0726a94464c";
  }

  static const char* value(const ::baxter_playfile_nodes::playfileSrvRequest_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x74e1052863ffa712ULL;
  static const uint64_t static_value2 = 0x32dcc0726a94464cULL;
};

template<class ContainerAllocator>
struct DataType< ::baxter_playfile_nodes::playfileSrvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "baxter_playfile_nodes/playfileSrvRequest";
  }

  static const char* value(const ::baxter_playfile_nodes::playfileSrvRequest_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::baxter_playfile_nodes::playfileSrvRequest_<ContainerAllocator> >
{
  static const char* value()
  {
    return "int32 PRE_POSE=0\n\
int32 DEMO_TRAJ=1\n\
int32 SHY=2\n\
int32 HUG=3\n\
int32 SHAKE=4\n\
int32 STICK_EM_UP=5\n\
int32 WAVE=6\n\
\n\
int32 playfile_code\n\
";
  }

  static const char* value(const ::baxter_playfile_nodes::playfileSrvRequest_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::baxter_playfile_nodes::playfileSrvRequest_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.playfile_code);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct playfileSrvRequest_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::baxter_playfile_nodes::playfileSrvRequest_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::baxter_playfile_nodes::playfileSrvRequest_<ContainerAllocator>& v)
  {
    s << indent << "playfile_code: ";
    Printer<int32_t>::stream(s, indent + "  ", v.playfile_code);
  }
};

} // namespace message_operations
} // namespace ros

#endif // BAXTER_PLAYFILE_NODES_MESSAGE_PLAYFILESRVREQUEST_H
