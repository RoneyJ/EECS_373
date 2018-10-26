// Generated by gencpp from file magic_object_finder/magicObjectFinderAction.msg
// DO NOT EDIT!


#ifndef MAGIC_OBJECT_FINDER_MESSAGE_MAGICOBJECTFINDERACTION_H
#define MAGIC_OBJECT_FINDER_MESSAGE_MAGICOBJECTFINDERACTION_H


#include <string>
#include <vector>
#include <map>

#include <ros/types.h>
#include <ros/serialization.h>
#include <ros/builtin_message_traits.h>
#include <ros/message_operations.h>

#include <magic_object_finder/magicObjectFinderActionGoal.h>
#include <magic_object_finder/magicObjectFinderActionResult.h>
#include <magic_object_finder/magicObjectFinderActionFeedback.h>

namespace magic_object_finder
{
template <class ContainerAllocator>
struct magicObjectFinderAction_
{
  typedef magicObjectFinderAction_<ContainerAllocator> Type;

  magicObjectFinderAction_()
    : action_goal()
    , action_result()
    , action_feedback()  {
    }
  magicObjectFinderAction_(const ContainerAllocator& _alloc)
    : action_goal(_alloc)
    , action_result(_alloc)
    , action_feedback(_alloc)  {
  (void)_alloc;
    }



   typedef  ::magic_object_finder::magicObjectFinderActionGoal_<ContainerAllocator>  _action_goal_type;
  _action_goal_type action_goal;

   typedef  ::magic_object_finder::magicObjectFinderActionResult_<ContainerAllocator>  _action_result_type;
  _action_result_type action_result;

   typedef  ::magic_object_finder::magicObjectFinderActionFeedback_<ContainerAllocator>  _action_feedback_type;
  _action_feedback_type action_feedback;





  typedef boost::shared_ptr< ::magic_object_finder::magicObjectFinderAction_<ContainerAllocator> > Ptr;
  typedef boost::shared_ptr< ::magic_object_finder::magicObjectFinderAction_<ContainerAllocator> const> ConstPtr;

}; // struct magicObjectFinderAction_

typedef ::magic_object_finder::magicObjectFinderAction_<std::allocator<void> > magicObjectFinderAction;

typedef boost::shared_ptr< ::magic_object_finder::magicObjectFinderAction > magicObjectFinderActionPtr;
typedef boost::shared_ptr< ::magic_object_finder::magicObjectFinderAction const> magicObjectFinderActionConstPtr;

// constants requiring out of line definition



template<typename ContainerAllocator>
std::ostream& operator<<(std::ostream& s, const ::magic_object_finder::magicObjectFinderAction_<ContainerAllocator> & v)
{
ros::message_operations::Printer< ::magic_object_finder::magicObjectFinderAction_<ContainerAllocator> >::stream(s, "", v);
return s;
}

} // namespace magic_object_finder

namespace ros
{
namespace message_traits
{



// BOOLTRAITS {'IsFixedSize': False, 'IsMessage': True, 'HasHeader': False}
// {'roscpp': ['/opt/ros/kinetic/share/roscpp/cmake/../msg'], 'magic_object_finder': ['/home/jproney/ros_ws/devel/share/magic_object_finder/msg'], 'std_msgs': ['/opt/ros/kinetic/share/std_msgs/cmake/../msg'], 'actionlib': ['/opt/ros/kinetic/share/actionlib/cmake/../msg'], 'trajectory_msgs': ['/opt/ros/kinetic/share/trajectory_msgs/cmake/../msg'], 'gazebo_msgs': ['/home/jproney/ros_ws/src/learning_ros_external_pkgs_kinetic/gazebo_ros_pkgs/gazebo_msgs/msg'], 'geometry_msgs': ['/opt/ros/kinetic/share/geometry_msgs/cmake/../msg'], 'sensor_msgs': ['/opt/ros/kinetic/share/sensor_msgs/cmake/../msg'], 'actionlib_msgs': ['/opt/ros/kinetic/share/actionlib_msgs/cmake/../msg']}

// !!!!!!!!!!! ['__class__', '__delattr__', '__dict__', '__doc__', '__eq__', '__format__', '__getattribute__', '__hash__', '__init__', '__module__', '__ne__', '__new__', '__reduce__', '__reduce_ex__', '__repr__', '__setattr__', '__sizeof__', '__str__', '__subclasshook__', '__weakref__', '_parsed_fields', 'constants', 'fields', 'full_name', 'has_header', 'header_present', 'names', 'package', 'parsed_fields', 'short_name', 'text', 'types']




template <class ContainerAllocator>
struct IsFixedSize< ::magic_object_finder::magicObjectFinderAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct IsFixedSize< ::magic_object_finder::magicObjectFinderAction_<ContainerAllocator> const>
  : FalseType
  { };

template <class ContainerAllocator>
struct IsMessage< ::magic_object_finder::magicObjectFinderAction_<ContainerAllocator> >
  : TrueType
  { };

template <class ContainerAllocator>
struct IsMessage< ::magic_object_finder::magicObjectFinderAction_<ContainerAllocator> const>
  : TrueType
  { };

template <class ContainerAllocator>
struct HasHeader< ::magic_object_finder::magicObjectFinderAction_<ContainerAllocator> >
  : FalseType
  { };

template <class ContainerAllocator>
struct HasHeader< ::magic_object_finder::magicObjectFinderAction_<ContainerAllocator> const>
  : FalseType
  { };


template<class ContainerAllocator>
struct MD5Sum< ::magic_object_finder::magicObjectFinderAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "0239f75a85aeb3e3a205ed02c54926b9";
  }

  static const char* value(const ::magic_object_finder::magicObjectFinderAction_<ContainerAllocator>&) { return value(); }
  static const uint64_t static_value1 = 0x0239f75a85aeb3e3ULL;
  static const uint64_t static_value2 = 0xa205ed02c54926b9ULL;
};

template<class ContainerAllocator>
struct DataType< ::magic_object_finder::magicObjectFinderAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "magic_object_finder/magicObjectFinderAction";
  }

  static const char* value(const ::magic_object_finder::magicObjectFinderAction_<ContainerAllocator>&) { return value(); }
};

template<class ContainerAllocator>
struct Definition< ::magic_object_finder::magicObjectFinderAction_<ContainerAllocator> >
{
  static const char* value()
  {
    return "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
magicObjectFinderActionGoal action_goal\n\
magicObjectFinderActionResult action_result\n\
magicObjectFinderActionFeedback action_feedback\n\
\n\
================================================================================\n\
MSG: magic_object_finder/magicObjectFinderActionGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalID goal_id\n\
magicObjectFinderGoal goal\n\
\n\
================================================================================\n\
MSG: std_msgs/Header\n\
# Standard metadata for higher-level stamped data types.\n\
# This is generally used to communicate timestamped data \n\
# in a particular coordinate frame.\n\
# \n\
# sequence ID: consecutively increasing ID \n\
uint32 seq\n\
#Two-integer timestamp that is expressed as:\n\
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')\n\
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')\n\
# time-handling sugar is provided by the client library\n\
time stamp\n\
#Frame this data is associated with\n\
# 0: no frame\n\
# 1: global frame\n\
string frame_id\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalID\n\
# The stamp should store the time at which this goal was requested.\n\
# It is used by an action server when it tries to preempt all\n\
# goals that were requested before a certain time\n\
time stamp\n\
\n\
# The id provides a way to associate feedback and\n\
# result message with specific goal requests. The id\n\
# specified must be unique.\n\
string id\n\
\n\
\n\
================================================================================\n\
MSG: magic_object_finder/magicObjectFinderGoal\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#objectFinder.action\n\
#goal:\n\
#get object ID codes from <object_manipulation_properties/object_ID_codes.h>\n\
#goal field to fill in: name of object of interest\n\
string object_name\n\
\n\
================================================================================\n\
MSG: magic_object_finder/magicObjectFinderActionResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
magicObjectFinderResult result\n\
\n\
================================================================================\n\
MSG: actionlib_msgs/GoalStatus\n\
GoalID goal_id\n\
uint8 status\n\
uint8 PENDING         = 0   # The goal has yet to be processed by the action server\n\
uint8 ACTIVE          = 1   # The goal is currently being processed by the action server\n\
uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing\n\
                            #   and has since completed its execution (Terminal State)\n\
uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)\n\
uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due\n\
                            #    to some failure (Terminal State)\n\
uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,\n\
                            #    because the goal was unattainable or invalid (Terminal State)\n\
uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing\n\
                            #    and has not yet completed execution\n\
uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,\n\
                            #    but the action server has not yet confirmed that the goal is canceled\n\
uint8 RECALLED        = 8   # The goal received a cancel request before it started executing\n\
                            #    and was successfully cancelled (Terminal State)\n\
uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be\n\
                            #    sent over the wire by an action server\n\
\n\
#Allow for the user to associate a string with GoalStatus for debugging\n\
string text\n\
\n\
\n\
================================================================================\n\
MSG: magic_object_finder/magicObjectFinderResult\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#result\n\
int32 SUCCESS =0 \n\
int32 OBJECT_FOUND=0 #synonym for SUCCESS\n\
int32 OBJECT_NOT_FOUND=1\n\
int32 OBJECT_FINDER_CANCELLED=4\n\
#return the identified pose here:\n\
int32 found_object_code\n\
geometry_msgs/PoseStamped object_pose\n\
\n\
================================================================================\n\
MSG: geometry_msgs/PoseStamped\n\
# A Pose with reference coordinate frame and timestamp\n\
Header header\n\
Pose pose\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Pose\n\
# A representation of pose in free space, composed of position and orientation. \n\
Point position\n\
Quaternion orientation\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Point\n\
# This contains the position of a point in free space\n\
float64 x\n\
float64 y\n\
float64 z\n\
\n\
================================================================================\n\
MSG: geometry_msgs/Quaternion\n\
# This represents an orientation in free space in quaternion form.\n\
\n\
float64 x\n\
float64 y\n\
float64 z\n\
float64 w\n\
\n\
================================================================================\n\
MSG: magic_object_finder/magicObjectFinderActionFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
\n\
Header header\n\
actionlib_msgs/GoalStatus status\n\
magicObjectFinderFeedback feedback\n\
\n\
================================================================================\n\
MSG: magic_object_finder/magicObjectFinderFeedback\n\
# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======\n\
#feedback: optional; \n\
#int32 OBJECT_FINDER_BUSY=3 \n\
#int32 fdbk\n\
\n\
\n\
\n\
\n\
";
  }

  static const char* value(const ::magic_object_finder::magicObjectFinderAction_<ContainerAllocator>&) { return value(); }
};

} // namespace message_traits
} // namespace ros

namespace ros
{
namespace serialization
{

  template<class ContainerAllocator> struct Serializer< ::magic_object_finder::magicObjectFinderAction_<ContainerAllocator> >
  {
    template<typename Stream, typename T> inline static void allInOne(Stream& stream, T m)
    {
      stream.next(m.action_goal);
      stream.next(m.action_result);
      stream.next(m.action_feedback);
    }

    ROS_DECLARE_ALLINONE_SERIALIZER
  }; // struct magicObjectFinderAction_

} // namespace serialization
} // namespace ros

namespace ros
{
namespace message_operations
{

template<class ContainerAllocator>
struct Printer< ::magic_object_finder::magicObjectFinderAction_<ContainerAllocator> >
{
  template<typename Stream> static void stream(Stream& s, const std::string& indent, const ::magic_object_finder::magicObjectFinderAction_<ContainerAllocator>& v)
  {
    s << indent << "action_goal: ";
    s << std::endl;
    Printer< ::magic_object_finder::magicObjectFinderActionGoal_<ContainerAllocator> >::stream(s, indent + "  ", v.action_goal);
    s << indent << "action_result: ";
    s << std::endl;
    Printer< ::magic_object_finder::magicObjectFinderActionResult_<ContainerAllocator> >::stream(s, indent + "  ", v.action_result);
    s << indent << "action_feedback: ";
    s << std::endl;
    Printer< ::magic_object_finder::magicObjectFinderActionFeedback_<ContainerAllocator> >::stream(s, indent + "  ", v.action_feedback);
  }
};

} // namespace message_operations
} // namespace ros

#endif // MAGIC_OBJECT_FINDER_MESSAGE_MAGICOBJECTFINDERACTION_H
