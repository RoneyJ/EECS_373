// Auto-generated. Do not edit!

// (in-package object_grabber.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let object_grabberGoal = require('./object_grabberGoal.js');
let actionlib_msgs = _finder('actionlib_msgs');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class object_grabberActionGoal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.goal_id = null;
      this.goal = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('goal_id')) {
        this.goal_id = initObj.goal_id
      }
      else {
        this.goal_id = new actionlib_msgs.msg.GoalID();
      }
      if (initObj.hasOwnProperty('goal')) {
        this.goal = initObj.goal
      }
      else {
        this.goal = new object_grabberGoal();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type object_grabberActionGoal
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [goal_id]
    bufferOffset = actionlib_msgs.msg.GoalID.serialize(obj.goal_id, buffer, bufferOffset);
    // Serialize message field [goal]
    bufferOffset = object_grabberGoal.serialize(obj.goal, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type object_grabberActionGoal
    let len;
    let data = new object_grabberActionGoal(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [goal_id]
    data.goal_id = actionlib_msgs.msg.GoalID.deserialize(buffer, bufferOffset);
    // Deserialize message field [goal]
    data.goal = object_grabberGoal.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += actionlib_msgs.msg.GoalID.getMessageSize(object.goal_id);
    length += object_grabberGoal.getMessageSize(object.goal);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'object_grabber/object_grabberActionGoal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '3ed7e2451ccf211a58f8be3a15ef1aeb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    Header header
    actionlib_msgs/GoalID goal_id
    object_grabberGoal goal
    
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    # 0: no frame
    # 1: global frame
    string frame_id
    
    ================================================================================
    MSG: actionlib_msgs/GoalID
    # The stamp should store the time at which this goal was requested.
    # It is used by an action server when it tries to preempt all
    # goals that were requested before a certain time
    time stamp
    
    # The id provides a way to associate feedback and
    # result message with specific goal requests. The id
    # specified must be unique.
    string id
    
    
    ================================================================================
    MSG: object_grabber/object_grabberGoal
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    #object_grabber action message
    #pass in an object code and the object's frame (w/rt named frame_id)
    #object_grabber will plan approach, grasp and lift of object
    #returns codes regarding outcome
    
    #action codes:
    int32 TEST_CODE = 0   #a simple "ping" to action server
    int32 MOVE_TO_WAITING_POSE = 1 #move to a pose, defined on param server, that is convenient
                                   #e.g., prepared to approach a surface, but out of way of sensors
    int32 PLAN_MOVE_TO_GRASP_POSE  =2   #expects parameters of current_object_pose, object_ID, grasp_option, approach_option
                                   #must send separate "grasp" command to gripper
    int32 PLAN_MOVE_FINE_TO_GRASP_POSE  =3   #as above, but finer/slower approach motion
    int32 PLAN_MOVE_OBJECT_JSPACE =4    #move a grasped object to a destination pose using simple, joint-space move
                                   #expects params: des_object_pose, object_ID, grasp_option (need to know how object is grasped)
    int32 PLAN_MOVE_OBJECT_CSPACE = 5   #move a grasped object with Cartesian motion to a desired object pose
                                   #params:  des_object_pose, object_ID, grasp_option
    int32 PLAN_MOVE_FINE_OBJECT_CSPACE = 6 #as above, but w/ finer, slower motion
    
    int32 PLAN_WITHDRAW_FROM_OBJECT = 7 #with object grasp released, perform departure from object using specified depart strategy
                                   #params: object_ID, grasp_option, depart_option
    int32 PLAN_WITHDRAW_FINE_FROM_OBJECT = 8 #as above, but slower/more precise motion
    
    int32 PLAN_OBJECT_GRASP = 9  #combine multiple elements above to acquire an object
    
    int32 CART_MOVE_CURRENT_TO_CART_GOAL = 10 #plan and execute a move from current pose to a cartesian goal, to be specified in "geometry_msgs/PoseStamped object_frame"
    
    int32 GRAB_OBJECT = 101 #plan and attempt to execute object acquisition, including grasp and lift
    int32 DROPOFF_OBJECT = 102 #plan and attempt to execute object placement and arm withdrawal
    int32 STRADDLE_OBJECT = 103 #plan and attempt to execute part of GRAB_OBJECT: move to grasp pose, then halt
    
    int32 SET_SPEED_FACTOR = 10    #use arg speed_factor to change time scale of trajectory plan; larger than 1.0--> slower
    
    #manipulation strategy options:
    int32 DEFAULT_GRASP_STRATEGY = 0
    #has corresponding default approach and depart strategies
    
    int32 EXECUTE_PLANNED_MOVE = 100 #accept arm-motion plan and enable its execution
    
    #generalized gripper commands:
    int32 GRIPPER_PREPARE_GRASP_OBJECT = 20 #may require gripper to prepare for grasp approach, e.g. open fingers
    int32 GRIPPER_GRASP_OBJECT = 21        #command to perform appropriate action to grasp object, 
                                   #assumes gripper is in appropriate pose, prepared to grasp object
                                   #client does not need to know what type of gripper is used
    int32 GRIPPER_RELEASE_OBJECT = 22      #command to gripper to release a grasped object
                                   #params: object ID and grasp option used; 
    int32 GRIPPER_IS_OBJECT_GRASPED = 23   #a query; may require object_ID, grasp option used, and/or corresponding grasp test parameters; 
                                   #should return true/false
    
    #goal:
    int32 action_code
    int32 object_id
    int32 grasp_option
    int32 approach_strategy
    int32 lift_object_strategy
    int32 dropoff_strategy
    int32 dropoff_withdraw_strategy
    geometry_msgs/PoseStamped object_frame  #should be w/rt system_ref_frame, or have tf to this frame available
    float64 speed_factor #default to 1.0
    float64[] gripper_test_params
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new object_grabberActionGoal(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.goal_id !== undefined) {
      resolved.goal_id = actionlib_msgs.msg.GoalID.Resolve(msg.goal_id)
    }
    else {
      resolved.goal_id = new actionlib_msgs.msg.GoalID()
    }

    if (msg.goal !== undefined) {
      resolved.goal = object_grabberGoal.Resolve(msg.goal)
    }
    else {
      resolved.goal = new object_grabberGoal()
    }

    return resolved;
    }
};

module.exports = object_grabberActionGoal;
