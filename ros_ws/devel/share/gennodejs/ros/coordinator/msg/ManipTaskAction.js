// Auto-generated. Do not edit!

// (in-package coordinator.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let ManipTaskActionGoal = require('./ManipTaskActionGoal.js');
let ManipTaskActionResult = require('./ManipTaskActionResult.js');
let ManipTaskActionFeedback = require('./ManipTaskActionFeedback.js');

//-----------------------------------------------------------

class ManipTaskAction {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.action_goal = null;
      this.action_result = null;
      this.action_feedback = null;
    }
    else {
      if (initObj.hasOwnProperty('action_goal')) {
        this.action_goal = initObj.action_goal
      }
      else {
        this.action_goal = new ManipTaskActionGoal();
      }
      if (initObj.hasOwnProperty('action_result')) {
        this.action_result = initObj.action_result
      }
      else {
        this.action_result = new ManipTaskActionResult();
      }
      if (initObj.hasOwnProperty('action_feedback')) {
        this.action_feedback = initObj.action_feedback
      }
      else {
        this.action_feedback = new ManipTaskActionFeedback();
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type ManipTaskAction
    // Serialize message field [action_goal]
    bufferOffset = ManipTaskActionGoal.serialize(obj.action_goal, buffer, bufferOffset);
    // Serialize message field [action_result]
    bufferOffset = ManipTaskActionResult.serialize(obj.action_result, buffer, bufferOffset);
    // Serialize message field [action_feedback]
    bufferOffset = ManipTaskActionFeedback.serialize(obj.action_feedback, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type ManipTaskAction
    let len;
    let data = new ManipTaskAction(null);
    // Deserialize message field [action_goal]
    data.action_goal = ManipTaskActionGoal.deserialize(buffer, bufferOffset);
    // Deserialize message field [action_result]
    data.action_result = ManipTaskActionResult.deserialize(buffer, bufferOffset);
    // Deserialize message field [action_feedback]
    data.action_feedback = ManipTaskActionFeedback.deserialize(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += ManipTaskActionGoal.getMessageSize(object.action_goal);
    length += ManipTaskActionResult.getMessageSize(object.action_result);
    length += ManipTaskActionFeedback.getMessageSize(object.action_feedback);
    return length;
  }

  static datatype() {
    // Returns string type for a message object
    return 'coordinator/ManipTaskAction';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b0d72a10459cfe87a5243cfd55aca4ab';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    ManipTaskActionGoal action_goal
    ManipTaskActionResult action_result
    ManipTaskActionFeedback action_feedback
    
    ================================================================================
    MSG: coordinator/ManipTaskActionGoal
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    Header header
    actionlib_msgs/GoalID goal_id
    ManipTaskGoal goal
    
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
    MSG: coordinator/ManipTaskGoal
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    #goal: specify object code, perception type (incl blind), dropoff location, optional pickup location
    #task codes:
    int32 PCL_VISION = 1 # could have more camera sources to refer to
    int32 BLIND_MANIP = 2 #expect pose specified in pickup_location field
    
    #action codes:
    int32 GET_PICKUP_POSE = 1 #low level ops; maybe don't implement
    int32 WAIT_FOR_FINDER = 101
    
    int32 FIND_TABLE_SURFACE = 2 #do this once, and re-use result for multiple objects
    int32 WAIT_FIND_TABLE_SURFACE = 102
    
    int32 GRAB_OBJECT = 3 #assumes use of current pickup pose,
                          #whether provided for "blind" manip, or found
                          #from use of PCL_VISION
    int32 WAIT_FOR_GRAB_OBJECT = 103    
                     
    int32 DROPOFF_OBJECT = 4 #must provide dropoff_frame in goal msg
    int32 WAIT_FOR_DROPOFF_OBJECT = 104
    
    #int32 MANIP_OBJECT = 5 #macro: does perception, grab, and dropoff
                           #MUST provide dropoff frame, and means to
                           #get pickup_frame
    
    int32 STRADDLE_OBJECT = 8 #test mode--simply straddle object, but don't grasp it
    int32 WAIT_FOR_STRADDLE_OBJECT = 108
    
    int32 CART_MOVE_TO_GRIPPER_POSE = 9
    int32 WAIT_FOR_CART_MOVE = 109
                           
    int32 NO_CURRENT_TASK = 6
    int32 MOVE_TO_PRE_POSE = 7
    int32 WAIT_FOR_MOVE_TO_PREPOSE = 107
    int32 WAIT_FOR_MOVE = 107 #generic wait-for-move status
    
    int32 ABORT= 666
    
    #goal specification:
    int32 action_code #what action should be performed?
    int32 object_code #refer to a-priori known object types by object-ID codes
    geometry_msgs/PoseStamped pickup_frame #specify object coords for pickup 
    geometry_msgs/PoseStamped dropoff_frame #specify desired drop-off coords of object's frame
    geometry_msgs/PoseStamped gripper_goal_frame #a goal frame to move gripper
    int32 perception_source  #e.g. name a camera source
    
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
    
    ================================================================================
    MSG: coordinator/ManipTaskActionResult
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    Header header
    actionlib_msgs/GoalStatus status
    ManipTaskResult result
    
    ================================================================================
    MSG: actionlib_msgs/GoalStatus
    GoalID goal_id
    uint8 status
    uint8 PENDING         = 0   # The goal has yet to be processed by the action server
    uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
    uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
                                #   and has since completed its execution (Terminal State)
    uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
    uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
                                #    to some failure (Terminal State)
    uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
                                #    because the goal was unattainable or invalid (Terminal State)
    uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
                                #    and has not yet completed execution
    uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
                                #    but the action server has not yet confirmed that the goal is canceled
    uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
                                #    and was successfully cancelled (Terminal State)
    uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
                                #    sent over the wire by an action server
    
    #Allow for the user to associate a string with GoalStatus for debugging
    string text
    
    
    ================================================================================
    MSG: coordinator/ManipTaskResult
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    #return codes:
    int32 MANIP_SUCCESS = 0
    int32 FAILED_PERCEPTION = 1
    int32 FAILED_PICKUP_PLAN =2 
    int32 FAILED_DROPOFF_PLAN=3
    int32 FAILED_PICKUP=4
    int32 FAILED_DROPOFF=5
    int32 DROPPED_OBJECT = 6
    int32 ABORTED = 7
    int32 PENDING = 8
    int32 FAILED_MOVE = 9
    
    int32 manip_return_code
    int32 object_grabber_return_code
    geometry_msgs/PoseStamped des_gripper_pose
    int32 object_finder_return_code
    geometry_msgs/PoseStamped object_pose
    
    
    ================================================================================
    MSG: coordinator/ManipTaskActionFeedback
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    
    Header header
    actionlib_msgs/GoalStatus status
    ManipTaskFeedback feedback
    
    ================================================================================
    MSG: coordinator/ManipTaskFeedback
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    #feedback codes:  perception_busy; planning_busy; pickup_busy; dropoff_busy
    int32 RECEIVED_NEW_TASK = 0
    int32 PERCEPTION_BUSY = 1
    int32 PICKUP_PLANNING_BUSY = 2
    int32 PICKUP_MOTION_BUSY = 3
    int32 PICKUP_SUCCESSFUL = 103
    int32 DROPOFF_PLANNING_BUSY = 4
    int32 DROPOFF_MOTION_BUSY = 5
    int32 NO_CURRENT_TASK = 6
    int32 ABORTED = 7
    int32 COMPLETED_MOVE = 8
    int32 COMPLETED_DROPOFF = 9
    int32 PREPOSE_MOVE_BUSY = 10
    int32 MOVE_BUSY = 11
    
    
    int32 feedback_status
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new ManipTaskAction(null);
    if (msg.action_goal !== undefined) {
      resolved.action_goal = ManipTaskActionGoal.Resolve(msg.action_goal)
    }
    else {
      resolved.action_goal = new ManipTaskActionGoal()
    }

    if (msg.action_result !== undefined) {
      resolved.action_result = ManipTaskActionResult.Resolve(msg.action_result)
    }
    else {
      resolved.action_result = new ManipTaskActionResult()
    }

    if (msg.action_feedback !== undefined) {
      resolved.action_feedback = ManipTaskActionFeedback.Resolve(msg.action_feedback)
    }
    else {
      resolved.action_feedback = new ManipTaskActionFeedback()
    }

    return resolved;
    }
};

module.exports = ManipTaskAction;
