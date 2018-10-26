// Auto-generated. Do not edit!

// (in-package object_finder.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class objectFinderResult {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.found_object_code = null;
      this.object_pose = null;
      this.object_id = null;
    }
    else {
      if (initObj.hasOwnProperty('found_object_code')) {
        this.found_object_code = initObj.found_object_code
      }
      else {
        this.found_object_code = 0;
      }
      if (initObj.hasOwnProperty('object_pose')) {
        this.object_pose = initObj.object_pose
      }
      else {
        this.object_pose = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('object_id')) {
        this.object_id = initObj.object_id
      }
      else {
        this.object_id = 0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type objectFinderResult
    // Serialize message field [found_object_code]
    bufferOffset = _serializer.int32(obj.found_object_code, buffer, bufferOffset);
    // Serialize message field [object_pose]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.object_pose, buffer, bufferOffset);
    // Serialize message field [object_id]
    bufferOffset = _serializer.int32(obj.object_id, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type objectFinderResult
    let len;
    let data = new objectFinderResult(null);
    // Deserialize message field [found_object_code]
    data.found_object_code = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [object_pose]
    data.object_pose = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [object_id]
    data.object_id = _deserializer.int32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.object_pose);
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'object_finder/objectFinderResult';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a7900c52b913a2038f3ebf889a359d6e';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======
    #result
    int32 SUCCESS =0 
    int32 OBJECT_FOUND=0 #synonym for SUCCESS
    int32 OBJECT_NOT_FOUND=1
    int32 OBJECT_CODE_NOT_RECOGNIZED=2
    int32 OBJECT_FINDER_BUSY=3
    int32 OBJECT_FINDER_CANCELLED=4
    
    #return the identified pose here:
    int32 found_object_code
    geometry_msgs/PoseStamped object_pose
    int32 object_id
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
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
    const resolved = new objectFinderResult(null);
    if (msg.found_object_code !== undefined) {
      resolved.found_object_code = msg.found_object_code;
    }
    else {
      resolved.found_object_code = 0
    }

    if (msg.object_pose !== undefined) {
      resolved.object_pose = geometry_msgs.msg.PoseStamped.Resolve(msg.object_pose)
    }
    else {
      resolved.object_pose = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.object_id !== undefined) {
      resolved.object_id = msg.object_id;
    }
    else {
      resolved.object_id = 0
    }

    return resolved;
    }
};

// Constants for message
objectFinderResult.Constants = {
  SUCCESS: 0,
  OBJECT_FOUND: 0,
  OBJECT_NOT_FOUND: 1,
  OBJECT_CODE_NOT_RECOGNIZED: 2,
  OBJECT_FINDER_BUSY: 3,
  OBJECT_FINDER_CANCELLED: 4,
}

module.exports = objectFinderResult;
