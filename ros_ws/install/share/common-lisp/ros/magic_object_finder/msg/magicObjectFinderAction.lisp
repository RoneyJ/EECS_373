; Auto-generated. Do not edit!


(cl:in-package magic_object_finder-msg)


;//! \htmlinclude magicObjectFinderAction.msg.html

(cl:defclass <magicObjectFinderAction> (roslisp-msg-protocol:ros-message)
  ((action_goal
    :reader action_goal
    :initarg :action_goal
    :type magic_object_finder-msg:magicObjectFinderActionGoal
    :initform (cl:make-instance 'magic_object_finder-msg:magicObjectFinderActionGoal))
   (action_result
    :reader action_result
    :initarg :action_result
    :type magic_object_finder-msg:magicObjectFinderActionResult
    :initform (cl:make-instance 'magic_object_finder-msg:magicObjectFinderActionResult))
   (action_feedback
    :reader action_feedback
    :initarg :action_feedback
    :type magic_object_finder-msg:magicObjectFinderActionFeedback
    :initform (cl:make-instance 'magic_object_finder-msg:magicObjectFinderActionFeedback)))
)

(cl:defclass magicObjectFinderAction (<magicObjectFinderAction>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <magicObjectFinderAction>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'magicObjectFinderAction)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name magic_object_finder-msg:<magicObjectFinderAction> is deprecated: use magic_object_finder-msg:magicObjectFinderAction instead.")))

(cl:ensure-generic-function 'action_goal-val :lambda-list '(m))
(cl:defmethod action_goal-val ((m <magicObjectFinderAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magic_object_finder-msg:action_goal-val is deprecated.  Use magic_object_finder-msg:action_goal instead.")
  (action_goal m))

(cl:ensure-generic-function 'action_result-val :lambda-list '(m))
(cl:defmethod action_result-val ((m <magicObjectFinderAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magic_object_finder-msg:action_result-val is deprecated.  Use magic_object_finder-msg:action_result instead.")
  (action_result m))

(cl:ensure-generic-function 'action_feedback-val :lambda-list '(m))
(cl:defmethod action_feedback-val ((m <magicObjectFinderAction>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader magic_object_finder-msg:action_feedback-val is deprecated.  Use magic_object_finder-msg:action_feedback instead.")
  (action_feedback m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <magicObjectFinderAction>) ostream)
  "Serializes a message object of type '<magicObjectFinderAction>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_goal) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_result) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'action_feedback) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <magicObjectFinderAction>) istream)
  "Deserializes a message object of type '<magicObjectFinderAction>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_goal) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_result) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'action_feedback) istream)
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<magicObjectFinderAction>)))
  "Returns string type for a message object of type '<magicObjectFinderAction>"
  "magic_object_finder/magicObjectFinderAction")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'magicObjectFinderAction)))
  "Returns string type for a message object of type 'magicObjectFinderAction"
  "magic_object_finder/magicObjectFinderAction")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<magicObjectFinderAction>)))
  "Returns md5sum for a message object of type '<magicObjectFinderAction>"
  "0239f75a85aeb3e3a205ed02c54926b9")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'magicObjectFinderAction)))
  "Returns md5sum for a message object of type 'magicObjectFinderAction"
  "0239f75a85aeb3e3a205ed02c54926b9")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<magicObjectFinderAction>)))
  "Returns full string definition for message of type '<magicObjectFinderAction>"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%magicObjectFinderActionGoal action_goal~%magicObjectFinderActionResult action_result~%magicObjectFinderActionFeedback action_feedback~%~%================================================================================~%MSG: magic_object_finder/magicObjectFinderActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%magicObjectFinderGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: magic_object_finder/magicObjectFinderGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#objectFinder.action~%#goal:~%#get object ID codes from <object_manipulation_properties/object_ID_codes.h>~%#goal field to fill in: name of object of interest~%string object_name~%~%================================================================================~%MSG: magic_object_finder/magicObjectFinderActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%magicObjectFinderResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: magic_object_finder/magicObjectFinderResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result~%int32 SUCCESS =0 ~%int32 OBJECT_FOUND=0 #synonym for SUCCESS~%int32 OBJECT_NOT_FOUND=1~%int32 OBJECT_FINDER_CANCELLED=4~%#return the identified pose here:~%int32 found_object_code~%geometry_msgs/PoseStamped object_pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: magic_object_finder/magicObjectFinderActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%magicObjectFinderFeedback feedback~%~%================================================================================~%MSG: magic_object_finder/magicObjectFinderFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback: optional; ~%#int32 OBJECT_FINDER_BUSY=3 ~%#int32 fdbk~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'magicObjectFinderAction)))
  "Returns full string definition for message of type 'magicObjectFinderAction"
  (cl:format cl:nil "# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%magicObjectFinderActionGoal action_goal~%magicObjectFinderActionResult action_result~%magicObjectFinderActionFeedback action_feedback~%~%================================================================================~%MSG: magic_object_finder/magicObjectFinderActionGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalID goal_id~%magicObjectFinderGoal goal~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%================================================================================~%MSG: actionlib_msgs/GoalID~%# The stamp should store the time at which this goal was requested.~%# It is used by an action server when it tries to preempt all~%# goals that were requested before a certain time~%time stamp~%~%# The id provides a way to associate feedback and~%# result message with specific goal requests. The id~%# specified must be unique.~%string id~%~%~%================================================================================~%MSG: magic_object_finder/magicObjectFinderGoal~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#objectFinder.action~%#goal:~%#get object ID codes from <object_manipulation_properties/object_ID_codes.h>~%#goal field to fill in: name of object of interest~%string object_name~%~%================================================================================~%MSG: magic_object_finder/magicObjectFinderActionResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%magicObjectFinderResult result~%~%================================================================================~%MSG: actionlib_msgs/GoalStatus~%GoalID goal_id~%uint8 status~%uint8 PENDING         = 0   # The goal has yet to be processed by the action server~%uint8 ACTIVE          = 1   # The goal is currently being processed by the action server~%uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing~%                            #   and has since completed its execution (Terminal State)~%uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)~%uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due~%                            #    to some failure (Terminal State)~%uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,~%                            #    because the goal was unattainable or invalid (Terminal State)~%uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing~%                            #    and has not yet completed execution~%uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,~%                            #    but the action server has not yet confirmed that the goal is canceled~%uint8 RECALLED        = 8   # The goal received a cancel request before it started executing~%                            #    and was successfully cancelled (Terminal State)~%uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be~%                            #    sent over the wire by an action server~%~%#Allow for the user to associate a string with GoalStatus for debugging~%string text~%~%~%================================================================================~%MSG: magic_object_finder/magicObjectFinderResult~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#result~%int32 SUCCESS =0 ~%int32 OBJECT_FOUND=0 #synonym for SUCCESS~%int32 OBJECT_NOT_FOUND=1~%int32 OBJECT_FINDER_CANCELLED=4~%#return the identified pose here:~%int32 found_object_code~%geometry_msgs/PoseStamped object_pose~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%================================================================================~%MSG: magic_object_finder/magicObjectFinderActionFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%~%Header header~%actionlib_msgs/GoalStatus status~%magicObjectFinderFeedback feedback~%~%================================================================================~%MSG: magic_object_finder/magicObjectFinderFeedback~%# ====== DO NOT MODIFY! AUTOGENERATED FROM AN ACTION DEFINITION ======~%#feedback: optional; ~%#int32 OBJECT_FINDER_BUSY=3 ~%#int32 fdbk~%~%~%~%~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <magicObjectFinderAction>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_goal))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_result))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'action_feedback))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <magicObjectFinderAction>))
  "Converts a ROS message object to a list"
  (cl:list 'magicObjectFinderAction
    (cl:cons ':action_goal (action_goal msg))
    (cl:cons ':action_result (action_result msg))
    (cl:cons ':action_feedback (action_feedback msg))
))
