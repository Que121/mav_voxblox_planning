; Auto-generated. Do not edit!


(cl:in-package voxblox_msgs-msg)


;//! \htmlinclude Mesh.msg.html

(cl:defclass <Mesh> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (block_edge_length
    :reader block_edge_length
    :initarg :block_edge_length
    :type cl:float
    :initform 0.0)
   (mesh_blocks
    :reader mesh_blocks
    :initarg :mesh_blocks
    :type (cl:vector voxblox_msgs-msg:MeshBlock)
   :initform (cl:make-array 0 :element-type 'voxblox_msgs-msg:MeshBlock :initial-element (cl:make-instance 'voxblox_msgs-msg:MeshBlock))))
)

(cl:defclass Mesh (<Mesh>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <Mesh>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'Mesh)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name voxblox_msgs-msg:<Mesh> is deprecated: use voxblox_msgs-msg:Mesh instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <Mesh>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voxblox_msgs-msg:header-val is deprecated.  Use voxblox_msgs-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'block_edge_length-val :lambda-list '(m))
(cl:defmethod block_edge_length-val ((m <Mesh>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voxblox_msgs-msg:block_edge_length-val is deprecated.  Use voxblox_msgs-msg:block_edge_length instead.")
  (block_edge_length m))

(cl:ensure-generic-function 'mesh_blocks-val :lambda-list '(m))
(cl:defmethod mesh_blocks-val ((m <Mesh>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader voxblox_msgs-msg:mesh_blocks-val is deprecated.  Use voxblox_msgs-msg:mesh_blocks instead.")
  (mesh_blocks m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <Mesh>) ostream)
  "Serializes a message object of type '<Mesh>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:let ((bits (roslisp-utils:encode-single-float-bits (cl:slot-value msg 'block_edge_length))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream))
  (cl:let ((__ros_arr_len (cl:length (cl:slot-value msg 'mesh_blocks))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_arr_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_arr_len) ostream))
  (cl:map cl:nil #'(cl:lambda (ele) (roslisp-msg-protocol:serialize ele ostream))
   (cl:slot-value msg 'mesh_blocks))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <Mesh>) istream)
  "Deserializes a message object of type '<Mesh>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'block_edge_length) (roslisp-utils:decode-single-float-bits bits)))
  (cl:let ((__ros_arr_len 0))
    (cl:setf (cl:ldb (cl:byte 8 0) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 8) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 16) __ros_arr_len) (cl:read-byte istream))
    (cl:setf (cl:ldb (cl:byte 8 24) __ros_arr_len) (cl:read-byte istream))
  (cl:setf (cl:slot-value msg 'mesh_blocks) (cl:make-array __ros_arr_len))
  (cl:let ((vals (cl:slot-value msg 'mesh_blocks)))
    (cl:dotimes (i __ros_arr_len)
    (cl:setf (cl:aref vals i) (cl:make-instance 'voxblox_msgs-msg:MeshBlock))
  (roslisp-msg-protocol:deserialize (cl:aref vals i) istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<Mesh>)))
  "Returns string type for a message object of type '<Mesh>"
  "voxblox_msgs/Mesh")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'Mesh)))
  "Returns string type for a message object of type 'Mesh"
  "voxblox_msgs/Mesh")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<Mesh>)))
  "Returns md5sum for a message object of type '<Mesh>"
  "4d589d8b6bcff9e45d11686cdd6227df")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'Mesh)))
  "Returns md5sum for a message object of type 'Mesh"
  "4d589d8b6bcff9e45d11686cdd6227df")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<Mesh>)))
  "Returns full string definition for message of type '<Mesh>"
  (cl:format cl:nil "# vertex positions rounded to nearest BLOCK_EDGE_LENGTH/65535 ~%# no alpha information~%~%std_msgs/Header header~%~%float32 block_edge_length~%~%voxblox_msgs/MeshBlock[] mesh_blocks~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: voxblox_msgs/MeshBlock~%# vertex positions rounded to nearest BLOCK_EDGE_LENGTH/65535 ~%# no alpha information~%~%# Index of meshed points in block map~%int64[3] index~%~%# Triangle information (always in groups of 3)~%uint16[] x~%uint16[] y~%uint16[] z~%~%# Color information may be missing~%uint8[] r~%uint8[] g~%uint8[] b~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'Mesh)))
  "Returns full string definition for message of type 'Mesh"
  (cl:format cl:nil "# vertex positions rounded to nearest BLOCK_EDGE_LENGTH/65535 ~%# no alpha information~%~%std_msgs/Header header~%~%float32 block_edge_length~%~%voxblox_msgs/MeshBlock[] mesh_blocks~%~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: voxblox_msgs/MeshBlock~%# vertex positions rounded to nearest BLOCK_EDGE_LENGTH/65535 ~%# no alpha information~%~%# Index of meshed points in block map~%int64[3] index~%~%# Triangle information (always in groups of 3)~%uint16[] x~%uint16[] y~%uint16[] z~%~%# Color information may be missing~%uint8[] r~%uint8[] g~%uint8[] b~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <Mesh>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     4
     4 (cl:reduce #'cl:+ (cl:slot-value msg 'mesh_blocks) :key #'(cl:lambda (ele) (cl:declare (cl:ignorable ele)) (cl:+ (roslisp-msg-protocol:serialization-length ele))))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <Mesh>))
  "Converts a ROS message object to a list"
  (cl:list 'Mesh
    (cl:cons ':header (header msg))
    (cl:cons ':block_edge_length (block_edge_length msg))
    (cl:cons ':mesh_blocks (mesh_blocks msg))
))
