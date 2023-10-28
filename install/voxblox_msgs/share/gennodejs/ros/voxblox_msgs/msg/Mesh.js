// Auto-generated. Do not edit!

// (in-package voxblox_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let MeshBlock = require('./MeshBlock.js');
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class Mesh {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.block_edge_length = null;
      this.mesh_blocks = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('block_edge_length')) {
        this.block_edge_length = initObj.block_edge_length
      }
      else {
        this.block_edge_length = 0.0;
      }
      if (initObj.hasOwnProperty('mesh_blocks')) {
        this.mesh_blocks = initObj.mesh_blocks
      }
      else {
        this.mesh_blocks = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Mesh
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [block_edge_length]
    bufferOffset = _serializer.float32(obj.block_edge_length, buffer, bufferOffset);
    // Serialize message field [mesh_blocks]
    // Serialize the length for message field [mesh_blocks]
    bufferOffset = _serializer.uint32(obj.mesh_blocks.length, buffer, bufferOffset);
    obj.mesh_blocks.forEach((val) => {
      bufferOffset = MeshBlock.serialize(val, buffer, bufferOffset);
    });
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Mesh
    let len;
    let data = new Mesh(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [block_edge_length]
    data.block_edge_length = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [mesh_blocks]
    // Deserialize array length for message field [mesh_blocks]
    len = _deserializer.uint32(buffer, bufferOffset);
    data.mesh_blocks = new Array(len);
    for (let i = 0; i < len; ++i) {
      data.mesh_blocks[i] = MeshBlock.deserialize(buffer, bufferOffset)
    }
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    object.mesh_blocks.forEach((val) => {
      length += MeshBlock.getMessageSize(val);
    });
    return length + 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'voxblox_msgs/Mesh';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4d589d8b6bcff9e45d11686cdd6227df';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # vertex positions rounded to nearest BLOCK_EDGE_LENGTH/65535 
    # no alpha information
    
    std_msgs/Header header
    
    float32 block_edge_length
    
    voxblox_msgs/MeshBlock[] mesh_blocks
    
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
    string frame_id
    
    ================================================================================
    MSG: voxblox_msgs/MeshBlock
    # vertex positions rounded to nearest BLOCK_EDGE_LENGTH/65535 
    # no alpha information
    
    # Index of meshed points in block map
    int64[3] index
    
    # Triangle information (always in groups of 3)
    uint16[] x
    uint16[] y
    uint16[] z
    
    # Color information may be missing
    uint8[] r
    uint8[] g
    uint8[] b
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Mesh(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.block_edge_length !== undefined) {
      resolved.block_edge_length = msg.block_edge_length;
    }
    else {
      resolved.block_edge_length = 0.0
    }

    if (msg.mesh_blocks !== undefined) {
      resolved.mesh_blocks = new Array(msg.mesh_blocks.length);
      for (let i = 0; i < resolved.mesh_blocks.length; ++i) {
        resolved.mesh_blocks[i] = MeshBlock.Resolve(msg.mesh_blocks[i]);
      }
    }
    else {
      resolved.mesh_blocks = []
    }

    return resolved;
    }
};

module.exports = Mesh;
