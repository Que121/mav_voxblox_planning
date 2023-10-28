// Auto-generated. Do not edit!

// (in-package voxblox_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class MeshBlock {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.index = null;
      this.x = null;
      this.y = null;
      this.z = null;
      this.r = null;
      this.g = null;
      this.b = null;
    }
    else {
      if (initObj.hasOwnProperty('index')) {
        this.index = initObj.index
      }
      else {
        this.index = new Array(3).fill(0);
      }
      if (initObj.hasOwnProperty('x')) {
        this.x = initObj.x
      }
      else {
        this.x = [];
      }
      if (initObj.hasOwnProperty('y')) {
        this.y = initObj.y
      }
      else {
        this.y = [];
      }
      if (initObj.hasOwnProperty('z')) {
        this.z = initObj.z
      }
      else {
        this.z = [];
      }
      if (initObj.hasOwnProperty('r')) {
        this.r = initObj.r
      }
      else {
        this.r = [];
      }
      if (initObj.hasOwnProperty('g')) {
        this.g = initObj.g
      }
      else {
        this.g = [];
      }
      if (initObj.hasOwnProperty('b')) {
        this.b = initObj.b
      }
      else {
        this.b = [];
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type MeshBlock
    // Check that the constant length array field [index] has the right length
    if (obj.index.length !== 3) {
      throw new Error('Unable to serialize array field index - length must be 3')
    }
    // Serialize message field [index]
    bufferOffset = _arraySerializer.int64(obj.index, buffer, bufferOffset, 3);
    // Serialize message field [x]
    bufferOffset = _arraySerializer.uint16(obj.x, buffer, bufferOffset, null);
    // Serialize message field [y]
    bufferOffset = _arraySerializer.uint16(obj.y, buffer, bufferOffset, null);
    // Serialize message field [z]
    bufferOffset = _arraySerializer.uint16(obj.z, buffer, bufferOffset, null);
    // Serialize message field [r]
    bufferOffset = _arraySerializer.uint8(obj.r, buffer, bufferOffset, null);
    // Serialize message field [g]
    bufferOffset = _arraySerializer.uint8(obj.g, buffer, bufferOffset, null);
    // Serialize message field [b]
    bufferOffset = _arraySerializer.uint8(obj.b, buffer, bufferOffset, null);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type MeshBlock
    let len;
    let data = new MeshBlock(null);
    // Deserialize message field [index]
    data.index = _arrayDeserializer.int64(buffer, bufferOffset, 3)
    // Deserialize message field [x]
    data.x = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [y]
    data.y = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [z]
    data.z = _arrayDeserializer.uint16(buffer, bufferOffset, null)
    // Deserialize message field [r]
    data.r = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [g]
    data.g = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    // Deserialize message field [b]
    data.b = _arrayDeserializer.uint8(buffer, bufferOffset, null)
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += 2 * object.x.length;
    length += 2 * object.y.length;
    length += 2 * object.z.length;
    length += object.r.length;
    length += object.g.length;
    length += object.b.length;
    return length + 48;
  }

  static datatype() {
    // Returns string type for a message object
    return 'voxblox_msgs/MeshBlock';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'a2921da2fac1fa36ed1e9d85b1e1b29c';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
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
    const resolved = new MeshBlock(null);
    if (msg.index !== undefined) {
      resolved.index = msg.index;
    }
    else {
      resolved.index = new Array(3).fill(0)
    }

    if (msg.x !== undefined) {
      resolved.x = msg.x;
    }
    else {
      resolved.x = []
    }

    if (msg.y !== undefined) {
      resolved.y = msg.y;
    }
    else {
      resolved.y = []
    }

    if (msg.z !== undefined) {
      resolved.z = msg.z;
    }
    else {
      resolved.z = []
    }

    if (msg.r !== undefined) {
      resolved.r = msg.r;
    }
    else {
      resolved.r = []
    }

    if (msg.g !== undefined) {
      resolved.g = msg.g;
    }
    else {
      resolved.g = []
    }

    if (msg.b !== undefined) {
      resolved.b = msg.b;
    }
    else {
      resolved.b = []
    }

    return resolved;
    }
};

module.exports = MeshBlock;
