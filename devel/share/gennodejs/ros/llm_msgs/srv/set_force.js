// Auto-generated. Do not edit!

// (in-package llm_msgs.srv)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------


//-----------------------------------------------------------

class set_forceRequest {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.force0Ratio = null;
      this.force1Ratio = null;
      this.force2Ratio = null;
      this.force3Ratio = null;
      this.force4Ratio = null;
      this.force5Ratio = null;
    }
    else {
      if (initObj.hasOwnProperty('force0Ratio')) {
        this.force0Ratio = initObj.force0Ratio
      }
      else {
        this.force0Ratio = 0.0;
      }
      if (initObj.hasOwnProperty('force1Ratio')) {
        this.force1Ratio = initObj.force1Ratio
      }
      else {
        this.force1Ratio = 0.0;
      }
      if (initObj.hasOwnProperty('force2Ratio')) {
        this.force2Ratio = initObj.force2Ratio
      }
      else {
        this.force2Ratio = 0.0;
      }
      if (initObj.hasOwnProperty('force3Ratio')) {
        this.force3Ratio = initObj.force3Ratio
      }
      else {
        this.force3Ratio = 0.0;
      }
      if (initObj.hasOwnProperty('force4Ratio')) {
        this.force4Ratio = initObj.force4Ratio
      }
      else {
        this.force4Ratio = 0.0;
      }
      if (initObj.hasOwnProperty('force5Ratio')) {
        this.force5Ratio = initObj.force5Ratio
      }
      else {
        this.force5Ratio = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type set_forceRequest
    // Serialize message field [force0Ratio]
    bufferOffset = _serializer.float32(obj.force0Ratio, buffer, bufferOffset);
    // Serialize message field [force1Ratio]
    bufferOffset = _serializer.float32(obj.force1Ratio, buffer, bufferOffset);
    // Serialize message field [force2Ratio]
    bufferOffset = _serializer.float32(obj.force2Ratio, buffer, bufferOffset);
    // Serialize message field [force3Ratio]
    bufferOffset = _serializer.float32(obj.force3Ratio, buffer, bufferOffset);
    // Serialize message field [force4Ratio]
    bufferOffset = _serializer.float32(obj.force4Ratio, buffer, bufferOffset);
    // Serialize message field [force5Ratio]
    bufferOffset = _serializer.float32(obj.force5Ratio, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type set_forceRequest
    let len;
    let data = new set_forceRequest(null);
    // Deserialize message field [force0Ratio]
    data.force0Ratio = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [force1Ratio]
    data.force1Ratio = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [force2Ratio]
    data.force2Ratio = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [force3Ratio]
    data.force3Ratio = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [force4Ratio]
    data.force4Ratio = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [force5Ratio]
    data.force5Ratio = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 24;
  }

  static datatype() {
    // Returns string type for a service object
    return 'llm_msgs/set_forceRequest';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'b85daa8a508267c29c0e9abef4453eb9';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 force0Ratio
    float32 force1Ratio
    float32 force2Ratio
    float32 force3Ratio
    float32 force4Ratio
    float32 force5Ratio
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new set_forceRequest(null);
    if (msg.force0Ratio !== undefined) {
      resolved.force0Ratio = msg.force0Ratio;
    }
    else {
      resolved.force0Ratio = 0.0
    }

    if (msg.force1Ratio !== undefined) {
      resolved.force1Ratio = msg.force1Ratio;
    }
    else {
      resolved.force1Ratio = 0.0
    }

    if (msg.force2Ratio !== undefined) {
      resolved.force2Ratio = msg.force2Ratio;
    }
    else {
      resolved.force2Ratio = 0.0
    }

    if (msg.force3Ratio !== undefined) {
      resolved.force3Ratio = msg.force3Ratio;
    }
    else {
      resolved.force3Ratio = 0.0
    }

    if (msg.force4Ratio !== undefined) {
      resolved.force4Ratio = msg.force4Ratio;
    }
    else {
      resolved.force4Ratio = 0.0
    }

    if (msg.force5Ratio !== undefined) {
      resolved.force5Ratio = msg.force5Ratio;
    }
    else {
      resolved.force5Ratio = 0.0
    }

    return resolved;
    }
};

class set_forceResponse {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.force_accepted = null;
    }
    else {
      if (initObj.hasOwnProperty('force_accepted')) {
        this.force_accepted = initObj.force_accepted
      }
      else {
        this.force_accepted = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type set_forceResponse
    // Serialize message field [force_accepted]
    bufferOffset = _serializer.bool(obj.force_accepted, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type set_forceResponse
    let len;
    let data = new set_forceResponse(null);
    // Deserialize message field [force_accepted]
    data.force_accepted = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 1;
  }

  static datatype() {
    // Returns string type for a service object
    return 'llm_msgs/set_forceResponse';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '6133e5998441599a96899c74dc63d85d';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool force_accepted
    
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new set_forceResponse(null);
    if (msg.force_accepted !== undefined) {
      resolved.force_accepted = msg.force_accepted;
    }
    else {
      resolved.force_accepted = false
    }

    return resolved;
    }
};

module.exports = {
  Request: set_forceRequest,
  Response: set_forceResponse,
  md5sum() { return '6b0fdeb4ed7ee4c97030abfd78488ebb'; },
  datatype() { return 'llm_msgs/set_force'; }
};
